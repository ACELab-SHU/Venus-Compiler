//===- RegAllocBase.cpp - Register Allocator Base Class -------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the RegAllocBase class which provides common functionality
// for LiveIntervalUnion-based register allocators.
//
//===----------------------------------------------------------------------===//

#include "RegAllocBase.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/CodeGen/LiveInterval.h"
#include "llvm/CodeGen/LiveIntervals.h"
#include "llvm/CodeGen/LiveRegMatrix.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/Spiller.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/CodeGen/VirtRegMap.h"
#include "llvm/Pass.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/Timer.h"
#include "llvm/Support/raw_ostream.h"
#include <cassert>
#include <limits>

#define GET_REGINFO_ENUM
#include "../lib/Target/RISCV/RISCVGenRegisterInfo.inc"
#define GET_INSTRINFO_ENUM
#include "../lib/Target/RISCV/RISCVGenInstrInfo.inc"

using namespace llvm;

#define DEBUG_TYPE "regalloc"

STATISTIC(NumNewQueued, "Number of new live ranges queued");

// Temporary verification option until we can put verification inside
// MachineVerifier.
static cl::opt<bool, true>
    VerifyRegAlloc("verify-regalloc", cl::location(RegAllocBase::VerifyEnabled),
                   cl::Hidden, cl::desc("Verify during register allocation"));

const char RegAllocBase::TimerGroupName[] = "regalloc";
const char RegAllocBase::TimerGroupDescription[] = "Register Allocation";
bool RegAllocBase::VerifyEnabled = false;

//===----------------------------------------------------------------------===//
//                         RegAllocBase Implementation
//===----------------------------------------------------------------------===//

// Pin the vtable to this file.
void RegAllocBase::anchor() {}

void RegAllocBase::init(VirtRegMap &vrm, LiveIntervals &lis,
                        LiveRegMatrix &mat) {
  TRI = &vrm.getTargetRegInfo();
  MRI = &vrm.getRegInfo();
  VRM = &vrm;
  LIS = &lis;
  Matrix = &mat;
  MRI->freezeReservedRegs(vrm.getMachineFunction());
  RegClassInfo.runOnMachineFunction(vrm.getMachineFunction());
}

// Visit all the live registers. If they are already assigned to a physical
// register, unify them with the corresponding LiveIntervalUnion, otherwise push
// them on the priority queue for later assignment.
void RegAllocBase::seedLiveRegs() {
  NamedRegionTimer T("seed", "Seed Live Regs", TimerGroupName,
                     TimerGroupDescription, TimePassesIsEnabled);
  unsigned VenusConsecRegs = 0;
  unsigned cmx_VenusConsecRegs = 0;
  float tempLI;
  unsigned virt_Reg_num0;
  for (unsigned i = 0, e = MRI->getNumVirtRegs(); i != e; ++i) {
    Register Reg = Register::index2VirtReg(i);

    if (MRI->reg_nodbg_empty(Reg))
      continue;
    if(CmxVirtRegWeigthtMap.find(Reg) != CmxVirtRegWeigthtMap.end())
      continue;
    // The following `if` condition is to set the LiveIntervals of consecutive
    // virtual venus registers into the same value.
    // TODO: It is only valid when `-regalloc=basic`
    // If it is an undef VNSR, do not care
    std::map<Register, std::list<MachineInstr *>> RegDefMap;
    MachineInstr *delimitMI;
    MachineInstr *currentMI;
    if (MRI->getRegClass(Reg)->getID() == RISCV::VNSRRegClassID &&
        !MRI->def_empty(Reg)) {
      bool isFirstConsecIntrinsic = false;
      std::list<MachineInstr *> MISet =
          MRI->getMultipleVRegDef(Reg);
      if (MISet.size() == 1){
        delimitMI = MISet.front();
        }
      else {
        if (RegDefMap.find(Reg) == RegDefMap.end())
          RegDefMap[Reg] = MISet;
        std::list<MachineInstr *> OldMISet = RegDefMap[Reg];
        for (auto &MI : OldMISet) {
          if (MI->getOpcode() == RISCV::venus_cmxmul_ivv) {
            MachineInstr *next_delimitMI;
            next_delimitMI = MI->getNextNode();
            LLVM_DEBUG(llvm::dbgs()  << "getNextNode:" << "\n");
            LLVM_DEBUG(next_delimitMI->dump());
            if (next_delimitMI->getOperand(0).isReg()){
              virt_Reg_num0 = Register::virtReg2Index(next_delimitMI->getOperand(0).getReg());
              unsigned current_reg = Register::virtReg2Index(MI->getOperand(0).getReg());
              if (virt_Reg_num0 == (current_reg + 2)){
                cmx_flag = true;
                }
              }
          }
        }
        delimitMI = OldMISet.front();
        OldMISet.pop_front();
        RegDefMap[Reg] = OldMISet;
      }
      
      const MachineBasicBlock::iterator MBBI =
          delimitMI->getParent()->begin();
      currentMI = delimitMI;
      LLVM_DEBUG(llvm::dbgs()  << "current:" << "\n");
      LLVM_DEBUG(currentMI->dump());
      // Look above to find venus_delimit
      while (!VenusConsecRegs &&
             delimitMI->getOpcode() != RISCV::venus_delimit) {
        // First check whether the prevNode is legal
        if (&(*MBBI) == delimitMI) break;
        delimitMI = delimitMI->getPrevNode();
        // LLVM_DEBUG(delimitMI->dump());

        // Then check whether the prevNode is venus_delimit
        if (delimitMI->getOpcode() == RISCV::venus_delimit)
          isFirstConsecIntrinsic = true;
        // The instructions in between must be ADDI or LUI or COPY
        if (delimitMI->getOpcode() != RISCV::ADDI &&
            delimitMI->getOpcode() != RISCV::LUI ) break;
      }

      if (isFirstConsecIntrinsic) {
        VenusConsecRegs = delimitMI->getOperand(0).getImm() - 1;
        tempLI = LIS->getInterval(Reg).weight();
        currentMI = delimitMI->getNextNode();
        VenusVirtRegWeightMap[Reg] = VenusConsecRegs + 1;
        if (cmx_flag) {
          for (unsigned i = 0; i < VenusConsecRegs + 1; ++i) {
            while ((currentMI->getOperand(0).isReg() &&
              MRI->getRegClass(currentMI->getOperand(0).getReg())->getID() != RISCV::VNSRRegClassID) ||
              currentMI->getOpcode() == 19 || !currentMI->getOperand(0).isReg()) {

              LLVM_DEBUG(llvm::dbgs() << "current222:\n");
              LLVM_DEBUG(currentMI->dump());
              currentMI = currentMI->getNextNode();
              }
          
            CmxVirtRegWeigthtMap[currentMI->getOperand(0).getReg()] = tempLI;
            LIS->getInterval(currentMI->getOperand(0).getReg()).setWeight(tempLI); 
            LLVM_DEBUG(dbgs() << "cmx:" << Register::virtReg2Index(Reg) << '\n');
            LLVM_DEBUG(dbgs() << "cmx:" << Register::virtReg2Index(currentMI->getOperand(0).getReg()) << '\n');
            LLVM_DEBUG(dbgs() << "tempLI:" << tempLI<< '\n');
            currentMI = currentMI->getNextNode();
          }
          currentMI = delimitMI->getNextNode();
          cmx_VenusConsecRegs = VenusConsecRegs;
          VenusConsecRegs = 0;
        }
      } else if (VenusConsecRegs) {
        LIS->getInterval(Reg).setWeight(tempLI);
        VenusConsecRegs--;
      }
    }
    
    if (cmx_flag) {
      for (unsigned i = 0; i < cmx_VenusConsecRegs + 1; ++i) {
        while ((currentMI->getOperand(0).isReg() &&
                MRI->getRegClass(currentMI->getOperand(0).getReg())->getID() != RISCV::VNSRRegClassID) ||
                currentMI->getOpcode() == 19 || !currentMI->getOperand(0).isReg()) {
        LLVM_DEBUG(llvm::dbgs() << "current222:\n");
        LLVM_DEBUG(currentMI->dump());
        currentMI = currentMI->getNextNode();
      }
        enqueue(&LIS->getInterval(currentMI->getOperand(0).getReg()));
        currentMI = currentMI->getNextNode();
      }
      cmx_flag = false;
    }
    else enqueue(&LIS->getInterval(Reg));
  }
}

// Top-level driver to manage the queue of unassigned VirtRegs and call the
// selectOrSplit implementation.
void RegAllocBase::allocatePhysRegs() {
  seedLiveRegs();

  // Continue assigning vregs one at a time to available physical registers.
  while (const LiveInterval *VirtReg = dequeue()) {
    LLVM_DEBUG(dbgs() << "number virtreg " << Register::virtReg2Index(VirtReg->reg()) << '\n');
    assert(!VRM->hasPhys(VirtReg->reg()) && "Register already assigned");

    // Unused registers can appear when the spiller coalesces snippets.
    if (MRI->reg_nodbg_empty(VirtReg->reg())) {
      LLVM_DEBUG(dbgs() << "Dropping unused " << *VirtReg << '\n');
      aboutToRemoveInterval(*VirtReg);
      LIS->removeInterval(VirtReg->reg());
      continue;
    }

    // Invalidate all interference queries, live ranges could have changed.
    Matrix->invalidateVirtRegs();

    // selectOrSplit requests the allocator to return an available physical
    // register if possible and populate a list of new live intervals that
    // result from splitting.
    // LLVM_DEBUG(dbgs() << "\nselectOrSplit "
    //                   << TRI->getRegClassName(MRI->getRegClass(VirtReg->reg()))
    //                   << ':' << *VirtReg << " w=" << VirtReg->weight() << '\n');

    using VirtRegVec = SmallVector<Register, 4>;

    VirtRegVec SplitVRegs;
    MCRegister AvailablePhysReg = selectOrSplit(*VirtReg, SplitVRegs);

    if (AvailablePhysReg == ~0u) {
      // selectOrSplit failed to find a register!
      // Probably caused by an inline asm.
      MachineInstr *MI = nullptr;
      for (MachineRegisterInfo::reg_instr_iterator
               I = MRI->reg_instr_begin(VirtReg->reg()),
               E = MRI->reg_instr_end();
           I != E;) {
        MI = &*(I++);
        if (MI->isInlineAsm())
          break;
      }

      const TargetRegisterClass *RC = MRI->getRegClass(VirtReg->reg());
      ArrayRef<MCPhysReg> AllocOrder = RegClassInfo.getOrder(RC);
      if (AllocOrder.empty())
        report_fatal_error("no registers from class available to allocate");
      else if (MI && MI->isInlineAsm()) {
        MI->emitError("inline assembly requires more registers than available");
      } else if (MI) {
        LLVMContext &Context =
            MI->getParent()->getParent()->getMMI().getModule()->getContext();
        Context.emitError("ran out of registers during register allocation");
      } else {
        report_fatal_error("ran out of registers during register allocation");
      }

      // Keep going after reporting the error.
      VRM->assignVirt2Phys(VirtReg->reg(), AllocOrder.front());
    } else if (AvailablePhysReg)
      Matrix->assign(*VirtReg, AvailablePhysReg);

    for (Register Reg : SplitVRegs) {
      assert(LIS->hasInterval(Reg));

      LiveInterval *SplitVirtReg = &LIS->getInterval(Reg);
      assert(!VRM->hasPhys(SplitVirtReg->reg()) && "Register already assigned");
      if (MRI->reg_nodbg_empty(SplitVirtReg->reg())) {
        assert(SplitVirtReg->empty() && "Non-empty but used interval");
        LLVM_DEBUG(dbgs() << "not queueing unused  " << *SplitVirtReg << '\n');
        aboutToRemoveInterval(*SplitVirtReg);
        LIS->removeInterval(SplitVirtReg->reg());
        continue;
      }
      LLVM_DEBUG(dbgs() << "queuing new interval: " << *SplitVirtReg << "\n");
      assert(Register::isVirtualRegister(SplitVirtReg->reg()) &&
             "expect split value in virtual register");
      enqueue(SplitVirtReg);
      ++NumNewQueued;
    }
  }
}

void RegAllocBase::postOptimization() {
  spiller().postOptimization();
  for (auto *DeadInst : DeadRemats) {
    LIS->RemoveMachineInstrFromMaps(*DeadInst);
    DeadInst->eraseFromParent();
  }
  DeadRemats.clear();
}

void RegAllocBase::enqueue(const LiveInterval *LI) {
  const Register Reg = LI->reg();

  assert(Reg.isVirtual() && "Can only enqueue virtual registers");

  if (VRM->hasPhys(Reg))
    return;

  const TargetRegisterClass &RC = *MRI->getRegClass(Reg);
  if (ShouldAllocateClass(*TRI, RC)) {
    LLVM_DEBUG(dbgs() << "Enqueuing " << printReg(Reg, TRI) << '\n');
    enqueueImpl(LI);
  } else {
    LLVM_DEBUG(dbgs() << "Not enqueueing " << printReg(Reg, TRI)
                      << " in skipped register class\n");
  }
}

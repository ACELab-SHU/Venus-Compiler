//===-- RegAllocBasic.cpp - Basic Register Allocator ----------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the RABasic function pass, which provides a minimal
// implementation of the basic register allocator.
//
//===----------------------------------------------------------------------===//

#include "AllocationOrder.h"
#include "LiveDebugVariables.h"
#include "RegAllocBase.h"
#include "llvm/Analysis/AliasAnalysis.h"
#include "llvm/CodeGen/CalcSpillWeights.h"
#include "llvm/CodeGen/LiveIntervals.h"
#include "llvm/CodeGen/LiveRangeEdit.h"
#include "llvm/CodeGen/LiveRegMatrix.h"
#include "llvm/CodeGen/LiveStacks.h"
#include "llvm/CodeGen/MachineBlockFrequencyInfo.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineLoopInfo.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/RegAllocRegistry.h"
#include "llvm/CodeGen/Spiller.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/CodeGen/VirtRegMap.h"
#include "llvm/Pass.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/raw_ostream.h"
#include <queue>

using namespace llvm;

#define DEBUG_TYPE "regalloc"

static RegisterRegAlloc basicRegAlloc("basic", "basic register allocator",
                                      createBasicRegisterAllocator);

namespace {
  struct CompSpillWeight {
    bool operator()(const LiveInterval *A, const LiveInterval *B) const {
      return A->weight() < B->weight();
    }
  };

  struct VenusCompSpillWeight {
    bool operator()(const LiveInterval *A, const LiveInterval *B) const {
      // FIXME: Not sure whether the `!=` is appropriate
      if (A->weight() != B->weight()) return A->weight() < B->weight();
      // if the LIs are the same, we prefer the smaller virt reg.
      else return A->reg() > B->reg();
    }
  };
}

namespace {
/// RABasic provides a minimal implementation of the basic register allocation
/// algorithm. It prioritizes live virtual registers by spill weight and spills
/// whenever a register is unavailable. This is not practical in production but
/// provides a useful baseline both for measuring other allocators and comparing
/// the speed of the basic algorithm against other styles of allocators.
class RABasic : public MachineFunctionPass,
                public RegAllocBase,
                private LiveRangeEdit::Delegate {
  // context
  MachineFunction *MF;

  // state
  std::unique_ptr<Spiller> SpillerInstance;
  std::priority_queue<const LiveInterval *, std::vector<const LiveInterval *>,
                      VenusCompSpillWeight>
      Queue;

  // Scratch space.  Allocated here to avoid repeated malloc calls in
  // selectOrSplit().
  BitVector UsableRegs;

  bool LRE_CanEraseVirtReg(Register) override;
  void LRE_WillShrinkVirtReg(Register) override;

public:
  RABasic(const RegClassFilterFunc F = allocateAllRegClasses);

  /// Return the pass name.
  StringRef getPassName() const override { return "Basic Register Allocator"; }

  /// RABasic analysis usage.
  void getAnalysisUsage(AnalysisUsage &AU) const override;

  void releaseMemory() override;

  Spiller &spiller() override { return *SpillerInstance; }

  void enqueueImpl(const LiveInterval *LI) override { Queue.push(LI); }

  const LiveInterval *dequeue() override {
    if (Queue.empty())
      return nullptr;
    const LiveInterval *LI = Queue.top();
    Queue.pop();
    return LI;
  }

  MCRegister selectOrSplit(const LiveInterval &VirtReg,
                           SmallVectorImpl<Register> &SplitVRegs) override;

  /// Perform register allocation.
  bool runOnMachineFunction(MachineFunction &mf) override;

  MachineFunctionProperties getRequiredProperties() const override {
    return MachineFunctionProperties().set(
        MachineFunctionProperties::Property::NoPHIs);
  }

  MachineFunctionProperties getClearedProperties() const override {
    return MachineFunctionProperties().set(
      MachineFunctionProperties::Property::IsSSA);
  }

  // Helper for spilling all live virtual registers currently unified under preg
  // that interfere with the most recently queried lvr.  Return true if spilling
  // was successful, and append any new spilled/split intervals to splitLVRs.
  bool spillInterferences(const LiveInterval &VirtReg, MCRegister PhysReg,
                          SmallVectorImpl<Register> &SplitVRegs);

  static char ID;
};

char RABasic::ID = 0;

} // end anonymous namespace

char &llvm::RABasicID = RABasic::ID;

INITIALIZE_PASS_BEGIN(RABasic, "regallocbasic", "Basic Register Allocator",
                      false, false)
INITIALIZE_PASS_DEPENDENCY(LiveDebugVariables)
INITIALIZE_PASS_DEPENDENCY(SlotIndexes)
INITIALIZE_PASS_DEPENDENCY(LiveIntervals)
INITIALIZE_PASS_DEPENDENCY(RegisterCoalescer)
INITIALIZE_PASS_DEPENDENCY(MachineScheduler)
INITIALIZE_PASS_DEPENDENCY(LiveStacks)
INITIALIZE_PASS_DEPENDENCY(AAResultsWrapperPass)
INITIALIZE_PASS_DEPENDENCY(MachineDominatorTree)
INITIALIZE_PASS_DEPENDENCY(MachineLoopInfo)
INITIALIZE_PASS_DEPENDENCY(VirtRegMap)
INITIALIZE_PASS_DEPENDENCY(LiveRegMatrix)
INITIALIZE_PASS_END(RABasic, "regallocbasic", "Basic Register Allocator", false,
                    false)

bool RABasic::LRE_CanEraseVirtReg(Register VirtReg) {
  LiveInterval &LI = LIS->getInterval(VirtReg);
  if (VRM->hasPhys(VirtReg)) {
    Matrix->unassign(LI);
    aboutToRemoveInterval(LI);
    return true;
  }
  // Unassigned virtreg is probably in the priority queue.
  // RegAllocBase will erase it after dequeueing.
  // Nonetheless, clear the live-range so that the debug
  // dump will show the right state for that VirtReg.
  LI.clear();
  return false;
}

void RABasic::LRE_WillShrinkVirtReg(Register VirtReg) {
  if (!VRM->hasPhys(VirtReg))
    return;

  // Register is assigned, put it back on the queue for reassignment.
  LiveInterval &LI = LIS->getInterval(VirtReg);
  Matrix->unassign(LI);
  enqueue(&LI);
}

RABasic::RABasic(RegClassFilterFunc F):
  MachineFunctionPass(ID),
  RegAllocBase(F) {
}

void RABasic::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.setPreservesCFG();
  AU.addRequired<AAResultsWrapperPass>();
  AU.addPreserved<AAResultsWrapperPass>();
  AU.addRequired<LiveIntervals>();
  AU.addPreserved<LiveIntervals>();
  AU.addPreserved<SlotIndexes>();
  AU.addRequired<LiveDebugVariables>();
  AU.addPreserved<LiveDebugVariables>();
  AU.addRequired<LiveStacks>();
  AU.addPreserved<LiveStacks>();
  AU.addRequired<MachineBlockFrequencyInfo>();
  AU.addPreserved<MachineBlockFrequencyInfo>();
  AU.addRequiredID(MachineDominatorsID);
  AU.addPreservedID(MachineDominatorsID);
  AU.addRequired<MachineLoopInfo>();
  AU.addPreserved<MachineLoopInfo>();
  AU.addRequired<VirtRegMap>();
  AU.addPreserved<VirtRegMap>();
  AU.addRequired<LiveRegMatrix>();
  AU.addPreserved<LiveRegMatrix>();
  MachineFunctionPass::getAnalysisUsage(AU);
}

void RABasic::releaseMemory() {
  SpillerInstance.reset();
}


// Spill or split all live virtual registers currently unified under PhysReg
// that interfere with VirtReg. The newly spilled or split live intervals are
// returned by appending them to SplitVRegs.
bool RABasic::spillInterferences(const LiveInterval &VirtReg,
                                 MCRegister PhysReg,
                                 SmallVectorImpl<Register> &SplitVRegs) {
  // Record each interference and determine if all are spillable before mutating
  // either the union or live intervals.
  SmallVector<const LiveInterval *, 8> Intfs;

  // Collect interferences assigned to any alias of the physical register.
  for (MCRegUnitIterator Units(PhysReg, TRI); Units.isValid(); ++Units) {
    LiveIntervalUnion::Query &Q = Matrix->query(VirtReg, *Units);
    for (const auto *Intf : reverse(Q.interferingVRegs())) {
      if (!Intf->isSpillable() || Intf->weight() > VirtReg.weight())
        return false;
      Intfs.push_back(Intf);
    }
  }
  LLVM_DEBUG(dbgs() << "spilling " << printReg(PhysReg, TRI)
                    << " interferences with " << VirtReg << "\n");
  assert(!Intfs.empty() && "expected interference");

  // Spill each interfering vreg allocated to PhysReg or an alias.
  for (unsigned i = 0, e = Intfs.size(); i != e; ++i) {
    const LiveInterval &Spill = *Intfs[i];

    // Skip duplicates.
    if (!VRM->hasPhys(Spill.reg()))
      continue;

    // Deallocate the interfering vreg by removing it from the union.
    // A LiveInterval instance may not be in a union during modification!
    Matrix->unassign(Spill);

    // Spill the extracted interval.
    LiveRangeEdit LRE(&Spill, SplitVRegs, *MF, *LIS, VRM, this, &DeadRemats);
    spiller().spill(LRE);
  }
  return true;
}

// Driver for the register assignment and splitting heuristics.
// Manages iteration over the LiveIntervalUnions.
//
// This is a minimal implementation of register assignment and splitting that
// spills whenever we run out of registers.
//
// selectOrSplit can only be called once per live virtual register. We then do a
// single interference test for each register the correct class until we find an
// available register. So, the number of interference tests in the worst case is
// |vregs| * |machineregs|. And since the number of interference tests is
// minimal, there is no value in caching them outside the scope of
// selectOrSplit().
MCRegister RABasic::selectOrSplit(const LiveInterval &VirtReg,
                                  SmallVectorImpl<Register> &SplitVRegs) {
  // Populate a list of physical register spill candidates.
  SmallVector<MCRegister, 8> PhysRegSpillCands;
  unsigned VenusConsecRegs = 1; 
  LLVM_DEBUG(dbgs() << "rightnow: " << Register::virtReg2Index(VirtReg.reg()) << '\n');
  if(Register::virtReg2Index(VirtReg.reg()) == 216) {  
    LLVM_DEBUG(dbgs() << "which136: " << Register::virtReg2Index(VirtReg.reg()) << '\n');    }
  if(Register::virtReg2Index(VirtReg.reg()) == 218) {  
  LLVM_DEBUG(dbgs() << "which22: " << Register::virtReg2Index(VirtReg.reg()) << '\n');    }

  auto It = VenusVirtRegWeightMap.find(VirtReg.reg());
  if (It != VenusVirtRegWeightMap.end()) {
    VenusConsecRegs = It->second; // Retrieve the number of registers that need to be allocated consecutively.
}

  // Check for an available register in this class.
  auto Order =
      AllocationOrder::create(VirtReg.reg(), *VRM, RegClassInfo, Matrix);

  if (!ConsecutiveRegs.empty()) {
    // If there are available registers in ConsecutiveRegs, allocate the first one.
    MCRegister AllocReg = *ConsecutiveRegs.begin();
    ConsecutiveRegs.erase(ConsecutiveRegs.begin()); // Remove this register after allocation.
    return AllocReg;
  }
  
  std::vector<MCRegister> TempRegs;  
  if (VenusConsecRegs > 1) {
    for (auto It = Order.begin(); It != Order.end(); ++It) { 
        MCRegister firstReg = *It;  
        LLVM_DEBUG(dbgs() << "firstReg: " << firstReg.id() << '\n');
        TempRegs.clear();
        auto CheckIt = It;  
        bool Found = true;
        unsigned fore_phyreg = 0;
        std::priority_queue<const LiveInterval *, std::vector<const LiveInterval *>, 
                    VenusCompSpillWeight> TempQueue = Queue;
        if (It == Order.end()) { 
          Found = false;
          assert(It == Order.end() && "CheckIt is not at Order.end(), check your logic!");
          break;
        } 

        // Check VenusConsecRegs consecutive registers.
        for (unsigned j = 0; j < VenusConsecRegs; j++) {
          if (CheckIt == Order.end()) { 
            Found = false;
            assert(CheckIt == Order.end() && "CheckIt is not at Order.end(), check your logic!");
            break;
          } 
          MCRegister PhysReg = *CheckIt;   
          if(j > 0) {
            const LiveInterval *CurVirtReg = TempQueue.top();
            if(fore_phyreg + 1 != PhysReg.id()){
              Found = false;
              break;  // If the condition is not met, switch to the next starting point.
            }
            if (!PhysReg.isValid() || 
              Matrix->checkInterference(*CurVirtReg, PhysReg) != LiveRegMatrix::IK_Free) {
              Found = false;
              break;  // If the condition is not met, switch to the next starting point.
            }
          TempQueue.pop();
          }
          else{
            if (!PhysReg.isValid() || 
                Matrix->checkInterference(VirtReg, PhysReg) != LiveRegMatrix::IK_Free) {
                Found = false;
                break;  // If the condition is not met, switch to the next starting point.
            }
          }
          TempRegs.push_back(PhysReg);  
          fore_phyreg = PhysReg.id();
          ++CheckIt;  // Continue checking the next register.
        }
        
        if (Found) {
          ConsecutiveRegs = TempRegs;  // Record the found register.
          ConsecutiveRegs.erase(ConsecutiveRegs.begin());
          return firstReg;  // Return the first register.
        }
    }
}
  
  for (MCRegister PhysReg : Order) {
    assert(PhysReg.isValid());
    // Check for interference in PhysReg
    switch (Matrix->checkInterference(VirtReg, PhysReg)) {
    case LiveRegMatrix::IK_Free:
      // PhysReg is available, allocate it.
      return PhysReg;

    case LiveRegMatrix::IK_VirtReg:
      // Only virtual registers in the way, we may be able to spill them.
      PhysRegSpillCands.push_back(PhysReg);
      continue;

    default:
      // RegMask or RegUnit interference.
      continue;
    }
  }

  // Try to spill another interfering reg with less spill weight.
  for (MCRegister &PhysReg : PhysRegSpillCands) {
    if (!spillInterferences(VirtReg, PhysReg, SplitVRegs))
      continue;

    assert(!Matrix->checkInterference(VirtReg, PhysReg) &&
           "Interference after spill.");
    // Tell the caller to allocate to this newly freed physical register.
    return PhysReg;
  }

  // No other spill candidates were found, so spill the current VirtReg.
  LLVM_DEBUG(dbgs() << "spilling: " << VirtReg << '\n');
  if (!VirtReg.isSpillable())
    return ~0u;
  LiveRangeEdit LRE(&VirtReg, SplitVRegs, *MF, *LIS, VRM, this, &DeadRemats);
  spiller().spill(LRE);

  // The live virtual register requesting allocation was spilled, so tell
  // the caller not to allocate anything during this round.
  return 0;
}

bool RABasic::runOnMachineFunction(MachineFunction &mf) {
  LLVM_DEBUG(dbgs() << "********** BASIC REGISTER ALLOCATION **********\n"
                    << "********** Function: " << mf.getName() << '\n');

  MF = &mf;
  RegAllocBase::init(getAnalysis<VirtRegMap>(),
                     getAnalysis<LiveIntervals>(),
                     getAnalysis<LiveRegMatrix>());
  VirtRegAuxInfo VRAI(*MF, *LIS, *VRM, getAnalysis<MachineLoopInfo>(),
                      getAnalysis<MachineBlockFrequencyInfo>());
  VRAI.calculateSpillWeightsAndHints();

  SpillerInstance.reset(createInlineSpiller(*this, *MF, *VRM, VRAI));

  allocatePhysRegs();
  postOptimization();

  // Diagnostic output before rewriting
  LLVM_DEBUG(dbgs() << "Post alloc VirtRegMap:\n" << *VRM << "\n");

  releaseMemory();
  return true;
}

FunctionPass* llvm::createBasicRegisterAllocator() {
  return new RABasic();
}

FunctionPass* llvm::createBasicRegisterAllocator(RegClassFilterFunc F) {
  return new RABasic(F);
}

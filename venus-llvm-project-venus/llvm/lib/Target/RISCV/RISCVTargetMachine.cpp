//===-- RISCVTargetMachine.cpp - Define TargetMachine for RISCV -----------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// Implements the info about RISCV target spec.
//
//===----------------------------------------------------------------------===//

#include "RISCVTargetMachine.h"
#include "MCTargetDesc/RISCVBaseInfo.h"
#include "RISCV.h"
#include "RISCVMachineFunctionInfo.h"
#include "RISCVMacroFusion.h"
#include "RISCVTargetObjectFile.h"
#include "RISCVTargetTransformInfo.h"
#include "TargetInfo/RISCVTargetInfo.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/Analysis/TargetTransformInfo.h"
#include "llvm/CodeGen/GlobalISel/IRTranslator.h"
#include "llvm/CodeGen/GlobalISel/InstructionSelect.h"
#include "llvm/CodeGen/GlobalISel/Legalizer.h"
#include "llvm/CodeGen/GlobalISel/RegBankSelect.h"
#include "llvm/CodeGen/MIRParser/MIParser.h"
#include "llvm/CodeGen/MIRYamlMapping.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"
#include "llvm/CodeGen/TargetPassConfig.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/InitializePasses.h"
#include "llvm/MC/TargetRegistry.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Target/TargetOptions.h"
#include "llvm/Transforms/IPO.h"
using namespace llvm;

static cl::opt<bool> EnableRedundantCopyElimination(
    "riscv-enable-copyelim",
    cl::desc("Enable the redundant copy elimination pass"), cl::init(true),
    cl::Hidden);

static cl::opt<bool> EnableVenusAluAnlysis(
    "enable-venus-alu-anlysis",
    cl::desc("Enable analyze task-alu pass"),
    cl::Optional,
    cl::init(false)
);

static cl::opt<bool>
    EnableVnsBindRemove("enable-vns-bind-remove",
                            cl::desc("Should enable remove assembly specific instruction Vns_bind"),
                            cl::Optional, cl::init(false));

static cl::opt<bool> DisableVenusInstModifyLoadStore(
    "venus-disable-vimls",
    cl::desc("Disable the Venus instruction modify load/store pass"),
    cl::init(false)
    );

static cl::opt<bool> DisableVenusMerge(
    "venus-disable-vm",
    cl::desc("Disable the Venus Merge pass"),
    cl::init(false)
);

static cl::opt<bool> EnableVenusRegShift(
    "venus-enable-vrs",
    cl::desc("Enable the Venus Register Shifting pass"),
    cl::init(false)
);

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeRISCVTarget() {
  RegisterTargetMachine<RISCVTargetMachine> X(getTheRISCV32Target());
  RegisterTargetMachine<RISCVTargetMachine> Y(getTheRISCV64Target());
  auto *PR = PassRegistry::getPassRegistry();
  initializeGlobalISel(*PR);
  initializeRISCVMakeCompressibleOptPass(*PR);
  initializeRISCVGatherScatterLoweringPass(*PR);
  initializeRISCVCodeGenPreparePass(*PR);
  initializeRISCVMergeBaseOffsetOptPass(*PR);
  initializeRISCVSExtWRemovalPass(*PR);
  initializeRISCVExpandPseudoPass(*PR);
  initializeRISCVInsertVSETVLIPass(*PR);
  initializeRISCVVenusMergePass(*PR);
  initializeRISCVVenusAluPass(*PR);
  initializeRISCVVenusInstModifyLoadStorePass(*PR);
  initializeRISCVVenusBindRemovePass(*PR);
  initializeRISCVVenusRegShiftPass(*PR);
}

static StringRef computeDataLayout(const Triple &TT) {
  if (TT.isArch64Bit())
    return "e-m:e-p:64:64-i64:64-i128:128-n64-S128";
  assert(TT.isArch32Bit() && "only RV32 and RV64 are currently supported");
  return "e-m:e-p:32:32-i64:64-n32-S128";
}

static Reloc::Model getEffectiveRelocModel(const Triple &TT,
                                           Optional<Reloc::Model> RM) {
  return RM.value_or(Reloc::Static);
}

RISCVTargetMachine::RISCVTargetMachine(const Target &T, const Triple &TT,
                                       StringRef CPU, StringRef FS,
                                       const TargetOptions &Options,
                                       Optional<Reloc::Model> RM,
                                       Optional<CodeModel::Model> CM,
                                       CodeGenOpt::Level OL, bool JIT)
    : LLVMTargetMachine(T, computeDataLayout(TT), TT, CPU, FS, Options,
                        getEffectiveRelocModel(TT, RM),
                        getEffectiveCodeModel(CM, CodeModel::Small), OL),
      TLOF(std::make_unique<RISCVELFTargetObjectFile>()) {
  initAsmInfo();

  // RISC-V supports the MachineOutliner.
  setMachineOutliner(true);
  setSupportsDefaultOutlining(true);
}

const RISCVSubtarget *
RISCVTargetMachine::getSubtargetImpl(const Function &F) const {
  Attribute CPUAttr = F.getFnAttribute("target-cpu");
  Attribute TuneAttr = F.getFnAttribute("tune-cpu");
  Attribute FSAttr = F.getFnAttribute("target-features");

  std::string CPU =
      CPUAttr.isValid() ? CPUAttr.getValueAsString().str() : TargetCPU;
  std::string TuneCPU =
      TuneAttr.isValid() ? TuneAttr.getValueAsString().str() : CPU;
  std::string FS =
      FSAttr.isValid() ? FSAttr.getValueAsString().str() : TargetFS;
  std::string Key = CPU + TuneCPU + FS;
  auto &I = SubtargetMap[Key];
  if (!I) {
    // This needs to be done before we create a new subtarget since any
    // creation will depend on the TM and the code generation flags on the
    // function that reside in TargetOptions.
    resetTargetOptions(F);
    auto ABIName = Options.MCOptions.getABIName();
    if (const MDString *ModuleTargetABI = dyn_cast_or_null<MDString>(
            F.getParent()->getModuleFlag("target-abi"))) {
      auto TargetABI = RISCVABI::getTargetABI(ABIName);
      if (TargetABI != RISCVABI::ABI_Unknown &&
          ModuleTargetABI->getString() != ABIName) {
        report_fatal_error("-target-abi option != target-abi module flag");
      }
      ABIName = ModuleTargetABI->getString();
    }
    I = std::make_unique<RISCVSubtarget>(TargetTriple, CPU, TuneCPU, FS, ABIName, *this);
  }
  return I.get();
}

TargetTransformInfo
RISCVTargetMachine::getTargetTransformInfo(const Function &F) const {
  return TargetTransformInfo(RISCVTTIImpl(this, F));
}

// A RISC-V hart has a single byte-addressable address space of 2^XLEN bytes
// for all memory accesses, so it is reasonable to assume that an
// implementation has no-op address space casts. If an implementation makes a
// change to this, they can override it here.
bool RISCVTargetMachine::isNoopAddrSpaceCast(unsigned SrcAS,
                                             unsigned DstAS) const {
  return true;
}

namespace {
class RISCVPassConfig : public TargetPassConfig {
public:
  RISCVPassConfig(RISCVTargetMachine &TM, PassManagerBase &PM)
      : TargetPassConfig(TM, PM) {}

  RISCVTargetMachine &getRISCVTargetMachine() const {
    return getTM<RISCVTargetMachine>();
  }

  ScheduleDAGInstrs *
  createMachineScheduler(MachineSchedContext *C) const override {
    const RISCVSubtarget &ST = C->MF->getSubtarget<RISCVSubtarget>();
    if (ST.hasMacroFusion()) {
      ScheduleDAGMILive *DAG = createGenericSchedLive(C);
      DAG->addMutation(createRISCVMacroFusionDAGMutation());
      return DAG;
    }
    return nullptr;
  }

  ScheduleDAGInstrs *
  createPostMachineScheduler(MachineSchedContext *C) const override {
    const RISCVSubtarget &ST = C->MF->getSubtarget<RISCVSubtarget>();
    if (ST.hasMacroFusion()) {
      ScheduleDAGMI *DAG = createGenericSchedPostRA(C);
      DAG->addMutation(createRISCVMacroFusionDAGMutation());
      return DAG;
    }
    return nullptr;
  }

  void addIRPasses() override;
  bool addPreISel() override;
  bool addInstSelector() override;
  bool addIRTranslator() override;
  bool addLegalizeMachineIR() override;
  bool addRegBankSelect() override;
  bool addGlobalInstructionSelect() override;
  void addPreEmitPass() override;
  void addPreEmitPass2() override;
  void addPreSched2() override;
  void addMachineSSAOptimization() override;
  void addPreRegAlloc() override;
  void addPostRegAlloc() override;
};
} // namespace

TargetPassConfig *RISCVTargetMachine::createPassConfig(PassManagerBase &PM) {
  return new RISCVPassConfig(*this, PM);
}

void RISCVPassConfig::addIRPasses() {
  addPass(createAtomicExpandPass());

  if (getOptLevel() != CodeGenOpt::None)
    addPass(createRISCVGatherScatterLoweringPass());

  if (getOptLevel() != CodeGenOpt::None)
    addPass(createRISCVCodeGenPreparePass());

  TargetPassConfig::addIRPasses();
}

bool RISCVPassConfig::addPreISel() {
  if (TM->getOptLevel() != CodeGenOpt::None) {
    // Add a barrier before instruction selection so that we will not get
    // deleted block address after enabling default outlining. See D99707 for
    // more details.
    addPass(createBarrierNoopPass());
  }
  return false;
}

bool RISCVPassConfig::addInstSelector() {
  addPass(createRISCVISelDag(getRISCVTargetMachine(), getOptLevel()));

  return false;
}

bool RISCVPassConfig::addIRTranslator() {
  addPass(new IRTranslator(getOptLevel()));
  return false;
}

bool RISCVPassConfig::addLegalizeMachineIR() {
  addPass(new Legalizer());
  return false;
}

bool RISCVPassConfig::addRegBankSelect() {
  addPass(new RegBankSelect());
  return false;
}

bool RISCVPassConfig::addGlobalInstructionSelect() {
  addPass(new InstructionSelect(getOptLevel()));
  return false;
}

void RISCVPassConfig::addPreSched2() {}

void RISCVPassConfig::addPreEmitPass() {
  addPass(&BranchRelaxationPassID);
  addPass(createRISCVMakeCompressibleOptPass());
}

void RISCVPassConfig::addPreEmitPass2() {
  addPass(createRISCVExpandPseudoPass());
  // Schedule the expansion of AMOs at the last possible moment, avoiding the
  // possibility for other passes to break the requirements for forward
  // progress in the LR/SC block.
  addPass(createRISCVExpandAtomicPseudoPass());
}

void RISCVPassConfig::addMachineSSAOptimization() {
  TargetPassConfig::addMachineSSAOptimization();

  if (TM->getTargetTriple().getArch() == Triple::riscv64)
    addPass(createRISCVSExtWRemovalPass());
}

void RISCVPassConfig::addPreRegAlloc() {
  if (TM->getOptLevel() != CodeGenOpt::None)
    addPass(createRISCVMergeBaseOffsetOptPass());
  addPass(createRISCVInsertVSETVLIPass());
}

void RISCVPassConfig::addPostRegAlloc() {
  if (TM->getOptLevel() != CodeGenOpt::None && EnableRedundantCopyElimination)
    addPass(createRISCVRedundantCopyEliminationPass());
  if (EnableVenusRegShift)
    addPass(createRISCVVenusRegShiftPass());
  if (!DisableVenusMerge)
    addPass(createRISCVVenusMergePass());
  if (!DisableVenusInstModifyLoadStore)
    addPass(createRISCVVenusInstModifyLoadStorePass());
  if (EnableVnsBindRemove)
    addPass(createRISCVVenusBindRemovePass());   
  if (EnableVenusAluAnlysis)
    addPass(createRISCVVenusAluPass());
}

yaml::MachineFunctionInfo *
RISCVTargetMachine::createDefaultFuncInfoYAML() const {
  return new yaml::RISCVMachineFunctionInfo();
}

yaml::MachineFunctionInfo *
RISCVTargetMachine::convertFuncInfoToYAML(const MachineFunction &MF) const {
  const auto *MFI = MF.getInfo<RISCVMachineFunctionInfo>();
  return new yaml::RISCVMachineFunctionInfo(*MFI);
}

bool RISCVTargetMachine::parseMachineFunctionInfo(
    const yaml::MachineFunctionInfo &MFI, PerFunctionMIParsingState &PFS,
    SMDiagnostic &Error, SMRange &SourceRange) const {
  const auto &YamlMFI =
      static_cast<const yaml::RISCVMachineFunctionInfo &>(MFI);
  PFS.MF.getInfo<RISCVMachineFunctionInfo>()->initializeBaseYamlFields(YamlMFI);
  return false;
}

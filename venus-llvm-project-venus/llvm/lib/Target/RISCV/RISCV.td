//===-- RISCV.td - Describe the RISCV Target Machine -------*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

include "llvm/Target/Target.td"

//===----------------------------------------------------------------------===//
// RISC-V subtarget features and instruction predicates.
//===----------------------------------------------------------------------===//

def FeatureStdExtM
    : SubtargetFeature<"m", "HasStdExtM", "true",
                       "'M' (Integer Multiplication and Division)">;
def HasStdExtM : Predicate<"Subtarget->hasStdExtM()">,
                           AssemblerPredicate<(all_of FeatureStdExtM),
                           "'M' (Integer Multiplication and Division)">;

def FeatureStdExtZmmul
    : SubtargetFeature<"zmmul", "HasStdExtZmmul", "true",
                       "'Zmmul' (Integer Multiplication)">;
def HasStdExtZmmul : Predicate<"Subtarget->hasStdExtZmmul()">,
                               AssemblerPredicate<(all_of FeatureStdExtZmmul),
                               "'Zmmul' (Integer Multiplication)">;

def HasStdExtMOrZmmul
    : Predicate<"Subtarget->hasStdExtM() || Subtarget->hasStdExtZmmul()">,
                AssemblerPredicate<(any_of FeatureStdExtM, FeatureStdExtZmmul),
                                   "'M' (Integer Multiplication and Division) or "
                                   "'Zmmul' (Integer Multiplication)">;

def FeatureStdExtA
    : SubtargetFeature<"a", "HasStdExtA", "true",
                       "'A' (Atomic Instructions)">;
def HasStdExtA : Predicate<"Subtarget->hasStdExtA()">,
                           AssemblerPredicate<(all_of FeatureStdExtA),
                           "'A' (Atomic Instructions)">;

def FeatureStdExtF
    : SubtargetFeature<"f", "HasStdExtF", "true",
                       "'F' (Single-Precision Floating-Point)">;
def HasStdExtF : Predicate<"Subtarget->hasStdExtF()">,
                           AssemblerPredicate<(all_of FeatureStdExtF),
                           "'F' (Single-Precision Floating-Point)">;

def FeatureStdExtD
    : SubtargetFeature<"d", "HasStdExtD", "true",
                       "'D' (Double-Precision Floating-Point)",
                       [FeatureStdExtF]>;
def HasStdExtD : Predicate<"Subtarget->hasStdExtD()">,
                           AssemblerPredicate<(all_of FeatureStdExtD),
                           "'D' (Double-Precision Floating-Point)">;

def FeatureStdExtZihintpause
    : SubtargetFeature<"zihintpause", "HasStdExtZihintpause", "true",
                       "'zihintpause' (Pause Hint)">;
def HasStdExtZihintpause : Predicate<"Subtarget->hasStdExtZihintpause()">,
                                     AssemblerPredicate<(all_of FeatureStdExtZihintpause),
                                     "'Zihintpause' (Pause Hint)">;

def FeatureStdExtZfhmin
    : SubtargetFeature<"zfhmin", "HasStdExtZfhmin", "true",
                       "'Zfhmin' (Half-Precision Floating-Point Minimal)",
                       [FeatureStdExtF]>;
def HasStdExtZfhmin : Predicate<"Subtarget->hasStdExtZfhmin()">,
                             AssemblerPredicate<(all_of FeatureStdExtZfhmin),
                             "'Zfhmin' (Half-Precision Floating-Point Minimal)">;

def FeatureStdExtZfh
    : SubtargetFeature<"zfh", "HasStdExtZfh", "true",
                       "'Zfh' (Half-Precision Floating-Point)",
                       [FeatureStdExtF]>;
def HasStdExtZfh : Predicate<"Subtarget->hasStdExtZfh()">,
                             AssemblerPredicate<(all_of FeatureStdExtZfh),
                             "'Zfh' (Half-Precision Floating-Point)">;

def HasStdExtZfhOrZfhmin
    : Predicate<"Subtarget->hasStdExtZfh() || Subtarget->hasStdExtZfhmin()">,
                AssemblerPredicate<(any_of FeatureStdExtZfh, FeatureStdExtZfhmin),
                                   "'Zfh' (Half-Precision Floating-Point) or "
                                   "'Zfhmin' (Half-Precision Floating-Point Minimal)">;

def FeatureStdExtZfinx
    : SubtargetFeature<"zfinx", "HasStdExtZfinx", "true",
                       "'Zfinx' (Float in Integer)">;
def HasStdExtZfinx : Predicate<"Subtarget->hasStdExtZfinx()">,
                               AssemblerPredicate<(all_of FeatureStdExtZfinx),
                               "'Zfinx' (Float in Integer)">;

def FeatureStdExtZdinx
    : SubtargetFeature<"zdinx", "HasStdExtZdinx", "true",
                       "'Zdinx' (Double in Integer)",
                       [FeatureStdExtZfinx]>;
def HasStdExtZdinx : Predicate<"Subtarget->hasStdExtZdinx()">,
                               AssemblerPredicate<(all_of FeatureStdExtZdinx),
                               "'Zdinx' (Double in Integer)">;

def FeatureStdExtZhinxmin
    : SubtargetFeature<"zhinxmin", "HasStdExtZhinxmin", "true",
                       "'Zhinxmin' (Half Float in Integer Minimal)",
                       [FeatureStdExtZfinx]>;
def HasStdExtZhinxmin : Predicate<"Subtarget->hasStdExtZhinxmin()">,
                                  AssemblerPredicate<(all_of FeatureStdExtZhinxmin),
                                  "'Zhinxmin' (Half Float in Integer Minimal)">;

def FeatureStdExtZhinx
    : SubtargetFeature<"zhinx", "HasStdExtZhinx", "true",
                       "'Zhinx' (Half Float in Integer)",
                       [FeatureStdExtZfinx]>;
def HasStdExtZhinx : Predicate<"Subtarget->hasStdExtZhinx()">,
                               AssemblerPredicate<(all_of FeatureStdExtZhinx),
                               "'Zhinx' (Half Float in Integer)">;

def HasStdExtZhinxOrZhinxmin
    : Predicate<"Subtarget->hasStdExtZhinx() || Subtarget->hasStdExtZhinxmin()">,
                AssemblerPredicate<(any_of FeatureStdExtZhinx, FeatureStdExtZhinxmin),
                                   "'Zhinx' (Half Float in Integer) or "
                                   "'Zhinxmin' (Half Float in Integer Minimal)">;

def FeatureStdExtC
    : SubtargetFeature<"c", "HasStdExtC", "true",
                       "'C' (Compressed Instructions)">;
def HasStdExtC : Predicate<"Subtarget->hasStdExtC()">,
                           AssemblerPredicate<(all_of FeatureStdExtC),
                           "'C' (Compressed Instructions)">;

def FeatureStdExtZba
    : SubtargetFeature<"zba", "HasStdExtZba", "true",
                       "'Zba' (Address Generation Instructions)">;
def HasStdExtZba : Predicate<"Subtarget->hasStdExtZba()">,
                             AssemblerPredicate<(all_of FeatureStdExtZba),
                             "'Zba' (Address Generation Instructions)">;
def NotHasStdExtZba : Predicate<"!Subtarget->hasStdExtZba()">;

def FeatureStdExtZbb
    : SubtargetFeature<"zbb", "HasStdExtZbb", "true",
                       "'Zbb' (Basic Bit-Manipulation)">;
def HasStdExtZbb : Predicate<"Subtarget->hasStdExtZbb()">,
                             AssemblerPredicate<(all_of FeatureStdExtZbb),
                             "'Zbb' (Basic Bit-Manipulation)">;

def FeatureStdExtZbc
    : SubtargetFeature<"zbc", "HasStdExtZbc", "true",
                       "'Zbc' (Carry-Less Multiplication)">;
def HasStdExtZbc : Predicate<"Subtarget->hasStdExtZbc()">,
                             AssemblerPredicate<(all_of FeatureStdExtZbc),
                             "'Zbc' (Carry-Less Multiplication)">;

def FeatureStdExtZbe
    : SubtargetFeature<"experimental-zbe", "HasStdExtZbe", "true",
                       "'Zbe' (Extract-Deposit 'Zb' Instructions)">;
def HasStdExtZbe : Predicate<"Subtarget->hasStdExtZbe()">,
                             AssemblerPredicate<(all_of FeatureStdExtZbe),
                             "'Zbe' (Extract-Deposit 'Zb' Instructions)">;

def FeatureStdExtZbf
    : SubtargetFeature<"experimental-zbf", "HasStdExtZbf", "true",
                       "'Zbf' (Bit-Field 'Zb' Instructions)">;
def HasStdExtZbf : Predicate<"Subtarget->hasStdExtZbf()">,
                             AssemblerPredicate<(all_of FeatureStdExtZbf),
                             "'Zbf' (Bit-Field 'Zb' Instructions)">;

def FeatureStdExtZbm
    : SubtargetFeature<"experimental-zbm", "HasStdExtZbm", "true",
                       "'Zbm' (Matrix 'Zb' Instructions)">;
def HasStdExtZbm : Predicate<"Subtarget->hasStdExtZbm()">,
                             AssemblerPredicate<(all_of FeatureStdExtZbm),
                             "'Zbm' (Matrix 'Zb' Instructions)">;

def FeatureStdExtZbp
    : SubtargetFeature<"experimental-zbp", "HasStdExtZbp", "true",
                       "'Zbp' (Permutation 'Zb' Instructions)">;
def HasStdExtZbp : Predicate<"Subtarget->hasStdExtZbp()">,
                             AssemblerPredicate<(all_of FeatureStdExtZbp),
                             "'Zbp' (Permutation 'Zb' Instructions)">;

def FeatureStdExtZbr
    : SubtargetFeature<"experimental-zbr", "HasStdExtZbr", "true",
                       "'Zbr' (Polynomial Reduction 'Zb' Instructions)">;
def HasStdExtZbr : Predicate<"Subtarget->hasStdExtZbr()">,
                             AssemblerPredicate<(all_of FeatureStdExtZbr),
                             "'Zbr' (Polynomial Reduction 'Zb' Instructions)">;

def FeatureStdExtZbs
    : SubtargetFeature<"zbs", "HasStdExtZbs", "true",
                       "'Zbs' (Single-Bit Instructions)">;
def HasStdExtZbs : Predicate<"Subtarget->hasStdExtZbs()">,
                             AssemblerPredicate<(all_of FeatureStdExtZbs),
                             "'Zbs' (Single-Bit Instructions)">;

def FeatureStdExtZbt
    : SubtargetFeature<"experimental-zbt", "HasStdExtZbt", "true",
                       "'Zbt' (Ternary 'Zb' Instructions)">;
def HasStdExtZbt : Predicate<"Subtarget->hasStdExtZbt()">,
                             AssemblerPredicate<(all_of FeatureStdExtZbt),
                             "'Zbt' (Ternary 'Zb' Instructions)">;

// Some instructions belong to both the basic and the permutation
// subextensions. They should be enabled if either has been specified.
def HasStdExtZbbOrZbp
    : Predicate<"Subtarget->hasStdExtZbb() || Subtarget->hasStdExtZbp()">,
                AssemblerPredicate<(any_of FeatureStdExtZbb, FeatureStdExtZbp),
                                   "'Zbb' (Basic Bit-Manipulation) or "
                                   "'Zbp' (Permutation 'Zb' Instructions)">;

def FeatureStdExtZbkb
    : SubtargetFeature<"zbkb", "HasStdExtZbkb", "true",
                       "'Zbkb' (Bitmanip instructions for Cryptography)">;
def HasStdExtZbkb : Predicate<"Subtarget->hasStdExtZbkb()">,
                             AssemblerPredicate<(all_of FeatureStdExtZbkb),
                             "'Zbkb' (Bitmanip instructions for Cryptography)">;

def FeatureStdExtZbkx
    : SubtargetFeature<"zbkx", "HasStdExtZbkx", "true",
                       "'Zbkx' (Crossbar permutation instructions)">;
def HasStdExtZbkx : Predicate<"Subtarget->hasStdExtZbkx()">,
                             AssemblerPredicate<(all_of FeatureStdExtZbkx),
                             "'Zbkx' (Crossbar permutation instructions)">;

def HasStdExtZbpOrZbkx
    : Predicate<"Subtarget->hasStdExtZbp() || Subtarget->hasStdExtZbkx()">,
                AssemblerPredicate<(any_of FeatureStdExtZbp, FeatureStdExtZbkx),
                                   "'Zbp' (Permutation 'Zb' Instructions) or "
                                   "'Zbkx' (Crossbar permutation instructions)">;

def HasStdExtZbpOrZbkb
    : Predicate<"Subtarget->hasStdExtZbp() || Subtarget->hasStdExtZbkb()">,
                AssemblerPredicate<(any_of FeatureStdExtZbp, FeatureStdExtZbkb),
                                   "'Zbp' (Permutation 'Zb' Instructions) or "
                                   "'Zbkb' (Bitmanip instructions for Cryptography)">;

def HasStdExtZbbOrZbkb
    : Predicate<"Subtarget->hasStdExtZbb() || Subtarget->hasStdExtZbkb()">,
                AssemblerPredicate<(any_of FeatureStdExtZbb, FeatureStdExtZbkb),
                                   "'Zbb' (Basic Bit-Manipulation) or "
                                   "'Zbkb' (Bitmanip instructions for Cryptography)">;

def HasStdExtZbbOrZbpOrZbkb
    : Predicate<"Subtarget->hasStdExtZbb() || Subtarget->hasStdExtZbp() || Subtarget->hasStdExtZbkb()">,
                AssemblerPredicate<(any_of FeatureStdExtZbb, FeatureStdExtZbp, FeatureStdExtZbkb),
                                   "'Zbb' (Basic Bit-Manipulation) or "
                                   "'Zbp' (Permutation 'Zb' Instructions) or "
                                   "'Zbkb' (Bitmanip instructions for Cryptography)">;

// The Carry-less multiply subextension for cryptography is a subset of basic carry-less multiply subextension. The former should be enabled if the latter is enabled.
def FeatureStdExtZbkc
    : SubtargetFeature<"zbkc", "HasStdExtZbkc", "true",
                       "'Zbkc' (Carry-less multiply instructions for Cryptography)">;
def HasStdExtZbkc
    : Predicate<"Subtarget->hasStdExtZbkc()">,
                             AssemblerPredicate<(all_of FeatureStdExtZbkc),
                             "'Zbkc' (Carry-less multiply instructions for Cryptography)">;

def HasStdExtZbcOrZbkc
    : Predicate<"Subtarget->hasStdExtZbc() || Subtarget->hasStdExtZbkc()">,
                AssemblerPredicate<(any_of FeatureStdExtZbc, FeatureStdExtZbkc),
                                   "'Zbc' (Carry-Less Multiplication) or "
                                   "'Zbkc' (Carry-less multiply instructions for Cryptography)">;

def FeatureStdExtZknd
    : SubtargetFeature<"zknd", "HasStdExtZknd", "true",
                       "'Zknd' (NIST Suite: AES Decryption)">;
def HasStdExtZknd : Predicate<"Subtarget->hasStdExtZknd()">,
                             AssemblerPredicate<(all_of FeatureStdExtZknd),
                             "'Zknd' (NIST Suite: AES Decryption)">;

def FeatureStdExtZkne
    : SubtargetFeature<"zkne", "HasStdExtZkne", "true",
                       "'Zkne' (NIST Suite: AES Encryption)">;
def HasStdExtZkne : Predicate<"Subtarget->hasStdExtZkne()">,
                             AssemblerPredicate<(all_of FeatureStdExtZkne),
                             "'Zkne' (NIST Suite: AES Encryption)">;

// Some instructions belong to both Zknd and Zkne subextensions.
// They should be enabled if either has been specified.
def HasStdExtZkndOrZkne
    : Predicate<"Subtarget->hasStdExtZknd() || Subtarget->hasStdExtZkne()">,
                AssemblerPredicate<(any_of FeatureStdExtZknd, FeatureStdExtZkne),
                                   "'Zknd' (NIST Suite: AES Decryption) or "
                                   "'Zkne' (NIST Suite: AES Encryption)">;

def FeatureStdExtZknh
    : SubtargetFeature<"zknh", "HasStdExtZknh", "true",
                       "'Zknh' (NIST Suite: Hash Function Instructions)">;
def HasStdExtZknh : Predicate<"Subtarget->hasStdExtZknh()">,
                             AssemblerPredicate<(all_of FeatureStdExtZknh),
                             "'Zknh' (NIST Suite: Hash Function Instructions)">;

def FeatureStdExtZksed
    : SubtargetFeature<"zksed", "HasStdExtZksed", "true",
                       "'Zksed' (ShangMi Suite: SM4 Block Cipher Instructions)">;
def HasStdExtZksed : Predicate<"Subtarget->hasStdExtZksed()">,
                             AssemblerPredicate<(all_of FeatureStdExtZksed),
                             "'Zksed' (ShangMi Suite: SM4 Block Cipher Instructions)">;

def FeatureStdExtZksh
    : SubtargetFeature<"zksh", "HasStdExtZksh", "true",
                       "'Zksh' (ShangMi Suite: SM3 Hash Function Instructions)">;
def HasStdExtZksh : Predicate<"Subtarget->hasStdExtZksh()">,
                             AssemblerPredicate<(all_of FeatureStdExtZksh),
                             "'Zksh' (ShangMi Suite: SM3 Hash Function Instructions)">;

def FeatureStdExtZkr
    : SubtargetFeature<"zkr", "HasStdExtZkr", "true",
                       "'Zkr' (Entropy Source Extension)">;
def HasStdExtZkr : Predicate<"Subtarget->hasStdExtZkr()">,
                             AssemblerPredicate<(all_of FeatureStdExtZkr),
                             "'Zkr' (Entropy Source Extension)">;

def FeatureStdExtZkn
    : SubtargetFeature<"zkn", "HasStdExtZkn", "true",
                       "'Zkn' (NIST Algorithm Suite)",
                       [FeatureStdExtZbkb,
                        FeatureStdExtZbkc,
                        FeatureStdExtZbkx,
                        FeatureStdExtZkne,
                        FeatureStdExtZknd,
                        FeatureStdExtZknh]>;

def FeatureStdExtZks
    : SubtargetFeature<"zks", "HasStdExtZks", "true",
                       "'Zks' (ShangMi Algorithm Suite)",
                       [FeatureStdExtZbkb,
                        FeatureStdExtZbkc,
                        FeatureStdExtZbkx,
                        FeatureStdExtZksed,
                        FeatureStdExtZksh]>;

def FeatureStdExtZkt
    : SubtargetFeature<"zkt", "HasStdExtZkt", "true",
                       "'Zkt' (Data Independent Execution Latency)">;

def FeatureStdExtZk
    : SubtargetFeature<"zk", "HasStdExtZk", "true",
                       "'Zk' (Standard scalar cryptography extension)",
                       [FeatureStdExtZkn,
                        FeatureStdExtZkr,
                        FeatureStdExtZkt]>;

def FeatureNoRVCHints
    : SubtargetFeature<"no-rvc-hints", "EnableRVCHintInstrs", "false",
                       "Disable RVC Hint Instructions.">;
def HasRVCHints : Predicate<"Subtarget->enableRVCHintInstrs()">,
                  AssemblerPredicate<(all_of(not FeatureNoRVCHints)),
                                      "RVC Hint Instructions">;

def FeatureStdExtZvl32b : SubtargetFeature<"zvl32b", "ZvlLen", "32",
                       "'Zvl' (Minimum Vector Length) 32">;

foreach i = { 6-15 } in {
    defvar I = !shl(1, i);
    def FeatureStdExtZvl#I#b :
        SubtargetFeature<"zvl"#I#"b", "ZvlLen", !cast<string>(I),
                        "'Zvl' (Minimum Vector Length) "#I,
                        [!cast<SubtargetFeature>("FeatureStdExtZvl"#!srl(I, 1)#"b")]>;
}

def FeatureStdExtZve32x
    : SubtargetFeature<"zve32x", "HasStdExtZve32x", "true",
                       "'Zve32x' (Vector Extensions for Embedded Processors "
                       "with maximal 32 EEW)",
                       [FeatureStdExtZvl32b]>;

def FeatureStdExtZve32f
    : SubtargetFeature<"zve32f", "HasStdExtZve32f", "true",
                       "'Zve32f' (Vector Extensions for Embedded Processors "
                       "with maximal 32 EEW and F extension)",
                       [FeatureStdExtZve32x]>;

def FeatureStdExtZve64x
    : SubtargetFeature<"zve64x", "HasStdExtZve64x", "true",
                       "'Zve64x' (Vector Extensions for Embedded Processors "
                       "with maximal 64 EEW)", [FeatureStdExtZve32x, FeatureStdExtZvl64b]>;

def FeatureStdExtZve64f
    : SubtargetFeature<"zve64f", "HasStdExtZve64f", "true",
                       "'Zve64f' (Vector Extensions for Embedded Processors "
                       "with maximal 64 EEW and F extension)",
                       [FeatureStdExtZve32f, FeatureStdExtZve64x]>;

def FeatureStdExtZve64d
    : SubtargetFeature<"zve64d", "HasStdExtZve64d", "true",
                       "'Zve64d' (Vector Extensions for Embedded Processors "
                       "with maximal 64 EEW, F and D extension)",
                       [FeatureStdExtZve64f]>;

def FeatureStdExtV
    : SubtargetFeature<"v", "HasStdExtV", "true",
                       "'V' (Vector Extension for Application Processors)",
                       [FeatureStdExtZvl128b, FeatureStdExtZve64d, FeatureStdExtF, FeatureStdExtD]>;

def HasVInstructions    : Predicate<"Subtarget->hasVInstructions()">,
      AssemblerPredicate<
          (any_of FeatureStdExtZve32x),
          "'V' (Vector Extension for Application Processors), 'Zve32x' or "
          "'Zve64x' (Vector Extensions for Embedded Processors)">;
def HasVInstructionsI64 : Predicate<"Subtarget->hasVInstructionsI64()">,
      AssemblerPredicate<
          (any_of FeatureStdExtZve64x),
          "'V' (Vector Extension for Application Processors) or 'Zve64x' "
          "(Vector Extensions for Embedded Processors)">;
def HasVInstructionsAnyF : Predicate<"Subtarget->hasVInstructionsAnyF()">,
      AssemblerPredicate<
          (any_of FeatureStdExtZve32f),
          "'V' (Vector Extension for Application Processors), 'Zve32f', "
          "'Zve64f' or 'Zve64d' (Vector Extensions for Embedded Processors)">;

def FeatureStdExtZvfh
    : SubtargetFeature<"experimental-zvfh", "HasStdExtZvfh", "true",
                       "'Zvfh' (Vector Half-Precision Floating-Point)",
                       [FeatureStdExtZve32f]>;

def FeatureStdExtZicbom
    : SubtargetFeature<"zicbom", "HasStdExtZicbom", "true",
                       "'Zicbom' (Cache-Block Management Instructions)">;
def HasStdExtZicbom : Predicate<"Subtarget->hasStdExtZicbom()">,
                                AssemblerPredicate<(all_of FeatureStdExtZicbom),
                                "'Zicbom' (Cache-Block Management Instructions)">;

def FeatureStdExtZicboz
    : SubtargetFeature<"zicboz", "HasStdExtZicboz", "true",
                       "'Zicboz' (Cache-Block Zero Instructions)">;
def HasStdExtZicboz : Predicate<"Subtarget->hasStdExtZicboz()">,
                                AssemblerPredicate<(all_of FeatureStdExtZicboz),
                                "'Zicboz' (Cache-Block Zero Instructions)">;

def FeatureStdExtZicbop
    : SubtargetFeature<"zicbop", "HasStdExtZicbop", "true",
                       "'Zicbop' (Cache-Block Prefetch Instructions)">;
def HasStdExtZicbop : Predicate<"Subtarget->hasStdExtZicbop()">,
                                AssemblerPredicate<(all_of FeatureStdExtZicbop),
                                "'Zicbop' (Cache-Block Prefetch Instructions)">;

def Feature64Bit
    : SubtargetFeature<"64bit", "HasRV64", "true", "Implements RV64">;
def IsRV64 : Predicate<"Subtarget->is64Bit()">,
                       AssemblerPredicate<(all_of Feature64Bit),
                       "RV64I Base Instruction Set">;
def IsRV32 : Predicate<"!Subtarget->is64Bit()">,
                       AssemblerPredicate<(all_of (not Feature64Bit)),
                       "RV32I Base Instruction Set">;

defvar RV32 = DefaultMode;
def RV64           : HwMode<"+64bit">;

def FeatureRV32E
    : SubtargetFeature<"e", "IsRV32E", "true",
                       "Implements RV32E (provides 16 rather than 32 GPRs)">;
def IsRV32E : Predicate<"Subtarget->isRV32E()">,
                        AssemblerPredicate<(all_of FeatureRV32E)>;

def FeatureRelax
    : SubtargetFeature<"relax", "EnableLinkerRelax", "true",
                       "Enable Linker relaxation.">;

foreach i = {1-31} in
    def FeatureReserveX#i :
        SubtargetFeature<"reserve-x"#i, "UserReservedRegister[RISCV::X"#i#"]",
                         "true", "Reserve X"#i>;

def FeatureSaveRestore : SubtargetFeature<"save-restore", "EnableSaveRestore",
                                          "true", "Enable save/restore.">;

def FeatureUnalignedScalarMem
   : SubtargetFeature<"unaligned-scalar-mem", "EnableUnalignedScalarMem",
                      "true", "Has reasonably performant unaligned scalar "
                      "loads and stores">;

def TuneLUIADDIFusion
    : SubtargetFeature<"lui-addi-fusion", "HasLUIADDIFusion",
                       "true", "Enable LUI+ADDI macrofusion">;

def TuneNoDefaultUnroll
    : SubtargetFeature<"no-default-unroll", "EnableDefaultUnroll", "false",
                       "Disable default unroll preference.">;

def TuneSiFive7 : SubtargetFeature<"sifive7", "RISCVProcFamily", "SiFive7",
                                   "SiFive 7-Series processors",
                                   [TuneNoDefaultUnroll]>;

def FeatureExtVenus : SubtargetFeature<"zvenus", "HasExtVenus", "true", "Venus WiPU Extension">;
def HasExtVenus : Predicate<"Subtarget->hasExtVenus()">,
                             AssemblerPredicate<(all_of FeatureExtVenus),
                             "'Zvenus' (WiPU Extension)">;

//===----------------------------------------------------------------------===//
// Named operands for CSR instructions.
//===----------------------------------------------------------------------===//

include "RISCVSystemOperands.td"

//===----------------------------------------------------------------------===//
// Registers, calling conventions, instruction descriptions.
//===----------------------------------------------------------------------===//

include "RISCVSchedule.td"
include "RISCVRegisterInfo.td"
include "RISCVCallingConv.td"
include "RISCVInstrInfo.td"
include "RISCVRegisterBanks.td"
include "RISCVSchedRocket.td"
include "RISCVSchedSiFive7.td"

//===----------------------------------------------------------------------===//
// RISC-V processors supported.
//===----------------------------------------------------------------------===//

def : ProcessorModel<"generic-rv32", NoSchedModel, []>;
def : ProcessorModel<"generic-rv64", NoSchedModel, [Feature64Bit]>;
// Support generic for compatibility with other targets. The triple will be used
// to change to the appropriate rv32/rv64 version.
def : ProcessorModel<"generic", NoSchedModel, []>;

def : ProcessorModel<"rocket-rv32", RocketModel, []>;
def : ProcessorModel<"rocket-rv64", RocketModel, [Feature64Bit]>;

def : ProcessorModel<"sifive-7-rv32", SiFive7Model, [],
                     [TuneSiFive7]>;
def : ProcessorModel<"sifive-7-rv64", SiFive7Model, [Feature64Bit],
                     [TuneSiFive7]>;

def : ProcessorModel<"sifive-e20", RocketModel, [FeatureStdExtM,
                                                 FeatureStdExtC]>;

def : ProcessorModel<"sifive-e21", RocketModel, [FeatureStdExtM,
                                                 FeatureStdExtA,
                                                 FeatureStdExtC]>;

def : ProcessorModel<"sifive-e24", RocketModel, [FeatureStdExtM,
                                                 FeatureStdExtA,
                                                 FeatureStdExtF,
                                                 FeatureStdExtC]>;

def : ProcessorModel<"sifive-e31", RocketModel, [FeatureStdExtM,
                                                 FeatureStdExtA,
                                                 FeatureStdExtC]>;

def : ProcessorModel<"sifive-e34", RocketModel, [FeatureStdExtM,
                                                 FeatureStdExtA,
                                                 FeatureStdExtF,
                                                 FeatureStdExtC]>;

def : ProcessorModel<"sifive-e76", SiFive7Model, [FeatureStdExtM,
                                                  FeatureStdExtA,
                                                  FeatureStdExtF,
                                                  FeatureStdExtC],
                     [TuneSiFive7]>;

def : ProcessorModel<"sifive-s21", RocketModel, [Feature64Bit,
                                                 FeatureStdExtM,
                                                 FeatureStdExtA,
                                                 FeatureStdExtC]>;

def : ProcessorModel<"sifive-s51", RocketModel, [Feature64Bit,
                                                 FeatureStdExtM,
                                                 FeatureStdExtA,
                                                 FeatureStdExtC]>;

def : ProcessorModel<"sifive-s54", RocketModel, [Feature64Bit,
                                                 FeatureStdExtM,
                                                 FeatureStdExtA,
                                                 FeatureStdExtF,
                                                 FeatureStdExtD,
                                                 FeatureStdExtC]>;

def : ProcessorModel<"sifive-s76", SiFive7Model, [Feature64Bit,
                                                  FeatureStdExtM,
                                                  FeatureStdExtA,
                                                  FeatureStdExtF,
                                                  FeatureStdExtD,
                                                  FeatureStdExtC],
                     [TuneSiFive7]>;

def : ProcessorModel<"sifive-u54", RocketModel, [Feature64Bit,
                                                 FeatureStdExtM,
                                                 FeatureStdExtA,
                                                 FeatureStdExtF,
                                                 FeatureStdExtD,
                                                 FeatureStdExtC]>;

def : ProcessorModel<"sifive-u74", SiFive7Model, [Feature64Bit,
                                                  FeatureStdExtM,
                                                  FeatureStdExtA,
                                                  FeatureStdExtF,
                                                  FeatureStdExtD,
                                                  FeatureStdExtC],
                     [TuneSiFive7]>;

//===----------------------------------------------------------------------===//
// Define the RISC-V target.
//===----------------------------------------------------------------------===//

def RISCVInstrInfo : InstrInfo {
  let guessInstructionProperties = 0;
}

def RISCVAsmParser : AsmParser {
  let ShouldEmitMatchRegisterAltName = 1;
  let AllowDuplicateRegisterNames = 1;
}

def RISCVAsmWriter : AsmWriter {
  int PassSubtarget = 1;
}

def RISCV : Target {
  let InstructionSet = RISCVInstrInfo;
  let AssemblyParsers = [RISCVAsmParser];
  let AssemblyWriters = [RISCVAsmWriter];
  let AllowRegisterRenaming = 1;
}

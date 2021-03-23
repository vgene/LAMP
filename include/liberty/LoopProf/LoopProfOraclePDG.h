#pragma once

#include "llvm/Pass.h"
#include "llvm/IR/Instruction.h"
#include "llvm/IR/Function.h"

#include "PDG.hpp"

namespace liberty
{
  using namespace llvm;
  class LoopProfOraclePDG : public ModulePass
  {
    public:
      LoopProfOraclePDG() : ModulePass(ID), valid(false) {}
      void getAnalysisUsage(AnalysisUsage &au) const;
      bool runOnModule(Module &mod);
      StringRef getPassName() const {
        return "loop-prof-oracle-pdg";
      }

      static char ID;
    private:
      bool valid;
  };
}

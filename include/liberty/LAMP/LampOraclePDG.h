#pragma once

#include "llvm/Pass.h"
#include "llvm/IR/Module.h"

#include "scaf/MemoryAnalysisModules/LoopAA.h"
#include "LAMPLoadProfile.h"

#include "PDG.hpp"

#include <set>

namespace liberty
{
  using namespace llvm;
  using namespace llvm::noelle;

  class LampOraclePDG : public ModulePass
  {
    private:
      LAMPLoadProfile *lamp;
      uint8_t disproveMemoryDep(Instruction *src, Instruction *dst,
          LoopAA::TemporalRelation FW, LoopAA::TemporalRelation RV,
          uint8_t depTypes, Loop *loop);

      uint8_t disproveIntraIterationMemoryDep(Instruction *src,
          Instruction *dst, uint8_t depTypes, Loop *loop);

      uint8_t disproveLoopCarriedMemoryDep(Instruction *src,
          Instruction *dst,
          uint8_t depTypes, Loop *loop);

    public:
      static char ID;
      LampOraclePDG() : ModulePass(ID), lamp(NULL) {}
      void getAnalysisUsage(AnalysisUsage &au) const;
      bool runOnModule(Module &mod);
      StringRef getPassName() const {
        return "lamp-oracle-pdg";
      }

      bool dumpAllOracleMemLoopPDG(Module &mod);

      std::unique_ptr<llvm::noelle::PDG> getLoopOracleMemPDG(Loop *loop);
      bool dumpOracleMemLoopPDG(Loop *loop);

      // std::unique_ptr<std::unordered_set<Edge>> verifyLoopPDG(Loop *loop, llvm::noelle::PDG *pdg);

      LoopAA::ModRefResult modref(
          const Instruction *A,
          LoopAA::TemporalRelation rel,
          const Instruction *B,
          const Loop *L,
          Remedies &R);
  };

}


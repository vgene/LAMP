#pragma once

#include "llvm/Pass.h"
#include "llvm/IR/Module.h"
#include "llvm/ADT/SmallBitVector.h"

#include "scaf/MemoryAnalysisModules/LoopAA.h"
#include "scaf/Utilities/ModuleLoops.h"

#include "PDG.hpp"

#include <map>
#include <set>

namespace liberty
{
  /*
   * Use profiling information from LLVM generated using both:
   *   1. `opt -pgo-instr-gen -instrprof`
   *   2. `clang -fprofile-generate`
   */
  class EdgeProfOraclePDG : public ModulePass
  {
    public:
      static char ID;
      EdgeProfOraclePDG() : ModulePass(ID) {}

      void getAnalysisUsage(AnalysisUsage &au) const;
      bool runOnModule(Module &mod);
      StringRef getPassName() const {
        return "edgeprof-oracle-pdg";
      }

      /*
       * Dump the PDG of all loops, constructed only from the edge profile
       */
      bool dumpAllOracleEdgeProfLoopPDG(Module &mod);

      /*
       * Dump the PDG of a specific loop, constructed only from the edge profile
       */
      bool dumpOracleEdgeProfLoopPDG(Loop *loop);

      /*
       *
       */
      void updateEdgeProfOraclePDG(Loop *loop, PDG *pdg);

      /*
       * TODO: Figure this out
       */
      std::unique_ptr<llvm::noelle::PDG> getEdgeProfOraclePDG(Loop *loop);

      /*
       * XXX: not sure why this is needed? Shouldn't this be in SCAF?
       */
      LoopAA::ModRefResult modref(
          const Instruction *A,
          LoopAA::TemporalRelation rel,
          const Instruction *B,
          const Loop *L,
          Remedies &R);

    private:
      /* -------- High-level functions -------- */

      /*
       * Determine if the provided control flow edge is speculated
       * not to run
       */
      bool isSpeculativelyDead(const Instruction *term, unsigned succNo);

      /*
       * Determine if the given basic block is speculatively dead
       */
      bool isSpeculativelyDead(const BasicBlock *bb);

      /*
       * Does the speculatively dead control flow edge sourced from this term
       * manifest during profiling
       */
      bool misspecInProfLoopExit(const Instruction *term);

      /*
       * Get and set loop header
       */
      void setLoopOfInterest(const BasicBlock *bb);
      const BasicBlock *getLoopHeaderOfInterest() const;

      /* -------- Lower-level functions --------*/

      /*
       * Does this basic bock dominate current loop header of interest
       */
      bool dominatesTargetHeader(const BasicBlock *bb);

      /*
       * Visit a function and populate internal data structures
       * TODO: Better comment
       */
      void visit(const Function *fcn);

      /* -------- Private types/variables -------- */
      ModuleLoops *mloops;

      struct LoopSpeculation
      {
        std::map<const Instruction *, SmallBitVector> deadEdges;
        std::set<const BasicBlock *> deadBlocks;
        std::set<const Function *> visited;
        std::set<const Instruction *> manifestedDeadExits;
      };

      std::map<const BasicBlock *, LoopSpeculation> loops; // XXX: What is this?

      const BasicBlock *loop_header;
  };
}

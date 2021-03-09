#define DEBUG_TYPE "lamp-oracle-aa"

#define LAMP_COLLECTS_OUTPUT_DEPENDENCES  (0)

#include "scaf/MemoryAnalysisModules/Introspection.h"
#include "liberty/LAMP/LampOraclePDG.h"

#include "llvm/ADT/Statistic.h"
#include "llvm/IR/IntrinsicInst.h"
#include "llvm/Analysis/LoopInfo.h"


#include "PDGPrinter.hpp"

namespace liberty
{
  using namespace llvm;
  using namespace llvm::noelle;

  STATISTIC(numQueries,       "Num queries");
  STATISTIC(numEligible,      "Num eligible queries");
  STATISTIC(numNoForwardFlow, "Num forward no-flow results");
  STATISTIC(numNoReverseFlow, "Num reverse no-flow results");

  static cl::opt<unsigned> Threshhold(
    "lamp-oracle-pdg-threshhold", cl::init(0),
    cl::NotHidden,
    cl::desc("Maximum number of observed flows to report NoModRef"));

  // create an empty NOELLE pdg
  // query each pair of nodes to LAMP
  //
  // Dump the PDG

  uint8_t LampOraclePDG::disproveMemoryDep(Instruction *src, Instruction *dst,
      LoopAA::TemporalRelation FW,
      LoopAA::TemporalRelation RV,
      uint8_t depTypes, Loop *loop) {

    uint8_t disprovedDeps = depTypes;
    if (!depTypes)
      // no dependences to disprove
      return disprovedDeps;

    if (!src->mayReadOrWriteMemory())
      return disprovedDeps;
    if (!dst->mayReadOrWriteMemory())
      return disprovedDeps;
    if (!src->mayWriteToMemory() && !dst->mayWriteToMemory())
      return disprovedDeps;

    bool loopCarried = FW != RV;

    // The remedies set will remain empty when no speculation is used
    Remedies R;

    // forward dep test
    LoopAA::ModRefResult forward = this->modref(src, FW, dst, loop, R);

    if (!src->mayWriteToMemory())
      forward = LoopAA::ModRefResult(forward & (~LoopAA::Mod));
    if (!src->mayReadFromMemory())
      forward = LoopAA::ModRefResult(forward & (~LoopAA::Ref));

    if (LoopAA::NoModRef == forward)
      return disprovedDeps;

    // reverse dep test
    LoopAA::ModRefResult reverse = forward;

    if (loopCarried || src != dst)
      reverse = this->modref(dst, RV, src, loop, R);

    if (!dst->mayWriteToMemory())
      reverse = LoopAA::ModRefResult(reverse & (~LoopAA::Mod));
    if (!dst->mayReadFromMemory())
      reverse = LoopAA::ModRefResult(reverse & (~LoopAA::Ref));

    if (LoopAA::NoModRef == reverse)
      return disprovedDeps;

    if (LoopAA::Ref == forward && LoopAA::Ref == reverse)
      return disprovedDeps; // RaR dep; who cares.

    // At this point, we know there is one or more of
    // a flow-, anti-, or output-dependence.

    bool RAW = (forward == LoopAA::Mod || forward == LoopAA::ModRef) &&
      (reverse == LoopAA::Ref || reverse == LoopAA::ModRef);
    bool WAR = (forward == LoopAA::Ref || forward == LoopAA::ModRef) &&
      (reverse == LoopAA::Mod || reverse == LoopAA::ModRef);
    bool WAW = (forward == LoopAA::Mod || forward == LoopAA::ModRef) &&
      (reverse == LoopAA::Mod || reverse == LoopAA::ModRef);

    // set to zero the bits for the reported deps by SCAF
    if (RAW) {
      disprovedDeps &= ~1;
    }
    if (WAW) {
      disprovedDeps &= ~(1 << 1);
    }
    if (WAR) {
      disprovedDeps &= ~(1 << 2);
    }

    // return the disproved dependences
    return disprovedDeps;
  }

  uint8_t LampOraclePDG::disproveIntraIterationMemoryDep(Instruction *src,
      Instruction *dst, uint8_t depTypes, Loop *loop) {
    return disproveMemoryDep(src, dst, LoopAA::Same, LoopAA::Same, depTypes,
        loop);
  }

  uint8_t LampOraclePDG::disproveLoopCarriedMemoryDep(Instruction *src,
      Instruction *dst, uint8_t depTypes, Loop *loop) {
    // there is always a feasible path for inter-iteration deps
    // (there is a path from any node in the loop to the header
    //  and the header dominates all the nodes of the loops)

    // only need to check for aliasing and kill-flow

    return disproveMemoryDep(src, dst, LoopAA::Before, LoopAA::After, depTypes,
        loop);
  }

  static bool isMemIntrinsic(const Instruction *inst)
  {
    return isa< MemIntrinsic >(inst);
  }

  static bool intrinsicMayRead(const Instruction *inst)
  {
    ImmutableCallSite cs(inst);
    StringRef  name = cs.getCalledFunction()->getName();
    if( name == "llvm.memset.p0i8.i32"
    ||  name == "llvm.memset.p0i8.i64" )
      return false;

    return true;
  }

  LoopAA::ModRefResult LampOraclePDG::modref(
    const Instruction *A,
    LoopAA::TemporalRelation rel,
    const Instruction *B,
    const Loop *L,
    Remedies &R)
  {
    ++numQueries;

    // Lamp profile data is loop sensitive.
    if( !L )
      // Inapplicable
      return LoopAA::ModRef;

    // INTROSPECT(ENTER(A,rel,B,L));

    LoopAA::ModRefResult result = LoopAA::ModRef;

    // Loop carried forward queries, or
    // LoopAA::Same queries.
    if( rel == LoopAA::Before || rel == LoopAA::Same )
    {
      // Lamp profile data is only collected for
      // loads and stores; not callsites.
      // Lamp collects FLOW and OUTPUT info, but
      // not ANTI or FALSE dependence data.
      // Thus, for LoopAA::Before/Same queries, we are looking
      // for Store -> Load/Store
      if( isa<StoreInst>(A) )
        // Stores don't ref
        result = LoopAA::Mod;

      else if( isMemIntrinsic(A) )
      {
        if( intrinsicMayRead(A) )
          result = LoopAA::ModRef;
        else
          result = LoopAA::Mod;
      }

      else
      {
        // Callsites, etc: inapplicable
        result = LoopAA::ModRef;
        // INTROSPECT(EXIT(A,rel,B,L,result));
        return result;
      }

      // Again, only Store vs (Load/Store)
      if( isa<LoadInst>(B) )
      {
        // okay
      }
      else if( isMemIntrinsic(B) && intrinsicMayRead(B) )
      {
        // okay
      }
      else
      {
        if( ! (LAMP_COLLECTS_OUTPUT_DEPENDENCES && isa<StoreInst>(B)) )
        {
          // inapplicable
          result = LoopAA::ModRefResult(result & LoopAA::ModRef);
          // INTROSPECT(EXIT(A,rel,B,L,result));
          return result;
        }
      }

      if( rel == LoopAA::Before )
      {
        ++numEligible;
        // Query profile data
        if( lamp->numObsInterIterDep(L->getHeader(), B, A ) <= Threshhold )
        {
          // No flow.
          result = LoopAA::ModRefResult(result & ~ LoopAA::Mod);
          ++numNoForwardFlow;
        }
      }

      else if( rel == LoopAA::Same )
      {
        ++numEligible;
        // Query profile data
        if( lamp->numObsIntraIterDep(L->getHeader(), B, A ) <= Threshhold )
        {
          // No flow
          result = LoopAA::ModRefResult(result & ~ LoopAA::Mod);
          ++numNoForwardFlow;
        }
      }
    }

    // Loop carried reverse queries.
    else if( rel == LoopAA::After )
    {
      // Lamp profile data is only collected for
      // loads and stores; not callsites.
      // Lamp collects FLOW and OUTPUT info, but
      // not ANTI or FALSE dependence data.
      // Thus, for LoopAA::After queries, we are looking
      // for Load/Store -> Store
      if( isa<LoadInst>(A) )
        // Anti or False: inapplicable
        result = LoopAA::Ref;

      else if( isMemIntrinsic(A) && intrinsicMayRead(A) )
        result = LoopAA::ModRef;

      else if( LAMP_COLLECTS_OUTPUT_DEPENDENCES && isa<StoreInst>(A) )
        // Stores don't ref
        result = LoopAA::Mod;

      else
      {
        // Callsites, etc: inapplicable
        result = LoopAA::ModRef;
        // INTROSPECT(EXIT(A,rel,B,L,result));
        return result;
      }


      // Again, only (Load/Store) vs Store
      if( isa<StoreInst>(B) )
      {
        // good
      }
      else if( isMemIntrinsic(B) )
      {
        // good
      }
      else
      {
        // inapplicable
        result = LoopAA::ModRefResult(result & LoopAA::ModRef);
        // INTROSPECT(EXIT(A,rel,B,L,result));
        return result;
      }

      ++numEligible;
      // Query profile data.
      if( lamp->numObsInterIterDep(L->getHeader(), A, B ) <= Threshhold )
      {
        // No flow.
        if( isa<LoadInst>(B) )
          result = LoopAA::ModRefResult(result & ~ LoopAA::Ref);

        else if( isa<StoreInst>(B) )
          result = LoopAA::ModRefResult(result & ~ LoopAA::Mod);

        ++numNoReverseFlow;
      }
    }

    if( result !=  LoopAA::NoModRef )
      // Chain.
      result = LoopAA::ModRefResult(result & LoopAA::ModRef );

    // INTROSPECT(EXIT(A,rel,B,L,result));
    return result;
  }

  void LampOraclePDG::getAnalysisUsage(AnalysisUsage &au) const {
    au.addRequired< LAMPLoadProfile >();
    au.addRequired< LoopInfoWrapperPass >();
    au.setPreservesAll();
  }

  bool LampOraclePDG::runOnModule(Module &mod) {
    lamp = &getAnalysis< LAMPLoadProfile >();

    dumpAllOracleMemLoopPDG(mod);
    return false;
  }

  bool LampOraclePDG::dumpAllOracleMemLoopPDG(Module &mod) {
    auto numLoops = 0;
    // iterate all the loops
    for (auto &F : mod) {
      if (F.isDeclaration())
        continue;
      LoopInfo &li= getAnalysis< LoopInfoWrapperPass >(F).getLoopInfo();

      // get all loops
      list<Loop*> loops( li.begin(), li.end() );
      while( !loops.empty() )
      {
        ++numLoops;

        Loop *loop = loops.front();
        loops.pop_front();

        dumpOracleMemLoopPDG(loop);

        loops.insert( loops.end(), loop->getSubLoops().begin(), loop->getSubLoops().end());
      }
    }

    if (numLoops > 0)
      return true;
    else
      return false;
  }

  bool LampOraclePDG::dumpOracleMemLoopPDG(Loop *loop) {
    std::unique_ptr<llvm::noelle::PDG> pdg = getLoopOracleMemPDG(loop);

    // create file name
    auto *header = loop->getHeader();
    auto *fcn = header->getParent();

    std::string pdgDotName = "pdg_" + fcn->getName().str() + "_" + header->getName().str() + ".dot";
    
    // dump pdg
    DGPrinter::writeClusteredGraph<PDG, Value>(pdgDotName, pdg.get());
    
    return true;
  }

  std::unique_ptr<PDG> LampOraclePDG::getLoopOracleMemPDG(Loop *loop) {
    auto pdg = std::make_unique<PDG>(loop);

    for (auto nodeI: make_range(pdg->begin_nodes(), pdg->end_nodes())) {
      // get src I
      Value *pdgValueI = nodeI->getT();
      Instruction *src = dyn_cast<Instruction>(pdgValueI);

      if (!src->mayReadOrWriteMemory())
        continue;

      for (auto nodeJ: make_range(pdg->begin_nodes(), pdg->end_nodes())) {
        // get dst I
        Value *pdgValueJ = nodeJ->getT();
        Instruction *dst = dyn_cast<Instruction>(pdgValueJ);

        uint8_t depTypes = 0b111;
        uint8_t lcDep = disproveLoopCarriedMemoryDep(src, dst, depTypes, loop);

        // RAW
        if (!(lcDep & 0b1)) {
          auto edge = pdg->addEdge((Value *)src, (Value *)dst);
          edge->setMemMustType(true, false, DG_DATA_RAW);
          edge->setLoopCarried(true);
        }

        // WAW
        if (!(lcDep & 0b10)) {
          auto edge = pdg->addEdge((Value *)src, (Value *)dst);
          edge->setMemMustType(true, false, DG_DATA_WAW);
          edge->setLoopCarried(true);
        }

        // WAR
        if (!(lcDep & 0b100)) {
          auto edge = pdg->addEdge((Value *)src, (Value *)dst);
          edge->setMemMustType(true, false, DG_DATA_WAR);
          edge->setLoopCarried(true);
        }

        depTypes = 0b111;
        lcDep = disproveIntraIterationMemoryDep(src, dst, depTypes, loop);

        // RAW
        if (!(lcDep & 0b1)) {
          auto edge = pdg->addEdge((Value *)src, (Value *)dst);
          edge->setMemMustType(true, false, DG_DATA_RAW);
          edge->setLoopCarried(false);
        }

        // WAW
        if (!(lcDep & 0b10)) {
          auto edge = pdg->addEdge((Value *)src, (Value *)dst);
          edge->setMemMustType(true, false, DG_DATA_WAW);
          edge->setLoopCarried(false);
        }

        // WAR
        if (!(lcDep & 0b100)) {
          auto edge = pdg->addEdge((Value *)src, (Value *)dst);
          edge->setMemMustType(true, false, DG_DATA_WAR);
          edge->setLoopCarried(false);
        }
      }
    }

    return pdg;
  }

  char LampOraclePDG::ID = 0;
  static RegisterPass< LampOraclePDG > rp("lamp-oracle-pdg", "LAMP Oracle PDG");
}



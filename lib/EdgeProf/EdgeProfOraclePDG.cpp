#define DEBUG_TYPE "edgeprof-oracle-pdg"

#include "liberty/EdgeProf/EdgeProfOraclePDG.h"
#include "liberty/LoopProf/Targets.h"

#include "scaf/Utilities/ModuleLoops.h"

#include "llvm/Analysis/BlockFrequencyInfo.h"
#include "llvm/Analysis/BranchProbabilityInfo.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/Support/Debug.h"

#include "PDGPrinter.hpp"


namespace liberty
{
  using namespace llvm;
  using namespace llvm::noelle;

  /* static cl::opt<double> MinSamples( */
  /*     "min-samples", */
  /*     cl::init(5.0), */
  /*     cl::Hidden, */
  /*     cl::desc("Minimum number of samples to consider edge")); */
  /* static cl::opt<double> MaxMisspec( */
  /*     "max-misspec", */
  /*     cl::init(0.00001), */
  /*     cl::Hidden, */
  /*     cl::desc("Max misspeculation rate (percentage/100) for non-exit edges")); */
  /* static cl::opt<double> MaxMisspecLoopExit( */
  /*     "max-misspec-loop-exit", */
  /*     cl::init(0.0), */
  /*     cl::Hidden, */
  /*     cl::desc("Max misspeculation rate (percentage/100) for loop exit edges")); */
  /* static cl::opt<double> MaxMisspecTargetLoopExit( */
  /*     "max-misspec-target-loop-exit", */
  /*     cl::init(0.1), */
  /*     cl::Hidden, */
  /*     cl::desc("Speculate loop exit not taken if exit is taken less than this ratio")); */

  static const double MinSamples = 5.0;
  static const double MaxMisspec = 0.00001;
  static const double MaxMisspecLoopExit = 0.0;
  static const double MaxMisspecTargetLoopExit = 0.1;

  STATISTIC(numSpecEdges,   "Speculatively dead edges");
  STATISTIC(numTotalBlocks, "Total basic blocks visited");
  STATISTIC(numSpecBlocks,  "Speculatively dead blocks");

  void EdgeProfOraclePDG::getAnalysisUsage(AnalysisUsage &au) const
  {
    au.addRequired<ModuleLoops>();
    au.addRequired<BlockFrequencyInfoWrapperPass>();
    au.addRequired<BranchProbabilityInfoWrapperPass>();
    /* au.addRequired<Targets>(); */
    au.setPreservesAll();
  }

  bool EdgeProfOraclePDG::runOnModule(Module &mod)
  {
    mloops = &getAnalysis<ModuleLoops>();
    dumpAllOracleEdgeProfLoopPDG(mod);
    return false;
  }

  bool EdgeProfOraclePDG::dumpAllOracleEdgeProfLoopPDG(Module &mod)
  {
    auto numLoops = 0;
    for ( auto &F : mod )
    {
      if ( F.isDeclaration() )
        continue;

      LoopInfo &li = getAnalysis<LoopInfoWrapperPass>(F).getLoopInfo();
      list<Loop *> loops(li.begin(), li.end());
      while ( !loops.empty() )
      {
        numLoops++;

        Loop *loop = loops.front();
        loops.pop_front();

        dumpOracleEdgeProfLoopPDG(loop);

        loops.insert(loops.end(), loop->getSubLoops().begin(), loop->getSubLoops().end());
      }
    }

    if ( numLoops > 0 )
      return true;
    else
      return false;
  }

  bool EdgeProfOraclePDG::dumpOracleEdgeProfLoopPDG(Loop *loop)
  {
    std::unique_ptr<PDG> pdg = getEdgeProfOraclePDG(loop);

    auto *header = loop->getHeader();
    auto *fcn = header->getParent();

    std::string pdgDotName = "pdg_" + fcn->getName().str() + "_" + header->getName().str() + ".dot";

    DGPrinter::writeClusteredGraph<PDG, Value>(pdgDotName, pdg.get());

    return true;
  }

  void EdgeProfOraclePDG::updateEdgeProfOraclePDG(Loop *loop, PDG *pdg)
  {
    setLoopOfInterest(loop->getHeader());

    for ( auto nodeI : make_range(pdg->begin_nodes(), pdg->end_nodes()) )
    {
      // get instruction
      Value *pdgValueI = nodeI->getT();
      Instruction *inst = dyn_cast<Instruction>(pdgValueI);

      if ( isSpeculativelyDead(inst->getParent()) )
      {
        nodeI->setSpeculativelyDead(true);
      }
    }
  }

  std::unique_ptr<PDG> EdgeProfOraclePDG::getEdgeProfOraclePDG(Loop *loop) {

    LLVM_DEBUG(
        errs() << "GENERATING PDG FOR " << loop->getHeader()->getParent()->getName()
               << " :: " << loop->getHeader()->getName() << "\n");

    auto pdg = std::make_unique<PDG>(loop);

    updateEdgeProfOraclePDG(loop, pdg.get());

    /* for ( auto nodeI : make_range(pdg->begin_nodes(), pdg->end_nodes()) ) */
    /* { */
    /*   // get src */
    /*   Value *pdgValueI = nodeI->getT(); */
    /*   Instruction *src = dyn_cast<Instruction>(pdgValueI); */

    /*   for ( auto nodeJ : make_range(pdg->begin_nodes(), pdg->end_nodes()) ) */
    /*   { */
    /*     // get dst */
    /*     Value *pdgValueJ = nodeJ->getT(); */
    /*     Instruction *dst = dyn_cast<Instruction>(pdgValueJ); */
    /*   } */
    /* } */

    return pdg;
  }

  void EdgeProfOraclePDG::setLoopOfInterest(const BasicBlock *bb)
  {
    // reachableCache.clear();
    loop_header = bb;
  }

  const BasicBlock *EdgeProfOraclePDG::getLoopHeaderOfInterest() const
  {
    assert(loop_header && "Did not set loop of interest!");
    return loop_header;
  }

  bool EdgeProfOraclePDG::dominatesTargetHeader(const BasicBlock *bb)
  {
    const Function *fcn = bb->getParent();
    auto targetHeader = getLoopHeaderOfInterest();
    if ( fcn != targetHeader->getParent() )
      return false;

    DominatorTree &dt = this->mloops->getAnalysis_DominatorTree(fcn);
    if ( dt.dominates(bb, targetHeader) )
    {
      /* LLVM_DEBUG( */
      /*     errs() << "BasicBlock " << bb->getName() << " dominates target header " */
      /*            << targetHeader->getName() << "\n"); */
      return true;
    }

    return false;
  }

  void EdgeProfOraclePDG::visit(const Function *fcn)
  {
    // Analyze every function AT MOST once
    if ( loops[getLoopHeaderOfInterest()].visited.count(fcn) )
      return;
    loops[getLoopHeaderOfInterest()].visited.insert(fcn);

    LLVM_DEBUG(errs() << "EdgeProfOraclePDG: visit( " << fcn->getName() << " )\n");

    // XXX Evil but should be ok since we aren't modifying IR
    Function *non_const_fcn = const_cast<Function *>(fcn);
    BlockFrequencyInfo &bfi = getAnalysis<BlockFrequencyInfoWrapperPass>(*non_const_fcn).getBFI();
    BranchProbabilityInfo &bpi = getAnalysis<BranchProbabilityInfoWrapperPass>(*non_const_fcn).getBPI();

    // For functions that are never invoked, speculate that the callsite is never
    // run but don't speculate about anything inside the function
    // XXX: We do speculate inside the function right now
    if ( !fcn->getEntryCount().hasValue() )
    {
      // CASE 0
      LLVM_DEBUG(errs() << "EdgeProfOraclePDG: function does not have profile data available\n");

      // XXX In LLVM 5.0 (and possible later)  getEntryCount() will return none
      // for zero counts. Thus no profile data vs never invoked functions are indistinguisable.
      // Re-read metadata to check
      MDNode *MD = fcn->getMetadata(LLVMContext::MD_prof);
      if ( MD && MD->getOperand(0) )
        if ( MDString *MDS = dyn_cast<MDString>(MD->getOperand(0)) )
          if ( MDS->getString().equals("function_entry_count") )
          {
            ConstantInt *CI = mdconst::extract<ConstantInt>(MD->getOperand(1));
            uint64_t count = CI->getValue().getZExtValue();

            // CASE 1
            // When a function is never invoked, make all its basic blocks speculatively dead.
            // This fix is required for privateer, where no profile data is collected for
            // speculatively dead code.
            if ( count == 0 )
            {
              auto &deadBlocks = loops[getLoopHeaderOfInterest()].deadBlocks;
              for ( auto i = fcn->begin(), e = fcn->end(); i != e; i++ )
              {
                const BasicBlock *bb = &*i;

                LLVM_DEBUG(
                    errs() << "EdgeProfOraclePDG: CASE 1 | Function "
                           << fcn->getName() << " never invoked. "
                           << bb->getName() << " is speculatively dead.\n");
                deadBlocks.insert(bb);
                numSpecBlocks++;
              }
            }
          }
      return;
    }

    // XXX: Is this some redundancy?
    uint64_t fcnt = fcn->getEntryCount().getCount();
    if ( fcnt == 0 ) // not possible in LLVM 5.0
    {
      /* LLVM_DEBUG( */
      /*   errs() << "EdgeProfOraclePDG: CASE 2 | Function " */
      /*          << fcn->getName() << " is never executed\n"); */

      // CASE 2
      // When a function is never invoked make all its basic blocks speculatively
      // dead. This fix is required for privateer
      auto &deadBlocks = loops[getLoopHeaderOfInterest()].deadBlocks;
      for ( auto i = fcn->begin(), e = fcn->end(); i != e; i++ )
      {
        const BasicBlock *bb = &*i;
        LLVM_DEBUG(
            errs() << "EdgeProfOraclePDG: CASE 2 (bug) | Function "
                   << fcn->getName() << " is never invoked. "
                   << bb->getName() << " is speculatively dead.\n");
        deadBlocks.insert(bb);
        numSpecBlocks++;
      }
    }

    LoopInfo &li = mloops->getAnalysis_LoopInfo(fcn);

    // For each conditional branch in this function which that executes at least once
    auto &deadEdges = loops[getLoopHeaderOfInterest()].deadEdges;
    auto &misspecInLoopProfExit = loops[getLoopHeaderOfInterest()].manifestedDeadExits;
    for ( auto i = fcn->begin(), e = fcn->end(); i != e; i++ )
    {
      const BasicBlock *pred = &*i;
      const Instruction *term = pred->getTerminator();
      const unsigned N = term->getNumSuccessors();

      if ( N < 2 ) // only a single successor ==> branch always taken
        continue;

      double pred_cnt;
      if ( bfi.getBlockProfileCount(pred).hasValue() )
        pred_cnt = bfi.getBlockProfileCount(pred).getValue();
      else
        pred_cnt = -1;

      // CASE 3
      if ( pred_cnt < MinSamples ) // ProfileInfo::MissingValue < 0 < MinSamples
      {
        LLVM_DEBUG(
            errs() << "EdgeProfOraclePDG: CASE 3 | Skipping branch at "
                   << fcn->getName() << " :: " << pred->getName()
                   << " because too few samples.\n");
        continue;
      }

      // For each control flow edge soured by this branch
      for ( unsigned sn = 0; sn < N; sn++ )
      {
        const BasicBlock *succ = term->getSuccessor(sn);
        auto prob = bpi.getEdgeProbability(pred, sn);
        const double rate = pred_cnt * prob.getNumerator() / prob.getDenominator();

        // CASE 4
        // getEdgeProbability() does not inform if weights are unknown but it
        // is assumed that if function or source block has a count then
        // the edge will have it as well
        if ( prob.isUnknown() )
        {
          LLVM_DEBUG(
              errs() << "EdgeProfOraclePDG: CASE 4 | Will not speculate in function "
                     << fcn->getName() << ". Edge weight unknown for edge "
                     << pred->getName() << " --> " << succ->getName() << "\n");
          continue;
        }

        // CASE 5
        if ( dominatesTargetHeader(succ) )
        {
          LLVM_DEBUG(
              errs() << "EdgeProfOraclePDG: CASE 5 | Will not speculate in function "
                     << fcn->getName() << ". Successor dominates target header for edge "
                     << pred->getName() << " --> " << succ->getName() << "\n");
          continue;
        }

        // Does this control-flow edge exit a subloop of our loop of interest?
        Loop *lpred = li.getLoopFor(pred);
        if ( lpred && !lpred->contains(succ)
                   && lpred->getHeader() != getLoopHeaderOfInterest())
        {
          // This control-flow edge exits a loop but that loop is not the loop-of-interest.
          // This case is special because we DO NOT want to speculate that this
          // loop is infinite UNDER ANY CIRCUMSTANCES.
          // If we did that the loop-of-interest would misspeculate during every iteration.

          // Even after running loop-simplify some loops are not in canonical form and
          // do not have dedicated exits. Thus, getUniqueExitBlock() will lead to an
          // assertion error. Instead check if the loop hasDedicatedExits().
          // If not, avoid this loop.

          // TODO: Make sure that ignoring loops not in canonical form is the right approach
          // here. Maybe we should still check the misspeculation rate and consider these
          // non-canonical exits.

          // CASE 6
          if ( !lpred->hasDedicatedExits() )
          {
            LLVM_DEBUG(
                errs() << "EdgeProfOraclePDG: CASE 6 | Will not speculate in function "
                       << fcn->getName() << ". Loop has no dedicated exits (i.e. not in canonical form). "
                       << "Avoid speculating the exit edge "
                       << pred->getName() << " --> " << succ->getName() << "\n");
            continue;
          }

          // CASE 7
          if ( lpred->getUniqueExitBlock() == pred )
          {
            LLVM_DEBUG(
                errs() << "EdgeProfOraclePDG: CASE 7 | Will not speculate in function "
                       << fcn->getName() << ". The unique exit of a loop at edge "
                       << fcn->getName() << " :: " << pred->getName() << "\n");
            continue;
          }

          // CASE 8
          if ( rate > pred_cnt * MaxMisspecLoopExit )
          {
            LLVM_DEBUG(
                errs() << "EdgeProfOraclePDG: CASE 8 | In function "
                       << fcn->getName() << ". Loop exit branch taken too frequently (> "
                       << MaxMisspecLoopExit << ") at "
                       << pred->getName() << ", " << (unsigned) rate << " / " << (unsigned) pred_cnt << "\n");
            continue;
          }
        }

        // Normal biased branches as well as branches which exit the loop-of-interest
        else
        {
          bool LoopOfInterestExit =
            lpred && !lpred->contains(succ)
                  && lpred->getHeader() == getLoopHeaderOfInterest();

          // CASE 9
          // loop exits too often
          if ( LoopOfInterestExit && rate > pred_cnt * MaxMisspecTargetLoopExit )
          {
            LLVM_DEBUG(
                errs() << "EdgeProfOraclePDG: CASE 9 | In function "
                       << fcn->getName() << ". Target loop exits too often. "
                       << pred->getName() << " --> " << succ->getName() << " cannot be speculated\n");
            continue;
          }

          // keep track of loop exits that will be speculated but manifest at least
          // one misspec during profiling
          if ( LoopOfInterestExit && rate  > pred_cnt * MaxMisspecLoopExit )
            misspecInLoopProfExit.insert(term);

          // CASE 10
          // Normal biasing threshold for branches that are not loop exits
          if ( !LoopOfInterestExit && rate > pred_cnt * MaxMisspec )
          {
            LLVM_DEBUG(
                errs() << "EdgeProfOraclePDG: CASE 10 | In function "
                       << fcn->getName() << ". Will not speculate edge "
                       << pred->getName() << " --> " << succ->getName()
                       << ", " << (unsigned) rate << " > " << (unsigned) pred_cnt << " * "
                       << format("%f", MaxMisspec) << "\n");
            continue;
          }
        }

        LLVM_DEBUG(
            errs() << "EdgeProfOraclePDG: DEAD_EDGE | In function "
                   << fcn->getName() << " -- speculating that "
                   << pred->getName() << " never branches to " << succ->getName()
                   << " [observed rate " << (unsigned) rate
                   << " / " << (unsigned) pred_cnt << " samples]\n");

        if ( !deadEdges.count(term) )
          deadEdges[term].resize(N);

        deadEdges[term].set(sn);
        numSpecEdges++;
      }
    }

    // Next, determine which blocks are still reachable without misspeculating
    std::vector<const BasicBlock *> fringe;
    std::set<const BasicBlock *> reachable;

    fringe.push_back(&fcn->getEntryBlock());
    while ( !fringe.empty() )
    {
      const BasicBlock *bb = fringe.back();
      fringe.pop_back();

      if ( reachable.count(bb) )
        continue;
      reachable.insert(bb);

      const Instruction *term = bb->getTerminator();
      for ( unsigned sn = 0, N = term->getNumSuccessors(); sn < N; sn++ )
      {
        // Skip speculated edges
        if ( isSpeculativelyDead(term, sn) )
          continue;

        // Add successor to fringe
        fringe.push_back(term->getSuccessor(sn));
      }
    }

    // Finally record blocks which are NOT reachable anymore
    auto &deadBlocks = loops[getLoopHeaderOfInterest()].deadBlocks;
    for ( auto i = fcn->begin(), e = fcn->end(); i != e; i++ )
    {
      const BasicBlock *bb = &*i;
      numTotalBlocks++;

      if ( !reachable.count(bb) )
      {
        LLVM_DEBUG(
            errs() << "EdgeProfOraclePDG: DEAD_BLOCK | In function "
                   << fcn->getName() << " :: " << bb->getName()
                   << " is speculatively dead.\n");
        deadBlocks.insert(bb);
        numSpecBlocks++;
      }
    }
  }

  bool EdgeProfOraclePDG::isSpeculativelyDead(const Instruction *term, unsigned succNo)
  {
    const BasicBlock *bb = term->getParent();
    const Function *fcn = bb->getParent();

    visit(fcn);

    const auto &deadEdges = loops[getLoopHeaderOfInterest()].deadEdges;
    auto i = deadEdges.find(term);
    if ( i == deadEdges.end() )
      return false;

    return i->second.test(succNo);
  }

  bool EdgeProfOraclePDG::isSpeculativelyDead(const BasicBlock *bb)
  {
    const Function *fcn = bb->getParent();

    visit(fcn);

    const auto &deadBlocks = loops[getLoopHeaderOfInterest()].deadBlocks;

    return deadBlocks.count(bb);
  }


  char EdgeProfOraclePDG::ID = 0;
  static RegisterPass<EdgeProfOraclePDG> rp("edgeprof-oracle-pdg", "EdgeProf Oracle PDG");
} // namespace liberty

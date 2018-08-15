/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2018, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file     EncCu.h
    \brief    Coding Unit (CU) encoder class (header)
*/

#ifndef __ENCCU__
#define __ENCCU__

// Include files
#include "CommonLib/CommonDef.h"
#include "CommonLib/IntraPrediction.h"
#include "CommonLib/InterPrediction.h"
#include "CommonLib/TrQuant.h"
#include "CommonLib/Unit.h"
#include "CommonLib/UnitPartitioner.h"

#if REUSE_CU_RESULTS
#include "DecoderLib/DecCu.h"
#endif

#include "CABACWriter.h"
#include "IntraSearch.h"
#include "InterSearch.h"
#include "RateCtrl.h"
#include "EncModeCtrl.h"
//! \ingroup EncoderLib
//! \{

class EncLib;
class HLSWriter;
class EncSlice;

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// CU encoder class
class EncCu
#if REUSE_CU_RESULTS
  : DecCu
#endif
{
private:

  struct CtxPair
  {
    Ctx start;
    Ctx best;
  };

  std::vector<CtxPair>  m_CtxBuffer;
  CtxPair*              m_CurrCtx;
  CtxCache*             m_CtxCache;

#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  int                   m_dataId;
#endif

  //  Data : encoder control
  int                   m_cuChromaQpOffsetIdxPlus1; // if 0, then cu_chroma_qp_offset_flag will be 0, otherwise cu_chroma_qp_offset_flag will be 1.

  XUCache               m_unitCache;

  CodingStructure    ***m_pTempCS;
  CodingStructure    ***m_pBestCS;
  //  Access channel
  EncCfg*               m_pcEncCfg;
  IntraSearch*          m_pcIntraSearch;
  InterSearch*          m_pcInterSearch;
  TrQuant*              m_pcTrQuant;
  RdCost*               m_pcRdCost;
  EncSlice*             m_pcSliceEncoder;

  CABACWriter*          m_CABACEstimator;
  RateCtrl*             m_pcRateCtrl;
#if JVET_K0357_AMVR
  CodingStructure    ***m_pImvTempCS;
#endif
  EncModeCtrl          *m_modeCtrl;

  PelStorage            m_acMergeBuffer[MRG_MAX_NUM_CANDS];

#if JVET_K0346
  MotionInfo            m_SubPuMiBuf      [( MAX_CU_SIZE * MAX_CU_SIZE ) >> ( MIN_CU_LOG2 << 1 )];
#endif
#if JVET_K0346
  unsigned int          m_subMergeBlkSize[10];
  unsigned int          m_subMergeBlkNum[10];
  unsigned int          m_prevPOC;
  bool                  m_clearSubMergeStatic;
#endif

#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  EncLib*               m_pcEncLib;
#endif

#if SHARP_LUMA_DELTA_QP
  void    updateLambda      ( Slice* slice, double dQP );
#endif

public:
  /// copy parameters from encoder class
  void  init                ( EncLib* pcEncLib, const SPS& sps PARL_PARAM( const int jId = 0 ) );

  /// create internal buffers
  void  create              ( EncCfg* encCfg );

  /// destroy internal buffers
  void  destroy             ();

  /// CTU analysis function
  void  compressCtu         ( CodingStructure& cs, const UnitArea& area, const unsigned ctuRsAddr, const int prevQP[], const int currQP[] );
  /// CTU encoding function
  int   updateCtuDataISlice ( const CPelBuf buf );

  EncModeCtrl* getModeCtrl  () { return m_modeCtrl; }

#if JVET_K0346
  void clearSubMergeStatics()
  {
    ::memset(m_subMergeBlkSize, 0, sizeof(m_subMergeBlkSize));
    ::memset(m_subMergeBlkNum, 0, sizeof(m_subMergeBlkNum));
  }

  void clearOneTLayerSubMergeStatics(unsigned int layer)
  {
    m_subMergeBlkSize[layer] = 0;
    m_subMergeBlkNum[layer] = 0;
  }
  unsigned int getSubMergeBlkSize(unsigned int layer) { return m_subMergeBlkSize[layer]; }
  unsigned int getSubMergeBlkNum(unsigned int layer) { return m_subMergeBlkNum[layer]; }
  void incrementSubMergeBlkSize(unsigned int layer, unsigned int inc) { m_subMergeBlkSize[layer] += inc; }
  void incrementSubMergeBlkNum(unsigned int layer, unsigned int inc) { m_subMergeBlkNum[layer] += inc; }
  void setPrevPOC(unsigned int poc) { m_prevPOC = poc; }
  unsigned int getPrevPOC() { return m_prevPOC; }
  void setClearSubMergeStatic(bool b) { m_clearSubMergeStatic = b; }
  bool getClearSubMergeStatic() { return m_clearSubMergeStatic; }
#endif

  ~EncCu();

protected:

  void xCompressCU            ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm );
#if ENABLE_SPLIT_PARALLELISM
  void xCompressCUParallel    ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm );
  void copyState              ( EncCu* other, Partitioner& pm, const UnitArea& currArea, const bool isDist );
#endif

  void xCheckBestMode         ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestmode );

  void xCheckModeSplit        ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestMode );

  void xCheckRDCostIntra      ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestMode );
  void xCheckIntraPCM         ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestMode );

  void xCheckDQP              ( CodingStructure& cs, Partitioner& partitioner, bool bKeepCtx = false);
  void xFillPCMBuffer         ( CodingUnit &cu);

#if JVET_K_AFFINE
  void xCheckRDCostAffineMerge2Nx2N
                              ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode );
#endif
  void xCheckRDCostInter      ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestMode );
#if JVET_K0357_AMVR
  bool xCheckRDCostInterIMV   ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestMode );
#endif
  void xEncodeDontSplit       ( CodingStructure &cs, Partitioner &partitioner);

  void xCheckRDCostMerge2Nx2N ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestMode );

#if JVET_K0357_AMVR
  void xEncodeInterResidual   ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode, int residualPass = 0, CodingStructure* imvCS = NULL, int emtMode = 1, bool* bestHasNonResi = NULL );
#else
  void xEncodeInterResidual   ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode, int residualPass, bool* bestHasNonResi );
#endif
#if REUSE_CU_RESULTS

  void xReuseCachedResult     ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &Partitioner );
#endif
};

//! \}

#endif // __ENCMB__

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

/** \file     IntraSearch.h
    \brief    intra search class (header)
*/

#ifndef __INTRASEARCH__
#define __INTRASEARCH__

// Include files

#include "CABACWriter.h"
#include "EncCfg.h"

#include "CommonLib/IntraPrediction.h"
#include "CommonLib/CrossCompPrediction.h"
#include "CommonLib/TrQuant.h"
#include "CommonLib/Unit.h"
#include "CommonLib/RdCost.h"

//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

class EncModeCtrl;

/// encoder search class
class IntraSearch : public IntraPrediction, CrossComponentPrediction
{
private:
#if !JVET_K0220_ENC_CTRL
  EncModeCtrl    *m_modeCtrl; //we need this to call the saveLoadTag functions for the EMT
#endif
  Pel*            m_pSharedPredTransformSkip[MAX_NUM_TBLOCKS];

  XUCache         m_unitCache;

  CodingStructure ****m_pSplitCS;
  CodingStructure ****m_pFullCS;

  CodingStructure ***m_pTempCS;
  CodingStructure ***m_pBestCS;

  CodingStructure **m_pSaveCS;

#if JVET_K1000_SIMPLIFIED_EMT
  //cost variables for the EMT algorithm and new modes list
  double m_bestModeCostStore[4];                                    // RD cost of the best mode for each PU using DCT2
  double m_modeCostStore    [4][NUM_LUMA_MODE];                         // RD cost of each mode for each PU using DCT2
  uint32_t   m_savedRdModeList  [4][NUM_LUMA_MODE], m_savedNumRdModes[4];

#endif
protected:
  // interface to option
  EncCfg*         m_pcEncCfg;

  // interface to classes
  TrQuant*        m_pcTrQuant;
  RdCost*         m_pcRdCost;

  // RD computation
  CABACWriter*    m_CABACEstimator;
  CtxCache*       m_CtxCache;

  bool            m_isInitialized;

public:

  IntraSearch();
  ~IntraSearch();

  void init                       ( EncCfg*        pcEncCfg,
                                    TrQuant*       pcTrQuant,
                                    RdCost*        pcRdCost,
                                    CABACWriter*   CABACEstimator,
                                    CtxCache*      ctxCache,
                                    const uint32_t     maxCUWidth,
                                    const uint32_t     maxCUHeight,
                                    const uint32_t     maxTotalCUDepth
                                  );

  void destroy                    ();

  CodingStructure****getSplitCSBuf() { return m_pSplitCS; }
  CodingStructure****getFullCSBuf () { return m_pFullCS; }
  CodingStructure  **getSaveCSBuf () { return m_pSaveCS; }

#if !JVET_K0220_ENC_CTRL
  void setModeCtrl                (EncModeCtrl *modeCtrl) { m_modeCtrl = modeCtrl; }

#endif
public:

  void estIntraPredLumaQT         ( CodingUnit &cu, Partitioner& pm );
  void estIntraPredChromaQT       (CodingUnit &cu, Partitioner& pm);
  void IPCMSearch                 (CodingStructure &cs, Partitioner& partitioner);

protected:

  // -------------------------------------------------------------------------------------------------------------------
  // T & Q & Q-1 & T-1
  // -------------------------------------------------------------------------------------------------------------------

  void xEncPCM                    (CodingStructure &cs, Partitioner& partitioner, const ComponentID &compID);

  // -------------------------------------------------------------------------------------------------------------------
  // Intra search
  // -------------------------------------------------------------------------------------------------------------------

  void xEncIntraHeader            (CodingStructure &cs, Partitioner& pm, const bool &bLuma, const bool &bChroma);
  void xEncSubdivCbfQT            (CodingStructure &cs, Partitioner& pm, const bool &bLuma, const bool &bChroma);
  uint64_t xGetIntraFracBitsQT      (CodingStructure &cs, Partitioner& pm, const bool &bLuma, const bool &bChroma);

  uint64_t xGetIntraFracBitsQTChroma(TransformUnit& tu, const ComponentID &compID);
  void xEncCoeffQT                (CodingStructure &cs, Partitioner& pm, const ComponentID &compID);

  uint64_t xFracModeBitsIntra       (PredictionUnit &pu, const uint32_t &uiMode, const ChannelType &compID);

  void xIntraCodingTUBlock        (TransformUnit &tu, const ComponentID &compID, const bool &checkCrossCPrediction, Distortion& ruiDist, const int &default0Save1Load2 = 0, uint32_t* numSig = nullptr );

  ChromaCbfs xRecurIntraChromaCodingQT  (CodingStructure &cs, Partitioner& pm);

  void xRecurIntraCodingLumaQT    ( CodingStructure &cs, Partitioner& pm );


  void encPredIntraDPCM( const ComponentID &compID, PelBuf &pOrg, PelBuf &pDst, const uint32_t &uiDirMode );
  static bool useDPCMForFirstPassIntraEstimation( const PredictionUnit &pu, const uint32_t &uiDirMode );
};// END CLASS DEFINITION EncSearch

//! \}

#endif // __ENCSEARCH__

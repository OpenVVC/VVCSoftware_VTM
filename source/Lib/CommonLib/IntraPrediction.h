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

/** \file     IntraPrediction.h
    \brief    prediction class (header)
*/

#ifndef __INTRAPREDICTION__
#define __INTRAPREDICTION__


// Include files
#include "Unit.h"
#include "Buffer.h"
#include "Picture.h"


//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// prediction class
enum PredBuf
{
  PRED_BUF_UNFILTERED = 0,
  PRED_BUF_FILTERED   = 1,
  NUM_PRED_BUF        = 2
};

static const uint32_t MAX_INTRA_FILTER_DEPTHS=8;

class IntraPrediction
{
private:

  Pel* m_piYuvExt[MAX_NUM_COMPONENT][NUM_PRED_BUF];
  int  m_iYuvExtSize;


  static const uint8_t m_aucIntraFilter[MAX_NUM_CHANNEL_TYPE][MAX_INTRA_FILTER_DEPTHS];

#if JVET_K0190
  unsigned m_auShiftLM[32]; // Table for substituting division operation by multiplication

#endif
  Pel* m_piTemp;
#if !JVET_K0190
#endif
protected:

  ChromaFormat  m_currChromaFormat;

#if JVET_K0500_WAIP
  int m_topRefLength;
  int m_leftRefLength;
#endif
  // prediction
  void xPredIntraPlanar           ( const CPelBuf &pSrc, PelBuf &pDst,                                                                                                         const SPS& sps );
  void xPredIntraDc               ( const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType,                                                                                          const bool enableBoundaryFilter = true );
#if HEVC_USE_HOR_VER_PREDFILTERING
  void xPredIntraAng              ( const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType, const uint32_t dirMode, const ClpRng& clpRng, const bool bEnableEdgeFilters, const SPS& sps, const bool enableBoundaryFilter = true );
#else
  void xPredIntraAng              ( const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType, const uint32_t dirMode, const ClpRng& clpRng, const SPS& sps, const bool enableBoundaryFilter = true );
#endif
  Pel  xGetPredValDc              ( const CPelBuf &pSrc, const Size &dstSize );

  void xFillReferenceSamples      ( const CPelBuf &recoBuf,      Pel* refBufUnfiltered, const CompArea &area, const CodingUnit &cu );
  void xFilterReferenceSamples    ( const Pel* refBufUnfiltered, Pel* refBufFiltered, const CompArea &area, const SPS &sps );

#if HEVC_USE_DC_PREDFILTERING
  // dc filtering
  void xDCPredFiltering           ( const CPelBuf &pSrc, PelBuf &pDst, const ChannelType &channelType );
#endif
#if JVET_K0500_WAIP
  static int getWideAngle         ( int width, int height, int predMode );
  void setReferenceArrayLengths   ( const CompArea &area );
#endif

  void destroy                    ();

  void xFilterGroup               ( Pel* pMulDst[], int i, Pel const* const piSrc, int iRecStride, bool bAboveAvaillable, bool bLeftAvaillable);
#if !JVET_K0190
#else
  void xGetLMParameters(const PredictionUnit &pu, const ComponentID compID, const CompArea& chromaArea, int& a, int& b, int& iShift);
#endif
public:
  IntraPrediction();
  virtual ~IntraPrediction();

  void init                       (ChromaFormat chromaFormatIDC, const unsigned bitDepthY);

  // Angular Intra
  void predIntraAng               ( const ComponentID compId, PelBuf &piPred, const PredictionUnit &pu, const bool useFilteredPredSamples );
  Pel*  getPredictorPtr           (const ComponentID compID, const bool bUseFilteredPredictions = false) { return m_piYuvExt[compID][bUseFilteredPredictions?PRED_BUF_FILTERED:PRED_BUF_UNFILTERED]; }
#if JVET_K0190
  // Cross-component Chroma
  void predIntraChromaLM(const ComponentID compID, PelBuf &piPred, const PredictionUnit &pu, const CompArea& chromaArea, int intraDir);
  void xGetLumaRecPixels(const PredictionUnit &pu, CompArea chromaArea);
#else
#endif
  /// set parameters from CU data for accessing intra data
  void initIntraPatternChType     (const CodingUnit &cu, const CompArea &area, const bool bFilterRefSamples = false );

static bool useFilteredIntraRefSamples( const ComponentID &compID, const PredictionUnit &pu, bool modeSpecific, const UnitArea &tuArea );
  static bool useDPCMForFirstPassIntraEstimation(const PredictionUnit &pu, const uint32_t &uiDirMode);
};

//! \}

#endif // __INTRAPREDICTION__

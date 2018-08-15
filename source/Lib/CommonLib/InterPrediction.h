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

/** \file     InterPrediction.h
    \brief    inter prediction class (header)
*/

#ifndef __INTERPREDICTION__
#define __INTERPREDICTION__


// Include files
#include "InterpolationFilter.h"
#include "WeightPrediction.h"

#include "Buffer.h"
#include "Unit.h"
#include "Picture.h"

#include "RdCost.h"
#include "ContextModelling.h"

// forward declaration
class Mv;

//! \ingroup CommonLib
//! \{


// ====================================================================================================================
// Class definition
// ====================================================================================================================


class InterPrediction : public WeightPrediction
{
private:


protected:
  InterpolationFilter  m_if;

  Pel*                 m_acYuvPred            [NUM_REF_PIC_LIST_01][MAX_NUM_COMPONENT];
  Pel*                 m_filteredBlock        [LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][MAX_NUM_COMPONENT];
  Pel*                 m_filteredBlockTmp     [LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][MAX_NUM_COMPONENT];


  ChromaFormat         m_currChromaFormat;

  ComponentID          m_maxCompIDToPred;      ///< tells the predictor to only process the components up to (inklusive) this one - useful to skip chroma components during RD-search

  RdCost*              m_pcRdCost;

  int                  m_iRefListIdx;
  

  void xPredInterUni            ( const PredictionUnit& pu, const RefPicList& eRefPicList, PelUnitBuf& pcYuvPred, const bool& bi );
  void xPredInterBi             ( PredictionUnit& pu, PelUnitBuf &pcYuvPred );
  void xPredInterBlk            ( const ComponentID& compID, const PredictionUnit& pu, const Picture* refPic, const Mv& _mv, PelUnitBuf& dstPic, const bool& bi, const ClpRng& clpRng
                                 );
  
  void xWeightedAverage         ( const PredictionUnit& pu, const CPelUnitBuf& pcYuvSrc0, const CPelUnitBuf& pcYuvSrc1, PelUnitBuf& pcYuvDst, const BitDepths& clipBitDepths, const ClpRngs& clpRngs );
#if JVET_K_AFFINE
  void xPredAffineBlk( const ComponentID& compID, const PredictionUnit& pu, const Picture* refPic, const Mv* _mv, PelUnitBuf& dstPic, const bool& bi, const ClpRng& clpRng );
#endif

  static bool xCheckIdenticalMotion( const PredictionUnit& pu );

#if JVET_K0346
  void xSubPuMC(PredictionUnit& pu, PelUnitBuf& predBuf, const RefPicList &eRefPicList = REF_PIC_LIST_X);
#endif

  void destroy();


#if JVET_K0346
  MotionInfo      m_SubPuMiBuf[(MAX_CU_SIZE * MAX_CU_SIZE) >> (MIN_CU_LOG2 << 1)];
#endif

public:
  InterPrediction();
  virtual ~InterPrediction();

  void    init                (RdCost* pcRdCost, ChromaFormat chromaFormatIDC);

  // inter
  void    motionCompensation  (PredictionUnit &pu, PelUnitBuf& predBuf, const RefPicList &eRefPicList = REF_PIC_LIST_X);
  void    motionCompensation  (PredictionUnit &pu, const RefPicList &eRefPicList = REF_PIC_LIST_X);
  void    motionCompensation  (CodingUnit &cu,     const RefPicList &eRefPicList = REF_PIC_LIST_X);

#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  void    cacheAssign( CacheModel *cache );
#endif

};

//! \}

#endif // __INTERPREDICTION__

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

/** \file     TrQuant.h
    \brief    transform and quantization class (header)
*/

#ifndef __TRQUANT__
#define __TRQUANT__

#include "CommonDef.h"
#include "Unit.h"
#include "ChromaFormat.h"
#include "Contexts.h"
#include "ContextModelling.h"

#include "UnitPartitioner.h"
#include "Quant.h"

//! \ingroup CommonLib
//! \{

#if JVET_K1000_SIMPLIFIED_EMT
typedef void FwdTrans(const TCoeff*, TCoeff*, int, int, int, int);
typedef void InvTrans(const TCoeff*, TCoeff*, int, int, int, int, const TCoeff, const TCoeff);
#else
typedef void FwdTrans(const TCoeff*, TCoeff*, int, int, int, int, int);
typedef void InvTrans(const TCoeff*, TCoeff*, int, int, int, int, int, const TCoeff, const TCoeff);
#endif

// ====================================================================================================================
// Class definition
// ====================================================================================================================


/// transform and quantization class
class TrQuant
{
public:
  TrQuant();
  ~TrQuant();

  // initialize class
  void init      (
                    const Quant* otherQuant,
                    const uint32_t uiMaxTrSize,
                    const bool bUseRDOQ             = false,
                    const bool bUseRDOQTS           = false,
#if T0196_SELECTIVE_RDOQ
                    const bool useSelectiveRDOQ     = false,
#endif
#if JVET_K0072
#else
#endif
                    const bool bEnc                 = false,
                    const bool useTransformSkipFast = false,
#if !INTRA67_3MPM
#endif
                    const bool rectTUs              = false
  );

#if JVET_K1000_SIMPLIFIED_EMT
  uint8_t getEmtTrIdx( TransformUnit tu, const ComponentID compID );
  uint8_t getEmtMode ( TransformUnit tu, const ComponentID compID );
#endif


protected:

public:

  void invTransformNxN  (TransformUnit &tu, const ComponentID &compID, PelBuf &pResi, const QpParam &cQPs);

  void transformNxN     (TransformUnit &tu, const ComponentID &compID, const QpParam &cQP, TCoeff &uiAbsSum, const Ctx &ctx);
  void rdpcmNxN         (TransformUnit &tu, const ComponentID &compID, const QpParam &cQP, TCoeff &uiAbsSum,       RDPCMMode &rdpcmMode);
  void applyForwardRDPCM(TransformUnit &tu, const ComponentID &compID, const QpParam &cQP, TCoeff &uiAbsSum, const RDPCMMode &rdpcmMode);

  void transformSkipQuantOneSample(TransformUnit &tu, const ComponentID &compID, const TCoeff &resiDiff, TCoeff &coeff,    const uint32_t &uiPos, const QpParam &cQP, const bool bUseHalfRoundingPoint);
  void invTrSkipDeQuantOneSample  (TransformUnit &tu, const ComponentID &compID, const TCoeff &pcCoeff,  Pel &reconSample, const uint32_t &uiPos, const QpParam &cQP);

  void invRdpcmNxN(TransformUnit& tu, const ComponentID &compID, PelBuf &pcResidual);
#if RDOQ_CHROMA_LAMBDA
  void   setLambdas  ( const double lambdas[MAX_NUM_COMPONENT] )   { m_quant->setLambdas( lambdas ); }
  void   selectLambda( const ComponentID compIdx )                 { m_quant->selectLambda( compIdx ); }
  void   getLambdas  ( double (&lambdas)[MAX_NUM_COMPONENT]) const { m_quant->getLambdas( lambdas ); }
#endif
  void   setLambda   ( const double dLambda )                      { m_quant->setLambda( dLambda ); }
  double getLambda   () const                                      { return m_quant->getLambda(); }

  Quant* getQuant() { return m_quant;  }


#if ENABLE_SPLIT_PARALLELISM
  void    copyState( const TrQuant& other );
#endif

protected:
  TCoeff*  m_plTempCoeff;
  uint32_t     m_uiMaxTrSize;
  bool     m_bEnc;
  bool     m_useTransformSkipFast;

#if !INTRA67_3MPM
#endif
  bool     m_rectTUs;

  bool     m_scalingListEnabledFlag;

private:
  Quant    *m_quant;          //!< Quantizer


  // forward Transform
  void xT        ( const TransformUnit &tu, const ComponentID &compID, const CPelBuf &resi, CoeffBuf &dstCoeff, const int iWidth, const int iHeight );

#if !JVET_K1000_SIMPLIFIED_EMT
#if HEVC_USE_4x4_DSTVII
  void (*m_fTr ) ( const int bitDepth, const Pel *residual, size_t stride, TCoeff *coeff, size_t width, size_t height, bool useDST, const int maxLog2TrDynamicRange );
  void (*m_fITr) ( const int bitDepth, const TCoeff *coeff, Pel *residual, size_t stride, size_t width, size_t height, bool useDST, const int maxLog2TrDynamicRange );
#else
  void (*m_fTr ) ( const int bitDepth, const Pel *residual, size_t stride, TCoeff *coeff, size_t width, size_t height, const int maxLog2TrDynamicRange );
  void (*m_fITr) ( const int bitDepth, const TCoeff *coeff, Pel *residual, size_t stride, size_t width, size_t height, const int maxLog2TrDynamicRange );
#endif
#endif

  // skipping Transform
  void xTransformSkip   (const TransformUnit &tu, const ComponentID &compID, const CPelBuf &resi, TCoeff* psCoeff);

  // quantization
  void xQuant           (TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pSrc, TCoeff &uiAbsSum, const QpParam &cQP, const Ctx& ctx);

  // dequantization
  void xDeQuant( const TransformUnit &tu,
                       CoeffBuf      &dstCoeff,
                 const ComponentID   &compID,
                 const QpParam       &cQP      );

  // inverse transform
  void xIT     ( const TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pCoeff, PelBuf &pResidual );

  // inverse skipping transform
  void xITransformSkip(
                 const CCoeffBuf     &plCoef,
                       PelBuf        &pResidual,
                 const TransformUnit &tu,
                 const ComponentID   &component);


#ifdef TARGET_SIMD_X86
  template<X86_VEXT vext>
  void _initTrQuantX86();
  void initTrQuantX86();
#endif
};// END CLASS DEFINITION TrQuant

//! \}

#endif // __TRQUANT__

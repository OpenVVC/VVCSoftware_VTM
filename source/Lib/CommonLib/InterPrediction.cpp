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

/** \file     Prediction.cpp
    \brief    prediction class
*/

#include "InterPrediction.h"

#include "Buffer.h"
#include "UnitTools.h"

#include <memory.h>
#include <algorithm>

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Constructor / destructor / initialize
// ====================================================================================================================

InterPrediction::InterPrediction()
:
  m_currChromaFormat( NUM_CHROMA_FORMAT )
, m_maxCompIDToPred ( MAX_NUM_COMPONENT )
, m_pcRdCost        ( nullptr )
{
  for( uint32_t ch = 0; ch < MAX_NUM_COMPONENT; ch++ )
  {
    for( uint32_t refList = 0; refList < NUM_REF_PIC_LIST_01; refList++ )
    {
      m_acYuvPred[refList][ch] = nullptr;
    }
  }

  for( uint32_t c = 0; c < MAX_NUM_COMPONENT; c++ )
  {
    for( uint32_t i = 0; i < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; i++ )
    {
      for( uint32_t j = 0; j < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; j++ )
      {
        m_filteredBlock[i][j][c] = nullptr;
      }

      m_filteredBlockTmp[i][c] = nullptr;
    }
  }

}

InterPrediction::~InterPrediction()
{
  destroy();
}

void InterPrediction::destroy()
{
  for( uint32_t i = 0; i < NUM_REF_PIC_LIST_01; i++ )
  {
    for( uint32_t c = 0; c < MAX_NUM_COMPONENT; c++ )
    {
      xFree( m_acYuvPred[i][c] );
      m_acYuvPred[i][c] = nullptr;
    }
  }

  for( uint32_t c = 0; c < MAX_NUM_COMPONENT; c++ )
  {
    for( uint32_t i = 0; i < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; i++ )
    {
      for( uint32_t j = 0; j < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; j++ )
      {
        xFree( m_filteredBlock[i][j][c] );
        m_filteredBlock[i][j][c] = nullptr;
      }

      xFree( m_filteredBlockTmp[i][c] );
      m_filteredBlockTmp[i][c] = nullptr;
    }
  }

}

void InterPrediction::init( RdCost* pcRdCost, ChromaFormat chromaFormatIDC )
{
  m_pcRdCost = pcRdCost;


  // if it has been initialised before, but the chroma format has changed, release the memory and start again.
  if( m_acYuvPred[REF_PIC_LIST_0][COMPONENT_Y] != nullptr && m_currChromaFormat != chromaFormatIDC )
  {
    destroy();
  }

  m_currChromaFormat = chromaFormatIDC;

  if( m_acYuvPred[REF_PIC_LIST_0][COMPONENT_Y] == nullptr ) // check if first is null (in which case, nothing initialised yet)
  {
    for( uint32_t c = 0; c < MAX_NUM_COMPONENT; c++ )
    {
      int extWidth  = MAX_CU_SIZE + 16;
      int extHeight = MAX_CU_SIZE + 1;
      for( uint32_t i = 0; i < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; i++ )
      {
        m_filteredBlockTmp[i][c] = ( Pel* ) xMalloc( Pel, ( extWidth + 4 ) * ( extHeight + 7 + 4 ) );

        for( uint32_t j = 0; j < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; j++ )
        {
          m_filteredBlock[i][j][c] = ( Pel* ) xMalloc( Pel, extWidth * extHeight );
        }
      }

      // new structure
      for( uint32_t i = 0; i < NUM_REF_PIC_LIST_01; i++ )
      {
        m_acYuvPred[i][c] = ( Pel* ) xMalloc( Pel, MAX_CU_SIZE * MAX_CU_SIZE );
      }
    }

    m_iRefListIdx = -1;
    
  }

#if !JVET_J0090_MEMORY_BANDWITH_MEASURE
  m_if.initInterpolationFilter( true );
#endif
}

#if JVET_K_AFFINE
bool checkIdenticalMotion( const PredictionUnit &pu, bool checkAffine )
#else
bool checkIdenticalMotion( const PredictionUnit &pu )
#endif
{
  const Slice &slice = *pu.cs->slice;

  if( slice.isInterB() && !pu.cs->pps->getWPBiPred() )
  {
    if( pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0 )
    {
      int RefPOCL0 = slice.getRefPic( REF_PIC_LIST_0, pu.refIdx[0] )->getPOC();
      int RefPOCL1 = slice.getRefPic( REF_PIC_LIST_1, pu.refIdx[1] )->getPOC();

      if( RefPOCL0 == RefPOCL1 )
      {
#if JVET_K_AFFINE
        if( !pu.cu->affine )
#endif
        {
          if( pu.mv[0] == pu.mv[1] )
          {
            return true;
          }
        }
#if JVET_K_AFFINE
        else
        {
          CHECK( !checkAffine, "In this case, checkAffine should be on." );
          const CMotionBuf &mb = pu.getMotionBuf();
#if JVET_K_AFFINE_BUG_FIXES
#if JVET_K0337_AFFINE_6PARA
          if ( (pu.cu->affineType == AFFINEMODEL_4PARAM && (mb.at( 0, 0 ).mv[0] == mb.at( 0, 0 ).mv[1]) && (mb.at( mb.width - 1, 0 ).mv[0] == mb.at( mb.width - 1, 0 ).mv[1]))
            || (pu.cu->affineType == AFFINEMODEL_6PARAM && (mb.at( 0, 0 ).mv[0] == mb.at( 0, 0 ).mv[1]) && (mb.at( mb.width - 1, 0 ).mv[0] == mb.at( mb.width - 1, 0 ).mv[1]) && (mb.at( 0, mb.height - 1 ).mv[0] == mb.at( 0, mb.height - 1 ).mv[1])) )
#else
          if ( (mb.at( 0, 0 ).mv[0] == mb.at( 0, 0 ).mv[1]) && (mb.at( mb.width - 1, 0 ).mv[0] == mb.at( mb.width - 1, 0 ).mv[1]) )
#endif
#else
          if( ( mb.at( 0, 0 ).mv[0] == mb.at( 0, 0 ).mv[1] ) && ( mb.at( mb.width - 1, 0 ).mv[0] == mb.at( mb.width - 1, 0 ).mv[1] ) && ( mb.at( 0, mb.height - 1 ).mv[0] == mb.at( 0, mb.height - 1 ).mv[1] ) )
#endif
          {
            return true;
          }
        }
#endif
      }
    }
  }

  return false;
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

bool InterPrediction::xCheckIdenticalMotion( const PredictionUnit &pu )
{
  const Slice &slice = *pu.cs->slice;

  if( slice.isInterB() && !pu.cs->pps->getWPBiPred() )
  {
    if( pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0 )
    {
      int RefPOCL0 = slice.getRefPic( REF_PIC_LIST_0, pu.refIdx[0] )->getPOC();
      int RefPOCL1 = slice.getRefPic( REF_PIC_LIST_1, pu.refIdx[1] )->getPOC();

      if( RefPOCL0 == RefPOCL1 )
      {
#if JVET_K_AFFINE
        if( !pu.cu->affine )
#endif
        {
          if( pu.mv[0] == pu.mv[1] )
          {
            return true;
          }
        }
#if JVET_K_AFFINE
        else
        {
          const CMotionBuf &mb = pu.getMotionBuf();
#if JVET_K_AFFINE_BUG_FIXES
#if JVET_K0337_AFFINE_6PARA
          if ( (pu.cu->affineType == AFFINEMODEL_4PARAM && (mb.at( 0, 0 ).mv[0] == mb.at( 0, 0 ).mv[1]) && (mb.at( mb.width - 1, 0 ).mv[0] == mb.at( mb.width - 1, 0 ).mv[1]))
            || (pu.cu->affineType == AFFINEMODEL_6PARAM && (mb.at( 0, 0 ).mv[0] == mb.at( 0, 0 ).mv[1]) && (mb.at( mb.width - 1, 0 ).mv[0] == mb.at( mb.width - 1, 0 ).mv[1]) && (mb.at( 0, mb.height - 1 ).mv[0] == mb.at( 0, mb.height - 1 ).mv[1])) )
#else
          if ( (mb.at( 0, 0 ).mv[0] == mb.at( 0, 0 ).mv[1]) && (mb.at( mb.width - 1, 0 ).mv[0] == mb.at( mb.width - 1, 0 ).mv[1]) )
#endif
#else
          if( ( mb.at( 0, 0 ).mv[0] == mb.at( 0, 0 ).mv[1] ) && ( mb.at( mb.width - 1, 0 ).mv[0] == mb.at( mb.width - 1, 0 ).mv[1] ) && ( mb.at( 0, mb.height - 1 ).mv[0] == mb.at( 0, mb.height - 1 ).mv[1] ) )
#endif
          {
            return true;
          }
        }
#endif
      }
    }
  }

  return false;
}

#if JVET_K0346
void InterPrediction::xSubPuMC( PredictionUnit& pu, PelUnitBuf& predBuf, const RefPicList &eRefPicList /*= REF_PIC_LIST_X*/ )
{
#if !JVET_K0346
  const SPSNext& spsNext  = pu.cs->sps->getSpsNext();
#endif

  // compute the location of the current PU
  Position puPos    = pu.lumaPos();
  Size puSize       = pu.lumaSize();

#if JVET_K0346
  int numPartLine, numPartCol, puHeight, puWidth;
#else
  int iNumPartLine, iNumPartCol, iPUHeight, iPUWidth;
#endif
  {
#if JVET_K0346
    const Slice& slice = *pu.cs->slice;
    numPartLine = std::max(puSize.width >> slice.getSubPuMvpSubblkLog2Size(), 1u);
    numPartCol  = std::max(puSize.height >> slice.getSubPuMvpSubblkLog2Size(), 1u);
    puHeight    = numPartCol == 1 ? puSize.height : 1 << slice.getSubPuMvpSubblkLog2Size();
    puWidth     = numPartLine == 1 ? puSize.width : 1 << slice.getSubPuMvpSubblkLog2Size();
#else
    iNumPartLine  = std::max( puSize.width  >> spsNext.getSubPuMvpLog2Size(), 1u );
    iNumPartCol   = std::max( puSize.height >> spsNext.getSubPuMvpLog2Size(), 1u );
    iPUHeight     = iNumPartCol  == 1 ? puSize.height : 1 << spsNext.getSubPuMvpLog2Size();
    iPUWidth      = iNumPartLine == 1 ? puSize.width  : 1 << spsNext.getSubPuMvpLog2Size();
#endif
  }

  PredictionUnit subPu;

  subPu.cs        = pu.cs;
  subPu.cu        = pu.cu;
  subPu.mergeType = MRG_TYPE_DEFAULT_N;

  // join sub-pus containing the same motion
#if JVET_K0346
  bool verMC = puSize.height > puSize.width;
  int  fstStart = (!verMC ? puPos.y : puPos.x);
  int  secStart = (!verMC ? puPos.x : puPos.y);
  int  fstEnd = (!verMC ? puPos.y + puSize.height : puPos.x + puSize.width);
  int  secEnd = (!verMC ? puPos.x + puSize.width : puPos.y + puSize.height);
  int  fstStep = (!verMC ? puHeight : puWidth);
  int  secStep = (!verMC ? puWidth : puHeight);

  for (int fstDim = fstStart; fstDim < fstEnd; fstDim += fstStep)
  {
    for (int secDim = secStart; secDim < secEnd; secDim += secStep)
    {
      int x = !verMC ? secDim : fstDim;
      int y = !verMC ? fstDim : secDim;
      const MotionInfo &curMi = pu.getMotionInfo(Position{ x, y });

      int length = secStep;
      int later  = secDim + secStep;

      while (later < secEnd)
      {
        const MotionInfo &laterMi = !verMC ? pu.getMotionInfo(Position{ later, fstDim }) : pu.getMotionInfo(Position{ fstDim, later });
        if (laterMi == curMi)
        {
          length += secStep;
        }
        else
        {
          break;
        }
        later += secStep;
      }
      int dx = !verMC ? length : puWidth;
      int dy = !verMC ? puHeight : length;

      subPu.UnitArea::operator=(UnitArea(pu.chromaFormat, Area(x, y, dx, dy)));
      subPu = curMi;
      PelUnitBuf subPredBuf = predBuf.subBuf(UnitAreaRelative(pu, subPu));

      motionCompensation(subPu, subPredBuf, eRefPicList);
      secDim = later - secStep;
    }
  }
#else
  for( int y = puPos.y; y < puPos.y + puSize.height; y += iPUHeight )
  {
    for( int x = puPos.x; x < puPos.x + puSize.width; x += iPUWidth )
    {
      const MotionInfo &curMi = pu.getMotionInfo( Position{ x, y } );

      int dx = iPUWidth;
      int dy = iPUHeight;

      subPu.UnitArea::operator=( UnitArea( pu.chromaFormat, Area( x, y, dx, dy ) ) );
      subPu                   = curMi;
      PelUnitBuf subPredBuf   = predBuf.subBuf( UnitAreaRelative( pu, subPu ) );

      motionCompensation( subPu, subPredBuf, eRefPicList );
    }
  }
#endif
}
#endif

void InterPrediction::xPredInterUni(const PredictionUnit& pu, const RefPicList& eRefPicList, PelUnitBuf& pcYuvPred, const bool& bi )
{
  const SPS &sps = *pu.cs->sps;

  int iRefIdx = pu.refIdx[eRefPicList];
  Mv mv[3];

#if JVET_K_AFFINE
  if( pu.cu->affine )
  {
    CHECK( iRefIdx < 0, "iRefIdx incorrect." );

    const CMotionBuf &mb = pu.getMotionBuf();
    mv[0] = mb.at( 0,            0             ).mv[eRefPicList];
    mv[1] = mb.at( mb.width - 1, 0             ).mv[eRefPicList];
    mv[2] = mb.at( 0,            mb.height - 1 ).mv[eRefPicList];
#if !JVET_K_AFFINE_BUG_FIXES
    clipMv(mv[1], pu.cu->lumaPos(), sps);
    clipMv(mv[2], pu.cu->lumaPos(), sps);
#endif
  }
  else
#endif
  {
    mv[0] = pu.mv[eRefPicList];
  }
#if JVET_K_AFFINE && JVET_K_AFFINE_BUG_FIXES
  if ( !pu.cu->affine )
#endif
  clipMv(mv[0], pu.cu->lumaPos(), sps);


  for( uint32_t comp = COMPONENT_Y; comp < pcYuvPred.bufs.size() && comp <= m_maxCompIDToPred; comp++ )
  {
    const ComponentID compID = ComponentID( comp );

#if JVET_K_AFFINE
    if ( pu.cu->affine )
    {
      xPredAffineBlk( compID, pu, pu.cu->slice->getRefPic( eRefPicList, iRefIdx ), mv, pcYuvPred, bi, pu.cu->slice->clpRng( compID ) );
    }
    else
#endif
    {
      xPredInterBlk( compID, pu, pu.cu->slice->getRefPic( eRefPicList, iRefIdx ), mv[0], pcYuvPred, bi, pu.cu->slice->clpRng( compID )
                    );
    }
  }
}

void InterPrediction::xPredInterBi(PredictionUnit& pu, PelUnitBuf &pcYuvPred)
{
  const PPS   &pps   = *pu.cs->pps;
  const Slice &slice = *pu.cs->slice;


  for (uint32_t refList = 0; refList < NUM_REF_PIC_LIST_01; refList++)
  {
    if( pu.refIdx[refList] < 0)
    {
      continue;
    }

    RefPicList eRefPicList = (refList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);

    CHECK( pu.refIdx[refList] >= slice.getNumRefIdx( eRefPicList ), "Invalid reference index" );
    m_iRefListIdx = refList;

    PelUnitBuf pcMbBuf = ( pu.chromaFormat == CHROMA_400 ?
                           PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[refList][0], pcYuvPred.Y())) :
                           PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[refList][0], pcYuvPred.Y()), PelBuf(m_acYuvPred[refList][1], pcYuvPred.Cb()), PelBuf(m_acYuvPred[refList][2], pcYuvPred.Cr())) );

    if (pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0)
    {
      xPredInterUni ( pu, eRefPicList, pcMbBuf, true
                     );
    }
    else
    {
      if( ( (pps.getUseWP() && slice.getSliceType() == P_SLICE) || (pps.getWPBiPred() && slice.getSliceType() == B_SLICE) ) )
      {
        xPredInterUni ( pu, eRefPicList, pcMbBuf, true );
      }
      else
      {
        xPredInterUni ( pu, eRefPicList, pcMbBuf, false );
      }
    }
  }


  CPelUnitBuf srcPred0 = ( pu.chromaFormat == CHROMA_400 ?
                           CPelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[0][0], pcYuvPred.Y())) :
                           CPelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[0][0], pcYuvPred.Y()), PelBuf(m_acYuvPred[0][1], pcYuvPred.Cb()), PelBuf(m_acYuvPred[0][2], pcYuvPred.Cr())) );
  CPelUnitBuf srcPred1 = ( pu.chromaFormat == CHROMA_400 ?
                           CPelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[1][0], pcYuvPred.Y())) :
                           CPelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[1][0], pcYuvPred.Y()), PelBuf(m_acYuvPred[1][1], pcYuvPred.Cb()), PelBuf(m_acYuvPred[1][2], pcYuvPred.Cr())) );
  if( pps.getWPBiPred() && slice.getSliceType() == B_SLICE )
  {
    xWeightedPredictionBi( pu, srcPred0, srcPred1, pcYuvPred, m_maxCompIDToPred );
  }
  else if( pps.getUseWP() && slice.getSliceType() == P_SLICE )
  {
    xWeightedPredictionUni( pu, srcPred0, REF_PIC_LIST_0, pcYuvPred, -1, m_maxCompIDToPred );
  }
  else
  {
    xWeightedAverage( pu, srcPred0, srcPred1, pcYuvPred, slice.getSPS()->getBitDepths(), slice.clpRngs() );
  }
}


void InterPrediction::xPredInterBlk ( const ComponentID& compID, const PredictionUnit& pu, const Picture* refPic, const Mv& _mv, PelUnitBuf& dstPic, const bool& bi, const ClpRng& clpRng
                                    )
{
  JVET_J0090_SET_REF_PICTURE( refPic, compID );
#if JVET_K0346 || JVET_K_AFFINE
  const ChromaFormat  chFmt = pu.chromaFormat;
  const bool          rndRes = !bi;

  int iAddPrecShift = 0;

  if (_mv.highPrec)
  {
    CHECKD(!pu.cs->sps->getSpsNext().getUseHighPrecMv(), "Found a high-precision motion vector, but the high-precision MV extension is disabled!");

    iAddPrecShift = VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;
  }

  int shiftHor = 2 + iAddPrecShift + ::getComponentScaleX(compID, chFmt);
  int shiftVer = 2 + iAddPrecShift + ::getComponentScaleY(compID, chFmt);

  int xFrac = _mv.hor & ((1 << shiftHor) - 1);
  int yFrac = _mv.ver & ((1 << shiftVer) - 1);

  xFrac <<= VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE - iAddPrecShift;
  yFrac <<= VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE - iAddPrecShift;

  CHECKD(!pu.cs->sps->getSpsNext().getUseHighPrecMv() && ((xFrac & 3) != 0), "Invalid fraction");
  CHECKD(!pu.cs->sps->getSpsNext().getUseHighPrecMv() && ((yFrac & 3) != 0), "Invalid fraction");
#else
  const ChromaFormat  chFmt  = pu.chromaFormat;
  const bool          rndRes = !bi;

  int shiftHor = 2 + ::getComponentScaleX( compID, chFmt );
  int shiftVer = 2 + ::getComponentScaleY( compID, chFmt );
  
  int xFrac = _mv.hor & ( ( 1 << shiftHor ) - 1 );
  int yFrac = _mv.ver & ( ( 1 << shiftVer ) - 1 );
#endif

  PelBuf &dstBuf  = dstPic.bufs[compID];
  unsigned width  = dstBuf.width;
  unsigned height = dstBuf.height;

  CPelBuf refBuf;
  {
    Position offset = pu.blocks[compID].pos().offset( _mv.getHor() >> shiftHor, _mv.getVer() >> shiftVer );
    refBuf = refPic->getRecoBuf( CompArea( compID, chFmt, offset, pu.blocks[compID].size() ) );
  }


  if( yFrac == 0 )
  {
    m_if.filterHor(compID, (Pel*) refBuf.buf, refBuf.stride, dstBuf.buf, dstBuf.stride, width, height, xFrac, rndRes, chFmt, clpRng);
  }
  else if( xFrac == 0 )
  {
    m_if.filterVer(compID, (Pel*) refBuf.buf, refBuf.stride, dstBuf.buf, dstBuf.stride, width, height, yFrac, true, rndRes, chFmt, clpRng);
  }
  else
  {
    PelBuf tmpBuf = PelBuf(m_filteredBlockTmp[0][compID], pu.blocks[compID]);

    int vFilterSize = isLuma(compID) ? NTAPS_LUMA : NTAPS_CHROMA;
    m_if.filterHor(compID, (Pel*) refBuf.buf - ((vFilterSize >> 1) - 1) * refBuf.stride, refBuf.stride, tmpBuf.buf, tmpBuf.stride, width, height + vFilterSize - 1, xFrac, false,         chFmt, clpRng);
    JVET_J0090_SET_CACHE_ENABLE( false );
    m_if.filterVer(compID, (Pel*) tmpBuf.buf + ((vFilterSize >> 1) - 1) * tmpBuf.stride, tmpBuf.stride, dstBuf.buf, dstBuf.stride, width, height,                   yFrac, false, rndRes, chFmt, clpRng);
    JVET_J0090_SET_CACHE_ENABLE( true );
  }

}

#if JVET_K_AFFINE
void InterPrediction::xPredAffineBlk( const ComponentID& compID, const PredictionUnit& pu, const Picture* refPic, const Mv* _mv, PelUnitBuf& dstPic, const bool& bi, const ClpRng& clpRng )
{
#if JVET_K0337_AFFINE_6PARA
  if ( (pu.cu->affineType == AFFINEMODEL_6PARAM && _mv[0] == _mv[1] && _mv[0] == _mv[2])
    || (pu.cu->affineType == AFFINEMODEL_4PARAM && _mv[0] == _mv[1])
    )
#else
  if( _mv[0] == _mv[1] )
#endif
  {
#if JVET_K_AFFINE_BUG_FIXES
    Mv mvTemp = _mv[0];
    clipMv( mvTemp, pu.cu->lumaPos(), *pu.cs->sps );
    xPredInterBlk( compID, pu, refPic, mvTemp, dstPic, bi, clpRng );
#else
    xPredInterBlk( compID, pu, refPic, _mv[0], dstPic, bi, clpRng );
#endif
    return;
  }

  JVET_J0090_SET_REF_PICTURE( refPic, compID );
  const ChromaFormat chFmt = pu.chromaFormat;
  int iScaleX = ::getComponentScaleX( compID, chFmt );
  int iScaleY = ::getComponentScaleY( compID, chFmt );

  Mv mvLT =_mv[0];
  Mv mvRT =_mv[1];
  Mv mvLB =_mv[2];

  mvLT.setHighPrec();
  mvRT.setHighPrec();
  mvLB.setHighPrec();

  // get affine sub-block width and height
  const int width  = pu.Y().width;
  const int height = pu.Y().height;
#if JVET_K0184_AFFINE_4X4
  int blockWidth = AFFINE_MIN_BLOCK_SIZE;
  int blockHeight = AFFINE_MIN_BLOCK_SIZE;
#else
  int blockWidth   = width;
  int blockHeight  = height;
  int mvWx = std::max<int>( abs((mvRT - mvLT).getHor()), abs((mvRT - mvLT).getVer()) );
  int mvWy = std::max<int>( abs((mvLB - mvLT).getHor()), abs((mvLB - mvLT).getVer()) );

  int iMvPrecision = 4;
  iMvPrecision -= VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;

  if (mvWx)
  {
    blockWidth = std::max<int>( (int)( ( width >> iMvPrecision ) / mvWx ), 1 );
    while (width % blockWidth)
    {
      blockWidth--;
    }
    blockWidth = std::max<int>( AFFINE_MIN_BLOCK_SIZE, blockWidth );
  }
  if (mvWy)
  {
    blockHeight = std::max<int>( (int)( ( height >> iMvPrecision ) / mvWy ), 1 );
    while (height % blockHeight)
    {
      blockHeight--;
    }
    blockHeight = std::max<int>( AFFINE_MIN_BLOCK_SIZE, blockHeight );
  }
#endif

  blockWidth  >>= iScaleX;
  blockHeight >>= iScaleY;
  const int cxWidth  = width  >> iScaleX;
  const int cxHeight = height >> iScaleY;
  const int iHalfBW  = blockWidth  >> 1;
  const int iHalfBH  = blockHeight >> 1;

  const int iBit = MAX_CU_DEPTH;
  int iDMvHorX, iDMvHorY, iDMvVerX, iDMvVerY;
  iDMvHorX = (mvRT - mvLT).getHor() << (iBit - g_aucLog2[cxWidth]);
  iDMvHorY = (mvRT - mvLT).getVer() << (iBit - g_aucLog2[cxWidth]);
#if JVET_K0337_AFFINE_6PARA
  if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
  {
    iDMvVerX = (mvLB - mvLT).getHor() << (iBit - g_aucLog2[cxHeight]);
    iDMvVerY = (mvLB - mvLT).getVer() << (iBit - g_aucLog2[cxHeight]);
  }
  else
  {
    iDMvVerX = -iDMvHorY;
    iDMvVerY = iDMvHorX;
  }
#else
  iDMvVerX = -iDMvHorY;
  iDMvVerY = iDMvHorX;
#endif

  int iMvScaleHor = mvLT.getHor() << iBit;
  int iMvScaleVer = mvLT.getVer() << iBit;
  const SPS &sps    = *pu.cs->sps;
  const int iMvShift = 4;
  const int iOffset  = 8;
  const int iHorMax = ( sps.getPicWidthInLumaSamples()     + iOffset -      pu.Y().x - 1 ) << iMvShift;
  const int iHorMin = (      -(int)pu.cs->pcv->maxCUWidth  - iOffset - (int)pu.Y().x + 1 ) << iMvShift;
  const int iVerMax = ( sps.getPicHeightInLumaSamples()    + iOffset -      pu.Y().y - 1 ) << iMvShift;
  const int iVerMin = (      -(int)pu.cs->pcv->maxCUHeight - iOffset - (int)pu.Y().y + 1 ) << iMvShift;

  PelBuf tmpBuf = PelBuf(m_filteredBlockTmp[0][compID], pu.blocks[compID]);
  const int vFilterSize = isLuma(compID) ? NTAPS_LUMA : NTAPS_CHROMA;

  const int shift = iBit - 4 + VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE + 2;

  // get prediction block by block
  for ( int h = 0; h < cxHeight; h += blockHeight )
  {
    for ( int w = 0; w < cxWidth; w += blockWidth )
    {
      int iMvScaleTmpHor = iMvScaleHor + iDMvHorX * (iHalfBW + w) + iDMvVerX * (iHalfBH + h);
      int iMvScaleTmpVer = iMvScaleVer + iDMvHorY * (iHalfBW + w) + iDMvVerY * (iHalfBH + h);
#if JVET_K_AFFINE_BUG_FIXES
      roundAffineMv( iMvScaleTmpHor, iMvScaleTmpVer, shift );
#else
      iMvScaleTmpHor >>= shift;
      iMvScaleTmpVer >>= shift;
#endif

      // clip and scale
      iMvScaleTmpHor = std::min<int>( iHorMax, std::max<int>( iHorMin, iMvScaleTmpHor ) );
      iMvScaleTmpVer = std::min<int>( iVerMax, std::max<int>( iVerMin, iMvScaleTmpVer ) );

      // get the MV in high precision
      int xFrac, yFrac, xInt, yInt;

      if (!iScaleX)
      {
        xInt  = iMvScaleTmpHor >> 4;
        xFrac = iMvScaleTmpHor & 15;
      }
      else
      {
        xInt  = iMvScaleTmpHor >> 5;
        xFrac = iMvScaleTmpHor & 31;
      }
      if (!iScaleY)
      {
        yInt  = iMvScaleTmpVer >> 4;
        yFrac = iMvScaleTmpVer & 15;
      }
      else
      {
        yInt  = iMvScaleTmpVer >> 5;
        yFrac = iMvScaleTmpVer & 31;
      }

      const CPelBuf refBuf = refPic->getRecoBuf( CompArea( compID, chFmt, pu.blocks[compID].offset(xInt + w, yInt + h), pu.blocks[compID] ) );
      PelBuf &dstBuf = dstPic.bufs[compID];

      if ( yFrac == 0 )
      {
        m_if.filterHor( compID, (Pel*) refBuf.buf, refBuf.stride, dstBuf.buf + w + h * dstBuf.stride, dstBuf.stride, blockWidth, blockHeight, xFrac, !bi, chFmt, clpRng );
      }
      else if ( xFrac == 0 )
      {
        m_if.filterVer( compID, (Pel*) refBuf.buf, refBuf.stride, dstBuf.buf + w + h * dstBuf.stride, dstBuf.stride, blockWidth, blockHeight, yFrac, true, !bi, chFmt, clpRng );
      }
      else
      {
        m_if.filterHor( compID, (Pel*) refBuf.buf - ((vFilterSize>>1) -1)*refBuf.stride, refBuf.stride, tmpBuf.buf, tmpBuf.stride, blockWidth, blockHeight+vFilterSize-1, xFrac, false,      chFmt, clpRng);
        JVET_J0090_SET_CACHE_ENABLE( false );
        m_if.filterVer( compID, tmpBuf.buf + ((vFilterSize>>1) -1)*tmpBuf.stride, tmpBuf.stride, dstBuf.buf + w + h * dstBuf.stride, dstBuf.stride, blockWidth, blockHeight, yFrac, false, !bi, chFmt, clpRng);
        JVET_J0090_SET_CACHE_ENABLE( true );
      }
    }
  }
}
#endif

int getMSB( unsigned x )
{
  int msb = 0, bits = ( sizeof(int) << 3 ), y = 1;
  while( x > 1u )
  {
    bits >>= 1;
    y      = x >> bits;
    if( y )
    {
      x    = y;
      msb += bits;
    }
  }
  msb += y;
  return msb;
}


void InterPrediction::xWeightedAverage( const PredictionUnit& pu, const CPelUnitBuf& pcYuvSrc0, const CPelUnitBuf& pcYuvSrc1, PelUnitBuf& pcYuvDst, const BitDepths& clipBitDepths, const ClpRngs& clpRngs )
{
  const int iRefIdx0 = pu.refIdx[0];
  const int iRefIdx1 = pu.refIdx[1];

  if( iRefIdx0 >= 0 && iRefIdx1 >= 0 )
  {
    pcYuvDst.addAvg( pcYuvSrc0, pcYuvSrc1, clpRngs );
  }
  else if( iRefIdx0 >= 0 && iRefIdx1 < 0 )
  {
    pcYuvDst.copyClip( pcYuvSrc0, clpRngs );
  }
  else if( iRefIdx0 < 0 && iRefIdx1 >= 0 )
  {
    pcYuvDst.copyClip( pcYuvSrc1, clpRngs );
  }
}

void InterPrediction::motionCompensation( PredictionUnit &pu, PelUnitBuf &predBuf, const RefPicList &eRefPicList )
{
        CodingStructure &cs = *pu.cs;
  const PPS &pps            = *cs.pps;
  const SliceType sliceType =  cs.slice->getSliceType();

  if( eRefPicList != REF_PIC_LIST_X )
  {
    if( ( ( sliceType == P_SLICE && pps.getUseWP() ) || ( sliceType == B_SLICE && pps.getWPBiPred() ) ) )
    {
      xPredInterUni         ( pu,          eRefPicList, predBuf, true );
      xWeightedPredictionUni( pu, predBuf, eRefPicList, predBuf, -1, m_maxCompIDToPred );
    }
    else
    {
      xPredInterUni( pu, eRefPicList, predBuf, false );
    }
  }
  else
  {
#if JVET_K0346
    if( pu.mergeType != MRG_TYPE_DEFAULT_N )
    {
      xSubPuMC( pu, predBuf, eRefPicList );
    }
    else if( xCheckIdenticalMotion( pu ) )
#else
    if( xCheckIdenticalMotion( pu ) )
#endif
    {
      xPredInterUni( pu, REF_PIC_LIST_0, predBuf, false );
    }
    else
    {
      xPredInterBi( pu, predBuf );
    }
  }
  return;
}

void InterPrediction::motionCompensation( CodingUnit &cu, const RefPicList &eRefPicList )
{
  for( auto &pu : CU::traversePUs( cu ) )
  {
    PelUnitBuf predBuf = cu.cs->getPredBuf( pu );
    motionCompensation( pu, predBuf, eRefPicList );
  }
}

void InterPrediction::motionCompensation( PredictionUnit &pu, const RefPicList &eRefPicList /*= REF_PIC_LIST_X*/ )
{
  PelUnitBuf predBuf = pu.cs->getPredBuf( pu );
  motionCompensation( pu, predBuf, eRefPicList );
}




#if JVET_J0090_MEMORY_BANDWITH_MEASURE
void InterPrediction::cacheAssign( CacheModel *cache )
{
  m_cacheModel = cache;
  m_if.cacheAssign( cache );
  m_if.initInterpolationFilter( !cache->isCacheEnable() );
}
#endif

//! \}

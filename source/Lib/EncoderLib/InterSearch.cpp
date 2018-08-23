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

/** \file     EncSearch.cpp
 *  \brief    encoder inter search class
 */

#include "InterSearch.h"


#include "CommonLib/CommonDef.h"
#include "CommonLib/Rom.h"
#include "CommonLib/MotionInfo.h"
#include "CommonLib/Picture.h"
#include "CommonLib/UnitTools.h"
#include "CommonLib/dtrace_next.h"
#include "CommonLib/dtrace_buffer.h"

#include "EncModeCtrl.h"
#include "EncLib.h"

#include <math.h>
#include <limits>


 //! \ingroup EncoderLib
 //! \{

static const Mv s_acMvRefineH[9] =
{
  Mv(  0,  0 ), // 0
  Mv(  0, -1 ), // 1
  Mv(  0,  1 ), // 2
  Mv( -1,  0 ), // 3
  Mv(  1,  0 ), // 4
  Mv( -1, -1 ), // 5
  Mv(  1, -1 ), // 6
  Mv( -1,  1 ), // 7
  Mv(  1,  1 )  // 8
};

static const Mv s_acMvRefineQ[9] =
{
  Mv(  0,  0 ), // 0
  Mv(  0, -1 ), // 1
  Mv(  0,  1 ), // 2
  Mv( -1, -1 ), // 5
  Mv(  1, -1 ), // 6
  Mv( -1,  0 ), // 3
  Mv(  1,  0 ), // 4
  Mv( -1,  1 ), // 7
  Mv(  1,  1 )  // 8
};


InterSearch::InterSearch()
  : m_modeCtrl                    (nullptr)
  , m_pSplitCS                    (nullptr)
  , m_pFullCS                     (nullptr)
  , m_pcEncCfg                    (nullptr)
  , m_pcTrQuant                   (nullptr)
  , m_iSearchRange                (0)
  , m_bipredSearchRange           (0)
  , m_motionEstimationSearchMethod(MESEARCH_FULL)
  , m_CABACEstimator              (nullptr)
  , m_CtxCache                    (nullptr)
  , m_pTempPel                    (nullptr)
  , m_isInitialized               (false)
{
  for (int i=0; i<MAX_NUM_REF_LIST_ADAPT_SR; i++)
  {
    memset (m_aaiAdaptSR[i], 0, MAX_IDX_ADAPT_SR * sizeof (int));
  }
  for (int i=0; i<AMVP_MAX_NUM_CANDS+1; i++)
  {
    memset (m_auiMVPIdxCost[i], 0, (AMVP_MAX_NUM_CANDS+1) * sizeof (uint32_t) );
  }

  setWpScalingDistParam( -1, REF_PIC_LIST_X, nullptr );
}


void InterSearch::destroy()
{
  CHECK(!m_isInitialized, "Not initialized");
  if ( m_pTempPel )
  {
    delete [] m_pTempPel;
    m_pTempPel = NULL;
  }

  m_pSplitCS = m_pFullCS = nullptr;

  m_pSaveCS = nullptr;

  for(uint32_t i = 0; i < NUM_REF_PIC_LIST_01; i++)
  {
    m_tmpPredStorage[i].destroy();
  }
  m_tmpStorageLCU.destroy();
  m_tmpAffiStorage.destroy();

  if ( m_tmpAffiError != NULL )
  {
    delete[] m_tmpAffiError;
  }
  if ( m_tmpAffiDeri[0] != NULL )
  {
    delete[] m_tmpAffiDeri[0];
  }
  if ( m_tmpAffiDeri[1] != NULL )
  {
    delete[] m_tmpAffiDeri[1];
  }
  m_isInitialized = false;
}

void InterSearch::setTempBuffers( CodingStructure ****pSplitCS, CodingStructure ****pFullCS, CodingStructure **pSaveCS )
{
  m_pSplitCS = pSplitCS;
  m_pFullCS  = pFullCS;
  m_pSaveCS  = pSaveCS;
}

#if ENABLE_SPLIT_PARALLELISM
void InterSearch::copyState( const InterSearch& other )
{
  if( !m_pcEncCfg->getQTBT() )
  {
    memcpy( m_integerMv2Nx2N, other.m_integerMv2Nx2N, sizeof( m_integerMv2Nx2N ) );
  }

  memcpy( m_aaiAdaptSR, other.m_aaiAdaptSR, sizeof( m_aaiAdaptSR ) );
}
#endif

InterSearch::~InterSearch()
{
  if (m_isInitialized)
  {
    destroy();
  }
}

void InterSearch::init( EncCfg*        pcEncCfg,
                        TrQuant*       pcTrQuant,
                        int            iSearchRange,
                        int            bipredSearchRange,
                        MESearchMethod motionEstimationSearchMethod,
                        const uint32_t     maxCUWidth,
                        const uint32_t     maxCUHeight,
                        const uint32_t     maxTotalCUDepth,
                        RdCost*        pcRdCost,
                        CABACWriter*   CABACEstimator,
                        CtxCache*      ctxCache
)
{
  CHECK(m_isInitialized, "Already initialized");
  m_pcEncCfg                     = pcEncCfg;
  m_pcTrQuant                    = pcTrQuant;
  m_iSearchRange                 = iSearchRange;
  m_bipredSearchRange            = bipredSearchRange;
  m_motionEstimationSearchMethod = motionEstimationSearchMethod;
  m_CABACEstimator               = CABACEstimator;
  m_CtxCache                     = ctxCache;

  for( uint32_t iDir = 0; iDir < MAX_NUM_REF_LIST_ADAPT_SR; iDir++ )
  {
    for( uint32_t iRefIdx = 0; iRefIdx < MAX_IDX_ADAPT_SR; iRefIdx++ )
    {
      m_aaiAdaptSR[iDir][iRefIdx] = iSearchRange;
    }
  }

  // initialize motion cost
  for( int iNum = 0; iNum < AMVP_MAX_NUM_CANDS + 1; iNum++ )
  {
    for( int iIdx = 0; iIdx < AMVP_MAX_NUM_CANDS; iIdx++ )
    {
      if( iIdx < iNum )
      {
        m_auiMVPIdxCost[iIdx][iNum] = xGetMvpIdxBits( iIdx, iNum );
      }
      else
      {
#if DISTORTION_TYPE_BUGFIX
        m_auiMVPIdxCost[iIdx][iNum] = MAX_UINT;
#else
        m_auiMVPIdxCost[iIdx][iNum] = MAX_INT;
#endif
      }
    }
  }

  const ChromaFormat cform = pcEncCfg->getChromaFormatIdc();
  InterPrediction::init( pcRdCost, cform );

  for( uint32_t i = 0; i < NUM_REF_PIC_LIST_01; i++ )
  {
    m_tmpPredStorage[i].create( UnitArea( cform, Area( 0, 0, MAX_CU_SIZE, MAX_CU_SIZE ) ) );
  }
  m_tmpStorageLCU.create( UnitArea( cform, Area( 0, 0, MAX_CU_SIZE, MAX_CU_SIZE ) ) );
  m_tmpAffiStorage.create( UnitArea( cform, Area( 0, 0, MAX_CU_SIZE, MAX_CU_SIZE ) ) );
#if JVET_K0367_AFFINE_FIX_POINT
  m_tmpAffiError = new Pel[MAX_CU_SIZE * MAX_CU_SIZE];
  m_tmpAffiDeri[0] = new int[MAX_CU_SIZE * MAX_CU_SIZE];
  m_tmpAffiDeri[1] = new int[MAX_CU_SIZE * MAX_CU_SIZE];
#else
  m_tmpAffiError   = new int   [MAX_CU_SIZE * MAX_CU_SIZE];
  m_tmpAffiDeri[0] = new double[MAX_CU_SIZE * MAX_CU_SIZE];
  m_tmpAffiDeri[1] = new double[MAX_CU_SIZE * MAX_CU_SIZE];
#endif
  m_pTempPel = new Pel[maxCUWidth*maxCUHeight];

  m_isInitialized = true;
}


inline void InterSearch::xTZSearchHelp( IntTZSearchStruct& rcStruct, const int iSearchX, const int iSearchY, const uint8_t ucPointNr, const uint32_t uiDistance )
{
  Distortion  uiSad = 0;

//  CHECK(!( !( rcStruct.searchRange.left > iSearchX || rcStruct.searchRange.right < iSearchX || rcStruct.searchRange.top > iSearchY || rcStruct.searchRange.bottom < iSearchY )), "Unspecified error");

  const Pel* const  piRefSrch = rcStruct.piRefY + iSearchY * rcStruct.iRefStride + iSearchX;

  m_cDistParam.cur.buf = piRefSrch;

  if( 1 == rcStruct.subShiftMode )
  {
    // motion cost
#if JVET_K0357_AMVR
    Distortion uiBitCost = m_pcRdCost->getCostOfVectorWithPredictor( iSearchX, iSearchY, rcStruct.imvShift );
#else
    Distortion uiBitCost = m_pcRdCost->getCostOfVectorWithPredictor( iSearchX, iSearchY );
#endif

    // Skip search if bit cost is already larger than best SAD
    if (uiBitCost < rcStruct.uiBestSad)
    {
      Distortion uiTempSad = m_cDistParam.distFunc( m_cDistParam );

      if((uiTempSad + uiBitCost) < rcStruct.uiBestSad)
      {
        // it's not supposed that any member of DistParams is manipulated beside cur.buf
        int subShift = m_cDistParam.subShift;
        const Pel* pOrgCpy = m_cDistParam.org.buf;
        uiSad += uiTempSad >> m_cDistParam.subShift;

        while( m_cDistParam.subShift > 0 )
        {
          int isubShift           = m_cDistParam.subShift -1;
          m_cDistParam.org.buf = rcStruct.pcPatternKey->buf + (rcStruct.pcPatternKey->stride << isubShift);
          m_cDistParam.cur.buf = piRefSrch + (rcStruct.iRefStride << isubShift);
          uiTempSad            = m_cDistParam.distFunc( m_cDistParam );
          uiSad               += uiTempSad >> m_cDistParam.subShift;

          if(((uiSad << isubShift) + uiBitCost) > rcStruct.uiBestSad)
          {
            break;
          }

          m_cDistParam.subShift--;
        }

        if(m_cDistParam.subShift == 0)
        {
          uiSad += uiBitCost;

          if( uiSad < rcStruct.uiBestSad )
          {
            rcStruct.uiBestSad      = uiSad;
            rcStruct.iBestX         = iSearchX;
            rcStruct.iBestY         = iSearchY;
            rcStruct.uiBestDistance = uiDistance;
            rcStruct.uiBestRound    = 0;
            rcStruct.ucPointNr      = ucPointNr;
            m_cDistParam.maximumDistortionForEarlyExit = uiSad;
          }
        }

        // restore org ptr
        m_cDistParam.org.buf  = pOrgCpy;
        m_cDistParam.subShift = subShift;
      }
    }
  }
  else
  {
    uiSad = m_cDistParam.distFunc( m_cDistParam );

    // only add motion cost if uiSad is smaller than best. Otherwise pointless
    // to add motion cost.
    if( uiSad < rcStruct.uiBestSad )
    {
      // motion cost
#if JVET_K0357_AMVR
      uiSad += m_pcRdCost->getCostOfVectorWithPredictor( iSearchX, iSearchY, rcStruct.imvShift );
#else
      uiSad += m_pcRdCost->getCostOfVectorWithPredictor( iSearchX, iSearchY );
#endif

      if( uiSad < rcStruct.uiBestSad )
      {
        rcStruct.uiBestSad      = uiSad;
        rcStruct.iBestX         = iSearchX;
        rcStruct.iBestY         = iSearchY;
        rcStruct.uiBestDistance = uiDistance;
        rcStruct.uiBestRound    = 0;
        rcStruct.ucPointNr      = ucPointNr;
        m_cDistParam.maximumDistortionForEarlyExit = uiSad;
      }
    }
  }
}



inline void InterSearch::xTZ2PointSearch( IntTZSearchStruct& rcStruct )
{
  const SearchRange& sr = rcStruct.searchRange;

  static const int xOffset[2][9] = { {  0, -1, -1,  0, -1, +1, -1, -1, +1 }, {  0,  0, +1, +1, -1, +1,  0, +1,  0 } };
  static const int yOffset[2][9] = { {  0,  0, -1, -1, +1, -1,  0, +1,  0 }, {  0, -1, -1,  0, -1, +1, +1, +1, +1 } };

  // 2 point search,                   //   1 2 3
  // check only the 2 untested points  //   4 0 5
  // around the start point            //   6 7 8
  const int iX1 = rcStruct.iBestX + xOffset[0][rcStruct.ucPointNr];
  const int iX2 = rcStruct.iBestX + xOffset[1][rcStruct.ucPointNr];

  const int iY1 = rcStruct.iBestY + yOffset[0][rcStruct.ucPointNr];
  const int iY2 = rcStruct.iBestY + yOffset[1][rcStruct.ucPointNr];

  if( iX1 >= sr.left && iX1 <= sr.right && iY1 >= sr.top && iY1 <= sr.bottom )
  {
    xTZSearchHelp( rcStruct, iX1, iY1, 0, 2 );
  }

  if( iX2 >= sr.left && iX2 <= sr.right && iY2 >= sr.top && iY2 <= sr.bottom )
  {
    xTZSearchHelp( rcStruct, iX2, iY2, 0, 2 );
  }
}


inline void InterSearch::xTZ8PointSquareSearch( IntTZSearchStruct& rcStruct, const int iStartX, const int iStartY, const int iDist )
{
  const SearchRange& sr = rcStruct.searchRange;
  // 8 point search,                   //   1 2 3
  // search around the start point     //   4 0 5
  // with the required  distance       //   6 7 8
  CHECK( iDist == 0 , "Invalid distance");
  const int iTop        = iStartY - iDist;
  const int iBottom     = iStartY + iDist;
  const int iLeft       = iStartX - iDist;
  const int iRight      = iStartX + iDist;
  rcStruct.uiBestRound += 1;

  if ( iTop >= sr.top ) // check top
  {
    if ( iLeft >= sr.left ) // check top left
    {
      xTZSearchHelp( rcStruct, iLeft, iTop, 1, iDist );
    }
    // top middle
    xTZSearchHelp( rcStruct, iStartX, iTop, 2, iDist );

    if ( iRight <= sr.right ) // check top right
    {
      xTZSearchHelp( rcStruct, iRight, iTop, 3, iDist );
    }
  } // check top
  if ( iLeft >= sr.left ) // check middle left
  {
    xTZSearchHelp( rcStruct, iLeft, iStartY, 4, iDist );
  }
  if ( iRight <= sr.right ) // check middle right
  {
    xTZSearchHelp( rcStruct, iRight, iStartY, 5, iDist );
  }
  if ( iBottom <= sr.bottom ) // check bottom
  {
    if ( iLeft >= sr.left ) // check bottom left
    {
      xTZSearchHelp( rcStruct, iLeft, iBottom, 6, iDist );
    }
    // check bottom middle
    xTZSearchHelp( rcStruct, iStartX, iBottom, 7, iDist );

    if ( iRight <= sr.right ) // check bottom right
    {
      xTZSearchHelp( rcStruct, iRight, iBottom, 8, iDist );
    }
  } // check bottom
}




inline void InterSearch::xTZ8PointDiamondSearch( IntTZSearchStruct& rcStruct,
                                                 const int iStartX,
                                                 const int iStartY,
                                                 const int iDist,
                                                 const bool bCheckCornersAtDist1 )
{
  const SearchRange& sr = rcStruct.searchRange;
  // 8 point search,                   //   1 2 3
  // search around the start point     //   4 0 5
  // with the required  distance       //   6 7 8
  CHECK( iDist == 0, "Invalid distance" );
  const int iTop        = iStartY - iDist;
  const int iBottom     = iStartY + iDist;
  const int iLeft       = iStartX - iDist;
  const int iRight      = iStartX + iDist;
  rcStruct.uiBestRound += 1;

  if ( iDist == 1 )
  {
    if ( iTop >= sr.top ) // check top
    {
      if (bCheckCornersAtDist1)
      {
        if ( iLeft >= sr.left) // check top-left
        {
          xTZSearchHelp( rcStruct, iLeft, iTop, 1, iDist );
        }
        xTZSearchHelp( rcStruct, iStartX, iTop, 2, iDist );
        if ( iRight <= sr.right ) // check middle right
        {
          xTZSearchHelp( rcStruct, iRight, iTop, 3, iDist );
        }
      }
      else
      {
        xTZSearchHelp( rcStruct, iStartX, iTop, 2, iDist );
      }
    }
    if ( iLeft >= sr.left ) // check middle left
    {
      xTZSearchHelp( rcStruct, iLeft, iStartY, 4, iDist );
    }
    if ( iRight <= sr.right ) // check middle right
    {
      xTZSearchHelp( rcStruct, iRight, iStartY, 5, iDist );
    }
    if ( iBottom <= sr.bottom ) // check bottom
    {
      if (bCheckCornersAtDist1)
      {
        if ( iLeft >= sr.left) // check top-left
        {
          xTZSearchHelp( rcStruct, iLeft, iBottom, 6, iDist );
        }
        xTZSearchHelp( rcStruct, iStartX, iBottom, 7, iDist );
        if ( iRight <= sr.right ) // check middle right
        {
          xTZSearchHelp( rcStruct, iRight, iBottom, 8, iDist );
        }
      }
      else
      {
        xTZSearchHelp( rcStruct, iStartX, iBottom, 7, iDist );
      }
    }
  }
  else
  {
    if ( iDist <= 8 )
    {
      const int iTop_2      = iStartY - (iDist>>1);
      const int iBottom_2   = iStartY + (iDist>>1);
      const int iLeft_2     = iStartX - (iDist>>1);
      const int iRight_2    = iStartX + (iDist>>1);

      if (  iTop >= sr.top && iLeft >= sr.left &&
           iRight <= sr.right && iBottom <= sr.bottom ) // check border
      {
        xTZSearchHelp( rcStruct, iStartX,  iTop,      2, iDist    );
        xTZSearchHelp( rcStruct, iLeft_2,  iTop_2,    1, iDist>>1 );
        xTZSearchHelp( rcStruct, iRight_2, iTop_2,    3, iDist>>1 );
        xTZSearchHelp( rcStruct, iLeft,    iStartY,   4, iDist    );
        xTZSearchHelp( rcStruct, iRight,   iStartY,   5, iDist    );
        xTZSearchHelp( rcStruct, iLeft_2,  iBottom_2, 6, iDist>>1 );
        xTZSearchHelp( rcStruct, iRight_2, iBottom_2, 8, iDist>>1 );
        xTZSearchHelp( rcStruct, iStartX,  iBottom,   7, iDist    );
      }
      else // check border
      {
        if ( iTop >= sr.top ) // check top
        {
          xTZSearchHelp( rcStruct, iStartX, iTop, 2, iDist );
        }
        if ( iTop_2 >= sr.top ) // check half top
        {
          if ( iLeft_2 >= sr.left ) // check half left
          {
            xTZSearchHelp( rcStruct, iLeft_2, iTop_2, 1, (iDist>>1) );
          }
          if ( iRight_2 <= sr.right ) // check half right
          {
            xTZSearchHelp( rcStruct, iRight_2, iTop_2, 3, (iDist>>1) );
          }
        } // check half top
        if ( iLeft >= sr.left ) // check left
        {
          xTZSearchHelp( rcStruct, iLeft, iStartY, 4, iDist );
        }
        if ( iRight <= sr.right ) // check right
        {
          xTZSearchHelp( rcStruct, iRight, iStartY, 5, iDist );
        }
        if ( iBottom_2 <= sr.bottom ) // check half bottom
        {
          if ( iLeft_2 >= sr.left ) // check half left
          {
            xTZSearchHelp( rcStruct, iLeft_2, iBottom_2, 6, (iDist>>1) );
          }
          if ( iRight_2 <= sr.right ) // check half right
          {
            xTZSearchHelp( rcStruct, iRight_2, iBottom_2, 8, (iDist>>1) );
          }
        } // check half bottom
        if ( iBottom <= sr.bottom ) // check bottom
        {
          xTZSearchHelp( rcStruct, iStartX, iBottom, 7, iDist );
        }
      } // check border
    }
    else // iDist > 8
    {
      if ( iTop >= sr.top && iLeft >= sr.left &&
           iRight <= sr.right && iBottom <= sr.bottom ) // check border
      {
        xTZSearchHelp( rcStruct, iStartX, iTop,    0, iDist );
        xTZSearchHelp( rcStruct, iLeft,   iStartY, 0, iDist );
        xTZSearchHelp( rcStruct, iRight,  iStartY, 0, iDist );
        xTZSearchHelp( rcStruct, iStartX, iBottom, 0, iDist );
        for ( int index = 1; index < 4; index++ )
        {
          const int iPosYT = iTop    + ((iDist>>2) * index);
          const int iPosYB = iBottom - ((iDist>>2) * index);
          const int iPosXL = iStartX - ((iDist>>2) * index);
          const int iPosXR = iStartX + ((iDist>>2) * index);
          xTZSearchHelp( rcStruct, iPosXL, iPosYT, 0, iDist );
          xTZSearchHelp( rcStruct, iPosXR, iPosYT, 0, iDist );
          xTZSearchHelp( rcStruct, iPosXL, iPosYB, 0, iDist );
          xTZSearchHelp( rcStruct, iPosXR, iPosYB, 0, iDist );
        }
      }
      else // check border
      {
        if ( iTop >= sr.top ) // check top
        {
          xTZSearchHelp( rcStruct, iStartX, iTop, 0, iDist );
        }
        if ( iLeft >= sr.left ) // check left
        {
          xTZSearchHelp( rcStruct, iLeft, iStartY, 0, iDist );
        }
        if ( iRight <= sr.right ) // check right
        {
          xTZSearchHelp( rcStruct, iRight, iStartY, 0, iDist );
        }
        if ( iBottom <= sr.bottom ) // check bottom
        {
          xTZSearchHelp( rcStruct, iStartX, iBottom, 0, iDist );
        }
        for ( int index = 1; index < 4; index++ )
        {
          const int iPosYT = iTop    + ((iDist>>2) * index);
          const int iPosYB = iBottom - ((iDist>>2) * index);
          const int iPosXL = iStartX - ((iDist>>2) * index);
          const int iPosXR = iStartX + ((iDist>>2) * index);

          if ( iPosYT >= sr.top ) // check top
          {
            if ( iPosXL >= sr.left ) // check left
            {
              xTZSearchHelp( rcStruct, iPosXL, iPosYT, 0, iDist );
            }
            if ( iPosXR <= sr.right ) // check right
            {
              xTZSearchHelp( rcStruct, iPosXR, iPosYT, 0, iDist );
            }
          } // check top
          if ( iPosYB <= sr.bottom ) // check bottom
          {
            if ( iPosXL >= sr.left ) // check left
            {
              xTZSearchHelp( rcStruct, iPosXL, iPosYB, 0, iDist );
            }
            if ( iPosXR <= sr.right ) // check right
            {
              xTZSearchHelp( rcStruct, iPosXR, iPosYB, 0, iDist );
            }
          } // check bottom
        } // for ...
      } // check border
    } // iDist <= 8
  } // iDist == 1
}

Distortion InterSearch::xPatternRefinement( const CPelBuf* pcPatternKey,
                                            Mv baseRefMv,
                                            int iFrac, Mv& rcMvFrac,
                                            bool bAllowUseOfHadamard )
{
  Distortion  uiDist;
  Distortion  uiDistBest  = std::numeric_limits<Distortion>::max();
  uint32_t        uiDirecBest = 0;

  Pel*  piRefPos;
  int iRefStride = pcPatternKey->width + 1;
  m_pcRdCost->setDistParam( m_cDistParam, *pcPatternKey, m_filteredBlock[0][0][0], iRefStride, m_lumaClpRng.bd, COMPONENT_Y, 0, 1, m_pcEncCfg->getUseHADME() && bAllowUseOfHadamard );

  const Mv* pcMvRefine = (iFrac == 2 ? s_acMvRefineH : s_acMvRefineQ);
  for (uint32_t i = 0; i < 9; i++)
  {
    Mv cMvTest = pcMvRefine[i];
    cMvTest += baseRefMv;

    int horVal = cMvTest.getHor() * iFrac;
    int verVal = cMvTest.getVer() * iFrac;
    piRefPos = m_filteredBlock[verVal & 3][horVal & 3][0];

    if (horVal == 2 && (verVal & 1) == 0)
    {
      piRefPos += 1;
    }
    if ((horVal & 1) == 0 && verVal == 2)
    {
      piRefPos += iRefStride;
    }
    cMvTest = pcMvRefine[i];
    cMvTest += rcMvFrac;


    m_cDistParam.cur.buf   = piRefPos;
    uiDist = m_cDistParam.distFunc( m_cDistParam );
#if JVET_K0357_AMVR
    uiDist += m_pcRdCost->getCostOfVectorWithPredictor( cMvTest.getHor(), cMvTest.getVer(), 0 );
#else
    uiDist += m_pcRdCost->getCostOfVectorWithPredictor( cMvTest.getHor(), cMvTest.getVer() );
#endif

    if ( uiDist < uiDistBest )
    {
      uiDistBest  = uiDist;
      uiDirecBest = i;
      m_cDistParam.maximumDistortionForEarlyExit = uiDist;
    }
  }

  rcMvFrac = pcMvRefine[uiDirecBest];

  return uiDistBest;
}

Distortion InterSearch::xGetInterPredictionError( PredictionUnit& pu, PelUnitBuf& origBuf, const RefPicList &eRefPicList )
{
  PelUnitBuf predBuf = m_tmpStorageLCU.getBuf( UnitAreaRelative(*pu.cu, pu) );

  motionCompensation( pu, predBuf, eRefPicList );

  DistParam cDistParam;
  cDistParam.applyWeight = false;

  m_pcRdCost->setDistParam( cDistParam, origBuf.Y(), predBuf.Y(), pu.cs->sps->getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, m_pcEncCfg->getUseHADME() && !pu.cu->transQuantBypass );

  return (Distortion)cDistParam.distFunc( cDistParam );
}

//! estimation of best merge coding
void InterSearch::xMergeEstimation( PredictionUnit& pu, PelUnitBuf& origBuf, int iPUIdx, uint32_t& uiMergeIdx, Distortion& ruiCost, MergeCtx &mergeCtx )
{
  PartSize partSize = pu.cu->partSize;

  if ( pu.cs->pps->getLog2ParallelMergeLevelMinus2() && partSize != SIZE_2Nx2N && pu.cu->lumaSize().width <= 8 )
  {
    if ( iPUIdx == 0 )
    {
      UnitArea unitArea = pu;

      pu.UnitArea::operator=( *pu.cu );
      pu.cu->partSize = SIZE_2Nx2N;

      PU::getInterMergeCandidates( pu, mergeCtx );

      pu.UnitArea::operator=( unitArea );
      pu.cu->partSize = partSize;
    }
  }
  else
  {
    PU::getInterMergeCandidates( pu, mergeCtx );
  }

  PU::restrictBiPredMergeCands( pu, mergeCtx );

  ruiCost = std::numeric_limits<Distortion>::max();
  for( uint32_t uiMergeCand = 0; uiMergeCand < mergeCtx.numValidMergeCand; ++uiMergeCand )
  {
    mergeCtx.setMergeInfo( pu, uiMergeCand );

    PU::spanMotionInfo( pu, mergeCtx );

    Distortion uiCostCand = xGetInterPredictionError( pu, origBuf );
    uint32_t       uiBitsCand = uiMergeCand + 1;

    if( uiMergeCand == m_pcEncCfg->getMaxNumMergeCand() - 1 )
    {
      uiBitsCand--;
    }
    uiCostCand = uiCostCand + m_pcRdCost->getCost( uiBitsCand );
    if ( uiCostCand < ruiCost )
    {
      ruiCost    = uiCostCand;
      uiMergeIdx = uiMergeCand;
    }
  }
}


//! search of the best candidate for inter prediction
void InterSearch::predInterSearch(CodingUnit& cu, Partitioner& partitioner)
{
  CodingStructure& cs = *cu.cs;

  AMVPInfo     amvp[2];
  Mv           cMvSrchRngLT;
  Mv           cMvSrchRngRB;

  Mv           cMvZero;

  Mv           cMv[2];
  Mv           cMvBi[2];
  Mv           cMvTemp[2][33];
#if JVET_K_AFFINE
  Mv           cMvHevcTemp[2][33];
#endif
  int          iNumPredDir = cs.slice->isInterP() ? 1 : 2;

  Mv           cMvPred[2][33];

  Mv           cMvPredBi[2][33];
  int          aaiMvpIdxBi[2][33];

  int          aaiMvpIdx[2][33];
  int          aaiMvpNum[2][33];

  AMVPInfo     aacAMVPInfo[2][33];

  int          iRefIdx[2]={0,0}; //If un-initialized, may cause SEGV in bi-directional prediction iterative stage.
  int          iRefIdxBi[2];

  uint32_t         uiMbBits[3] = {1, 1, 0};

  uint32_t         uiLastMode = 0;
#if JVET_K_AFFINE
  uint32_t         uiLastModeTemp = 0;
#endif
  int          iRefStart, iRefEnd;

  int          bestBiPRefIdxL1 = 0;
  int          bestBiPMvpL1    = 0;
  Distortion   biPDistTemp     = std::numeric_limits<Distortion>::max();

  MergeCtx     mergeCtx;

  // Loop over Prediction Units
  CHECK(!cu.firstPU, "CU does not contain any PUs");
  uint32_t         puIdx = 0;
  auto &pu = *cu.firstPU;

  {
    // motion estimation only evaluates luma component
    m_maxCompIDToPred = MAX_NUM_COMPONENT;
//    m_maxCompIDToPred = COMPONENT_Y;

    CHECK(pu.cu != &cu, "PU is contained in another CU");

#if JVET_K0346
    if (cu.cs->sps->getSpsNext().getUseSubPuMvp())
    {
      Size bufSize = g_miScaling.scale(pu.lumaSize());
      mergeCtx.subPuMvpMiBuf = MotionBuf(m_SubPuMiBuf, bufSize);
    }
#endif

    PU::spanMotionInfo( pu );
#if JVET_K_AFFINE
    Distortion   uiHevcCost = std::numeric_limits<Distortion>::max();
    Distortion   uiAffineCost = std::numeric_limits<Distortion>::max();
#endif
    Distortion   uiCost[2] = { std::numeric_limits<Distortion>::max(), std::numeric_limits<Distortion>::max() };
    Distortion   uiCostBi  =   std::numeric_limits<Distortion>::max();
    Distortion   uiCostTemp;

    uint32_t         uiBits[3];
    uint32_t         uiBitsTemp;
    Distortion   bestBiPDist = std::numeric_limits<Distortion>::max();

    Distortion   uiCostTempL0[MAX_NUM_REF];
    for (int iNumRef=0; iNumRef < MAX_NUM_REF; iNumRef++)
    {
      uiCostTempL0[iNumRef] = std::numeric_limits<Distortion>::max();
    }
    uint32_t         uiBitsTempL0[MAX_NUM_REF];

    Mv           mvValidList1;
    int          refIdxValidList1 = 0;
    uint32_t         bitsValidList1   = MAX_UINT;
    Distortion   costValidList1   = std::numeric_limits<Distortion>::max();

    PelUnitBuf origBuf = pu.cs->getOrgBuf( pu );

    xGetBlkBits( cu.partSize, cs.slice->isInterP(), puIdx, uiLastMode, uiMbBits );

    m_pcRdCost->selectMotionLambda( cu.transQuantBypass );

#if !JVET_K0220_ENC_CTRL
    bool bFastSkipBi = false;
    if( auto slsCtrl = dynamic_cast< SaveLoadEncInfoCtrl* >( m_modeCtrl ) )
    {
      bFastSkipBi = ( LOAD_ENC_INFO == slsCtrl->getSaveLoadTag( pu ) && 3 != slsCtrl->getSaveLoadInterDir( pu ) );
    }

#endif
#if JVET_K_AFFINE
#if !JVET_K0220_ENC_CTRL
    bool bFastSkipAffine = false;
    if( pu.cs->sps->getSpsNext().getUseQTBT() && m_pcEncCfg->getUseSaveLoadEncInfo() )
    {
      SaveLoadEncInfoCtrl* modeCtrl = dynamic_cast<SaveLoadEncInfoCtrl*>( m_modeCtrl );
      bFastSkipAffine = modeCtrl && LOAD_ENC_INFO == modeCtrl->getSaveLoadTag( pu ) && !modeCtrl->getSaveLoadAffineFlag( pu );
    }
#endif
#endif
#if JVET_K0357_AMVR
    unsigned imvShift = pu.cu->imv << 1;
#endif
      //  Uni-directional prediction
      for ( int iRefList = 0; iRefList < iNumPredDir; iRefList++ )
      {
        RefPicList  eRefPicList = ( iRefList ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );

        for ( int iRefIdxTemp = 0; iRefIdxTemp < cs.slice->getNumRefIdx(eRefPicList); iRefIdxTemp++ )
        {
          uiBitsTemp = uiMbBits[iRefList];
          if ( cs.slice->getNumRefIdx(eRefPicList) > 1 )
          {
            uiBitsTemp += iRefIdxTemp+1;
            if ( iRefIdxTemp == cs.slice->getNumRefIdx(eRefPicList)-1 )
            {
              uiBitsTemp--;
            }
          }
          xEstimateMvPredAMVP( pu, origBuf, eRefPicList, iRefIdxTemp, cMvPred[iRefList][iRefIdxTemp], amvp[eRefPicList], false, &biPDistTemp);

          aaiMvpIdx[iRefList][iRefIdxTemp] = pu.mvpIdx[eRefPicList];
          aaiMvpNum[iRefList][iRefIdxTemp] = pu.mvpNum[eRefPicList];

          if(cs.slice->getMvdL1ZeroFlag() && iRefList==1 && biPDistTemp < bestBiPDist)
          {
            bestBiPDist = biPDistTemp;
            bestBiPMvpL1 = aaiMvpIdx[iRefList][iRefIdxTemp];
            bestBiPRefIdxL1 = iRefIdxTemp;
          }

          uiBitsTemp += m_auiMVPIdxCost[aaiMvpIdx[iRefList][iRefIdxTemp]][AMVP_MAX_NUM_CANDS];

          if ( m_pcEncCfg->getFastMEForGenBLowDelayEnabled() && iRefList == 1 )    // list 1
          {
            if ( cs.slice->getList1IdxToList0Idx( iRefIdxTemp ) >= 0 )
            {
              cMvTemp[1][iRefIdxTemp] = cMvTemp[0][cs.slice->getList1IdxToList0Idx( iRefIdxTemp )];
              uiCostTemp = uiCostTempL0[cs.slice->getList1IdxToList0Idx( iRefIdxTemp )];
              /*first subtract the bit-rate part of the cost of the other list*/
              uiCostTemp -= m_pcRdCost->getCost( uiBitsTempL0[cs.slice->getList1IdxToList0Idx( iRefIdxTemp )] );
              /*correct the bit-rate part of the current ref*/
              m_pcRdCost->setPredictor  ( cMvPred[iRefList][iRefIdxTemp] );
#if JVET_K0357_AMVR
              uiBitsTemp += m_pcRdCost->getBitsOfVectorWithPredictor( cMvTemp[1][iRefIdxTemp].getHor(), cMvTemp[1][iRefIdxTemp].getVer(), imvShift );
#else
              uiBitsTemp += m_pcRdCost->getBitsOfVectorWithPredictor( cMvTemp[1][iRefIdxTemp].getHor(), cMvTemp[1][iRefIdxTemp].getVer() );
#endif
              /*calculate the correct cost*/
              uiCostTemp += m_pcRdCost->getCost( uiBitsTemp );
            }
            else
            {
              xMotionEstimation( pu, origBuf, eRefPicList, cMvPred[iRefList][iRefIdxTemp], iRefIdxTemp, cMvTemp[iRefList][iRefIdxTemp], aaiMvpIdx[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp, amvp[eRefPicList] );
            }
          }
          else
          {
            xMotionEstimation( pu, origBuf, eRefPicList, cMvPred[iRefList][iRefIdxTemp], iRefIdxTemp, cMvTemp[iRefList][iRefIdxTemp], aaiMvpIdx[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp, amvp[eRefPicList] );
          }
          xCopyAMVPInfo( &amvp[eRefPicList], &aacAMVPInfo[iRefList][iRefIdxTemp]); // must always be done ( also when AMVP_MODE = AM_NONE )
#if JVET_K0357_AMVR
          xCheckBestMVP( eRefPicList, cMvTemp[iRefList][iRefIdxTemp], cMvPred[iRefList][iRefIdxTemp], aaiMvpIdx[iRefList][iRefIdxTemp], amvp[eRefPicList], uiBitsTemp, uiCostTemp, pu.cu->imv );
#else
          xCheckBestMVP( eRefPicList, cMvTemp[iRefList][iRefIdxTemp], cMvPred[iRefList][iRefIdxTemp], aaiMvpIdx[iRefList][iRefIdxTemp], amvp[eRefPicList], uiBitsTemp, uiCostTemp );
#endif

          if ( iRefList == 0 )
          {
            uiCostTempL0[iRefIdxTemp] = uiCostTemp;
            uiBitsTempL0[iRefIdxTemp] = uiBitsTemp;
          }
          if ( uiCostTemp < uiCost[iRefList] )
          {
            uiCost[iRefList] = uiCostTemp;
            uiBits[iRefList] = uiBitsTemp; // storing for bi-prediction

            // set motion
            cMv    [iRefList] = cMvTemp[iRefList][iRefIdxTemp];
            iRefIdx[iRefList] = iRefIdxTemp;
          }

          if ( iRefList == 1 && uiCostTemp < costValidList1 && cs.slice->getList1IdxToList0Idx( iRefIdxTemp ) < 0 )
          {
            costValidList1 = uiCostTemp;
            bitsValidList1 = uiBitsTemp;

            // set motion
            mvValidList1     = cMvTemp[iRefList][iRefIdxTemp];
            refIdxValidList1 = iRefIdxTemp;
          }
        }
      }

#if JVET_K0220_ENC_CTRL
      if (cu.Y().width > 8 && cu.Y().height > 8 && cu.partSize == SIZE_2Nx2N && cu.slice->getSPS()->getSpsNext().getUseAffine() 
#if JVET_K0357_AMVR
        && cu.imv == 0
#endif
        )
#else
      if (cu.Y().width > 8 && cu.Y().height > 8 && cu.partSize == SIZE_2Nx2N && cu.slice->getSPS()->getSpsNext().getUseAffine() 
#if JVET_K0357_AMVR
        && cu.imv == 0
#endif
        && !bFastSkipAffine)
#endif
      {
        ::memcpy( cMvHevcTemp, cMvTemp, sizeof( cMvTemp ) );
      }
#if JVET_K_AFFINE
#if JVET_K0220_ENC_CTRL
      if ( cu.Y().width > 8 && cu.Y().height > 8 && cu.partSize == SIZE_2Nx2N && cu.slice->getSPS()->getSpsNext().getUseAffine() )
#else
      if ( cu.Y().width > 8 && cu.Y().height > 8 && cu.partSize == SIZE_2Nx2N && cu.slice->getSPS()->getSpsNext().getUseAffine() && !bFastSkipAffine )
#endif
      {
        ::memcpy( cMvHevcTemp, cMvTemp, sizeof( cMvTemp ) );
      }
#endif
      //  Bi-predictive Motion estimation
#if JVET_K0220_ENC_CTRL
      if( ( cs.slice->isInterB() ) && ( PU::isBipredRestriction( pu ) == false ) )
#else
      if ( (cs.slice->isInterB()) && ( PU::isBipredRestriction(pu) == false)  && !bFastSkipBi )
#endif
      {
        cMvBi[0] = cMv[0];
        cMvBi[1] = cMv[1];
        iRefIdxBi[0] = iRefIdx[0];
        iRefIdxBi[1] = iRefIdx[1];

        ::memcpy( cMvPredBi,   cMvPred,   sizeof( cMvPred   ) );
        ::memcpy( aaiMvpIdxBi, aaiMvpIdx, sizeof( aaiMvpIdx ) );

        uint32_t uiMotBits[2];

        if(cs.slice->getMvdL1ZeroFlag())
        {
          xCopyAMVPInfo(&aacAMVPInfo[1][bestBiPRefIdxL1], &amvp[REF_PIC_LIST_1]);
          aaiMvpIdxBi[1][bestBiPRefIdxL1] = bestBiPMvpL1;
          cMvPredBi  [1][bestBiPRefIdxL1] = amvp[REF_PIC_LIST_1].mvCand[bestBiPMvpL1];

          cMvBi    [1] = cMvPredBi[1][bestBiPRefIdxL1];
          iRefIdxBi[1] = bestBiPRefIdxL1;
          pu.mv    [REF_PIC_LIST_1] = cMvBi[1];
          pu.refIdx[REF_PIC_LIST_1] = iRefIdxBi[1];
          pu.mvpIdx[REF_PIC_LIST_1] = bestBiPMvpL1;

          PelUnitBuf predBufTmp = m_tmpPredStorage[REF_PIC_LIST_1].getBuf( UnitAreaRelative(cu, pu) );
          motionCompensation( pu, predBufTmp, REF_PIC_LIST_1 );

          uiMotBits[0] = uiBits[0] - uiMbBits[0];
          uiMotBits[1] = uiMbBits[1];

          if ( cs.slice->getNumRefIdx(REF_PIC_LIST_1) > 1 )
          {
            uiMotBits[1] += bestBiPRefIdxL1 + 1;
            if ( bestBiPRefIdxL1 == cs.slice->getNumRefIdx(REF_PIC_LIST_1)-1 )
            {
              uiMotBits[1]--;
            }
          }

          uiMotBits[1] += m_auiMVPIdxCost[aaiMvpIdxBi[1][bestBiPRefIdxL1]][AMVP_MAX_NUM_CANDS];

          uiBits[2] = uiMbBits[2] + uiMotBits[0] + uiMotBits[1];

          cMvTemp[1][bestBiPRefIdxL1] = cMvBi[1];
        }
        else
        {
          uiMotBits[0] = uiBits[0] - uiMbBits[0];
          uiMotBits[1] = uiBits[1] - uiMbBits[1];
          uiBits[2] = uiMbBits[2] + uiMotBits[0] + uiMotBits[1];
        }

        // 4-times iteration (default)
        int iNumIter = 4;

        // fast encoder setting: only one iteration
        if ( m_pcEncCfg->getFastInterSearchMode()==FASTINTERSEARCH_MODE1 || m_pcEncCfg->getFastInterSearchMode()==FASTINTERSEARCH_MODE2 || cs.slice->getMvdL1ZeroFlag() )
        {
          iNumIter = 1;
        }

        for ( int iIter = 0; iIter < iNumIter; iIter++ )
        {
          int         iRefList    = iIter % 2;

          if ( m_pcEncCfg->getFastInterSearchMode()==FASTINTERSEARCH_MODE1 || m_pcEncCfg->getFastInterSearchMode()==FASTINTERSEARCH_MODE2 )
          {
            if( uiCost[0] <= uiCost[1] )
            {
              iRefList = 1;
            }
            else
            {
              iRefList = 0;
            }
          }
          else if ( iIter == 0 )
          {
            iRefList = 0;
          }
          if ( iIter == 0 && !cs.slice->getMvdL1ZeroFlag())
          {
            pu.mv    [1 - iRefList] = cMv    [1 - iRefList];
            pu.refIdx[1 - iRefList] = iRefIdx[1 - iRefList];

            PelUnitBuf predBufTmp = m_tmpPredStorage[1 - iRefList].getBuf( UnitAreaRelative(cu, pu) );
            motionCompensation( pu, predBufTmp, RefPicList(1 - iRefList) );
          }

          RefPicList  eRefPicList = ( iRefList ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );

          if(cs.slice->getMvdL1ZeroFlag())
          {
            iRefList = 0;
            eRefPicList = REF_PIC_LIST_0;
          }

          bool bChanged = false;

          iRefStart = 0;
          iRefEnd   = cs.slice->getNumRefIdx(eRefPicList)-1;

          for ( int iRefIdxTemp = iRefStart; iRefIdxTemp <= iRefEnd; iRefIdxTemp++ )
          {
            uiBitsTemp = uiMbBits[2] + uiMotBits[1-iRefList];
            if ( cs.slice->getNumRefIdx(eRefPicList) > 1 )
            {
              uiBitsTemp += iRefIdxTemp+1;
              if ( iRefIdxTemp == cs.slice->getNumRefIdx(eRefPicList)-1 )
              {
                uiBitsTemp--;
              }
            }
            uiBitsTemp += m_auiMVPIdxCost[aaiMvpIdxBi[iRefList][iRefIdxTemp]][AMVP_MAX_NUM_CANDS];

            // call ME
            xCopyAMVPInfo(&aacAMVPInfo[iRefList][iRefIdxTemp], &amvp[eRefPicList] );
            xMotionEstimation ( pu, origBuf, eRefPicList, cMvPredBi[iRefList][iRefIdxTemp], iRefIdxTemp, cMvTemp[iRefList][iRefIdxTemp], aaiMvpIdxBi[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp, amvp[eRefPicList], true );
#if JVET_K0357_AMVR
            xCheckBestMVP( eRefPicList, cMvTemp[iRefList][iRefIdxTemp], cMvPredBi[iRefList][iRefIdxTemp], aaiMvpIdxBi[iRefList][iRefIdxTemp], amvp[eRefPicList], uiBitsTemp, uiCostTemp, pu.cu->imv);
#else
            xCheckBestMVP( eRefPicList, cMvTemp[iRefList][iRefIdxTemp], cMvPredBi[iRefList][iRefIdxTemp], aaiMvpIdxBi[iRefList][iRefIdxTemp], amvp[eRefPicList], uiBitsTemp, uiCostTemp);
#endif
            if ( uiCostTemp < uiCostBi )
            {
              bChanged = true;

              cMvBi[iRefList]     = cMvTemp[iRefList][iRefIdxTemp];
              iRefIdxBi[iRefList] = iRefIdxTemp;

              uiCostBi            = uiCostTemp;
              uiMotBits[iRefList] = uiBitsTemp - uiMbBits[2] - uiMotBits[1-iRefList];
              uiBits[2]           = uiBitsTemp;

              if(iNumIter!=1)
              {
                //  Set motion
                pu.mv    [eRefPicList] = cMvBi    [iRefList];
                pu.refIdx[eRefPicList] = iRefIdxBi[iRefList];

                PelUnitBuf predBufTmp = m_tmpPredStorage[iRefList].getBuf( UnitAreaRelative(cu, pu) );
                motionCompensation( pu, predBufTmp, eRefPicList );
              }
            }
          } // for loop-iRefIdxTemp

          if ( !bChanged )
          {
            if ( uiCostBi <= uiCost[0] && uiCostBi <= uiCost[1] )
            {
              xCopyAMVPInfo(&aacAMVPInfo[0][iRefIdxBi[0]], &amvp[REF_PIC_LIST_0]);
#if JVET_K0357_AMVR
              xCheckBestMVP( REF_PIC_LIST_0, cMvBi[0], cMvPredBi[0][iRefIdxBi[0]], aaiMvpIdxBi[0][iRefIdxBi[0]], amvp[eRefPicList], uiBits[2], uiCostBi, pu.cu->imv);
#else
              xCheckBestMVP( REF_PIC_LIST_0, cMvBi[0], cMvPredBi[0][iRefIdxBi[0]], aaiMvpIdxBi[0][iRefIdxBi[0]], amvp[eRefPicList], uiBits[2], uiCostBi);
#endif
              if(!cs.slice->getMvdL1ZeroFlag())
              {
                xCopyAMVPInfo(&aacAMVPInfo[1][iRefIdxBi[1]], &amvp[REF_PIC_LIST_1]);
#if JVET_K0357_AMVR
                xCheckBestMVP( REF_PIC_LIST_1, cMvBi[1], cMvPredBi[1][iRefIdxBi[1]], aaiMvpIdxBi[1][iRefIdxBi[1]], amvp[eRefPicList], uiBits[2], uiCostBi, pu.cu->imv);
#else
                xCheckBestMVP( REF_PIC_LIST_1, cMvBi[1], cMvPredBi[1][iRefIdxBi[1]], aaiMvpIdxBi[1][iRefIdxBi[1]], amvp[eRefPicList], uiBits[2], uiCostBi);
#endif
              }
            }
            break;
          }
        } // for loop-iter
      } // if (B_SLICE)



      //  Clear Motion Field
    pu.mv    [REF_PIC_LIST_0] = Mv();
    pu.mv    [REF_PIC_LIST_1] = Mv();
    pu.mvd   [REF_PIC_LIST_0] = cMvZero;
    pu.mvd   [REF_PIC_LIST_1] = cMvZero;
    pu.refIdx[REF_PIC_LIST_0] = NOT_VALID;
    pu.refIdx[REF_PIC_LIST_1] = NOT_VALID;
    pu.mvpIdx[REF_PIC_LIST_0] = NOT_VALID;
    pu.mvpIdx[REF_PIC_LIST_1] = NOT_VALID;
    pu.mvpNum[REF_PIC_LIST_0] = NOT_VALID;
    pu.mvpNum[REF_PIC_LIST_1] = NOT_VALID;


    uint32_t uiMEBits = 0;

    // Set Motion Field

    cMv    [1] = mvValidList1;
    iRefIdx[1] = refIdxValidList1;
    uiBits [1] = bitsValidList1;
    uiCost [1] = costValidList1;

#if JVET_K_AFFINE
      uiLastModeTemp = uiLastMode;
#endif
      if ( uiCostBi <= uiCost[0] && uiCostBi <= uiCost[1])
      {
        uiLastMode = 2;
        pu.mv    [REF_PIC_LIST_0] = cMvBi[0];
        pu.mv    [REF_PIC_LIST_1] = cMvBi[1];
        pu.mvd   [REF_PIC_LIST_0] = cMvBi[0] - cMvPredBi[0][iRefIdxBi[0]];
        pu.mvd   [REF_PIC_LIST_1] = cMvBi[1] - cMvPredBi[1][iRefIdxBi[1]];
        pu.refIdx[REF_PIC_LIST_0] = iRefIdxBi[0];
        pu.refIdx[REF_PIC_LIST_1] = iRefIdxBi[1];
        pu.mvpIdx[REF_PIC_LIST_0] = aaiMvpIdxBi[0][iRefIdxBi[0]];
        pu.mvpIdx[REF_PIC_LIST_1] = aaiMvpIdxBi[1][iRefIdxBi[1]];
        pu.mvpNum[REF_PIC_LIST_0] = aaiMvpNum[0][iRefIdxBi[0]];
        pu.mvpNum[REF_PIC_LIST_1] = aaiMvpNum[1][iRefIdxBi[1]];
        pu.interDir = 3;

        uiMEBits = uiBits[2];
      }
      else if ( uiCost[0] <= uiCost[1] )
      {
        uiLastMode = 0;
        pu.mv    [REF_PIC_LIST_0] = cMv[0];
        pu.mvd   [REF_PIC_LIST_0] = cMv[0] - cMvPred[0][iRefIdx[0]];
        pu.refIdx[REF_PIC_LIST_0] = iRefIdx[0];
        pu.mvpIdx[REF_PIC_LIST_0] = aaiMvpIdx[0][iRefIdx[0]];
        pu.mvpNum[REF_PIC_LIST_0] = aaiMvpNum[0][iRefIdx[0]];
        pu.interDir = 1;

        uiMEBits = uiBits[0];
      }
      else
      {
        uiLastMode = 1;
        pu.mv    [REF_PIC_LIST_1] = cMv[1];
        pu.mvd   [REF_PIC_LIST_1] = cMv[1] - cMvPred[1][iRefIdx[1]];
        pu.refIdx[REF_PIC_LIST_1] = iRefIdx[1];
        pu.mvpIdx[REF_PIC_LIST_1] = aaiMvpIdx[1][iRefIdx[1]];
        pu.mvpNum[REF_PIC_LIST_1] = aaiMvpNum[1][iRefIdx[1]];
        pu.interDir = 2;

        uiMEBits = uiBits[1];
      }

    if ( cu.partSize != SIZE_2Nx2N )
    {
      uint32_t uiMRGIndex    = 0;

      // calculate ME cost
      Distortion uiMEError = xGetInterPredictionError( pu, origBuf );
      Distortion uiMECost = uiMEError + m_pcRdCost->getCost( uiMEBits );
      // save ME result.
      InterPredictionData savedPU = pu;

      // find Merge result
      Distortion uiMRGCost = std::numeric_limits<Distortion>::max();

      pu.initData();
      xMergeEstimation( pu, origBuf, puIdx, uiMRGIndex, uiMRGCost, mergeCtx );

      if( uiMRGCost < uiMECost )
      {
        // set Merge result
        mergeCtx.setMergeInfo( pu, uiMRGIndex );
      }
      else
      {
        pu = savedPU;
      }
#if JVET_K_AFFINE
      uiHevcCost = ( uiMRGCost < uiMECost ) ? uiMRGCost : uiMECost;
#endif
    }
#if JVET_K_AFFINE
    if( cu.cs->pcv->only2Nx2N || cu.partSize == SIZE_2Nx2N )
    {
      uiHevcCost = ( uiCostBi <= uiCost[0] && uiCostBi <= uiCost[1] ) ? uiCostBi : ( ( uiCost[0] <= uiCost[1] ) ? uiCost[0] : uiCost[1] );
    }
#endif
    CHECK( !( !cu.cs->pcv->only2Nx2N || cu.partSize == SIZE_2Nx2N ), "Unexpected part size for QTBT." );
#if JVET_K_AFFINE
#if JVET_K0220_ENC_CTRL
#if JVET_K0357_AMVR
    if (cu.Y().width > 8 && cu.Y().height > 8 && cu.partSize == SIZE_2Nx2N && cu.slice->getSPS()->getSpsNext().getUseAffine() && cu.imv == 0)
#else
    if (cu.Y().width > 8 && cu.Y().height > 8 && cu.partSize == SIZE_2Nx2N && cu.slice->getSPS()->getSpsNext().getUseAffine())
#endif
#else
#if JVET_K0357_AMVR
    if (cu.Y().width > 8 && cu.Y().height > 8 && cu.partSize == SIZE_2Nx2N && cu.slice->getSPS()->getSpsNext().getUseAffine() && cu.imv == 0 && !bFastSkipAffine)
#else
    if (cu.Y().width > 8 && cu.Y().height > 8 && cu.partSize == SIZE_2Nx2N && cu.slice->getSPS()->getSpsNext().getUseAffine() && !bFastSkipAffine)
#endif
#endif
    {
      // save normal hevc result
      uint32_t uiMRGIndex = pu.mergeIdx;
      bool bMergeFlag = pu.mergeFlag;
      uint32_t uiInterDir = pu.interDir;

      Mv cMvd[2];
      uint32_t uiMvpIdx[2], uiMvpNum[2];
      uiMvpIdx[0] = pu.mvpIdx[REF_PIC_LIST_0];
      uiMvpIdx[1] = pu.mvpIdx[REF_PIC_LIST_1];
      uiMvpNum[0] = pu.mvpNum[REF_PIC_LIST_0];
      uiMvpNum[1] = pu.mvpNum[REF_PIC_LIST_1];
      cMvd[0]     = pu.mvd[REF_PIC_LIST_0];
      cMvd[1]     = pu.mvd[REF_PIC_LIST_1];

      MvField cHevcMvField[2];
      cHevcMvField[0].setMvField( pu.mv[REF_PIC_LIST_0], pu.refIdx[REF_PIC_LIST_0] );
      cHevcMvField[1].setMvField( pu.mv[REF_PIC_LIST_1], pu.refIdx[REF_PIC_LIST_1] );

      // do affine ME & Merge
#if JVET_K0185_AFFINE_6PARA_ENC
      cu.affineType = AFFINEMODEL_4PARAM;
      Mv acMvAffine4Para[2][33][3];
      int refIdx4Para[2] = { -1, -1 };

#if JVET_K0220_ENC_CTRL
      xPredAffineInterSearch( pu, origBuf, puIdx, uiLastModeTemp, uiAffineCost, cMvHevcTemp, acMvAffine4Para, refIdx4Para );
#else
      xPredAffineInterSearch( pu, origBuf, puIdx, uiLastModeTemp, uiAffineCost, cMvHevcTemp, bFastSkipBi, acMvAffine4Para, refIdx4Para );
#endif
      if ( cu.slice->getSPS()->getSpsNext().getUseAffineType() )
      {
        if ( uiAffineCost < uiHevcCost * 1.05 ) ///< condition for 6 parameter affine ME
        {
          // save 4 parameter results
          Mv bestMv[2][3], bestMvd[2][3];
          int bestMvpIdx[2], bestMvpNum[2], bestRefIdx[2];
          uint8_t bestInterDir;

          bestInterDir = pu.interDir;
          bestRefIdx[0] = pu.refIdx[0];
          bestRefIdx[1] = pu.refIdx[1];
          bestMvpIdx[0] = pu.mvpIdx[0];
          bestMvpIdx[1] = pu.mvpIdx[1];
          bestMvpNum[0] = pu.mvpNum[0];
          bestMvpNum[1] = pu.mvpNum[1];

          const CMotionBuf &mb = pu.getMotionBuf();
          for ( int refList = 0; refList < 2; refList++ )
          {
            bestMv[refList][0] = mb.at( 0, 0 ).mv[refList];
            bestMv[refList][1] = mb.at( mb.width - 1, 0 ).mv[refList];
            bestMv[refList][2] = mb.at( 0, mb.height - 1 ).mv[refList];

            bestMvd[refList][0] = pu.mvdAffi[refList][0];
            bestMvd[refList][1] = pu.mvdAffi[refList][1];
            bestMvd[refList][2] = pu.mvdAffi[refList][2];
          }

          refIdx4Para[0] = bestRefIdx[0];
          refIdx4Para[1] = bestRefIdx[1];

          Distortion uiAffine6Cost = std::numeric_limits<Distortion>::max();
          cu.affineType = AFFINEMODEL_6PARAM;
#if JVET_K0220_ENC_CTRL
          xPredAffineInterSearch( pu, origBuf, puIdx, uiLastModeTemp, uiAffine6Cost, cMvHevcTemp, acMvAffine4Para, refIdx4Para );
#else
          xPredAffineInterSearch( pu, origBuf, puIdx, uiLastModeTemp, uiAffine6Cost, cMvHevcTemp, bFastSkipBi, acMvAffine4Para, refIdx4Para );
#endif

          // reset to 4 parameter affine inter mode
          if ( uiAffineCost <= uiAffine6Cost )
          {
            cu.affineType = AFFINEMODEL_4PARAM;
            pu.interDir = bestInterDir;
            pu.refIdx[0] = bestRefIdx[0];
            pu.refIdx[1] = bestRefIdx[1];
            pu.mvpIdx[0] = bestMvpIdx[0];
            pu.mvpIdx[1] = bestMvpIdx[1];
            pu.mvpNum[0] = bestMvpNum[0];
            pu.mvpNum[1] = bestMvpNum[1];

            for ( int verIdx = 0; verIdx < 3; verIdx++ )
            {
              pu.mvdAffi[REF_PIC_LIST_0][verIdx] = bestMvd[0][verIdx];
              pu.mvdAffi[REF_PIC_LIST_1][verIdx] = bestMvd[1][verIdx];
            }

            PU::setAllAffineMv( pu, bestMv[0][0], bestMv[0][1], bestMv[0][2], REF_PIC_LIST_0 );
            PU::setAllAffineMv( pu, bestMv[1][0], bestMv[1][1], bestMv[1][2], REF_PIC_LIST_1 );
          }
          else
          {
            uiAffineCost = uiAffine6Cost;
          }
        }

        uiAffineCost += m_pcRdCost->getCost( 1 ); // add one bit for affine_type
      }
#else
#if JVET_K0220_ENC_CTRL
      xPredAffineInterSearch( pu, origBuf, puIdx, uiLastModeTemp, uiAffineCost, cMvHevcTemp );
#else
      xPredAffineInterSearch( pu, origBuf, puIdx, uiLastModeTemp, uiAffineCost, cMvHevcTemp, bFastSkipBi );
#endif
#endif

      if ( uiHevcCost <= uiAffineCost )
      {
        // set hevc me result
        cu.affine = false;
        pu.mergeFlag = bMergeFlag;
        pu.mergeIdx = uiMRGIndex;
        pu.interDir = uiInterDir;
        pu.mv    [REF_PIC_LIST_0] = cHevcMvField[0].mv;
        pu.refIdx[REF_PIC_LIST_0] = cHevcMvField[0].refIdx;
        pu.mv    [REF_PIC_LIST_1] = cHevcMvField[1].mv;
        pu.refIdx[REF_PIC_LIST_1] = cHevcMvField[1].refIdx;
        pu.mvpIdx[REF_PIC_LIST_0] = uiMvpIdx[0];
        pu.mvpIdx[REF_PIC_LIST_1] = uiMvpIdx[1];
        pu.mvpNum[REF_PIC_LIST_0] = uiMvpNum[0];
        pu.mvpNum[REF_PIC_LIST_1] = uiMvpNum[1];
        pu.mvd[REF_PIC_LIST_0] = cMvd[0];
        pu.mvd[REF_PIC_LIST_1] = cMvd[1];
      }
      else
      {
        CHECK( !cu.affine, "Wrong." );
        uiLastMode = uiLastModeTemp;
      }
    }
#endif
    m_maxCompIDToPred = MAX_NUM_COMPONENT;

    {
      PU::spanMotionInfo( pu, mergeCtx );
    }

    //  MC
    PelUnitBuf predBuf = pu.cs->getPredBuf(pu);
    motionCompensation( pu, predBuf, REF_PIC_LIST_X );
    puIdx++;
  }

  setWpScalingDistParam( -1, REF_PIC_LIST_X, cu.cs->slice );

  return;
}




// AMVP
void InterSearch::xEstimateMvPredAMVP( PredictionUnit& pu, PelUnitBuf& origBuf, RefPicList eRefPicList, int iRefIdx, Mv& rcMvPred, AMVPInfo& rAMVPInfo, bool bFilled, Distortion* puiDistBiP )
{
  Mv         cBestMv;
  int        iBestIdx   = 0;
  Distortion uiBestCost = std::numeric_limits<Distortion>::max();
  int        i;

  AMVPInfo*  pcAMVPInfo = &rAMVPInfo;

  // Fill the MV Candidates
  if (!bFilled)
  {
    PU::fillMvpCand( pu, eRefPicList, iRefIdx, *pcAMVPInfo );
  }

  // initialize Mvp index & Mvp
  iBestIdx = 0;
  cBestMv  = pcAMVPInfo->mvCand[0];

  PelUnitBuf predBuf = m_tmpStorageLCU.getBuf( UnitAreaRelative(*pu.cu, pu) );

  //-- Check Minimum Cost.
  for( i = 0 ; i < pcAMVPInfo->numCand; i++)
  {
    Distortion uiTmpCost = xGetTemplateCost( pu, origBuf, predBuf, pcAMVPInfo->mvCand[i], i, AMVP_MAX_NUM_CANDS, eRefPicList, iRefIdx );
    if( uiBestCost > uiTmpCost )
    {
      uiBestCost     = uiTmpCost;
      cBestMv        = pcAMVPInfo->mvCand[i];
      iBestIdx       = i;
      (*puiDistBiP)  = uiTmpCost;
    }
  }

  // Setting Best MVP
  rcMvPred = cBestMv;
  pu.mvpIdx[eRefPicList] = iBestIdx;
  pu.mvpNum[eRefPicList] = pcAMVPInfo->numCand;

  return;
}

uint32_t InterSearch::xGetMvpIdxBits(int iIdx, int iNum)
{
  CHECK(iIdx < 0 || iNum < 0 || iIdx >= iNum, "Invalid parameters");

  if (iNum == 1)
  {
    return 0;
  }

  uint32_t uiLength = 1;
  int iTemp = iIdx;
  if ( iTemp == 0 )
  {
    return uiLength;
  }

  bool bCodeLast = ( iNum-1 > iTemp );

  uiLength += (iTemp-1);

  if( bCodeLast )
  {
    uiLength++;
  }

  return uiLength;
}

void InterSearch::xGetBlkBits( PartSize eCUMode, bool bPSlice, int iPartIdx, uint32_t uiLastMode, uint32_t uiBlkBit[3])
{
  if ( eCUMode == SIZE_2Nx2N )
  {
    uiBlkBit[0] = (! bPSlice) ? 3 : 1;
    uiBlkBit[1] = 3;
    uiBlkBit[2] = 5;
  }
  else
  {
    THROW("Wrong part size!");
  }
}

void InterSearch::xCopyAMVPInfo (AMVPInfo* pSrc, AMVPInfo* pDst)
{
  pDst->numCand = pSrc->numCand;
  for (int i = 0; i < pSrc->numCand; i++)
  {
    pDst->mvCand[i] = pSrc->mvCand[i];
  }
}

#if JVET_K0357_AMVR
void InterSearch::xCheckBestMVP ( RefPicList eRefPicList, Mv cMv, Mv& rcMvPred, int& riMVPIdx, AMVPInfo& amvpInfo, uint32_t& ruiBits, Distortion& ruiCost, const uint8_t imv )
#else
void InterSearch::xCheckBestMVP ( RefPicList eRefPicList, Mv cMv, Mv& rcMvPred, int& riMVPIdx, AMVPInfo& amvpInfo, uint32_t& ruiBits, Distortion& ruiCost )
#endif
{
#if JVET_K0357_AMVR
  if( imv > 0 )
  {
    return;
  }
  unsigned imvshift = imv << 1;
#endif

  AMVPInfo* pcAMVPInfo = &amvpInfo;

  CHECK(pcAMVPInfo->mvCand[riMVPIdx] != rcMvPred, "Invalid MV prediction candidate");

  if (pcAMVPInfo->numCand < 2)
  {
    return;
  }

  m_pcRdCost->setCostScale ( 0    );

  int iBestMVPIdx = riMVPIdx;

  m_pcRdCost->setPredictor( rcMvPred );
#if JVET_K0357_AMVR
  int iOrgMvBits = m_pcRdCost->getBitsOfVectorWithPredictor(cMv.getHor(), cMv.getVer(), imvshift);
#else
  int iOrgMvBits = m_pcRdCost->getBitsOfVectorWithPredictor(cMv.getHor(), cMv.getVer());
#endif
  iOrgMvBits += m_auiMVPIdxCost[riMVPIdx][AMVP_MAX_NUM_CANDS];
  int iBestMvBits = iOrgMvBits;

  for (int iMVPIdx = 0; iMVPIdx < pcAMVPInfo->numCand; iMVPIdx++)
  {
    if (iMVPIdx == riMVPIdx)
    {
      continue;
    }

    m_pcRdCost->setPredictor( pcAMVPInfo->mvCand[iMVPIdx] );
#if JVET_K0357_AMVR
    int iMvBits = m_pcRdCost->getBitsOfVectorWithPredictor(cMv.getHor(), cMv.getVer(), imvshift);
#else
    int iMvBits = m_pcRdCost->getBitsOfVectorWithPredictor(cMv.getHor(), cMv.getVer());
#endif
    iMvBits += m_auiMVPIdxCost[iMVPIdx][AMVP_MAX_NUM_CANDS];

    if (iMvBits < iBestMvBits)
    {
      iBestMvBits = iMvBits;
      iBestMVPIdx = iMVPIdx;
    }
  }

  if (iBestMVPIdx != riMVPIdx)  //if changed
  {
    rcMvPred = pcAMVPInfo->mvCand[iBestMVPIdx];

    riMVPIdx = iBestMVPIdx;
    uint32_t uiOrgBits = ruiBits;
    ruiBits = uiOrgBits - iOrgMvBits + iBestMvBits;
    ruiCost = (ruiCost - m_pcRdCost->getCost( uiOrgBits ))  + m_pcRdCost->getCost( ruiBits );
  }
}


Distortion InterSearch::xGetTemplateCost( const PredictionUnit& pu,
                                          PelUnitBuf& origBuf,
                                          PelUnitBuf& predBuf,
                                          Mv          cMvCand,
                                          int         iMVPIdx,
                                          int         iMVPNum,
                                          RefPicList  eRefPicList,
                                          int         iRefIdx
)
{
  Distortion uiCost = std::numeric_limits<Distortion>::max();

  const Picture* picRef = pu.cu->slice->getRefPic( eRefPicList, iRefIdx );

  clipMv( cMvCand, pu.cu->lumaPos(), *pu.cs->sps );


  // prediction pattern
  const bool bi = pu.cu->slice->testWeightPred() && pu.cu->slice->getSliceType()==P_SLICE;


  xPredInterBlk( COMPONENT_Y, pu, picRef, cMvCand, predBuf, bi, pu.cu->slice->clpRng( COMPONENT_Y )
                );


  if ( bi )
  {
    xWeightedPredictionUni( pu, predBuf, eRefPicList, predBuf, iRefIdx, m_maxCompIDToPred );
  }

  // calc distortion

  uiCost  = m_pcRdCost->getDistPart( origBuf.Y(), predBuf.Y(), pu.cs->sps->getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, DF_SAD );
  uiCost += m_pcRdCost->getCost( m_auiMVPIdxCost[iMVPIdx][iMVPNum] );

  return uiCost;
}

#if JVET_K_AFFINE
Distortion InterSearch::xGetAffineTemplateCost( PredictionUnit& pu, PelUnitBuf& origBuf, PelUnitBuf& predBuf, Mv acMvCand[3], int iMVPIdx, int iMVPNum, RefPicList eRefPicList, int iRefIdx )
{
  Distortion uiCost = std::numeric_limits<Distortion>::max();

  const Picture* picRef = pu.cu->slice->getRefPic( eRefPicList, iRefIdx );

  // prediction pattern
  const bool bi = pu.cu->slice->testWeightPred() && pu.cu->slice->getSliceType()==P_SLICE;
  xPredAffineBlk( COMPONENT_Y, pu, picRef, acMvCand, predBuf, bi, pu.cu->slice->clpRng( COMPONENT_Y ) );
  if( bi )
  {
    xWeightedPredictionUni( pu, predBuf, eRefPicList, predBuf, iRefIdx, m_maxCompIDToPred );
  }

  // calc distortion

  uiCost  = m_pcRdCost->getDistPart( origBuf.Y(), predBuf.Y(), pu.cs->sps->getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, DF_SAD );
  uiCost += m_pcRdCost->getCost( m_auiMVPIdxCost[iMVPIdx][iMVPNum] );
  DTRACE( g_trace_ctx, D_COMMON, " (%d) affineTemplateCost=%d\n", DTRACE_GET_COUNTER(g_trace_ctx,D_COMMON), uiCost );
  return uiCost;
}
#endif

void InterSearch::xMotionEstimation(PredictionUnit& pu, PelUnitBuf& origBuf, RefPicList eRefPicList, Mv& rcMvPred, int iRefIdxPred, Mv& rcMv, int& riMVPIdx, uint32_t& ruiBits, Distortion& ruiCost, const AMVPInfo& amvpInfo, bool bBi)
{
  Mv cMvHalf, cMvQter;

  CHECK(eRefPicList >= MAX_NUM_REF_LIST_ADAPT_SR || iRefIdxPred>=int(MAX_IDX_ADAPT_SR), "Invalid reference picture list");
  m_iSearchRange = m_aaiAdaptSR[eRefPicList][iRefIdxPred];

  int    iSrchRng   = (bBi ? m_bipredSearchRange : m_iSearchRange);
  double fWeight    = 1.0;

  PelUnitBuf  origBufTmp = m_tmpStorageLCU.getBuf( UnitAreaRelative(*pu.cu, pu) );
  PelUnitBuf* pBuf       = &origBuf;

  if(bBi) // Bi-predictive ME
  {
    // NOTE: Other buf contains predicted signal from another direction
    PelUnitBuf otherBuf = m_tmpPredStorage[1 - (int)eRefPicList].getBuf( UnitAreaRelative(*pu.cu, pu ));
    origBufTmp.copyFrom(origBuf);
    origBufTmp.removeHighFreq(otherBuf, m_pcEncCfg->getClipForBiPredMeEnabled(), pu.cu->slice->clpRngs() );

    pBuf = &origBufTmp;

    fWeight = 0.5;
  }
  m_cDistParam.isBiPred = bBi;

  //  Search key pattern initialization
  CPelBuf  tmpPattern   = pBuf->Y();
  CPelBuf* pcPatternKey = &tmpPattern;

  m_lumaClpRng = pu.cs->slice->clpRng( COMPONENT_Y );

  CPelBuf buf = pu.cu->slice->getRefPic(eRefPicList, iRefIdxPred)->getRecoBuf(pu.blocks[COMPONENT_Y]);

  IntTZSearchStruct cStruct;
  cStruct.pcPatternKey  = pcPatternKey;
  cStruct.iRefStride    = buf.stride;
  cStruct.piRefY        = buf.buf;
#if JVET_K0357_AMVR
  cStruct.imvShift      = pu.cu->imv << 1;
#endif
  auto blkCache = dynamic_cast<CacheBlkInfoCtrl*>( m_modeCtrl );

  bool bQTBTMV  = false;
  bool bQTBTMV2 = false;
  Mv cIntMv;
  if( !bBi )
  {
    bool bValid = blkCache && blkCache->getMv( pu, eRefPicList, iRefIdxPred, cIntMv );
    if( bValid )
    {
      bQTBTMV2 = true;
      cIntMv <<= 2;
    }
  }


  m_pcRdCost->setPredictor( rcMvPred );

  m_pcRdCost->setCostScale(2);

  {
    setWpScalingDistParam(iRefIdxPred, eRefPicList, pu.cu->slice);
  }

  //  Do integer search
  if( ( m_motionEstimationSearchMethod == MESEARCH_FULL ) || bBi || bQTBTMV )
  {
    if( !bQTBTMV )
    {
      xSetSearchRange( pu, ( bBi ? rcMv : rcMvPred ), iSrchRng, cStruct.searchRange );
    }
    cStruct.subShiftMode = m_pcEncCfg->getFastInterSearchMode() == FASTINTERSEARCH_MODE1 || m_pcEncCfg->getFastInterSearchMode() == FASTINTERSEARCH_MODE3 ? 2 : 0;
    xPatternSearch( cStruct, rcMv, ruiCost);
  }
  else if( bQTBTMV2 )
  {
    rcMv = cIntMv;

    cStruct.subShiftMode = ( !m_pcEncCfg->getRestrictMESampling() && m_pcEncCfg->getMotionEstimationSearchMethod() == MESEARCH_SELECTIVE ) ? 1 :
                            ( m_pcEncCfg->getFastInterSearchMode() == FASTINTERSEARCH_MODE1 || m_pcEncCfg->getFastInterSearchMode() == FASTINTERSEARCH_MODE3 ) ? 2 : 0;
    xTZSearch( pu, cStruct, rcMv, ruiCost, NULL, false, true );
  }
  else
  {
    cStruct.subShiftMode = ( !m_pcEncCfg->getRestrictMESampling() && m_pcEncCfg->getMotionEstimationSearchMethod() == MESEARCH_SELECTIVE ) ? 1 :
                            ( m_pcEncCfg->getFastInterSearchMode() == FASTINTERSEARCH_MODE1 || m_pcEncCfg->getFastInterSearchMode() == FASTINTERSEARCH_MODE3 ) ? 2 : 0;
    rcMv = rcMvPred;
    const Mv *pIntegerMv2Nx2NPred = 0;
    if( !pu.cs->pcv->only2Nx2N && ( pu.cu->partSize != SIZE_2Nx2N || pu.cu->qtDepth != 0 ) )
    {
      pIntegerMv2Nx2NPred = &( m_integerMv2Nx2N[eRefPicList][iRefIdxPred] );
    }
    xPatternSearchFast( pu, cStruct, rcMv, ruiCost, pIntegerMv2Nx2NPred );
    if( blkCache )
    {
      blkCache->setMv( pu.cs->area, eRefPicList, iRefIdxPred, rcMv );
    }
    else if( pu.cu->partSize == SIZE_2Nx2N )
    {
      m_integerMv2Nx2N[eRefPicList][iRefIdxPred] = rcMv;
    }
  }

  DTRACE( g_trace_ctx, D_ME, "%d %d %d :MECostFPel<L%d,%d>: %d,%d,%dx%d,%2d: %d", DTRACE_GET_COUNTER( g_trace_ctx, D_ME ), pu.cu->slice->getPOC(), 0, ( int ) eRefPicList, ( int ) bBi, pu.Y().x, pu.Y().y, pu.Y().width, pu.Y().height, pu.cu->partSize, ruiCost );
  // sub-pel refinement for sub-pel resolution
#if JVET_K0357_AMVR
  if( pu.cu->imv == 0 )
#endif
  {
    xPatternSearchFracDIF( pu, eRefPicList, iRefIdxPred, cStruct, rcMv, cMvHalf, cMvQter, ruiCost );
    m_pcRdCost->setCostScale( 0 );
    rcMv <<= 2;
    rcMv  += ( cMvHalf <<= 1 );
    rcMv  += cMvQter;
#if JVET_K0357_AMVR
    uint32_t uiMvBits = m_pcRdCost->getBitsOfVectorWithPredictor( rcMv.getHor(), rcMv.getVer(), cStruct.imvShift );
#else
    uint32_t uiMvBits = m_pcRdCost->getBitsOfVectorWithPredictor( rcMv.getHor(), rcMv.getVer() );
#endif
    ruiBits += uiMvBits;
    ruiCost = ( Distortion ) ( floor( fWeight * ( ( double ) ruiCost - ( double ) m_pcRdCost->getCost( uiMvBits ) ) ) + ( double ) m_pcRdCost->getCost( ruiBits ) );
  }
#if JVET_K0357_AMVR
  else // integer refinement for integer-pel and 4-pel resolution
  {
    xPatternSearchIntRefine( pu, cStruct, rcMv, rcMvPred, riMVPIdx, ruiBits, ruiCost, amvpInfo, fWeight);
  }
  DTRACE( g_trace_ctx, D_ME, "   MECost<L%d,%d>: %6d (%d)  MV:%d,%d\n", ( int ) eRefPicList, ( int ) bBi, ruiCost, ruiBits, rcMv.getHor() << ( pu.cs->sps->getSpsNext().getUseHighPrecMv() ? 2 : 0 ), rcMv.getVer() << ( pu.cs->sps->getSpsNext().getUseHighPrecMv() ? 2 : 0 ) );
#else
  DTRACE( g_trace_ctx, D_ME, "   MECost<L%d,%d>: %6d (%d)  MV:%d,%d\n", ( int ) eRefPicList, ( int ) bBi, ruiCost, ruiBits, rcMv.getHor(), rcMv.getVer() );
#endif
}



void InterSearch::xSetSearchRange ( const PredictionUnit& pu,
                                    const Mv& cMvPred,
                                    const int iSrchRng,
                                    SearchRange& sr )
{
#if JVET_K0346 || JVET_K_AFFINE
  const int iMvShift = cMvPred.highPrec ? 4 : 2;
#else
  const int iMvShift = 2;
#endif
  Mv cFPMvPred = cMvPred;
  clipMv( cFPMvPred, pu.cu->lumaPos(), *pu.cs->sps );

#if JVET_K0346 || JVET_K_AFFINE
  Mv mvTL( cFPMvPred.getHor() - ( iSrchRng << iMvShift ), cFPMvPred.getVer() - ( iSrchRng << iMvShift ), cFPMvPred.highPrec );
  Mv mvBR( cFPMvPred.getHor() + ( iSrchRng << iMvShift ), cFPMvPred.getVer() + ( iSrchRng << iMvShift ), cFPMvPred.highPrec );
#else
  Mv mvTL( cFPMvPred.getHor() - ( iSrchRng << iMvShift ), cFPMvPred.getVer() - ( iSrchRng << iMvShift ) );
  Mv mvBR( cFPMvPred.getHor() + ( iSrchRng << iMvShift ), cFPMvPred.getVer() + ( iSrchRng << iMvShift ) );
#endif

  clipMv( mvTL, pu.cu->lumaPos(), *pu.cs->sps );
  clipMv( mvBR, pu.cu->lumaPos(), *pu.cs->sps );

  mvTL.divideByPowerOf2( iMvShift );
  mvBR.divideByPowerOf2( iMvShift );

  sr.left   = mvTL.hor;
  sr.top    = mvTL.ver;
  sr.right  = mvBR.hor;
  sr.bottom = mvBR.ver;
}


void InterSearch::xPatternSearch( IntTZSearchStruct&    cStruct,
                                  Mv&            rcMv,
                                  Distortion&    ruiSAD )
{
  Distortion  uiSad;
  Distortion  uiSadBest = std::numeric_limits<Distortion>::max();
  int         iBestX = 0;
  int         iBestY = 0;

  //-- jclee for using the SAD function pointer
  m_pcRdCost->setDistParam( m_cDistParam, *cStruct.pcPatternKey, cStruct.piRefY, cStruct.iRefStride, m_lumaClpRng.bd, COMPONENT_Y, cStruct.subShiftMode );

  const SearchRange& sr = cStruct.searchRange;

  const Pel* piRef = cStruct.piRefY + (sr.top * cStruct.iRefStride);
  for ( int y = sr.top; y <= sr.bottom; y++ )
  {
    for ( int x = sr.left; x <= sr.right; x++ )
    {
      //  find min. distortion position
      m_cDistParam.cur.buf = piRef + x;

      uiSad = m_cDistParam.distFunc( m_cDistParam );

      // motion cost
#if JVET_K0357_AMVR
      uiSad += m_pcRdCost->getCostOfVectorWithPredictor( x, y, cStruct.imvShift );
#else
      uiSad += m_pcRdCost->getCostOfVectorWithPredictor( x, y );
#endif

      if ( uiSad < uiSadBest )
      {
        uiSadBest = uiSad;
        iBestX    = x;
        iBestY    = y;
        m_cDistParam.maximumDistortionForEarlyExit = uiSad;
      }
    }
    piRef += cStruct.iRefStride;
  }

#if JVET_K0346 || JVET_K_AFFINE
  CHECK( rcMv.highPrec, "Unexpected high precision MV." );
#endif
  rcMv.set( iBestX, iBestY );

  cStruct.uiBestSad = uiSadBest; // th for testing
#if JVET_K0357_AMVR
  ruiSAD = uiSadBest - m_pcRdCost->getCostOfVectorWithPredictor( iBestX, iBestY, cStruct.imvShift );
#else
  ruiSAD = uiSadBest - m_pcRdCost->getCostOfVectorWithPredictor( iBestX, iBestY );
#endif
  return;
}


void InterSearch::xPatternSearchFast( const PredictionUnit& pu,
                                      IntTZSearchStruct&    cStruct,
                                      Mv&                   rcMv,
                                      Distortion&           ruiSAD,
                                      const Mv* const       pIntegerMv2Nx2NPred )
{
  switch ( m_motionEstimationSearchMethod )
  {
  case MESEARCH_DIAMOND:
    xTZSearch         ( pu, cStruct, rcMv, ruiSAD, pIntegerMv2Nx2NPred, false );
    break;

  case MESEARCH_SELECTIVE:
    xTZSearchSelective( pu, cStruct, rcMv, ruiSAD, pIntegerMv2Nx2NPred );
    break;

  case MESEARCH_DIAMOND_ENHANCED:
    xTZSearch         ( pu, cStruct, rcMv, ruiSAD, pIntegerMv2Nx2NPred, true );
    break;

  case MESEARCH_FULL: // shouldn't get here.
  default:
    break;
  }
}


void InterSearch::xTZSearch( const PredictionUnit& pu,
                             IntTZSearchStruct&    cStruct,
                             Mv&                   rcMv,
                             Distortion&           ruiSAD,
                             const Mv* const       pIntegerMv2Nx2NPred,
                             const bool            bExtendedSettings,
                             const bool            bFastSettings)
{
  const bool bUseRasterInFastMode                    = true; //toggle this to further reduce runtime

  const bool bUseAdaptiveRaster                      = bExtendedSettings;
  const int  iRaster                                 = (bFastSettings && bUseRasterInFastMode) ? 8 : 5;
  const bool bTestZeroVector                         = true && !bFastSettings;
  const bool bTestZeroVectorStart                    = bExtendedSettings;
  const bool bTestZeroVectorStop                     = false;
  const bool bFirstSearchDiamond                     = true;  // 1 = xTZ8PointDiamondSearch   0 = xTZ8PointSquareSearch
  const bool bFirstCornersForDiamondDist1            = bExtendedSettings;
  const bool bFirstSearchStop                        = m_pcEncCfg->getFastMEAssumingSmootherMVEnabled();
  const uint32_t uiFirstSearchRounds                     = bFastSettings ? (bUseRasterInFastMode?3:2) : 3;     // first search stop X rounds after best match (must be >=1)
  const bool bEnableRasterSearch                     = bFastSettings ? bUseRasterInFastMode : true;
  const bool bAlwaysRasterSearch                     = bExtendedSettings;  // true: BETTER but factor 2 slower
  const bool bRasterRefinementEnable                 = false; // enable either raster refinement or star refinement
  const bool bRasterRefinementDiamond                = false; // 1 = xTZ8PointDiamondSearch   0 = xTZ8PointSquareSearch
  const bool bRasterRefinementCornersForDiamondDist1 = bExtendedSettings;
  const bool bStarRefinementEnable                   = true;  // enable either star refinement or raster refinement
  const bool bStarRefinementDiamond                  = true;  // 1 = xTZ8PointDiamondSearch   0 = xTZ8PointSquareSearch
  const bool bStarRefinementCornersForDiamondDist1   = bExtendedSettings;
  const bool bStarRefinementStop                     = false || bFastSettings;
  const uint32_t uiStarRefinementRounds                  = 2;  // star refinement stop X rounds after best match (must be >=1)
  const bool bNewZeroNeighbourhoodTest               = bExtendedSettings;

  int iSearchRange = m_iSearchRange;

  clipMv( rcMv, pu.cu->lumaPos(), *pu.cs->sps );
  rcMv.divideByPowerOf2(2);

  // init TZSearchStruct
#if DISTORTION_TYPE_BUGFIX
  cStruct.uiBestSad = std::numeric_limits<Distortion>::max();
#else
  cStruct.uiBestSad   = MAX_UINT;
#endif

  //
  m_cDistParam.maximumDistortionForEarlyExit = cStruct.uiBestSad;
  m_pcRdCost->setDistParam( m_cDistParam, *cStruct.pcPatternKey, cStruct.piRefY, cStruct.iRefStride, m_lumaClpRng.bd, COMPONENT_Y, cStruct.subShiftMode );

  // distortion


  // set rcMv (Median predictor) as start point and as best point
  xTZSearchHelp( cStruct, rcMv.getHor(), rcMv.getVer(), 0, 0 );

  // test whether zero Mv is better start point than Median predictor
  if ( bTestZeroVector )
  {
    if ((rcMv.getHor() != 0 || rcMv.getVer() != 0) &&
      (0 != cStruct.iBestX || 0 != cStruct.iBestY))
    {
      // only test 0-vector if not obviously previously tested.
      xTZSearchHelp( cStruct, 0, 0, 0, 0 );
    }
  }

  SearchRange& sr = cStruct.searchRange;

  if (pIntegerMv2Nx2NPred != 0)
  {
    Mv integerMv2Nx2NPred = *pIntegerMv2Nx2NPred;
    integerMv2Nx2NPred <<= 2;
    clipMv( integerMv2Nx2NPred, pu.cu->lumaPos(), *pu.cs->sps );
    integerMv2Nx2NPred.divideByPowerOf2(2);

    if ((rcMv != integerMv2Nx2NPred) &&
      (integerMv2Nx2NPred.getHor() != cStruct.iBestX || integerMv2Nx2NPred.getVer() != cStruct.iBestY))
    {
      // only test integerMv2Nx2NPred if not obviously previously tested.
      xTZSearchHelp( cStruct, integerMv2Nx2NPred.getHor(), integerMv2Nx2NPred.getVer(), 0, 0);
    }
  }
  {
    // set search range
    Mv currBestMv(cStruct.iBestX, cStruct.iBestY );
    currBestMv <<= 2;
    xSetSearchRange( pu, currBestMv, m_iSearchRange>>(bFastSettings?1:0), sr );
  }

  // start search
  int  iDist = 0;
  int  iStartX = cStruct.iBestX;
  int  iStartY = cStruct.iBestY;

  const bool bBestCandidateZero = (cStruct.iBestX == 0) && (cStruct.iBestY == 0);

  // first search around best position up to now.
  // The following works as a "subsampled/log" window search around the best candidate
  for ( iDist = 1; iDist <= iSearchRange; iDist*=2 )
  {
    if ( bFirstSearchDiamond == 1 )
    {
      xTZ8PointDiamondSearch ( cStruct, iStartX, iStartY, iDist, bFirstCornersForDiamondDist1 );
    }
    else
    {
      xTZ8PointSquareSearch  ( cStruct, iStartX, iStartY, iDist );
    }

    if ( bFirstSearchStop && ( cStruct.uiBestRound >= uiFirstSearchRounds ) ) // stop criterion
    {
      break;
    }
  }

  if (!bNewZeroNeighbourhoodTest)
  {
    // test whether zero Mv is a better start point than Median predictor
    if ( bTestZeroVectorStart && ((cStruct.iBestX != 0) || (cStruct.iBestY != 0)) )
    {
      xTZSearchHelp( cStruct, 0, 0, 0, 0 );
      if ( (cStruct.iBestX == 0) && (cStruct.iBestY == 0) )
      {
        // test its neighborhood
        for ( iDist = 1; iDist <= iSearchRange; iDist*=2 )
        {
          xTZ8PointDiamondSearch( cStruct, 0, 0, iDist, false );
          if ( bTestZeroVectorStop && (cStruct.uiBestRound > 0) ) // stop criterion
          {
            break;
          }
        }
      }
    }
  }
  else
  {
    // Test also zero neighbourhood but with half the range
    // It was reported that the original (above) search scheme using bTestZeroVectorStart did not
    // make sense since one would have already checked the zero candidate earlier
    // and thus the conditions for that test would have not been satisfied
    if (bTestZeroVectorStart == true && bBestCandidateZero != true)
    {
      for ( iDist = 1; iDist <= (iSearchRange >> 1); iDist*=2 )
      {
        xTZ8PointDiamondSearch( cStruct, 0, 0, iDist, false );
        if ( bTestZeroVectorStop && (cStruct.uiBestRound > 2) ) // stop criterion
        {
          break;
        }
      }
    }
  }

  // calculate only 2 missing points instead 8 points if cStruct.uiBestDistance == 1
  if ( cStruct.uiBestDistance == 1 )
  {
    cStruct.uiBestDistance = 0;
    xTZ2PointSearch( cStruct );
  }

  // raster search if distance is too big
  if (bUseAdaptiveRaster)
  {
    int iWindowSize     = iRaster;
    SearchRange localsr = sr;

    if (!(bEnableRasterSearch && ( ((int)(cStruct.uiBestDistance) >= iRaster))))
    {
      iWindowSize ++;
      localsr.left   /= 2;
      localsr.right  /= 2;
      localsr.top    /= 2;
      localsr.bottom /= 2;
    }
    cStruct.uiBestDistance = iWindowSize;
    for ( iStartY = localsr.top; iStartY <= localsr.bottom; iStartY += iWindowSize )
    {
      for ( iStartX = localsr.left; iStartX <= localsr.right; iStartX += iWindowSize )
      {
        xTZSearchHelp( cStruct, iStartX, iStartY, 0, iWindowSize );
      }
    }
  }
  else
  {
    if ( bEnableRasterSearch && ( ((int)(cStruct.uiBestDistance) >= iRaster) || bAlwaysRasterSearch ) )
    {
      cStruct.uiBestDistance = iRaster;
      for ( iStartY = sr.top; iStartY <= sr.bottom; iStartY += iRaster )
      {
        for ( iStartX = sr.left; iStartX <= sr.right; iStartX += iRaster )
        {
          xTZSearchHelp( cStruct, iStartX, iStartY, 0, iRaster );
        }
      }
    }
  }

  // raster refinement

  if ( bRasterRefinementEnable && cStruct.uiBestDistance > 0 )
  {
    while ( cStruct.uiBestDistance > 0 )
    {
      iStartX = cStruct.iBestX;
      iStartY = cStruct.iBestY;
      if ( cStruct.uiBestDistance > 1 )
      {
        iDist = cStruct.uiBestDistance >>= 1;
        if ( bRasterRefinementDiamond == 1 )
        {
          xTZ8PointDiamondSearch ( cStruct, iStartX, iStartY, iDist, bRasterRefinementCornersForDiamondDist1 );
        }
        else
        {
          xTZ8PointSquareSearch  ( cStruct, iStartX, iStartY, iDist );
        }
      }

      // calculate only 2 missing points instead 8 points if cStruct.uiBestDistance == 1
      if ( cStruct.uiBestDistance == 1 )
      {
        cStruct.uiBestDistance = 0;
        if ( cStruct.ucPointNr != 0 )
        {
          xTZ2PointSearch( cStruct );
        }
      }
    }
  }

  // star refinement
  if ( bStarRefinementEnable && cStruct.uiBestDistance > 0 )
  {
    while ( cStruct.uiBestDistance > 0 )
    {
      iStartX = cStruct.iBestX;
      iStartY = cStruct.iBestY;
      cStruct.uiBestDistance = 0;
      cStruct.ucPointNr = 0;
      for ( iDist = 1; iDist < iSearchRange + 1; iDist*=2 )
      {
        if ( bStarRefinementDiamond == 1 )
        {
          xTZ8PointDiamondSearch ( cStruct, iStartX, iStartY, iDist, bStarRefinementCornersForDiamondDist1 );
        }
        else
        {
          xTZ8PointSquareSearch  ( cStruct, iStartX, iStartY, iDist );
        }
        if ( bStarRefinementStop && (cStruct.uiBestRound >= uiStarRefinementRounds) ) // stop criterion
        {
          break;
        }
      }

      // calculate only 2 missing points instead 8 points if cStrukt.uiBestDistance == 1
      if ( cStruct.uiBestDistance == 1 )
      {
        cStruct.uiBestDistance = 0;
        if ( cStruct.ucPointNr != 0 )
        {
          xTZ2PointSearch( cStruct );
        }
      }
    }
  }

  // write out best match
#if JVET_K0346 || JVET_K_AFFINE
  CHECK( rcMv.highPrec, "Unexpected high precision MV." );
#endif
  rcMv.set( cStruct.iBestX, cStruct.iBestY );
#if JVET_K0357_AMVR
  ruiSAD = cStruct.uiBestSad - m_pcRdCost->getCostOfVectorWithPredictor( cStruct.iBestX, cStruct.iBestY, cStruct.imvShift );
#else
  ruiSAD = cStruct.uiBestSad - m_pcRdCost->getCostOfVectorWithPredictor( cStruct.iBestX, cStruct.iBestY );
#endif
}


void InterSearch::xTZSearchSelective( const PredictionUnit& pu,
                                      IntTZSearchStruct&    cStruct,
                                      Mv                    &rcMv,
                                      Distortion            &ruiSAD,
                                      const Mv* const       pIntegerMv2Nx2NPred )
{
  const bool bTestZeroVector          = true;
  const bool bEnableRasterSearch      = true;
  const bool bAlwaysRasterSearch      = false;  // 1: BETTER but factor 15x slower
  const bool bStarRefinementEnable    = true;   // enable either star refinement or raster refinement
  const bool bStarRefinementDiamond   = true;   // 1 = xTZ8PointDiamondSearch   0 = xTZ8PointSquareSearch
  const bool bStarRefinementStop      = false;
  const uint32_t uiStarRefinementRounds   = 2;  // star refinement stop X rounds after best match (must be >=1)
  const int  iSearchRange             = m_iSearchRange;
  const int  iSearchRangeInitial      = m_iSearchRange >> 2;
  const int  uiSearchStep             = 4;
  const int  iMVDistThresh            = 8;

  int   iStartX                 = 0;
  int   iStartY                 = 0;
  int   iDist                   = 0;

  clipMv( rcMv, pu.cu->lumaPos(), *pu.cs->sps );

  rcMv.divideByPowerOf2(2);

  // init TZSearchStruct
#if DISTORTION_TYPE_BUGFIX
  cStruct.uiBestSad = std::numeric_limits<Distortion>::max();
#else
  cStruct.uiBestSad = MAX_UINT;
#endif
  cStruct.iBestX = 0;
  cStruct.iBestY = 0;

  m_cDistParam.maximumDistortionForEarlyExit = cStruct.uiBestSad;
  m_pcRdCost->setDistParam( m_cDistParam, *cStruct.pcPatternKey, cStruct.piRefY, cStruct.iRefStride, m_lumaClpRng.bd, COMPONENT_Y, cStruct.subShiftMode );


  // set rcMv (Median predictor) as start point and as best point
  xTZSearchHelp( cStruct, rcMv.getHor(), rcMv.getVer(), 0, 0 );

  // test whether zero Mv is better start point than Median predictor
  if ( bTestZeroVector )
  {
    xTZSearchHelp( cStruct, 0, 0, 0, 0 );
  }

  SearchRange& sr = cStruct.searchRange;

  if ( pIntegerMv2Nx2NPred != 0 )
  {
    Mv integerMv2Nx2NPred = *pIntegerMv2Nx2NPred;
    integerMv2Nx2NPred <<= 2;
    clipMv( integerMv2Nx2NPred, pu.cu->lumaPos(), *pu.cs->sps );
    integerMv2Nx2NPred.divideByPowerOf2(2);

    xTZSearchHelp( cStruct, integerMv2Nx2NPred.getHor(), integerMv2Nx2NPred.getVer(), 0, 0);

  }
  {
    // set search range
    Mv currBestMv(cStruct.iBestX, cStruct.iBestY );
    currBestMv <<= 2;
    xSetSearchRange( pu, currBestMv, m_iSearchRange, sr );
  }

  // Initial search
  int iBestX = cStruct.iBestX;
  int iBestY = cStruct.iBestY;
  int iFirstSrchRngHorLeft    = ((iBestX - iSearchRangeInitial) > sr.left)   ? (iBestX - iSearchRangeInitial) : sr.left;
  int iFirstSrchRngVerTop     = ((iBestY - iSearchRangeInitial) > sr.top)    ? (iBestY - iSearchRangeInitial) : sr.top;
  int iFirstSrchRngHorRight   = ((iBestX + iSearchRangeInitial) < sr.right)  ? (iBestX + iSearchRangeInitial) : sr.right;
  int iFirstSrchRngVerBottom  = ((iBestY + iSearchRangeInitial) < sr.bottom) ? (iBestY + iSearchRangeInitial) : sr.bottom;

  for ( iStartY = iFirstSrchRngVerTop; iStartY <= iFirstSrchRngVerBottom; iStartY += uiSearchStep )
  {
    for ( iStartX = iFirstSrchRngHorLeft; iStartX <= iFirstSrchRngHorRight; iStartX += uiSearchStep )
    {
      xTZSearchHelp( cStruct, iStartX, iStartY, 0, 0 );
      xTZ8PointDiamondSearch ( cStruct, iStartX, iStartY, 1, false );
      xTZ8PointDiamondSearch ( cStruct, iStartX, iStartY, 2, false );
    }
  }

  int iMaxMVDistToPred = (abs(cStruct.iBestX - iBestX) > iMVDistThresh || abs(cStruct.iBestY - iBestY) > iMVDistThresh);

  //full search with early exit if MV is distant from predictors
  if ( bEnableRasterSearch && (iMaxMVDistToPred || bAlwaysRasterSearch) )
  {
    for ( iStartY = sr.top; iStartY <= sr.bottom; iStartY += 1 )
    {
      for ( iStartX = sr.left; iStartX <= sr.right; iStartX += 1 )
      {
        xTZSearchHelp( cStruct, iStartX, iStartY, 0, 1 );
      }
    }
  }
  //Smaller MV, refine around predictor
  else if ( bStarRefinementEnable && cStruct.uiBestDistance > 0 )
  {
    // start refinement
    while ( cStruct.uiBestDistance > 0 )
    {
      iStartX = cStruct.iBestX;
      iStartY = cStruct.iBestY;
      cStruct.uiBestDistance = 0;
      cStruct.ucPointNr = 0;
      for ( iDist = 1; iDist < iSearchRange + 1; iDist*=2 )
      {
        if ( bStarRefinementDiamond == 1 )
        {
          xTZ8PointDiamondSearch ( cStruct, iStartX, iStartY, iDist, false );
        }
        else
        {
          xTZ8PointSquareSearch  ( cStruct, iStartX, iStartY, iDist );
        }
        if ( bStarRefinementStop && (cStruct.uiBestRound >= uiStarRefinementRounds) ) // stop criterion
        {
          break;
        }
      }

      // calculate only 2 missing points instead 8 points if cStrukt.uiBestDistance == 1
      if ( cStruct.uiBestDistance == 1 )
      {
        cStruct.uiBestDistance = 0;
        if ( cStruct.ucPointNr != 0 )
        {
          xTZ2PointSearch( cStruct );
        }
      }
    }
  }

  // write out best match
#if JVET_K0346 || JVET_K_AFFINE
  CHECK( rcMv.highPrec, "Unexpected high precision MV." );
#endif
  rcMv.set( cStruct.iBestX, cStruct.iBestY );
#if JVET_K0357_AMVR
  ruiSAD = cStruct.uiBestSad - m_pcRdCost->getCostOfVectorWithPredictor( cStruct.iBestX, cStruct.iBestY, cStruct.imvShift );
#else
  ruiSAD = cStruct.uiBestSad - m_pcRdCost->getCostOfVectorWithPredictor( cStruct.iBestX, cStruct.iBestY );
#endif
}

#if JVET_K0357_AMVR
void InterSearch::xPatternSearchIntRefine(PredictionUnit& pu, IntTZSearchStruct&  cStruct, Mv& rcMv, Mv& rcMvPred, int& riMVPIdx, uint32_t& ruiBits, Distortion& ruiCost, const AMVPInfo& amvpInfo, double fWeight)
{

  CHECK( pu.cu->imv == 0,                       "xPatternSearchIntRefine(): IMV not used.");
  CHECK( amvpInfo.mvCand[riMVPIdx] != rcMvPred, "xPatternSearchIntRefine(): MvPred issue.");

  const SPS &sps = *pu.cs->sps;
  m_pcRdCost->setDistParam( m_cDistParam, *cStruct.pcPatternKey, cStruct.piRefY, cStruct.iRefStride, m_lumaClpRng.bd, COMPONENT_Y, 0, 1, m_pcEncCfg->getUseHADME() && !pu.cu->transQuantBypass );

  // input MV rcMV has integer resolution
  // -> shift it to QPEL
  rcMv <<= 2;
  // -> set MV scale for cost calculation to QPEL (0)
  m_pcRdCost->setCostScale ( 0 );

  Distortion  uiDist, uiSATD = 0;
  Distortion  uiBestDist  = std::numeric_limits<Distortion>::max();
  // subtract old MVP costs because costs for all newly tested MVPs are added in here
  ruiBits -= m_auiMVPIdxCost[riMVPIdx][AMVP_MAX_NUM_CANDS];

  Mv cBestMv = rcMv;
  Mv cBaseMvd[2];
  int iBestBits = 0;
  int iBestMVPIdx = riMVPIdx;
  int testPos[9][2] = { { 0, 0}, { -1, -1},{ -1, 0},{ -1, 1},{ 0, -1},{ 0, 1},{ 1, -1},{ 1, 0},{ 1, 1} };


  cBaseMvd[0] = (rcMv - amvpInfo.mvCand[0]);
  cBaseMvd[1] = (rcMv - amvpInfo.mvCand[1]);
  CHECK( (cBaseMvd[0].getHor() & 0x03) != 0 || (cBaseMvd[0].getVer() & 0x03) != 0 , "xPatternSearchIntRefine(): AMVP cand 0 Mvd issue.");
  CHECK( (cBaseMvd[1].getHor() & 0x03) != 0 || (cBaseMvd[1].getVer() & 0x03) != 0 , "xPatternSearchIntRefine(): AMVP cand 1 Mvd issue.");

  roundMV(cBaseMvd[0], cStruct.imvShift);
  roundMV(cBaseMvd[1], cStruct.imvShift);

  int mvOffset = 1 << cStruct.imvShift;

  // test best integer position and all 8 neighboring positions
  for (int pos = 0; pos < 9; pos ++)
  {
    Mv cTestMv[2];
    // test both AMVP candidates for each position
    for (int iMVPIdx = 0; iMVPIdx < amvpInfo.numCand; iMVPIdx++)
    {
      cTestMv[iMVPIdx].set(testPos[pos][0]*mvOffset, testPos[pos][1]*mvOffset);
      cTestMv[iMVPIdx] += cBaseMvd[iMVPIdx];
      cTestMv[iMVPIdx] += amvpInfo.mvCand[iMVPIdx];

      if ( iMVPIdx == 0 || cTestMv[0] != cTestMv[1])
      {
        Mv cTempMV = cTestMv[iMVPIdx];
        clipMv(cTempMV, pu.cu->lumaPos(), sps);

        m_cDistParam.cur.buf = cStruct.piRefY  + cStruct.iRefStride * (cTempMV.getVer() >>  2) + (cTempMV.getHor() >> 2);
        uiDist = uiSATD = (Distortion) (m_cDistParam.distFunc( m_cDistParam ) * fWeight);
      }
      else
      {
        uiDist = uiSATD;
      }

      int iMvBits = m_auiMVPIdxCost[iMVPIdx][AMVP_MAX_NUM_CANDS];
      m_pcRdCost->setPredictor( amvpInfo.mvCand[iMVPIdx] );
      iMvBits += m_pcRdCost->getBitsOfVectorWithPredictor( cTestMv[iMVPIdx].getHor(), cTestMv[iMVPIdx].getVer(), cStruct.imvShift );
      uiDist += m_pcRdCost->getCostOfVectorWithPredictor( cTestMv[iMVPIdx].getHor(), cTestMv[iMVPIdx].getVer(), cStruct.imvShift );

      if (uiDist < uiBestDist)
      {
        uiBestDist = uiDist;
        cBestMv = cTestMv[iMVPIdx];
        iBestMVPIdx = iMVPIdx;
        iBestBits = iMvBits;
      }
    }
  }

  rcMv = cBestMv;
  rcMvPred = amvpInfo.mvCand[iBestMVPIdx];
  riMVPIdx = iBestMVPIdx;
  m_pcRdCost->setPredictor( rcMvPred );

  ruiBits += iBestBits;
  // taken from JEM 5.0
  // verify since it makes no sence to subtract Lamda*(Rmvd+Rmvpidx) from D+Lamda(Rmvd)
  // this would take the rate for the MVP idx out of the cost calculation
  // however this rate is always 1 so impact is small
  ruiCost = uiBestDist - m_pcRdCost->getCost(iBestBits) + m_pcRdCost->getCost(ruiBits);
  // taken from JEM 5.0
  // verify since it makes no sense to add rate for MVDs twicce
  ruiBits += m_pcRdCost->getBitsOfVectorWithPredictor(rcMv.getHor(), rcMv.getVer(), cStruct.imvShift);

  return;
}
#endif

void InterSearch::xPatternSearchFracDIF(
  const PredictionUnit& pu,
  RefPicList            eRefPicList,
  int                   iRefIdx,
  IntTZSearchStruct&    cStruct,
  const Mv&             rcMvInt,
  Mv&                   rcMvHalf,
  Mv&                   rcMvQter,
  Distortion&           ruiCost
)
{
  const bool bIsLosslessCoded = pu.cu->transQuantBypass;

  //  Reference pattern initialization (integer scale)
  int         iOffset    = rcMvInt.getHor() + rcMvInt.getVer() * cStruct.iRefStride;
  CPelBuf cPatternRoi(cStruct.piRefY + iOffset, cStruct.iRefStride, *cStruct.pcPatternKey);


#if JVET_K0357_AMVR
  if( cStruct.imvShift )
  {
    m_pcRdCost->setDistParam( m_cDistParam, *cStruct.pcPatternKey, cStruct.piRefY + iOffset, cStruct.iRefStride, m_lumaClpRng.bd, COMPONENT_Y, 0, 1, m_pcEncCfg->getUseHADME() && !bIsLosslessCoded );
    ruiCost = m_cDistParam.distFunc( m_cDistParam );
    ruiCost += m_pcRdCost->getCostOfVectorWithPredictor( rcMvInt.getHor(), rcMvInt.getVer(), cStruct.imvShift );
    return;
  }
#endif

  //  Half-pel refinement
  m_pcRdCost->setCostScale(1);
  xExtDIFUpSamplingH ( &cPatternRoi );

  rcMvHalf = rcMvInt;   rcMvHalf <<= 1;    // for mv-cost
  Mv baseRefMv(0, 0);
  ruiCost = xPatternRefinement(cStruct.pcPatternKey, baseRefMv, 2, rcMvHalf, !bIsLosslessCoded);

  //  quarter-pel refinement
  m_pcRdCost->setCostScale( 0 );
  xExtDIFUpSamplingQ ( &cPatternRoi, rcMvHalf );
  baseRefMv = rcMvHalf;
  baseRefMv <<= 1;

  rcMvQter = rcMvInt;    rcMvQter <<= 1;    // for mv-cost
  rcMvQter += rcMvHalf;  rcMvQter <<= 1;
  ruiCost = xPatternRefinement( cStruct.pcPatternKey, baseRefMv, 1, rcMvQter, !bIsLosslessCoded );
}

#if JVET_K_AFFINE

void InterSearch::xPredAffineInterSearch( PredictionUnit&       pu,
                                          PelUnitBuf&           origBuf,
                                          int                   puIdx,
                                          uint32_t&                 lastMode,
                                          Distortion&           affineCost,
#if JVET_K0220_ENC_CTRL
                                          Mv                    hevcMv[2][33]
#else
                                          Mv                    hevcMv[2][33],
                                          bool                  bFastSkipBi
#endif
#if JVET_K0185_AFFINE_6PARA_ENC
                                        , Mv                    mvAffine4Para[2][33][3]
                                        , int                   refIdx4Para[2]
#endif
                                         )
{
  const Slice &slice = *pu.cu->slice;

  affineCost = std::numeric_limits<Distortion>::max();

  Mv        cMvZero;
  Mv        aacMv[2][3];
  Mv        cMvBi[2][3];
  Mv        cMvTemp[2][33][3];

  int       iNumPredDir = slice.isInterP() ? 1 : 2;

  int mvNum = 2;
#if JVET_K0185_AFFINE_6PARA_ENC
  mvNum = pu.cu->affineType ? 3 : 2;
#endif

  // Mvp
  Mv        cMvPred[2][33][3];
  Mv        cMvPredBi[2][33][3];
  int       aaiMvpIdxBi[2][33];
  int       aaiMvpIdx[2][33];
  int       aaiMvpNum[2][33];

  AffineAMVPInfo aacAffineAMVPInfo[2][33];
  AffineAMVPInfo affiAMVPInfoTemp[2];

  int           iRefIdx[2]={0,0}; // If un-initialized, may cause SEGV in bi-directional prediction iterative stage.
  int           iRefIdxBi[2];

  uint32_t          uiMbBits[3] = {1, 1, 0};

  int           iRefStart, iRefEnd;

  PartSize      ePartSize = pu.cu->partSize;

  int           bestBiPRefIdxL1 = 0;
  int           bestBiPMvpL1 = 0;
#if DISTORTION_TYPE_BUGFIX
  Distortion biPDistTemp = std::numeric_limits<Distortion>::max();
#else
  uint32_t          biPDistTemp = MAX_INT;
#endif

  Distortion    uiCost[2] = { std::numeric_limits<Distortion>::max(), std::numeric_limits<Distortion>::max() };
  Distortion    uiCostBi  = std::numeric_limits<Distortion>::max();
  Distortion    uiCostTemp;

  uint32_t          uiBits[3];
  uint32_t          uiBitsTemp;
  Distortion    bestBiPDist = std::numeric_limits<Distortion>::max();

  Distortion    uiCostTempL0[MAX_NUM_REF];
  for (int iNumRef=0; iNumRef < MAX_NUM_REF; iNumRef++)
  {
    uiCostTempL0[iNumRef] = std::numeric_limits<Distortion>::max();
  }
#if DISTORTION_TYPE_BUGFIX
  uint32_t uiBitsTempL0[MAX_NUM_REF];
#else
  Distortion    uiBitsTempL0[MAX_NUM_REF];
#endif

  Mv            mvValidList1[4];
  int           refIdxValidList1 = 0;
  uint32_t          bitsValidList1 = MAX_UINT;
#if DISTORTION_TYPE_BUGFIX
  Distortion costValidList1 = std::numeric_limits<Distortion>::max();
#else
  uint32_t          costValidList1 = MAX_UINT;
#endif
  Mv            mvHevc[3];

  xGetBlkBits( ePartSize, slice.isInterP(), puIdx, lastMode, uiMbBits);

  pu.cu->affine = true;
  pu.mergeFlag = false;

  // Uni-directional prediction
  for ( int iRefList = 0; iRefList < iNumPredDir; iRefList++ )
  {
    RefPicList  eRefPicList = ( iRefList ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );

    for ( int iRefIdxTemp = 0; iRefIdxTemp < slice.getNumRefIdx(eRefPicList); iRefIdxTemp++ )
    {
      // Get RefIdx bits
      uiBitsTemp = uiMbBits[iRefList];
      if ( slice.getNumRefIdx(eRefPicList) > 1 )
      {
        uiBitsTemp += iRefIdxTemp+1;
        if ( iRefIdxTemp == slice.getNumRefIdx(eRefPicList)-1 )
        {
          uiBitsTemp--;
        }
      }

      // Do Affine AMVP
      xEstimateAffineAMVP( pu, affiAMVPInfoTemp[eRefPicList], origBuf, eRefPicList, iRefIdxTemp, cMvPred[iRefList][iRefIdxTemp], &biPDistTemp );
      aaiMvpIdx[iRefList][iRefIdxTemp] = pu.mvpIdx[eRefPicList];
      aaiMvpNum[iRefList][iRefIdxTemp] = pu.mvpNum[eRefPicList];;
#if JVET_K0185_AFFINE_6PARA_ENC // reuse refidx of 4-para
      if ( pu.cu->affineType == AFFINEMODEL_6PARAM && refIdx4Para[iRefList] != iRefIdxTemp )
      {
        xCopyAffineAMVPInfo( affiAMVPInfoTemp[eRefPicList], aacAffineAMVPInfo[iRefList][iRefIdxTemp] );
        continue;
      }
#endif

      // set hevc ME result as start search position when it is best than mvp
      for ( int i=0; i<3; i++ )
      {
        mvHevc[i] = hevcMv[iRefList][iRefIdxTemp];
      }
      PelUnitBuf predBuf = m_tmpStorageLCU.getBuf( UnitAreaRelative(*pu.cu, pu) );

#if DISTORTION_TYPE_BUGFIX
      Distortion uiCandCost = xGetAffineTemplateCost(pu, origBuf, predBuf, mvHevc, aaiMvpIdx[iRefList][iRefIdxTemp],
                                                     AMVP_MAX_NUM_CANDS, eRefPicList, iRefIdxTemp);
#else
      uint32_t uiCandCost = xGetAffineTemplateCost( pu, origBuf, predBuf, mvHevc, aaiMvpIdx[iRefList][iRefIdxTemp], AMVP_MAX_NUM_CANDS, eRefPicList, iRefIdxTemp );
#endif
#if JVET_K0185_AFFINE_6PARA_ENC // use 4-parameter results as start point
      if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
      {
        Mv mvFour[3];
        mvFour[0] = mvAffine4Para[iRefList][iRefIdxTemp][0];
        mvFour[1] = mvAffine4Para[iRefList][iRefIdxTemp][1];

        int shift = MAX_CU_DEPTH;
        int vx2 = (mvFour[0].getHor() << shift) - ((mvFour[1].getVer() - mvFour[0].getVer()) << (shift + g_aucLog2[pu.lheight()] - g_aucLog2[pu.lwidth()]));
        int vy2 = (mvFour[0].getVer() << shift) + ((mvFour[1].getHor() - mvFour[0].getHor()) << (shift + g_aucLog2[pu.lheight()] - g_aucLog2[pu.lwidth()]));
        vx2 >>= shift;
        vy2 >>= shift;
        mvFour[2] = Mv( vx2, vy2, true );
        mvFour[2].roundMV2SignalPrecision();

        Distortion uiCandCostInherit = xGetAffineTemplateCost( pu, origBuf, predBuf, mvFour, aaiMvpIdx[iRefList][iRefIdxTemp], AMVP_MAX_NUM_CANDS, eRefPicList, iRefIdxTemp );
        if ( uiCandCostInherit < uiCandCost )
        {
          uiCandCost = uiCandCostInherit;
          for ( int i = 0; i < 3; i++ )
          {
            mvHevc[i] = mvFour[i];
          }
        }
      }
#endif

      if ( uiCandCost < biPDistTemp )
      {
        ::memcpy( cMvTemp[iRefList][iRefIdxTemp], mvHevc, sizeof(Mv)*3 );
      }
      else
      {
        ::memcpy( cMvTemp[iRefList][iRefIdxTemp], cMvPred[iRefList][iRefIdxTemp], sizeof(Mv)*3 );
      }

      // GPB list 1, save the best MvpIdx, RefIdx and Cost
      if ( slice.getMvdL1ZeroFlag() && iRefList==1 && biPDistTemp < bestBiPDist )
      {
        bestBiPDist = biPDistTemp;
        bestBiPMvpL1 = aaiMvpIdx[iRefList][iRefIdxTemp];
        bestBiPRefIdxL1 = iRefIdxTemp;
      }

      // Update bits
      uiBitsTemp += m_auiMVPIdxCost[aaiMvpIdx[iRefList][iRefIdxTemp]][AMVP_MAX_NUM_CANDS];

      if ( m_pcEncCfg->getFastMEForGenBLowDelayEnabled() && iRefList == 1 )   // list 1
      {
#if JVET_K0185_AFFINE_6PARA_ENC
        if ( slice.getList1IdxToList0Idx( iRefIdxTemp ) >= 0 && (pu.cu->affineType != AFFINEMODEL_6PARAM || slice.getList1IdxToList0Idx( iRefIdxTemp ) == refIdx4Para[0]) )
#else
        if ( slice.getList1IdxToList0Idx( iRefIdxTemp ) >= 0 )
#endif
        {
          int iList1ToList0Idx = slice.getList1IdxToList0Idx( iRefIdxTemp );
          ::memcpy( cMvTemp[1][iRefIdxTemp], cMvTemp[0][iList1ToList0Idx], sizeof(Mv)*3 );
          uiCostTemp = uiCostTempL0[iList1ToList0Idx];

          uiCostTemp -= m_pcRdCost->getCost( uiBitsTempL0[iList1ToList0Idx] );
          for (int iVerIdx = 0; iVerIdx < mvNum; iVerIdx++)
          {
            m_pcRdCost->setPredictor( cMvPred[iRefList][iRefIdxTemp][iVerIdx] );
            const int shift = cMvTemp[1][iRefIdxTemp][iVerIdx].highPrec ? VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE : 0;
#if JVET_K0337_AFFINE_MVD_PREDICTION
            Mv secondPred;
            if ( iVerIdx != 0 )
            {
              secondPred = cMvPred[iRefList][iRefIdxTemp][iVerIdx] + (cMvTemp[1][iRefIdxTemp][0] - cMvPred[1][iRefIdxTemp][0]);
              m_pcRdCost->setPredictor( secondPred );
            }
#endif
#if JVET_K0357_AMVR
            uiBitsTemp += m_pcRdCost->getBitsOfVectorWithPredictor( cMvTemp[1][iRefIdxTemp][iVerIdx].getHor()>>shift, cMvTemp[1][iRefIdxTemp][iVerIdx].getVer()>>shift, 0 );
#else
            uiBitsTemp += m_pcRdCost->getBitsOfVectorWithPredictor( cMvTemp[1][iRefIdxTemp][iVerIdx].getHor() >> shift, cMvTemp[1][iRefIdxTemp][iVerIdx].getVer() >> shift );
#endif
          }
          /*calculate the correct cost*/
          uiCostTemp += m_pcRdCost->getCost( uiBitsTemp );
          DTRACE( g_trace_ctx, D_COMMON, " (%d) uiCostTemp=%d\n", DTRACE_GET_COUNTER(g_trace_ctx,D_COMMON), uiCostTemp );
        }
        else
        {
          xAffineMotionEstimation( pu, origBuf, eRefPicList, cMvPred[iRefList][iRefIdxTemp], iRefIdxTemp, cMvTemp[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp );
        }
      }
      else
      {
        xAffineMotionEstimation( pu, origBuf, eRefPicList, cMvPred[iRefList][iRefIdxTemp], iRefIdxTemp, cMvTemp[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp );
      }

      // Set best AMVP Index
      xCopyAffineAMVPInfo( affiAMVPInfoTemp[eRefPicList], aacAffineAMVPInfo[iRefList][iRefIdxTemp] );
      xCheckBestAffineMVP( pu, affiAMVPInfoTemp[eRefPicList], eRefPicList, cMvTemp[iRefList][iRefIdxTemp], cMvPred[iRefList][iRefIdxTemp], aaiMvpIdx[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp );

      if ( iRefList == 0 )
      {
        uiCostTempL0[iRefIdxTemp] = uiCostTemp;
        uiBitsTempL0[iRefIdxTemp] = uiBitsTemp;
      }
      DTRACE( g_trace_ctx, D_COMMON, " (%d) uiCostTemp=%d, uiCost[iRefList]=%d\n", DTRACE_GET_COUNTER(g_trace_ctx,D_COMMON), uiCostTemp, uiCost[iRefList] );
      if ( uiCostTemp < uiCost[iRefList] )
      {
        uiCost[iRefList] = uiCostTemp;
        uiBits[iRefList] = uiBitsTemp; // storing for bi-prediction

        // set best motion
        ::memcpy( aacMv[iRefList], cMvTemp[iRefList][iRefIdxTemp], sizeof(Mv) * 3 );
        iRefIdx[iRefList] = iRefIdxTemp;
      }

      if ( iRefList == 1 && uiCostTemp < costValidList1 && slice.getList1IdxToList0Idx( iRefIdxTemp ) < 0 )
      {
        costValidList1 = uiCostTemp;
        bitsValidList1 = uiBitsTemp;

        // set motion
        memcpy( mvValidList1, cMvTemp[iRefList][iRefIdxTemp], sizeof(Mv)*3 );
        refIdxValidList1 = iRefIdxTemp;
      }
    } // End refIdx loop
  } // end Uni-prediction

#if JVET_K0185_AFFINE_6PARA_ENC // save 4-parameter UNI-ME results
  if ( pu.cu->affineType == AFFINEMODEL_4PARAM )
  {
    ::memcpy( mvAffine4Para, cMvTemp, sizeof( cMvTemp ) );
  }
#endif

  // Bi-directional prediction
#if JVET_K0220_ENC_CTRL
  if ( slice.isInterB() && !PU::isBipredRestriction(pu) )
#else
  if ( slice.isInterB() && !PU::isBipredRestriction(pu) && !bFastSkipBi )
#endif
  {
    // Set as best list0 and list1
    iRefIdxBi[0] = iRefIdx[0];
    iRefIdxBi[1] = iRefIdx[1];

    ::memcpy( cMvBi,       aacMv,     sizeof(aacMv)     );
    ::memcpy( cMvPredBi,   cMvPred,   sizeof(cMvPred)   );
    ::memcpy( aaiMvpIdxBi, aaiMvpIdx, sizeof(aaiMvpIdx) );

    uint32_t uiMotBits[2];

    if ( slice.getMvdL1ZeroFlag() ) // GPB, list 1 only use Mvp
    {
      xCopyAffineAMVPInfo( aacAffineAMVPInfo[1][bestBiPRefIdxL1], affiAMVPInfoTemp[REF_PIC_LIST_1] );
      pu.mvpIdx[REF_PIC_LIST_1] = bestBiPMvpL1;
      aaiMvpIdxBi[1][bestBiPRefIdxL1] = bestBiPMvpL1;

      // Set Mv for list1
      Mv pcMvTemp[3] = { affiAMVPInfoTemp[REF_PIC_LIST_1].mvCandLT[bestBiPMvpL1],
                         affiAMVPInfoTemp[REF_PIC_LIST_1].mvCandRT[bestBiPMvpL1],
                         affiAMVPInfoTemp[REF_PIC_LIST_1].mvCandLB[bestBiPMvpL1] };
      ::memcpy( cMvPredBi[1][bestBiPRefIdxL1], pcMvTemp, sizeof(Mv)*3 );
      ::memcpy( cMvBi[1],                      pcMvTemp, sizeof(Mv)*3 );
      ::memcpy( cMvTemp[1][bestBiPRefIdxL1],   pcMvTemp, sizeof(Mv)*3 );
      iRefIdxBi[1] = bestBiPRefIdxL1;

      // Get list1 prediction block
      PU::setAllAffineMv( pu, cMvBi[1][0], cMvBi[1][1], cMvBi[1][2], REF_PIC_LIST_1 );
      pu.refIdx[REF_PIC_LIST_1] = iRefIdxBi[1];

      PelUnitBuf predBufTmp = m_tmpPredStorage[REF_PIC_LIST_1].getBuf( UnitAreaRelative(*pu.cu, pu) );
      motionCompensation( pu, predBufTmp, REF_PIC_LIST_1 );

      // Update bits
      uiMotBits[0] = uiBits[0] - uiMbBits[0];
      uiMotBits[1] = uiMbBits[1];

      if( slice.getNumRefIdx(REF_PIC_LIST_1) > 1 )
      {
        uiMotBits[1] += bestBiPRefIdxL1+1;
        if( bestBiPRefIdxL1 == slice.getNumRefIdx(REF_PIC_LIST_1)-1 )
        {
          uiMotBits[1]--;
        }
      }
      uiMotBits[1] += m_auiMVPIdxCost[aaiMvpIdxBi[1][bestBiPRefIdxL1]][AMVP_MAX_NUM_CANDS];
      uiBits[2] = uiMbBits[2] + uiMotBits[0] + uiMotBits[1];
    }
    else
    {
      uiMotBits[0] = uiBits[0] - uiMbBits[0];
      uiMotBits[1] = uiBits[1] - uiMbBits[1];
      uiBits[2] = uiMbBits[2] + uiMotBits[0] + uiMotBits[1];
    }

    // 4-times iteration (default)
    int iNumIter = 4;
    // fast encoder setting or GPB: only one iteration
    if ( m_pcEncCfg->getFastInterSearchMode()==FASTINTERSEARCH_MODE1 || m_pcEncCfg->getFastInterSearchMode()==FASTINTERSEARCH_MODE2 || slice.getMvdL1ZeroFlag() )
    {
      iNumIter = 1;
    }

    for ( int iIter = 0; iIter < iNumIter; iIter++ )
    {
      // Set RefList
      int iRefList = iIter % 2;
      if ( m_pcEncCfg->getFastInterSearchMode()==FASTINTERSEARCH_MODE1 || m_pcEncCfg->getFastInterSearchMode()==FASTINTERSEARCH_MODE2 )
      {
        if( uiCost[0] <= uiCost[1] )
        {
          iRefList = 1;
        }
        else
        {
          iRefList = 0;
        }
      }
      else if ( iIter == 0 )
      {
        iRefList = 0;
      }

      // First iterate, get prediction block of opposite direction
      if( iIter == 0 && !slice.getMvdL1ZeroFlag() )
      {
        PU::setAllAffineMv( pu, aacMv[1-iRefList][0], aacMv[1-iRefList][1], aacMv[1-iRefList][2], RefPicList(1-iRefList) );
        pu.refIdx[1-iRefList] = iRefIdx[1-iRefList];

        PelUnitBuf predBufTmp = m_tmpPredStorage[1 - iRefList].getBuf( UnitAreaRelative(*pu.cu, pu) );
        motionCompensation( pu, predBufTmp, RefPicList(1 - iRefList) );
      }

      RefPicList eRefPicList = ( iRefList ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );

      if ( slice.getMvdL1ZeroFlag() ) // GPB, fix List 1, search List 0
      {
        iRefList = 0;
        eRefPicList = REF_PIC_LIST_0;
      }

      bool bChanged = false;

      iRefStart = 0;
      iRefEnd   = slice.getNumRefIdx(eRefPicList) - 1;

      for ( int iRefIdxTemp = iRefStart; iRefIdxTemp <= iRefEnd; iRefIdxTemp++ )
      {
#if JVET_K0185_AFFINE_6PARA_ENC // reuse refidx of 4-para
        if ( pu.cu->affineType == AFFINEMODEL_6PARAM && refIdx4Para[iRefList] != iRefIdxTemp )
        {
          continue;
        }
#endif

        // update bits
        uiBitsTemp = uiMbBits[2] + uiMotBits[1-iRefList];
        if( slice.getNumRefIdx(eRefPicList) > 1 )
        {
          uiBitsTemp += iRefIdxTemp+1;
          if ( iRefIdxTemp == slice.getNumRefIdx(eRefPicList)-1 )
          {
            uiBitsTemp--;
          }
        }
        uiBitsTemp += m_auiMVPIdxCost[aaiMvpIdxBi[iRefList][iRefIdxTemp]][AMVP_MAX_NUM_CANDS];

        // call Affine ME
        xAffineMotionEstimation( pu, origBuf, eRefPicList, cMvPredBi[iRefList][iRefIdxTemp], iRefIdxTemp, cMvTemp[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp, true );
        xCopyAffineAMVPInfo( aacAffineAMVPInfo[iRefList][iRefIdxTemp], affiAMVPInfoTemp[eRefPicList] );
        xCheckBestAffineMVP( pu, affiAMVPInfoTemp[eRefPicList], eRefPicList, cMvTemp[iRefList][iRefIdxTemp], cMvPredBi[iRefList][iRefIdxTemp], aaiMvpIdxBi[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp );

        if ( uiCostTemp < uiCostBi )
        {
          bChanged = true;
          ::memcpy( cMvBi[iRefList], cMvTemp[iRefList][iRefIdxTemp], sizeof(Mv)*3 );
          iRefIdxBi[iRefList] = iRefIdxTemp;

          uiCostBi            = uiCostTemp;
          uiMotBits[iRefList] = uiBitsTemp - uiMbBits[2] - uiMotBits[1-iRefList];
          uiBits[2]           = uiBitsTemp;

          if ( iNumIter != 1 ) // MC for next iter
          {
            //  Set motion
            PU::setAllAffineMv( pu, cMvBi[iRefList][0], cMvBi[iRefList][1], cMvBi[iRefList][2], eRefPicList );
            pu.refIdx[eRefPicList] = iRefIdxBi[eRefPicList];
            PelUnitBuf predBufTmp = m_tmpPredStorage[iRefList].getBuf( UnitAreaRelative(*pu.cu, pu) );
            motionCompensation( pu, predBufTmp, eRefPicList );
          }
        }
      } // for loop-iRefIdxTemp

      if ( !bChanged )
      {
        if ( uiCostBi <= uiCost[0] && uiCostBi <= uiCost[1] )
        {
          xCopyAffineAMVPInfo( aacAffineAMVPInfo[0][iRefIdxBi[0]], affiAMVPInfoTemp[REF_PIC_LIST_0] );
          xCheckBestAffineMVP( pu, affiAMVPInfoTemp[REF_PIC_LIST_0], REF_PIC_LIST_0, cMvBi[0], cMvPredBi[0][iRefIdxBi[0]], aaiMvpIdxBi[0][iRefIdxBi[0]], uiBits[2], uiCostBi );

          if ( !slice.getMvdL1ZeroFlag() )
          {
            xCopyAffineAMVPInfo( aacAffineAMVPInfo[1][iRefIdxBi[1]], affiAMVPInfoTemp[REF_PIC_LIST_1] );
            xCheckBestAffineMVP( pu, affiAMVPInfoTemp[REF_PIC_LIST_1], REF_PIC_LIST_1, cMvBi[1], cMvPredBi[1][iRefIdxBi[1]], aaiMvpIdxBi[1][iRefIdxBi[1]], uiBits[2], uiCostBi );
          }
        }
        break;
      }
    } // for loop-iter
  } // if (B_SLICE)

  pu.mv    [REF_PIC_LIST_0] = Mv();
  pu.mv    [REF_PIC_LIST_1] = Mv();
  pu.mvd   [REF_PIC_LIST_0] = cMvZero;
  pu.mvd   [REF_PIC_LIST_1] = cMvZero;
  pu.refIdx[REF_PIC_LIST_0] = NOT_VALID;
  pu.refIdx[REF_PIC_LIST_1] = NOT_VALID;
  pu.mvpIdx[REF_PIC_LIST_0] = NOT_VALID;
  pu.mvpIdx[REF_PIC_LIST_1] = NOT_VALID;
  pu.mvpNum[REF_PIC_LIST_0] = NOT_VALID;
  pu.mvpNum[REF_PIC_LIST_1] = NOT_VALID;

  for ( int verIdx = 0; verIdx < 3; verIdx++ )
  {
    pu.mvdAffi[REF_PIC_LIST_0][verIdx] = cMvZero;
    pu.mvdAffi[REF_PIC_LIST_1][verIdx] = cMvZero;
  }

  // Set Motion Field
  memcpy( aacMv[1], mvValidList1, sizeof(Mv)*3 );
  iRefIdx[1] = refIdxValidList1;
  uiBits[1]  = bitsValidList1;
  uiCost[1]  = costValidList1;

  // Affine ME result set
  if ( uiCostBi <= uiCost[0] && uiCostBi <= uiCost[1] ) // Bi
  {
    lastMode = 2;
    affineCost = uiCostBi;

    PU::setAllAffineMv( pu, cMvBi[0][0], cMvBi[0][1], cMvBi[0][2], REF_PIC_LIST_0 );
    PU::setAllAffineMv( pu, cMvBi[1][0], cMvBi[1][1], cMvBi[1][2], REF_PIC_LIST_1 );
    pu.refIdx[REF_PIC_LIST_0] = iRefIdxBi[0];
    pu.refIdx[REF_PIC_LIST_1] = iRefIdxBi[1];

    for ( int verIdx = 0; verIdx < mvNum; verIdx++ )
    {
      pu.mvdAffi[REF_PIC_LIST_0][verIdx] = cMvBi[0][verIdx] - cMvPredBi[0][iRefIdxBi[0]][verIdx];
      pu.mvdAffi[REF_PIC_LIST_1][verIdx] = cMvBi[1][verIdx] - cMvPredBi[1][iRefIdxBi[1]][verIdx];
#if JVET_K0337_AFFINE_MVD_PREDICTION
      if ( verIdx != 0 )
      {
        pu.mvdAffi[0][verIdx] = pu.mvdAffi[0][verIdx] - pu.mvdAffi[0][0];
        pu.mvdAffi[1][verIdx] = pu.mvdAffi[1][verIdx] - pu.mvdAffi[1][0];
      }
#endif
    }

    pu.interDir = 3;

    pu.mvpIdx[REF_PIC_LIST_0] = aaiMvpIdxBi[0][iRefIdxBi[0]];
    pu.mvpNum[REF_PIC_LIST_0] = aaiMvpNum[0][iRefIdxBi[0]];
    pu.mvpIdx[REF_PIC_LIST_1] = aaiMvpIdxBi[1][iRefIdxBi[1]];
    pu.mvpNum[REF_PIC_LIST_1] = aaiMvpNum[1][iRefIdxBi[1]];
  }
  else if ( uiCost[0] <= uiCost[1] ) // List 0
  {
    lastMode = 0;
    affineCost = uiCost[0];

    PU::setAllAffineMv( pu, aacMv[0][0], aacMv[0][1], aacMv[0][2], REF_PIC_LIST_0 );
    pu.refIdx[REF_PIC_LIST_0] = iRefIdx[0];

    for ( int verIdx = 0; verIdx < mvNum; verIdx++ )
    {
      pu.mvdAffi[REF_PIC_LIST_0][verIdx] = aacMv[0][verIdx] - cMvPred[0][iRefIdx[0]][verIdx];
#if JVET_K0337_AFFINE_MVD_PREDICTION
      if ( verIdx != 0 )
      {
        pu.mvdAffi[0][verIdx] = pu.mvdAffi[0][verIdx] - pu.mvdAffi[0][0];
      }
#endif
    }
    pu.interDir = 1;

    pu.mvpIdx[REF_PIC_LIST_0] = aaiMvpIdx[0][iRefIdx[0]];
    pu.mvpNum[REF_PIC_LIST_0] = aaiMvpNum[0][iRefIdx[0]];
  }
  else
  {
    lastMode = 1;
    affineCost = uiCost[1];

    PU::setAllAffineMv( pu, aacMv[1][0], aacMv[1][1], aacMv[1][2], REF_PIC_LIST_1 );
    pu.refIdx[REF_PIC_LIST_1] = iRefIdx[1];

    for ( int verIdx = 0; verIdx < mvNum; verIdx++ )
    {
      pu.mvdAffi[REF_PIC_LIST_1][verIdx] = aacMv[1][verIdx] - cMvPred[1][iRefIdx[1]][verIdx];
#if JVET_K0337_AFFINE_MVD_PREDICTION
      if ( verIdx != 0 )
      {
        pu.mvdAffi[1][verIdx] = pu.mvdAffi[1][verIdx] - pu.mvdAffi[1][0];
      }
#endif
    }
    pu.interDir = 2;

    pu.mvpIdx[REF_PIC_LIST_1] = aaiMvpIdx[1][iRefIdx[1]];
    pu.mvpNum[REF_PIC_LIST_1] = aaiMvpNum[1][iRefIdx[1]];
  }
}

void solveEqual( double** dEqualCoeff, int iOrder, double* dAffinePara )
{
#if JVET_K_AFFINE_BUG_FIXES
  for ( int k = 0; k < iOrder; k++ )
  {
    dAffinePara[k] = 0.;
  }
#endif

  // row echelon
  for ( int i = 1; i < iOrder; i++ )
  {
    // find column max
    double temp = fabs(dEqualCoeff[i][i-1]);
    int tempIdx = i;
    for ( int j = i+1; j < iOrder+1; j++ )
    {
      if ( fabs(dEqualCoeff[j][i-1]) > temp )
      {
        temp = fabs(dEqualCoeff[j][i-1]);
        tempIdx = j;
      }
    }

    // swap line
    if ( tempIdx != i )
    {
      for ( int j = 0; j < iOrder+1; j++ )
      {
        dEqualCoeff[0][j] = dEqualCoeff[i][j];
        dEqualCoeff[i][j] = dEqualCoeff[tempIdx][j];
        dEqualCoeff[tempIdx][j] = dEqualCoeff[0][j];
      }
    }

    // elimination first column
#if JVET_K_AFFINE_BUG_FIXES
    if ( dEqualCoeff[i][i - 1] == 0. )
    {
      return;
    }
#endif
    for ( int j = i+1; j < iOrder+1; j++ )
    {
      for ( int k = i; k < iOrder+1; k++ )
      {
        dEqualCoeff[j][k] = dEqualCoeff[j][k] - dEqualCoeff[i][k] * dEqualCoeff[j][i-1] / dEqualCoeff[i][i-1];
      }
    }
  }

#if JVET_K_AFFINE_BUG_FIXES
  if ( dEqualCoeff[iOrder][iOrder - 1] == 0. )
  {
    return;
  }
#endif
  dAffinePara[iOrder-1] = dEqualCoeff[iOrder][iOrder] / dEqualCoeff[iOrder][iOrder-1];
  for ( int i = iOrder-2; i >= 0; i-- )
  {
#if JVET_K_AFFINE_BUG_FIXES
    if ( dEqualCoeff[i + 1][i] == 0. )
    {
      for ( int k = 0; k < iOrder; k++ )
      {
        dAffinePara[k] = 0.;
      }
      return;
    }
#endif
    double temp = 0;
    for ( int j = i+1; j < iOrder; j++ )
    {
      temp += dEqualCoeff[i+1][j] * dAffinePara[j];
    }
    dAffinePara[i] = ( dEqualCoeff[i+1][iOrder] - temp ) / dEqualCoeff[i+1][i];
  }
}

void InterSearch::xCheckBestAffineMVP( PredictionUnit &pu, AffineAMVPInfo &affineAMVPInfo, RefPicList eRefPicList, Mv acMv[3], Mv acMvPred[3], int& riMVPIdx, uint32_t& ruiBits, Distortion& ruiCost )
{
  if ( affineAMVPInfo.numCand < 2 )
  {
    return;
  }

#if JVET_K0185_AFFINE_6PARA_ENC
  int mvNum = pu.cu->affineType ? 3 : 2;
#endif

  m_pcRdCost->selectMotionLambda( pu.cu->transQuantBypass );
  m_pcRdCost->setCostScale ( 0 );

  int iBestMVPIdx = riMVPIdx;

  // Get origin MV bits
  int iOrgMvBits = 0;
#if JVET_K0185_AFFINE_6PARA_ENC
  for ( int iVerIdx = 0; iVerIdx < mvNum; iVerIdx++ )
#else
  for ( int iVerIdx=0; iVerIdx<2; iVerIdx++ )
#endif
  {
    m_pcRdCost->setPredictor ( acMvPred[iVerIdx] );
    const int shift = acMv[iVerIdx].highPrec ? VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE : 0;

#if JVET_K0337_AFFINE_MVD_PREDICTION
    Mv secondPred;
    if ( iVerIdx != 0 )
    {
      secondPred = acMvPred[iVerIdx] + (acMv[0] - acMvPred[0]);
      m_pcRdCost->setPredictor( secondPred );
    }
#endif
#if JVET_K0357_AMVR
    iOrgMvBits += m_pcRdCost->getBitsOfVectorWithPredictor( acMv[iVerIdx].getHor()>>shift, acMv[iVerIdx].getVer()>>shift, 0 );
#else
    iOrgMvBits += m_pcRdCost->getBitsOfVectorWithPredictor( acMv[iVerIdx].getHor() >> shift, acMv[iVerIdx].getVer() >> shift );
#endif
  }
  iOrgMvBits += m_auiMVPIdxCost[riMVPIdx][AMVP_MAX_NUM_CANDS];

  int iBestMvBits = iOrgMvBits;
  for (int iMVPIdx = 0; iMVPIdx < affineAMVPInfo.numCand; iMVPIdx++)
  {
    if (iMVPIdx == riMVPIdx)
    {
      continue;
    }

    int iMvBits = 0;
#if JVET_K0185_AFFINE_6PARA_ENC
    for ( int iVerIdx = 0; iVerIdx < mvNum; iVerIdx++ )
#else
    for ( int iVerIdx=0; iVerIdx<2; iVerIdx++ )
#endif
    {
#if JVET_K0185_AFFINE_6PARA_ENC
      m_pcRdCost->setPredictor( iVerIdx == 2 ? affineAMVPInfo.mvCandLB[iMVPIdx] :
        (iVerIdx == 1 ? affineAMVPInfo.mvCandRT[iMVPIdx] : affineAMVPInfo.mvCandLT[iMVPIdx]) );
#else
      m_pcRdCost->setPredictor ( iVerIdx ? affineAMVPInfo.mvCandRT[iMVPIdx] : affineAMVPInfo.mvCandLT[iMVPIdx] );
#endif
      const int shift = acMv[iVerIdx].highPrec ? VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE : 0;

#if JVET_K0337_AFFINE_MVD_PREDICTION
      Mv secondPred;
      if ( iVerIdx != 0 )
      {
#if JVET_K0185_AFFINE_6PARA_ENC
        secondPred = (iVerIdx == 1 ? affineAMVPInfo.mvCandRT[iMVPIdx] : affineAMVPInfo.mvCandLT[iMVPIdx]) + (acMv[0] - affineAMVPInfo.mvCandLT[iMVPIdx]);
#else
        secondPred = affineAMVPInfo.mvCandRT[iMVPIdx] + (acMv[0] - affineAMVPInfo.mvCandLT[iMVPIdx]);
#endif
        m_pcRdCost->setPredictor( secondPred );
      }
#endif
#if JVET_K0357_AMVR
      iMvBits += m_pcRdCost->getBitsOfVectorWithPredictor( acMv[iVerIdx].getHor()>>shift, acMv[iVerIdx].getVer()>>shift, 0 );
#else
      iMvBits += m_pcRdCost->getBitsOfVectorWithPredictor( acMv[iVerIdx].getHor() >> shift, acMv[iVerIdx].getVer() >> shift );
#endif
    }
    iMvBits += m_auiMVPIdxCost[iMVPIdx][AMVP_MAX_NUM_CANDS];

    if (iMvBits < iBestMvBits)
    {
      iBestMvBits = iMvBits;
      iBestMVPIdx = iMVPIdx;
    }
  }

  if (iBestMVPIdx != riMVPIdx)  // if changed
  {
    acMvPred[0] = affineAMVPInfo.mvCandLT[iBestMVPIdx];
    acMvPred[1] = affineAMVPInfo.mvCandRT[iBestMVPIdx];
    acMvPred[2] = affineAMVPInfo.mvCandLB[iBestMVPIdx];
    riMVPIdx = iBestMVPIdx;
    uint32_t uiOrgBits = ruiBits;
    ruiBits = uiOrgBits - iOrgMvBits + iBestMvBits;
    ruiCost = (ruiCost - m_pcRdCost->getCost( uiOrgBits )) + m_pcRdCost->getCost( ruiBits );
  }
}

void InterSearch::xAffineMotionEstimation( PredictionUnit& pu,
                                           PelUnitBuf&     origBuf,
                                           RefPicList      eRefPicList,
                                           Mv              acMvPred[3],
                                           int             iRefIdxPred,
                                           Mv              acMv[3],
                                           uint32_t&           ruiBits,
                                           Distortion&     ruiCost,
                                           bool            bBi )
{
  const int width  = pu.Y().width;
  const int height = pu.Y().height;

  const Picture* refPic = pu.cu->slice->getRefPic(eRefPicList, iRefIdxPred);

  // Set Origin YUV: pcYuv
  PelUnitBuf*   pBuf = &origBuf;
  double        fWeight       = 1.0;

  PelUnitBuf  origBufTmp = m_tmpStorageLCU.getBuf( UnitAreaRelative( *pu.cu, pu ) );

  // if Bi, set to ( 2 * Org - ListX )
  if ( bBi )
  {
    // NOTE: Other buf contains predicted signal from another direction
    PelUnitBuf otherBuf = m_tmpPredStorage[1 - (int)eRefPicList].getBuf( UnitAreaRelative( *pu.cu, pu ) );
    origBufTmp.copyFrom(origBuf);
    origBufTmp.removeHighFreq(otherBuf, m_pcEncCfg->getClipForBiPredMeEnabled(), pu.cu->slice->clpRngs());
    pBuf = &origBufTmp;

    fWeight = 0.5;
  }

  // pred YUV
  PelUnitBuf  predBuf = m_tmpAffiStorage.getBuf( UnitAreaRelative(*pu.cu, pu) );

  // Set start Mv position, use input mv as started search mv
  Mv acMvTemp[3];
  ::memcpy( acMvTemp, acMv, sizeof(Mv)*3 );
  acMvTemp[0].setHighPrec();
  acMvTemp[1].setHighPrec();
  acMvTemp[2].setHighPrec();

  // Set delta mv
  // malloc buffer
#if JVET_K0185_AFFINE_6PARA_ENC
  int iParaNum = pu.cu->affineType ? 7 : 5;
  int affineParaNum = iParaNum - 1;
  int mvNum = pu.cu->affineType ? 3 : 2;
#else
  static const int iParaNum = 5;
#endif
  double **pdEqualCoeff;
  pdEqualCoeff = new double *[iParaNum];
  for ( int i = 0; i < iParaNum; i++ )
  {
    pdEqualCoeff[i] = new double[iParaNum];
  }

#if JVET_K0367_AFFINE_FIX_POINT
  int64_t  i64EqualCoeff[7][7];
  Pel    *piError = m_tmpAffiError;
  int    *pdDerivate[2];
#else
  int    *piError = m_tmpAffiError;
  double *pdDerivate[2];
#endif
  pdDerivate[0] = m_tmpAffiDeri[0];
  pdDerivate[1] = m_tmpAffiDeri[1];

  Distortion uiCostBest = std::numeric_limits<Distortion>::max();
  uint32_t uiBitsBest = 0;

  // do motion compensation with origin mv
  clipMv( acMvTemp[0], pu.cu->lumaPos(), *pu.cs->sps );
  clipMv( acMvTemp[1], pu.cu->lumaPos(), *pu.cs->sps );
#if JVET_K0185_AFFINE_6PARA_ENC
  if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
  {
    clipMv( acMvTemp[2], pu.cu->lumaPos(), *pu.cs->sps );
  }
#endif
#if !JVET_K_AFFINE_BUG_FIXES
  int vx2 =  - ( acMvTemp[1].getVer() - acMvTemp[0].getVer() ) * height / width + acMvTemp[0].getHor();
  int vy2 =    ( acMvTemp[1].getHor() - acMvTemp[0].getHor() ) * height / width + acMvTemp[0].getVer();
  acMvTemp[2] = Mv( vx2, vy2, true );
  clipMv( acMvTemp[2], pu.cu->lumaPos(), *pu.cs->sps );
#endif
  xPredAffineBlk( COMPONENT_Y, pu, refPic, acMvTemp, predBuf, false, pu.cs->slice->clpRng( COMPONENT_Y ) );

  // get error
  uiCostBest = m_pcRdCost->getDistPart( predBuf.Y(), pBuf->Y(), pu.cs->sps->getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, DF_HAD );

  // get cost with mv
  m_pcRdCost->setCostScale(0);
  uiBitsBest = ruiBits;
  DTRACE( g_trace_ctx, D_COMMON, " (%d) xx uiBitsBest=%d\n", DTRACE_GET_COUNTER(g_trace_ctx,D_COMMON), uiBitsBest );
#if JVET_K0185_AFFINE_6PARA_ENC
  for ( int i = 0; i < mvNum; i++ )
#else
  for ( int i=0; i<2; i++ )
#endif
  {
    DTRACE( g_trace_ctx, D_COMMON, "#mvPredForBits=(%d,%d) \n", acMvPred[i].getHor(), acMvPred[i].getVer() );
    m_pcRdCost->setPredictor( acMvPred[i] );
    DTRACE( g_trace_ctx, D_COMMON, "#mvForBits=(%d,%d) \n", acMvTemp[i].getHor(), acMvTemp[i].getVer() );

    const int shift = acMvTemp[i].highPrec ? VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE : 0;
#if JVET_K0337_AFFINE_MVD_PREDICTION
    Mv secondPred;
    if ( i != 0 )
    {
      secondPred = acMvPred[i] + (acMvTemp[0] - acMvPred[0]);
      m_pcRdCost->setPredictor( secondPred );
    }
#endif
#if JVET_K0357_AMVR
    uiBitsBest += m_pcRdCost->getBitsOfVectorWithPredictor( acMvTemp[i].getHor()>>shift, acMvTemp[i].getVer()>>shift, 0 );
#else
    uiBitsBest += m_pcRdCost->getBitsOfVectorWithPredictor( acMvTemp[i].getHor() >> shift, acMvTemp[i].getVer() >> shift );
#endif
    DTRACE( g_trace_ctx, D_COMMON, " (%d) yy uiBitsBest=%d\n", DTRACE_GET_COUNTER(g_trace_ctx,D_COMMON), uiBitsBest );
  }
  uiCostBest = (Distortion)( floor( fWeight * (double)uiCostBest ) + (double)m_pcRdCost->getCost( uiBitsBest ) );

  DTRACE( g_trace_ctx, D_COMMON, " (%d) uiBitsBest=%d, uiCostBest=%d\n", DTRACE_GET_COUNTER(g_trace_ctx,D_COMMON), uiBitsBest, uiCostBest );

  ::memcpy( acMv, acMvTemp, sizeof(Mv) * 3 );

  const int bufStride = pBuf->Y().stride;
  const int predBufStride = predBuf.Y().stride;

#if JVET_K0185_AFFINE_6PARA_ENC
  int iIterTime;
  if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
  {
    iIterTime = bBi ? 3 : 4;
  }
  else
  {
    iIterTime = bBi ? 3 : 5;
  }

  if ( !pu.cu->cs->sps->getSpsNext().getUseAffineType() )
  {
    iIterTime = bBi ? 5 : 7;
  }
#else
  int iIterTime = bBi ? 5 : 7;
#endif
  for ( int iter=0; iter<iIterTime; iter++ )    // iterate loop
  {
    /*********************************************************************************
     *                         use gradient to update mv
     *********************************************************************************/
    // get Error Matrix
    Pel* pOrg  = pBuf->Y().buf;
    Pel* pPred = predBuf.Y().buf;
    for ( int j=0; j< height; j++ )
    {
      for ( int i=0; i< width; i++ )
      {
        piError[i + j * width] = pOrg[i] - pPred[i];
      }
      pOrg  += bufStride;
      pPred += predBufStride;
    }

    // sobel x direction
    // -1 0 1
    // -2 0 2
    // -1 0 1
    pPred = predBuf.Y().buf;
#if JVET_K0367_AFFINE_FIX_POINT
    m_HorizontalSobelFilter( pPred, predBufStride, pdDerivate[0], width, width, height );
#else
    for ( int j = 1; j < height-1; j++ )
    {
      for ( int k = 1; k < width-1; k++ )
      {
        int iCenter = j*predBufStride + k;
        pdDerivate[0][j*width + k] = (double)( pPred[iCenter + 1 - predBufStride] - pPred[iCenter - 1 - predBufStride]
                                                  + ( pPred[iCenter + 1] << 1 )      - ( pPred[iCenter - 1] << 1 )
                                                  + pPred[iCenter + 1 + predBufStride] - pPred[iCenter - 1 + predBufStride] ) / 8 ;
      }
      pdDerivate[0][ j*width ]             = pdDerivate[0][ j*width + 1 ];
      pdDerivate[0][ j*width + width - 1 ] = pdDerivate[0][ j*width + width - 2 ];
    }

    pdDerivate[0][0]                        = pdDerivate[0][width+1];
    pdDerivate[0][width-1]                  = pdDerivate[0][width+width-2];
    pdDerivate[0][(height-1)*width]         = pdDerivate[0][(height-2)*width+1];
    pdDerivate[0][(height-1)*width+width-1] = pdDerivate[0][(height-2)*width+width-2];

    for ( int j = 1; j < width - 1; j++ )
    {
      pdDerivate[0][j] = pdDerivate[0][width+j];
      pdDerivate[0][(height-1)*width+j] = pdDerivate[0][(height-2)*width+j];
    }
#endif

    // sobel y direction
    // -1 -2 -1
    //  0  0  0
    //  1  2  1
#if JVET_K0367_AFFINE_FIX_POINT
    m_VerticalSobelFilter( pPred, predBufStride, pdDerivate[1], width, width, height );
#else
    for ( int k=1; k < width-1; k++ )
    {
      for ( int j = 1; j < height-1; j++ )
      {
        int iCenter = j*predBufStride + k;
        pdDerivate[1][j*width + k] = (double)( pPred[iCenter + predBufStride - 1]    -   pPred[iCenter - predBufStride - 1]
                                           + ( pPred[iCenter + predBufStride] << 1 ) - ( pPred[iCenter - predBufStride] << 1 )
                                           +   pPred[iCenter + predBufStride + 1]    -   pPred[iCenter - predBufStride + 1] ) / 8;
      }
      pdDerivate[1][k] = pdDerivate[1][ width + k ];
      pdDerivate[1][ (height - 1) * width + k ] = pdDerivate[1][ (height - 2) * width + k ];
    }

    pdDerivate[1][0]           = pdDerivate[1][width+1];
    pdDerivate[1][width-1] = pdDerivate[1][width + width-2];
    pdDerivate[1][(height-1)*width]         = pdDerivate[1][(height-2)*width+1];
    pdDerivate[1][(height-1)*width+width-1] = pdDerivate[1][(height-2)*width+(width-2)];

    for ( int j=1; j < height-1; j++ )
    {
      pdDerivate[1][j*width] = pdDerivate[1][j*width+1];
      pdDerivate[1][j*width+width-1] = pdDerivate[1][j*width+width-2];
    }
#endif

    // solve delta x and y
#if JVET_K0367_AFFINE_FIX_POINT
    for ( int row = 0; row < iParaNum; row++ )
    {
      memset( &i64EqualCoeff[row][0], 0, iParaNum * sizeof( int64_t ) );
    }

    m_EqualCoeffComputer( piError, width, pdDerivate, width, i64EqualCoeff, width, height
#if JVET_K0185_AFFINE_6PARA_ENC
      , (pu.cu->affineType == AFFINEMODEL_6PARAM)
#else
      , false
#endif
    );

    for ( int row = 0; row < iParaNum; row++ )
    {
      for ( int i = 0; i < iParaNum; i++ )
      {
        pdEqualCoeff[row][i] = (double)i64EqualCoeff[row][i];
      }
    }
#else
    for ( int m = 0; m != iParaNum; m++ )
    {
      for ( int n = 0; n != iParaNum; n++ )
      {
        pdEqualCoeff[m][n] = 0.0;
      }
    }

    for ( int j = 0; j != height; j++ )
    {
      for ( int k = 0; k != width; k++ )
      {
        int iIdx = j * width + k;
#if JVET_K0185_AFFINE_6PARA_ENC
        double dC[6];
        if ( pu.cu->affineType )
        {
          dC[0] = pdDerivate[0][iIdx];
          dC[1] = k * pdDerivate[0][iIdx];
          dC[2] = pdDerivate[1][iIdx];
          dC[3] = k * pdDerivate[1][iIdx];
          dC[4] = j * pdDerivate[0][iIdx];
          dC[5] = j * pdDerivate[1][iIdx];
        }
        else
        {
          dC[0] = pdDerivate[0][iIdx];
          dC[1] = k * pdDerivate[0][iIdx] + j * pdDerivate[1][iIdx];
          dC[2] = pdDerivate[1][iIdx];
          dC[3] = j * pdDerivate[0][iIdx] - k * pdDerivate[1][iIdx];
        }

        for ( int col = 0; col < affineParaNum; col++ )
        {
          for ( int row = 0; row < affineParaNum; row++ )
          {
            pdEqualCoeff[col + 1][row] += dC[col] * dC[row];
          }
          pdEqualCoeff[col + 1][affineParaNum] += (double)(piError[iIdx] * dC[col]);
        }
#else
        double dC[4];
        dC[0] = pdDerivate[0][iIdx];
        dC[1] = k * pdDerivate[0][iIdx] + j * pdDerivate[1][iIdx];
        dC[2] = pdDerivate[1][iIdx];
        dC[3] = j * pdDerivate[0][iIdx] - k * pdDerivate[1][iIdx];

        for ( int col=0; col<4; col++ )
        {
          for ( int row=0; row<4; row++ )
          {
            pdEqualCoeff[col+1][row] += dC[col] * dC[row];
          }
          pdEqualCoeff[col+1][4] += (double)( piError[iIdx] * dC[col] );
        }
#endif
      }
    }
#endif

#if JVET_K0185_AFFINE_6PARA_ENC
    double dAffinePara[6];
    double dDeltaMv[6];
    Mv acDeltaMv[3];

    solveEqual( pdEqualCoeff, affineParaNum, dAffinePara );

    // convert to delta mv
    dDeltaMv[0] = dAffinePara[0];
    dDeltaMv[2] = dAffinePara[2];
    if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
    {
      dDeltaMv[1] = dAffinePara[1] * width + dAffinePara[0];
      dDeltaMv[3] = dAffinePara[3] * width + dAffinePara[2];
      dDeltaMv[4] = dAffinePara[4] * height + dAffinePara[0];
      dDeltaMv[5] = dAffinePara[5] * height + dAffinePara[2];
    }
    else
    {
      dDeltaMv[1] = dAffinePara[1] * width + dAffinePara[0];
      dDeltaMv[3] = -dAffinePara[3] * width + dAffinePara[2];
    }

    acDeltaMv[0] = Mv( (int)(dDeltaMv[0] * 4 + SIGN( dDeltaMv[0] ) * 0.5) << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE, (int)(dDeltaMv[2] * 4 + SIGN( dDeltaMv[2] ) * 0.5) << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE, true );
    acDeltaMv[1] = Mv( (int)(dDeltaMv[1] * 4 + SIGN( dDeltaMv[1] ) * 0.5) << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE, (int)(dDeltaMv[3] * 4 + SIGN( dDeltaMv[3] ) * 0.5) << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE, true );

    if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
    {
      acDeltaMv[2] = Mv( (int)(dDeltaMv[4] * 4 + SIGN( dDeltaMv[4] ) * 0.5) << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE, (int)(dDeltaMv[5] * 4 + SIGN( dDeltaMv[5] ) * 0.5) << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE, true );
    }
#else
    double dAffinePara[4];
    solveEqual( pdEqualCoeff, 4, dAffinePara );

    // convert to delta mv
    double dDeltaMv[4];
    dDeltaMv[0] = dAffinePara[0];
    dDeltaMv[2] = dAffinePara[2];

    dDeltaMv[1] =   dAffinePara[1] * width + dAffinePara[0];
    dDeltaMv[3] = - dAffinePara[3] * width + dAffinePara[2];

    Mv acDeltaMv[3];
    acDeltaMv[0] = Mv( (int)(dDeltaMv[0] * 4 + SIGN(dDeltaMv[0]) * 0.5 ) << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE, (int)(dDeltaMv[2] * 4 + SIGN(dDeltaMv[2]) * 0.5 ) << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE, true );
    acDeltaMv[1] = Mv( (int)(dDeltaMv[1] * 4 + SIGN(dDeltaMv[1]) * 0.5 ) << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE, (int)(dDeltaMv[3] * 4 + SIGN(dDeltaMv[3]) * 0.5 ) << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE, true );
#endif

    bool bAllZero = false;
#if JVET_K0185_AFFINE_6PARA_ENC
    for ( int i = 0; i < mvNum; i++ )
#else
    for ( int i=0; i<2; i++ )
#endif
    {
      if ( acDeltaMv[i].getHor() != 0 || acDeltaMv[i].getVer() != 0 )
      {
        bAllZero = false;
        break;
      }
      bAllZero = true;
    }

    if ( bAllZero )
      break;

    // do motion compensation with updated mv
#if JVET_K0185_AFFINE_6PARA_ENC
    for ( int i = 0; i < mvNum; i++ )
#else
    for ( int i=0; i<2; i++ )
#endif
    {
      acMvTemp[i] += acDeltaMv[i];
#if JVET_K_AFFINE_BUG_FIXES
      acMvTemp[i].hor = Clip3( -32768, 32767, acMvTemp[i].hor );
      acMvTemp[i].ver = Clip3( -32768, 32767, acMvTemp[i].ver );
      acMvTemp[i].roundMV2SignalPrecision();
#endif
      clipMv(acMvTemp[i], pu.cu->lumaPos(), *pu.cs->sps);
    }
#if !JVET_K_AFFINE_BUG_FIXES
    int vx2, vy2;
    vx2 =  - ( acMvTemp[1].getVer() - acMvTemp[0].getVer() ) * height / width + acMvTemp[0].getHor();
    vy2 =    ( acMvTemp[1].getHor() - acMvTemp[0].getHor() ) * height / width + acMvTemp[0].getVer();
    acMvTemp[2].set( vx2, vy2 );
    clipMv(acMvTemp[2], pu.cu->lumaPos(), *pu.cs->sps);
#endif
    xPredAffineBlk( COMPONENT_Y, pu, refPic, acMvTemp, predBuf, false, pu.cu->slice->clpRng( COMPONENT_Y ) );

    // get error
    Distortion uiCostTemp = m_pcRdCost->getDistPart( predBuf.Y(), pBuf->Y(), pu.cs->sps->getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, DF_HAD );
    DTRACE( g_trace_ctx, D_COMMON, " (%d) uiCostTemp=%d\n", DTRACE_GET_COUNTER(g_trace_ctx,D_COMMON), uiCostTemp );

    // get cost with mv
    m_pcRdCost->setCostScale(0);
    uint32_t uiBitsTemp = ruiBits;
#if JVET_K0185_AFFINE_6PARA_ENC
    for ( int i = 0; i < mvNum; i++ )
#else
    for ( int i=0; i<2; i++ )
#endif
    {
      m_pcRdCost->setPredictor( acMvPred[i] );
      const int shift = acMvTemp[i].highPrec ? VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE : 0;
#if JVET_K0337_AFFINE_MVD_PREDICTION
      Mv secondPred;
      if ( i != 0 )
      {
        secondPred = acMvPred[i] + (acMvTemp[0] - acMvPred[0]);
        m_pcRdCost->setPredictor( secondPred );
      }
#endif
#if JVET_K0357_AMVR
      uiBitsTemp += m_pcRdCost->getBitsOfVectorWithPredictor( acMvTemp[i].getHor()>>shift, acMvTemp[i].getVer()>>shift, 0 );
#else
      uiBitsTemp += m_pcRdCost->getBitsOfVectorWithPredictor( acMvTemp[i].getHor() >> shift, acMvTemp[i].getVer() >> shift );
#endif
    }

    uiCostTemp = (Distortion)( floor( fWeight * (double)uiCostTemp ) + (double)m_pcRdCost->getCost( uiBitsTemp ) );

    // store best cost and mv
    if ( uiCostTemp < uiCostBest )
    {
      uiCostBest = uiCostTemp;
      uiBitsBest = uiBitsTemp;
      memcpy( acMv, acMvTemp, sizeof(Mv) * 3 );
    }
  }

  // free buffer
  for ( int i=0; i<iParaNum; i++ )
    delete []pdEqualCoeff[i];
  delete []pdEqualCoeff;

  ruiBits = uiBitsBest;
  ruiCost = uiCostBest;
  DTRACE( g_trace_ctx, D_COMMON, " (%d) uiBitsBest=%d, uiCostBest=%d\n", DTRACE_GET_COUNTER(g_trace_ctx,D_COMMON), uiBitsBest, uiCostBest );

}

void InterSearch::xEstimateAffineAMVP( PredictionUnit&  pu,
                                       AffineAMVPInfo&  affineAMVPInfo,
                                       PelUnitBuf&      origBuf,
                                       RefPicList       eRefPicList,
                                       int              iRefIdx,
                                       Mv               acMvPred[3],
                                       Distortion*      puiDistBiP )
{
  Mv         bestMvLT, bestMvRT, bestMvLB;
  int        iBestIdx = 0;
  Distortion uiBestCost = std::numeric_limits<Distortion>::max();

  // Fill the MV Candidates
  PU::fillAffineMvpCand( pu, eRefPicList, iRefIdx, affineAMVPInfo );
  CHECK( affineAMVPInfo.numCand == 0, "Assertion failed." );

  PelUnitBuf predBuf = m_tmpStorageLCU.getBuf( UnitAreaRelative(*pu.cu, pu) );

  // initialize Mvp index & Mvp
  iBestIdx = 0;
  for( int i = 0 ; i < affineAMVPInfo.numCand; i++ )
  {
    Mv mv[3] = { affineAMVPInfo.mvCandLT[i], affineAMVPInfo.mvCandRT[i], affineAMVPInfo.mvCandLB[i] };
    Mv mvTrace[3] = { affineAMVPInfo.mvCandLT[i], affineAMVPInfo.mvCandRT[i], affineAMVPInfo.mvCandLB[i] };
    mvTrace[0].setHighPrec();
    mvTrace[1].setHighPrec();
    mvTrace[2].setHighPrec();

    Distortion uiTmpCost = xGetAffineTemplateCost( pu, origBuf, predBuf, mv, i, AMVP_MAX_NUM_CANDS, eRefPicList, iRefIdx );

    if ( uiBestCost > uiTmpCost )
    {
      uiBestCost = uiTmpCost;
      bestMvLT = affineAMVPInfo.mvCandLT[i];
      bestMvRT = affineAMVPInfo.mvCandRT[i];
      bestMvLB = affineAMVPInfo.mvCandLB[i];
      iBestIdx  = i;
      *puiDistBiP = uiTmpCost;
    }
  }

  // Setting Best MVP
  acMvPred[0] = bestMvLT;
  acMvPred[1] = bestMvRT;
  acMvPred[2] = bestMvLB;

  pu.mvpIdx[eRefPicList] = iBestIdx;
  pu.mvpNum[eRefPicList] = affineAMVPInfo.numCand;
  DTRACE( g_trace_ctx, D_COMMON, "#estAffi=%d \n", affineAMVPInfo.numCand );
}

void InterSearch::xCopyAffineAMVPInfo (AffineAMVPInfo& src, AffineAMVPInfo& dst)
{
  dst.numCand = src.numCand;
  DTRACE( g_trace_ctx, D_COMMON, " (%d) #copyAffi=%d \n", DTRACE_GET_COUNTER( g_trace_ctx, D_COMMON ), src.numCand );
  ::memcpy( dst.mvCandLT, src.mvCandLT, sizeof(Mv)*src.numCand );
  ::memcpy( dst.mvCandRT, src.mvCandRT, sizeof(Mv)*src.numCand );
  ::memcpy( dst.mvCandLB, src.mvCandLB, sizeof(Mv)*src.numCand );
}
#endif


/**
* \brief Generate half-sample interpolated block
*
* \param pattern Reference picture ROI
* \param biPred    Flag indicating whether block is for biprediction
*/
void InterSearch::xExtDIFUpSamplingH( CPelBuf* pattern )
{
  const ClpRng& clpRng = m_lumaClpRng;
  int width      = pattern->width;
  int height     = pattern->height;
  int srcStride  = pattern->stride;

  int intStride = width + 1;
  int dstStride = width + 1;
  Pel *intPtr;
  Pel *dstPtr;
  int filterSize = NTAPS_LUMA;
  int halfFilterSize = (filterSize>>1);
  const Pel *srcPtr = pattern->buf - halfFilterSize*srcStride - 1;

  const ChromaFormat chFmt = m_currChromaFormat;

#if JVET_K0346 || JVET_K_AFFINE
  m_if.filterHor(COMPONENT_Y, srcPtr, srcStride, m_filteredBlockTmp[0][0], intStride, width + 1, height + filterSize, 0 << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE, false, chFmt, clpRng);
  m_if.filterHor(COMPONENT_Y, srcPtr, srcStride, m_filteredBlockTmp[2][0], intStride, width + 1, height + filterSize, 2 << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE, false, chFmt, clpRng);

  intPtr = m_filteredBlockTmp[0][0] + halfFilterSize * intStride + 1;
  dstPtr = m_filteredBlock[0][0][0];
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width + 0, height + 0, 0 << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE, false, true, chFmt, clpRng);

  intPtr = m_filteredBlockTmp[0][0] + (halfFilterSize - 1) * intStride + 1;
  dstPtr = m_filteredBlock[2][0][0];
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width + 0, height + 1, 2 << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE, false, true, chFmt, clpRng);

  intPtr = m_filteredBlockTmp[2][0] + halfFilterSize * intStride;
  dstPtr = m_filteredBlock[0][2][0];
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width + 1, height + 0, 0 << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE, false, true, chFmt, clpRng);

  intPtr = m_filteredBlockTmp[2][0] + (halfFilterSize - 1) * intStride;
  dstPtr = m_filteredBlock[2][2][0];
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width + 1, height + 1, 2 << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE, false, true, chFmt, clpRng);
#else
  m_if.filterHor(COMPONENT_Y, srcPtr, srcStride, m_filteredBlockTmp[0][0], intStride, width + 1, height + filterSize, 0, false, chFmt, clpRng);
  m_if.filterHor(COMPONENT_Y, srcPtr, srcStride, m_filteredBlockTmp[2][0], intStride, width + 1, height + filterSize, 2, false, chFmt, clpRng);

  intPtr = m_filteredBlockTmp[0][0] + halfFilterSize * intStride + 1;
  dstPtr = m_filteredBlock[0][0][0];
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width + 0, height + 0, 0, false, true, chFmt, clpRng);

  intPtr = m_filteredBlockTmp[0][0] + (halfFilterSize - 1) * intStride + 1;
  dstPtr = m_filteredBlock[2][0][0];
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width + 0, height + 1, 2, false, true, chFmt, clpRng);

  intPtr = m_filteredBlockTmp[2][0] + halfFilterSize * intStride;
  dstPtr = m_filteredBlock[0][2][0];
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width + 1, height + 0, 0, false, true, chFmt, clpRng);

  intPtr = m_filteredBlockTmp[2][0] + (halfFilterSize - 1) * intStride;
  dstPtr = m_filteredBlock[2][2][0];
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width + 1, height + 1, 2, false, true, chFmt, clpRng);
#endif
}





/**
* \brief Generate quarter-sample interpolated blocks
*
* \param pattern    Reference picture ROI
* \param halfPelRef Half-pel mv
* \param biPred     Flag indicating whether block is for biprediction
*/
void InterSearch::xExtDIFUpSamplingQ( CPelBuf* pattern, Mv halfPelRef )
{
  const ClpRng& clpRng = m_lumaClpRng;
  int width      = pattern->width;
  int height     = pattern->height;
  int srcStride  = pattern->stride;

  Pel const* srcPtr;
  int intStride = width + 1;
  int dstStride = width + 1;
  Pel *intPtr;
  Pel *dstPtr;
  int filterSize = NTAPS_LUMA;

  int halfFilterSize = (filterSize>>1);

  int extHeight = (halfPelRef.getVer() == 0) ? height + filterSize : height + filterSize-1;

  const ChromaFormat chFmt = m_currChromaFormat;

  // Horizontal filter 1/4
  srcPtr = pattern->buf - halfFilterSize * srcStride - 1;
  intPtr = m_filteredBlockTmp[1][0];
  if (halfPelRef.getVer() > 0)
  {
    srcPtr += srcStride;
  }
  if (halfPelRef.getHor() >= 0)
  {
    srcPtr += 1;
  }
#if JVET_K0346 || JVET_K_AFFINE
  m_if.filterHor(COMPONENT_Y, srcPtr, srcStride, intPtr, intStride, width, extHeight, 1 << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE, false, chFmt, clpRng);
#else
  m_if.filterHor(COMPONENT_Y, srcPtr, srcStride, intPtr, intStride, width, extHeight, 1, false, chFmt, clpRng);
#endif

  // Horizontal filter 3/4
  srcPtr = pattern->buf - halfFilterSize*srcStride - 1;
  intPtr = m_filteredBlockTmp[3][0];
  if (halfPelRef.getVer() > 0)
  {
    srcPtr += srcStride;
  }
  if (halfPelRef.getHor() > 0)
  {
    srcPtr += 1;
  }
#if JVET_K0346 || JVET_K_AFFINE
  m_if.filterHor(COMPONENT_Y, srcPtr, srcStride, intPtr, intStride, width, extHeight, 3 << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE, false, chFmt, clpRng);
#else
  m_if.filterHor(COMPONENT_Y, srcPtr, srcStride, intPtr, intStride, width, extHeight, 3, false, chFmt, clpRng);
#endif

  // Generate @ 1,1
  intPtr = m_filteredBlockTmp[1][0] + (halfFilterSize-1) * intStride;
  dstPtr = m_filteredBlock[1][1][0];
  if (halfPelRef.getVer() == 0)
  {
    intPtr += intStride;
  }
#if JVET_K0346 || JVET_K_AFFINE
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 1 << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE, false, true, chFmt, clpRng);
#else
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 1, false, true, chFmt, clpRng);
#endif

  // Generate @ 3,1
  intPtr = m_filteredBlockTmp[1][0] + (halfFilterSize-1) * intStride;
  dstPtr = m_filteredBlock[3][1][0];
#if JVET_K0346 || JVET_K_AFFINE
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 3 << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE, false, true, chFmt, clpRng);
#else
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 3, false, true, chFmt, clpRng);
#endif

  if (halfPelRef.getVer() != 0)
  {
    // Generate @ 2,1
    intPtr = m_filteredBlockTmp[1][0] + (halfFilterSize - 1) * intStride;
    dstPtr = m_filteredBlock[2][1][0];
    if (halfPelRef.getVer() == 0)
    {
      intPtr += intStride;
    }
#if JVET_K0346 || JVET_K_AFFINE
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 2 << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE, false, true, chFmt, clpRng);
#else
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 2, false, true, chFmt, clpRng);
#endif

    // Generate @ 2,3
    intPtr = m_filteredBlockTmp[3][0] + (halfFilterSize - 1) * intStride;
    dstPtr = m_filteredBlock[2][3][0];
    if (halfPelRef.getVer() == 0)
    {
      intPtr += intStride;
    }
#if JVET_K0346 || JVET_K_AFFINE
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 2 << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE, false, true, chFmt, clpRng);
#else
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 2, false, true, chFmt, clpRng);
#endif
  }
  else
  {
    // Generate @ 0,1
    intPtr = m_filteredBlockTmp[1][0] + halfFilterSize * intStride;
    dstPtr = m_filteredBlock[0][1][0];
#if JVET_K0346 || JVET_K_AFFINE
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 0 << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE, false, true, chFmt, clpRng);
#else
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 0, false, true, chFmt, clpRng);
#endif

    // Generate @ 0,3
    intPtr = m_filteredBlockTmp[3][0] + halfFilterSize * intStride;
    dstPtr = m_filteredBlock[0][3][0];
#if JVET_K0346 || JVET_K_AFFINE
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 0 << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE, false, true, chFmt, clpRng);
#else
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 0, false, true, chFmt, clpRng);
#endif
  }

  if (halfPelRef.getHor() != 0)
  {
    // Generate @ 1,2
    intPtr = m_filteredBlockTmp[2][0] + (halfFilterSize - 1) * intStride;
    dstPtr = m_filteredBlock[1][2][0];
    if (halfPelRef.getHor() > 0)
    {
      intPtr += 1;
    }
    if (halfPelRef.getVer() >= 0)
    {
      intPtr += intStride;
    }
#if JVET_K0346 || JVET_K_AFFINE
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 1 << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE, false, true, chFmt, clpRng);
#else
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 1, false, true, chFmt, clpRng);
#endif

    // Generate @ 3,2
    intPtr = m_filteredBlockTmp[2][0] + (halfFilterSize - 1) * intStride;
    dstPtr = m_filteredBlock[3][2][0];
    if (halfPelRef.getHor() > 0)
    {
      intPtr += 1;
    }
    if (halfPelRef.getVer() > 0)
    {
      intPtr += intStride;
    }
#if JVET_K0346 || JVET_K_AFFINE
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 3 << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE, false, true, chFmt, clpRng);
#else
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 3, false, true, chFmt, clpRng);
#endif
  }
  else
  {
    // Generate @ 1,0
    intPtr = m_filteredBlockTmp[0][0] + (halfFilterSize - 1) * intStride + 1;
    dstPtr = m_filteredBlock[1][0][0];
    if (halfPelRef.getVer() >= 0)
    {
      intPtr += intStride;
    }
#if JVET_K0346 || JVET_K_AFFINE
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 1 << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE, false, true, chFmt, clpRng);
#else
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 1, false, true, chFmt, clpRng);
#endif

    // Generate @ 3,0
    intPtr = m_filteredBlockTmp[0][0] + (halfFilterSize - 1) * intStride + 1;
    dstPtr = m_filteredBlock[3][0][0];
    if (halfPelRef.getVer() > 0)
    {
      intPtr += intStride;
    }
#if JVET_K0346 || JVET_K_AFFINE
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 3 << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE, false, true, chFmt, clpRng);
#else
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 3, false, true, chFmt, clpRng);
#endif
  }

  // Generate @ 1,3
  intPtr = m_filteredBlockTmp[3][0] + (halfFilterSize - 1) * intStride;
  dstPtr = m_filteredBlock[1][3][0];
  if (halfPelRef.getVer() == 0)
  {
    intPtr += intStride;
  }
#if JVET_K0346 || JVET_K_AFFINE
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 1 << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE, false, true, chFmt, clpRng);
#else
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 1, false, true, chFmt, clpRng);
#endif

  // Generate @ 3,3
  intPtr = m_filteredBlockTmp[3][0] + (halfFilterSize - 1) * intStride;
  dstPtr = m_filteredBlock[3][3][0];
#if JVET_K0346 || JVET_K_AFFINE
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 3 << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE, false, true, chFmt, clpRng);
#else
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 3, false, true, chFmt, clpRng);
#endif
}





//! set wp tables
void InterSearch::setWpScalingDistParam( int iRefIdx, RefPicList eRefPicListCur, Slice *pcSlice )
{
  if ( iRefIdx<0 )
  {
    m_cDistParam.applyWeight = false;
    return;
  }

  WPScalingParam  *wp0 , *wp1;

  m_cDistParam.applyWeight = ( pcSlice->getSliceType()==P_SLICE && pcSlice->testWeightPred() ) || ( pcSlice->getSliceType()==B_SLICE && pcSlice->testWeightBiPred() ) ;

  if ( !m_cDistParam.applyWeight )
  {
    return;
  }

  int iRefIdx0 = ( eRefPicListCur == REF_PIC_LIST_0 ) ? iRefIdx : (-1);
  int iRefIdx1 = ( eRefPicListCur == REF_PIC_LIST_1 ) ? iRefIdx : (-1);

  getWpScaling( pcSlice, iRefIdx0, iRefIdx1, wp0 , wp1 );

  if ( iRefIdx0 < 0 )
  {
    wp0 = NULL;
  }
  if ( iRefIdx1 < 0 )
  {
    wp1 = NULL;
  }

  m_cDistParam.wpCur  = NULL;

  if ( eRefPicListCur == REF_PIC_LIST_0 )
  {
    m_cDistParam.wpCur = wp0;
  }
  else
  {
    m_cDistParam.wpCur = wp1;
  }
}

void InterSearch::xEncodeInterResidualQT(CodingStructure &cs, Partitioner &partitioner, const ComponentID &compID)
{
  const UnitArea& currArea    = partitioner.currArea();
  const TransformUnit &currTU = *cs.getTU(currArea.lumaPos(), partitioner.chType);
  const CodingUnit &cu        = *currTU.cu;
#if ENABLE_BMS
  const unsigned currDepth    = partitioner.currTrDepth;

  const bool bSubdiv          = currDepth != currTU.depth;
#endif

  if (compID == MAX_NUM_TBLOCKS)  // we are not processing a channel, instead we always recurse and code the CBFs
  {
#if ENABLE_BMS
    if( cs.pcv->noRQT )
    {
#if ENABLE_BMS
      if( partitioner.canSplit( TU_MAX_TR_SPLIT, cs ) )
      {
        CHECK( !bSubdiv, "Not performing the implicit TU split" );
      }
      else
#endif
      CHECK( bSubdiv, "transformsplit not supported" );
    }
#endif
    CHECK(CU::isIntra(cu), "Inter search provided with intra CU");

    if( cu.chromaFormat != CHROMA_400 )
    {
#if ENABLE_BMS
      const bool firstCbfOfCU = ( currDepth == 0 );
      {
        if( firstCbfOfCU || TU::getCbfAtDepth( currTU, COMPONENT_Cb, currDepth - 1 ) )
        {
          const bool  chroma_cbf = TU::getCbfAtDepth( currTU, COMPONENT_Cb, currDepth );
          m_CABACEstimator->cbf_comp( cs, chroma_cbf, currArea.blocks[COMPONENT_Cb], currDepth );
        }
        if( firstCbfOfCU || TU::getCbfAtDepth( currTU, COMPONENT_Cr, currDepth - 1 ) )
        {
          const bool  chroma_cbf = TU::getCbfAtDepth( currTU, COMPONENT_Cr, currDepth );
#if JVET_K0072
          m_CABACEstimator->cbf_comp( cs, chroma_cbf, currArea.blocks[COMPONENT_Cr], currDepth, TU::getCbfAtDepth( currTU, COMPONENT_Cb, currDepth ) );
#else
          m_CABACEstimator->cbf_comp( cs, chroma_cbf, currArea.blocks[COMPONENT_Cr], currDepth );
#endif
        }
      }
    }

    if( !bSubdiv )
    {
      m_CABACEstimator->cbf_comp( cs, TU::getCbfAtDepth( currTU, COMPONENT_Y, currDepth ), currArea.Y(), currDepth );
    }
#else
      m_CABACEstimator->cbf_comp( cs, TU::getCbf( currTU, COMPONENT_Cb ), currArea.blocks[COMPONENT_Cb] );
#if JVET_K0072
      m_CABACEstimator->cbf_comp( cs, TU::getCbf( currTU, COMPONENT_Cr ), currArea.blocks[COMPONENT_Cr], TU::getCbf( currTU, COMPONENT_Cb ) );
#else
      m_CABACEstimator->cbf_comp( cs, TU::getCbf( currTU, COMPONENT_Cr ), currArea.blocks[COMPONENT_Cr] );
#endif
    }

    m_CABACEstimator->cbf_comp( cs, TU::getCbf( currTU, COMPONENT_Y ), currArea.Y() );
#endif
  }

#if ENABLE_BMS
  if (!bSubdiv)
#endif
  {
    if (compID != MAX_NUM_TBLOCKS) // we have already coded the CBFs, so now we code coefficients
    {
      if( currArea.blocks[compID].valid() )
      {
        if( TU::hasCrossCompPredInfo( currTU, compID ) )
        {
          m_CABACEstimator->cross_comp_pred( currTU, compID );
        }
        if( TU::getCbf( currTU, compID ) )
        {
          m_CABACEstimator->residual_coding( currTU, compID );
        }
      }
    }
  }
#if ENABLE_BMS
  else
  {
    if( compID == MAX_NUM_TBLOCKS || TU::getCbfAtDepth( currTU, compID, currDepth ) )
    {
#if ENABLE_BMS
      if( partitioner.canSplit( TU_MAX_TR_SPLIT, cs ) )
      {
        partitioner.splitCurrArea( TU_MAX_TR_SPLIT, cs );
      }
      else
#endif
        THROW( "Implicit TU split not available!" );

      do
      {
        xEncodeInterResidualQT( cs, partitioner, compID );
      } while( partitioner.nextPart( cs ) );

      partitioner.exitCurrSplit();
    }
  }
#endif
}

void InterSearch::xEstimateInterResidualQT(CodingStructure &cs, Partitioner &partitioner, Distortion *puiZeroDist /*= NULL*/)
{
  const UnitArea& currArea = partitioner.currArea();
  const SPS &sps           = *cs.sps;
  const PPS &pps           = *cs.pps;
  const uint32_t numValidComp  = getNumberValidComponents( sps.getChromaFormatIdc() );
  const uint32_t numTBlocks    = getNumberValidTBlocks   ( *cs.pcv );
#if ENABLE_BMS
  const unsigned currDepth = partitioner.currTrDepth;

  bool bCheckSplit = false, bCheckFull = false;
#if ENABLE_BMS
  if( cs.pcv->noRQT )
  {
    bCheckFull  = !partitioner.canSplit( TU_MAX_TR_SPLIT, cs );
    bCheckSplit = !bCheckFull;
  }
#endif

  // get temporary data
  CodingStructure *csSplit = nullptr;
  CodingStructure *csFull  = nullptr;
  if (bCheckSplit)
  {
    csSplit = &cs;
  }
  else if (bCheckFull)
  {
    csFull = &cs;
  }
#else
  bool bCheckFull = true;
  CodingStructure *csFull = &cs;
#endif

  Distortion uiSingleDist         = 0;
  Distortion uiSingleDistComp [3] = { 0, 0, 0 };
  TCoeff     uiAbsSum         [3] = { 0, 0, 0 };

  const TempCtx ctxStart  ( m_CtxCache, m_CABACEstimator->getCtx() );
  TempCtx       ctxBest   ( m_CtxCache );

  if (bCheckFull)
  {
    TransformUnit &tu = csFull->addTU(currArea, partitioner.chType);
#if ENABLE_BMS
    tu.depth          = currDepth;
#endif
#if JVET_K1000_SIMPLIFIED_EMT
    tu.emtIdx         = 0;
#endif

    double minCost            [MAX_NUM_TBLOCKS];
    bool   checkTransformSkip [MAX_NUM_TBLOCKS];

    m_CABACEstimator->resetBits();

    memset(m_pTempPel, 0, sizeof(Pel) * tu.Y().area()); // not necessary needed for inside of recursion (only at the beginning)

    for (uint32_t i = 0; i < numTBlocks; i++)
    {
      minCost[i] = MAX_DOUBLE;
    }

    CodingStructure &saveCS = *m_pSaveCS[0];
    saveCS.pcv     = cs.pcv;
    saveCS.picture = cs.picture;
    saveCS.area.repositionTo(currArea);
    saveCS.clearTUs();

    TransformUnit &bestTU = saveCS.addTU( currArea, partitioner.chType );


    for( uint32_t c = 0; c < numTBlocks; c++ )
    {
      const ComponentID compID    = ComponentID(c);
      const CompArea&   compArea  = tu.blocks[compID];
      const int channelBitDepth   = sps.getBitDepth(toChannelType(compID));

      checkTransformSkip[compID]  = false;

      if( !tu.blocks[compID].valid() )
      {
        continue;
      }

      checkTransformSkip[compID] = pps.getUseTransformSkip() && TU::hasTransformSkipFlag( *tu.cs, tu.blocks[compID] ) && !cs.isLossless;
#if JVET_K1000_SIMPLIFIED_EMT
      if( isLuma(compID) )
      {
        checkTransformSkip[compID]  &= !tu.cu->emtFlag;
      }
#endif

      const bool isCrossCPredictionAvailable = TU::hasCrossCompPredInfo( tu, compID );

      int8_t preCalcAlpha = 0;
      const CPelBuf lumaResi = csFull->getResiBuf(tu.Y());

      if (isCrossCPredictionAvailable)
      {
        csFull->getResiBuf( compArea ).copyFrom( cs.getOrgResiBuf( compArea ) );
        preCalcAlpha = xCalcCrossComponentPredictionAlpha( tu, compID, m_pcEncCfg->getUseReconBasedCrossCPredictionEstimate() );
      }

      const int crossCPredictionModesToTest = preCalcAlpha != 0 ? 2 : 1;
#if JVET_K1000_SIMPLIFIED_EMT
      const int numEmtTransformCandidates   = isLuma(compID) && tu.cu->emtFlag && sps.getSpsNext().getUseInterEMT() ? 4 : 1;
      const int numTransformCandidates      = checkTransformSkip[compID] ? ( numEmtTransformCandidates + 1 ) : numEmtTransformCandidates;
#else
      const int numTransformCandidates      = checkTransformSkip[compID] ? 2 : 1;
#endif
      int lastTransformModeIndex            = numTransformCandidates - 1; //lastTransformModeIndex is the mode for transformSkip (if transformSkip is active)
      const bool isOneMode                  = crossCPredictionModesToTest == 1 && numTransformCandidates == 1;

      bool isLastBest = isOneMode;
      for( int transformMode = 0; transformMode < numTransformCandidates; transformMode++ )
      {
        for( int crossCPredictionModeId = 0; crossCPredictionModeId < crossCPredictionModesToTest; crossCPredictionModeId++ )
        {
          const bool isFirstMode  = transformMode == 0 && crossCPredictionModeId == 0;
          const bool isLastMode   = ( transformMode + 1 ) == numTransformCandidates && ( crossCPredictionModeId + 1 ) == crossCPredictionModesToTest;
          const bool bUseCrossCPrediction = crossCPredictionModeId != 0;

          // copy the original residual into the residual buffer
          csFull->getResiBuf(compArea).copyFrom(cs.getOrgResiBuf(compArea));

          m_CABACEstimator->getCtx() = ctxStart;
          m_CABACEstimator->resetBits();

#if JVET_K1000_SIMPLIFIED_EMT
          if( isLuma( compID ) ) tu.emtIdx = transformMode;
#endif
          tu.transformSkip[compID]  = checkTransformSkip[compID] && transformMode == lastTransformModeIndex;
          tu.compAlpha[compID]      = bUseCrossCPrediction ? preCalcAlpha : 0;

          const QpParam cQP(tu, compID);  // note: uses tu.transformSkip[compID]

#if RDOQ_CHROMA_LAMBDA
          m_pcTrQuant->selectLambda(compID);
#endif

          TCoeff     currAbsSum = 0;
          uint64_t   currCompFracBits = 0;
          Distortion currCompDist = 0;
          double     currCompCost = 0;
          uint64_t   nonCoeffFracBits = 0;
          Distortion nonCoeffDist = 0;
          double     nonCoeffCost = 0;

          if (bUseCrossCPrediction)
          {
            PelBuf resiBuf = csFull->getResiBuf( compArea );
            crossComponentPrediction( tu, compID, lumaResi, resiBuf, resiBuf, false );
          }

          m_pcTrQuant->transformNxN(tu, compID, cQP, currAbsSum, m_CABACEstimator->getCtx());

          if (isFirstMode || (currAbsSum == 0))
          {
            const CPelBuf zeroBuf(m_pTempPel, compArea);
            const CPelBuf orgResi = csFull->getOrgResiBuf( compArea );

            if (bUseCrossCPrediction)
            {
              PelBuf resi = csFull->getResiBuf( compArea );
              crossComponentPrediction( tu, compID, lumaResi, zeroBuf, resi, true );
              nonCoeffDist = m_pcRdCost->getDistPart( orgResi, resi, channelBitDepth, compID, DF_SSE );
            }
            else
            {
              nonCoeffDist = m_pcRdCost->getDistPart( zeroBuf, orgResi, channelBitDepth, compID, DF_SSE ); // initialized with zero residual distortion
            }

#if JVET_K0072
            const bool prevCbf = ( compID == COMPONENT_Cr ? tu.cbf[COMPONENT_Cb] : false );
#if ENABLE_BMS
            m_CABACEstimator->cbf_comp( *csFull, false, compArea, currDepth, prevCbf );
#else
            m_CABACEstimator->cbf_comp( *csFull, false, compArea, prevCbf );
#endif
#else
#if ENABLE_BMS
            m_CABACEstimator->cbf_comp( *csFull, false, compArea, currDepth );
#else
            m_CABACEstimator->cbf_comp( *csFull, false, compArea );
#endif
#endif

            if( isCrossCPredictionAvailable )
            {
              m_CABACEstimator->cross_comp_pred( tu, compID );
            }

            nonCoeffFracBits = m_CABACEstimator->getEstFracBits();
#if WCG_EXT
            if( m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() )
            {
              nonCoeffCost   = m_pcRdCost->calcRdCost(nonCoeffFracBits, nonCoeffDist, false);
            }
            else
#endif
            nonCoeffCost     = m_pcRdCost->calcRdCost(nonCoeffFracBits, nonCoeffDist);
          }

          if ((puiZeroDist != NULL) && isFirstMode)
          {
            *puiZeroDist += nonCoeffDist; // initialized with zero residual distortion
          }

          if (currAbsSum > 0) //if non-zero coefficients are present, a residual needs to be derived for further prediction
          {
            if (isFirstMode)
            {
              m_CABACEstimator->getCtx() = ctxStart;
              m_CABACEstimator->resetBits();
            }

#if JVET_K0072
            const bool prevCbf = ( compID == COMPONENT_Cr ? tu.cbf[COMPONENT_Cb] : false );
#if ENABLE_BMS
            m_CABACEstimator->cbf_comp( *csFull, true, compArea, currDepth, prevCbf );
#else
            m_CABACEstimator->cbf_comp( *csFull, true, compArea, prevCbf );
#endif
#else
#if ENABLE_BMS
            m_CABACEstimator->cbf_comp( *csFull, true, compArea, currDepth );
#else
            m_CABACEstimator->cbf_comp( *csFull, true, compArea );
#endif
#endif

            if( isCrossCPredictionAvailable )
            {
              m_CABACEstimator->cross_comp_pred( tu, compID );
            }
            m_CABACEstimator->residual_coding( tu, compID );

            currCompFracBits = m_CABACEstimator->getEstFracBits();

            PelBuf resiBuf     = csFull->getResiBuf(compArea);
            CPelBuf orgResiBuf = csFull->getOrgResiBuf(compArea);

            m_pcTrQuant->invTransformNxN(tu, compID, resiBuf, cQP);

            if (bUseCrossCPrediction)
            {
              crossComponentPrediction( tu, compID, lumaResi, resiBuf, resiBuf, true );
            }

            currCompDist = m_pcRdCost->getDistPart(orgResiBuf, resiBuf, channelBitDepth, compID, DF_SSE);
            
#if WCG_EXT
            currCompCost = m_pcRdCost->calcRdCost(currCompFracBits, currCompDist, false);
#else
            currCompCost = m_pcRdCost->calcRdCost(currCompFracBits, currCompDist);
#endif

            if (csFull->isLossless)
            {
              nonCoeffCost = MAX_DOUBLE;
            }
          }
          else if( ( transformMode == lastTransformModeIndex ) && checkTransformSkip[compID] && !bUseCrossCPrediction )
          {
            currCompCost = MAX_DOUBLE;
          }
          else
          {
            currCompFracBits = nonCoeffFracBits;
            currCompDist     = nonCoeffDist;
            currCompCost     = nonCoeffCost;

            tu.cbf[compID] = 0;
          }

          // evaluate
          if( ( currCompCost < minCost[compID] ) || ( transformMode == lastTransformModeIndex && checkTransformSkip[compID] && currCompCost == minCost[compID] ) )
          {
            // copy component
            if (isFirstMode && ((nonCoeffCost < currCompCost) || (currAbsSum == 0))) // check for forced null
            {
              tu.getCoeffs( compID ).fill( 0 );
              csFull->getResiBuf( compArea ).fill( 0 );
              tu.cbf[compID]   = 0;

              currAbsSum       = 0;
              currCompFracBits = nonCoeffFracBits;
              currCompDist     = nonCoeffDist;
              currCompCost     = nonCoeffCost;
            }

            uiAbsSum[compID]         = currAbsSum;
            uiSingleDistComp[compID] = currCompDist;
            minCost[compID]          = currCompCost;

            if (uiAbsSum[compID] == 0)
            {
              if (bUseCrossCPrediction)
              {
                const CPelBuf zeroBuf( m_pTempPel, compArea );
                PelBuf resiBuf = csFull->getResiBuf( compArea );

                crossComponentPrediction( tu, compID, lumaResi, zeroBuf, resiBuf, true );
              }
            }

            if( !isLastMode )
            {
              bestTU.copyComponentFrom( tu, compID );
              saveCS.getResiBuf( compArea ).copyFrom( csFull->getResiBuf( compArea ) );
            }

            isLastBest = isLastMode;
          }
        }
      }

      if( !isLastBest )
      {
        // copy component
        tu.copyComponentFrom( bestTU, compID );
        csFull->getResiBuf( compArea ).copyFrom( saveCS.getResiBuf( compArea ) );
      }
    } // component loop

    m_CABACEstimator->getCtx() = ctxStart;
    m_CABACEstimator->resetBits();

    static const ComponentID cbf_getComp[3] = { COMPONENT_Cb, COMPONENT_Cr, COMPONENT_Y };
    for( unsigned c = 0; c < numTBlocks; c++)
    {
      const ComponentID compID = cbf_getComp[c];
      if( tu.blocks[compID].valid() )
      {
#if ENABLE_BMS
#if JVET_K0072
        const bool prevCbf = ( compID == COMPONENT_Cr ? TU::getCbfAtDepth( tu, COMPONENT_Cb, currDepth ) : false );
        m_CABACEstimator->cbf_comp( *csFull, TU::getCbfAtDepth( tu, compID, currDepth ), tu.blocks[compID], currDepth, prevCbf );
#else
        m_CABACEstimator->cbf_comp( *csFull, TU::getCbfAtDepth( tu, compID, currDepth ), tu.blocks[compID], currDepth );
#endif
#else
#if JVET_K0072
        const bool prevCbf = ( compID == COMPONENT_Cr ? TU::getCbf( tu, COMPONENT_Cb ) : false );
        m_CABACEstimator->cbf_comp( *csFull, TU::getCbf( tu, compID ), tu.blocks[compID], prevCbf );
#else
        m_CABACEstimator->cbf_comp( *csFull, TU::getCbf( tu, compID ), tu.blocks[compID] );
#endif
#endif
      }
    }

    for (uint32_t ch = 0; ch < numValidComp; ch++)
    {
      const ComponentID compID = ComponentID(ch);

      if (tu.blocks[compID].valid())
      {
        if( cs.pps->getPpsRangeExtension().getCrossComponentPredictionEnabledFlag() && isChroma(compID) && uiAbsSum[COMPONENT_Y] )
        {
          m_CABACEstimator->cross_comp_pred( tu, compID );
        }
        if( TU::getCbf( tu, compID ) )
        {
          m_CABACEstimator->residual_coding( tu, compID );
        }
        uiSingleDist += uiSingleDistComp[compID];
      }
    }

    csFull->fracBits += m_CABACEstimator->getEstFracBits();
    csFull->dist     += uiSingleDist;
#if WCG_EXT
    if( m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() )
    {
      csFull->cost    = m_pcRdCost->calcRdCost(csFull->fracBits, csFull->dist, false);
    }
    else
#endif
    csFull->cost      = m_pcRdCost->calcRdCost(csFull->fracBits, csFull->dist);
  } // check full
#if ENABLE_BMS

  // code sub-blocks
  if( bCheckSplit )
  {
    if( bCheckFull )
    {
      m_CABACEstimator->getCtx() = ctxStart;
    }

#if ENABLE_BMS
    if( partitioner.canSplit( TU_MAX_TR_SPLIT, cs ) )
    {
      partitioner.splitCurrArea( TU_MAX_TR_SPLIT, cs );
    }
    else
#endif
      THROW( "Implicit TU split not available!" );

    do
    {
      xEstimateInterResidualQT(*csSplit, partitioner, bCheckFull ? nullptr : puiZeroDist);

      csSplit->cost = m_pcRdCost->calcRdCost( csSplit->fracBits, csSplit->dist );
#if JVET_K1000_SIMPLIFIED_EMT
      if( csFull && csSplit->cost >= csFull->cost && m_pcEncCfg->getFastInterEMT() )
      {
        break;
      }
#endif
    } while( partitioner.nextPart( *csSplit ) );

    partitioner.exitCurrSplit();

    unsigned        anyCbfSet   =   0;
    unsigned        compCbf[3]  = { 0, 0, 0 };

#if JVET_K1000_SIMPLIFIED_EMT
    bool isSplit = bCheckFull ? false : true;
    if( !bCheckFull || ( csSplit->cost < csFull->cost && m_pcEncCfg->getFastInterEMT() ) || !m_pcEncCfg->getFastInterEMT() )
#else
    if( !bCheckFull )
#endif
    {
      for( auto &currTU : csSplit->traverseTUs( currArea, partitioner.chType ) )
      {
        for( unsigned ch = 0; ch < numTBlocks; ch++ )
        {
          compCbf[ ch ] |= ( TU::getCbfAtDepth( currTU, ComponentID(ch), currDepth + 1 ) ? 1 : 0 );
        }
      }

      {

        for( auto &currTU : csSplit->traverseTUs( currArea, partitioner.chType ) )
        {
          TU::setCbfAtDepth   ( currTU, COMPONENT_Y,  currDepth, compCbf[ COMPONENT_Y  ] );
          if( currArea.chromaFormat != CHROMA_400 )
          {
            TU::setCbfAtDepth ( currTU, COMPONENT_Cb, currDepth, compCbf[ COMPONENT_Cb ] );
            TU::setCbfAtDepth ( currTU, COMPONENT_Cr, currDepth, compCbf[ COMPONENT_Cr ] );
          }
        }

        anyCbfSet    = compCbf[ COMPONENT_Y  ];
        if( currArea.chromaFormat != CHROMA_400 )
        {
          anyCbfSet |= compCbf[ COMPONENT_Cb ];
          anyCbfSet |= compCbf[ COMPONENT_Cr ];
        }
      }

      m_CABACEstimator->getCtx() = ctxStart;
      m_CABACEstimator->resetBits();

      // when compID isn't a channel, code Cbfs:
      xEncodeInterResidualQT( *csSplit, partitioner, MAX_NUM_TBLOCKS );
      for (uint32_t ch = 0; ch < numValidComp; ch++)
      {
        xEncodeInterResidualQT( *csSplit, partitioner, ComponentID( ch ) );
      }

      csSplit->fracBits = m_CABACEstimator->getEstFracBits();
      csSplit->cost     = m_pcRdCost->calcRdCost(csSplit->fracBits, csSplit->dist);

      if( bCheckFull && anyCbfSet && csSplit->cost < csFull->cost )
      {
        cs.useSubStructure( *csSplit, partitioner.chType, currArea, false, false, false, true );
        cs.cost = csSplit->cost;
      }
    }

#if JVET_K1000_SIMPLIFIED_EMT
    if( ( !isSplit && m_pcEncCfg->getFastInterEMT() ) || ( !m_pcEncCfg->getFastInterEMT() && !( !bCheckFull || ( anyCbfSet && csSplit->cost < csFull->cost ) ) ) )
#else
    if( !( !bCheckFull || ( anyCbfSet && csSplit->cost < csFull->cost ) ) )
#endif
    {
      CHECK( !bCheckFull, "Error!" );
      cs.useSubStructure( *csFull, partitioner.chType, currArea, false, false, false, true );
      cs.cost = csFull->cost;
      m_CABACEstimator->getCtx() = ctxBest;
    }

    if( csSplit && csFull )
    {
      csSplit->releaseIntermediateData();
      csFull ->releaseIntermediateData();
    }
  }
#endif
}

void InterSearch::encodeResAndCalcRdInterCU(CodingStructure &cs, Partitioner &partitioner, const bool &skipResidual)
{
  CodingUnit &cu = *cs.getCU( partitioner.chType );

  const ChromaFormat format     = cs.area.chromaFormat;;
  const int  numValidComponents = getNumberValidComponents(format);
  const SPS &sps                = *cs.sps;
  const PPS &pps                = *cs.pps;

  if( skipResidual ) //  No residual coding : SKIP mode
  {
    cu.skip    = true;
    cu.rootCbf = false;
    cs.getResiBuf().fill(0);
    {
      cs.getRecoBuf().copyFrom(cs.getPredBuf() );
    }


    // add an empty TU
    cs.addTU(cs.area, partitioner.chType);

    Distortion distortion = 0;

    for (int comp = 0; comp < numValidComponents; comp++)
    {
      const ComponentID compID = ComponentID(comp);

      CPelBuf reco = cs.getRecoBuf (compID);
      CPelBuf org  = cs.getOrgBuf  (compID);
#if WCG_EXT
      if( m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() )
      {
        const CPelBuf orgLuma = cs.getOrgBuf( cs.area.blocks[COMPONENT_Y] );
        distortion += m_pcRdCost->getDistPart( org, reco, sps.getBitDepth( toChannelType( compID ) ), compID, DF_SSE_WTD, &orgLuma );
      }
      else
#endif
      distortion += m_pcRdCost->getDistPart( org, reco, sps.getBitDepth( toChannelType( compID ) ), compID, DF_SSE );
    }

    m_CABACEstimator->resetBits();

    if( pps.getTransquantBypassEnabledFlag() )
    {
      m_CABACEstimator->cu_transquant_bypass_flag( cu );
    }

    PredictionUnit &pu = *cs.getPU( partitioner.chType );

    m_CABACEstimator->cu_skip_flag  ( cu );
#if JVET_K_AFFINE
    m_CABACEstimator->affine_flag( cu );
#endif
    m_CABACEstimator->merge_idx     ( pu );


    cs.dist     = distortion;
    cs.fracBits = m_CABACEstimator->getEstFracBits();
    cs.cost     = m_pcRdCost->calcRdCost(cs.fracBits, cs.dist);

    return;
  }

  //  Residual coding.
  cs.getResiBuf().copyFrom (cs.getOrgBuf());
  cs.getResiBuf().subtract (cs.getPredBuf());

  Distortion zeroDistortion = 0;

  const TempCtx ctxStart( m_CtxCache, m_CABACEstimator->getCtx() );

  cs.getOrgResiBuf().copyFrom(cs.getResiBuf());

  xEstimateInterResidualQT(cs, partitioner, &zeroDistortion);

  TransformUnit &firstTU = *cs.getTU( partitioner.chType );

  cu.rootCbf = false;
  m_CABACEstimator->resetBits();
  m_CABACEstimator->rqt_root_cbf( cu );
  const uint64_t  zeroFracBits = m_CABACEstimator->getEstFracBits();
  double zeroCost;
  {
#if WCG_EXT
    if( m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() )
    {
      zeroCost = cs.isLossless ? ( cs.cost + 1 ) : m_pcRdCost->calcRdCost( zeroFracBits, zeroDistortion, false );
    }
    else
#endif
    zeroCost = cs.isLossless ? ( cs.cost + 1 ) : m_pcRdCost->calcRdCost( zeroFracBits, zeroDistortion );
  }

  const int  numValidTBlocks   = ::getNumberValidTBlocks( *cs.pcv );
  for (uint32_t i = 0; i < numValidTBlocks; i++)
  {
    cu.rootCbf |= TU::getCbf( firstTU, ComponentID( i ) );
  }

  // -------------------------------------------------------
  // If a block full of 0's is efficient, then just use 0's.
  // The costs at this point do not include header bits.

  if (zeroCost < cs.cost || !cu.rootCbf)
  {
    cu.rootCbf = false;

    cs.clearTUs();

    // add a new "empty" TU spanning the whole CU
    TransformUnit& tu = cs.addTU(cu, partitioner.chType);

    for (int comp = 0; comp < numValidComponents; comp++)
    {
      tu.rdpcm[comp] = RDPCM_OFF;
    }
    cu.firstTU = cu.lastTU = &tu;
  }


  // all decisions now made. Fully encode the CU, including the headers:
  m_CABACEstimator->getCtx() = ctxStart;

  uint64_t finalFracBits = xGetSymbolFracBitsInter( cs, partitioner );
  // we've now encoded the CU, and so have a valid bit cost
  if (!cu.rootCbf)
  {
    cs.getResiBuf().fill(0); // Clear the residual image, if we didn't code it.
  }

  cs.getRecoBuf().reconstruct(cs.getPredBuf(), cs.getResiBuf(), cs.slice->clpRngs());

  // update with clipped distortion and cost (previously unclipped reconstruction values were used)
  Distortion finalDistortion = 0;

  for (int comp = 0; comp < numValidComponents; comp++)
  {
    const ComponentID compID = ComponentID(comp);

    CPelBuf reco = cs.getRecoBuf (compID);
    CPelBuf org  = cs.getOrgBuf  (compID);

#if WCG_EXT
    if( m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() )
    {
      const CPelBuf orgLuma = cs.getOrgBuf( cs.area.blocks[COMPONENT_Y] );
      finalDistortion += m_pcRdCost->getDistPart( org, reco, sps.getBitDepth( toChannelType( compID ) ), compID, DF_SSE_WTD, &orgLuma );
    }
    else
#endif
    {
      finalDistortion += m_pcRdCost->getDistPart( org, reco, sps.getBitDepth( toChannelType( compID ) ), compID, DF_SSE );
    }
  }

  cs.dist     = finalDistortion;
  cs.fracBits = finalFracBits;
  cs.cost     = m_pcRdCost->calcRdCost(cs.fracBits, cs.dist);

  CHECK(cs.tus.size() == 0, "No TUs present");
}

uint64_t InterSearch::xGetSymbolFracBitsInter(CodingStructure &cs, Partitioner &partitioner)
{
  uint64_t fracBits   = 0;
  CodingUnit &cu    = *cs.getCU( partitioner.chType );

  m_CABACEstimator->resetBits();

  if( cu.partSize == SIZE_2Nx2N && cu.firstPU->mergeFlag && !cu.rootCbf )
  {
    cu.skip = true;

    if( cs.pps->getTransquantBypassEnabledFlag() )
    {
      m_CABACEstimator->cu_transquant_bypass_flag( cu );
    }

    m_CABACEstimator->cu_skip_flag  ( cu );
#if JVET_K_AFFINE
    m_CABACEstimator->affine_flag   ( cu );
#endif
    m_CABACEstimator->merge_idx     ( *cu.firstPU );
    fracBits   += m_CABACEstimator->getEstFracBits();
  }
  else
  {
    CHECK( cu.skip, "Skip flag has to be off at this point!" );

    if( cs.pps->getTransquantBypassEnabledFlag() )
    {
      m_CABACEstimator->cu_transquant_bypass_flag( cu );
    }

    m_CABACEstimator->cu_skip_flag( cu );
    m_CABACEstimator->pred_mode   ( cu );
    m_CABACEstimator->cu_pred_data( cu );
    CUCtx cuCtx;
    cuCtx.isDQPCoded = true;
    cuCtx.isChromaQpAdjCoded = true;
    m_CABACEstimator->cu_residual ( cu, partitioner, cuCtx );
    fracBits       += m_CABACEstimator->getEstFracBits();
  }

  return fracBits;
}



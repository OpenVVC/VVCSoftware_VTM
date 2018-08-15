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

/** \file     LoopFilter.cpp
    \brief    deblocking filter
*/

#include "LoopFilter.h"
#include "Slice.h"
#include "Mv.h"
#include "Unit.h"
#include "UnitTools.h"
#include "UnitPartitioner.h"
#include "dtrace_codingstruct.h"
#include "dtrace_buffer.h"

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Constants
// ====================================================================================================================

//#define   EDGE_VER    0
//#define   EDGE_HOR    1

#define DEBLOCK_SMALLEST_BLOCK  8


#define DEFAULT_INTRA_TC_OFFSET 2 ///< Default intra TC offset

// ====================================================================================================================
// Tables
// ====================================================================================================================

const uint8_t LoopFilter::sm_tcTable[MAX_QP + 1 + DEFAULT_INTRA_TC_OFFSET] =
{
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,2,2,2,2,3,3,3,3,4,4,4,5,5,6,6,7,8,9,10,11,13,14,16,18,20,22,24
#if JVET_K0251_QP_EXT
  , 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48
#endif
};

const uint8_t LoopFilter::sm_betaTable[MAX_QP + 1] =
{
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,6,7,8,9,10,11,12,13,14,15,16,17,18,20,22,24,26,28,30,32,34,36,38,40,42,44,46,48,50,52,54,56,58,60,62,64
#if JVET_K0251_QP_EXT
  , 66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 86, 88
#endif
};

inline static uint32_t getRasterIdx(const Position& pos, const PreCalcValues& pcv)
{
  return ( ( pos.x & pcv.maxCUWidthMask ) >> pcv.minCUWidthLog2 ) + ( ( pos.y & pcv.maxCUHeightMask ) >> pcv.minCUHeightLog2 ) * pcv.partsInCtuWidth;
}

// ====================================================================================================================
// utility functions
// ====================================================================================================================

#if HEVC_TILES_WPP
static bool isAvailableLeft( const CodingUnit& cu, const CodingUnit& cu2, const bool bEnforceSliceRestriction, const bool bEnforceTileRestriction )
{
  return ( ( !bEnforceSliceRestriction || CU::isSameSlice( cu, cu2 ) ) && ( !bEnforceTileRestriction || CU::isSameTile( cu, cu2 ) ) );
}

static bool isAvailableAbove( const CodingUnit& cu, const CodingUnit& cu2, const bool bEnforceSliceRestriction, const bool bEnforceTileRestriction )
{
  return ( !bEnforceSliceRestriction || CU::isSameSlice( cu, cu2 ) ) && ( !bEnforceTileRestriction || CU::isSameTile( cu, cu2 ) );
}
#else
static bool isAvailable( const CodingUnit& cu, const CodingUnit& cu2, const bool bEnforceSliceRestriction )
{
  return ( !bEnforceSliceRestriction || CU::isSameSlice( cu, cu2 ) );
}
#endif


// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

LoopFilter::LoopFilter()
{
}

LoopFilter::~LoopFilter()
{
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================
void LoopFilter::create( const unsigned uiMaxCUDepth )
{
  destroy();
  const unsigned numPartitions = 1 << ( uiMaxCUDepth << 1 );
  for( int edgeDir = 0; edgeDir < NUM_EDGE_DIR; edgeDir++ )
  {
    m_aapucBS       [edgeDir].resize( numPartitions );
    m_aapbEdgeFilter[edgeDir].resize( numPartitions );
  }
}

void LoopFilter::destroy()
{
  for( int edgeDir = 0; edgeDir < NUM_EDGE_DIR; edgeDir++ )
  {
    m_aapucBS       [edgeDir].clear();
    m_aapbEdgeFilter[edgeDir].clear();
  }
}

/**
 - call deblocking function for every CU
 .
 \param  pcPic   picture class (Pic) pointer
 */
void LoopFilter::loopFilterPic( CodingStructure& cs
                                )
{
  const PreCalcValues& pcv = *cs.pcv;

  DTRACE_UPDATE( g_trace_ctx, ( std::make_pair( "poc", cs.slice->getPOC() ) ) );
#if ENABLE_TRACING
  for( int y = 0; y < pcv.heightInCtus; y++ )
  {
    for( int x = 0; x < pcv.widthInCtus; x++ )
    {
      const UnitArea ctuArea( pcv.chrFormat, Area( x << pcv.maxCUWidthLog2, y << pcv.maxCUHeightLog2, pcv.maxCUWidth, pcv.maxCUWidth ) );
      DTRACE    ( g_trace_ctx, D_CRC, "CTU %d %d", ctuArea.Y().x, ctuArea.Y().y );
      DTRACE_CRC( g_trace_ctx, D_CRC, cs, cs.picture->getRecoBuf( clipArea( ctuArea, *cs.picture ) ), &ctuArea.Y() );
    }
  }
#endif

  for( int y = 0; y < pcv.heightInCtus; y++ )
  {
    for( int x = 0; x < pcv.widthInCtus; x++ )
    {
      memset( m_aapucBS       [EDGE_VER].data(), 0,     m_aapucBS       [EDGE_VER].byte_size() );
      memset( m_aapbEdgeFilter[EDGE_VER].data(), false, m_aapbEdgeFilter[EDGE_VER].byte_size() );

      const UnitArea ctuArea( pcv.chrFormat, Area( x << pcv.maxCUWidthLog2, y << pcv.maxCUHeightLog2, pcv.maxCUWidth, pcv.maxCUWidth ) );

      // CU-based deblocking
      for( auto &currCU : cs.traverseCUs( CS::getArea( cs, ctuArea, CH_L ), CH_L ) )
      {
        xDeblockCU( currCU, EDGE_VER );
      }

      if( CS::isDualITree( cs ) )
      {
        memset( m_aapucBS       [EDGE_VER].data(), 0,     m_aapucBS       [EDGE_VER].byte_size() );
        memset( m_aapbEdgeFilter[EDGE_VER].data(), false, m_aapbEdgeFilter[EDGE_VER].byte_size() );

        for( auto &currCU : cs.traverseCUs( CS::getArea( cs, ctuArea, CH_C ), CH_C ) )
        {
          xDeblockCU( currCU, EDGE_VER );
        }
      }
    }
  }

  // Vertical filtering
  for( int y = 0; y < pcv.heightInCtus; y++ )
  {
    for( int x = 0; x < pcv.widthInCtus; x++ )
    {
      memset( m_aapucBS       [EDGE_HOR].data(), 0,     m_aapucBS       [EDGE_HOR].byte_size() );
      memset( m_aapbEdgeFilter[EDGE_HOR].data(), false, m_aapbEdgeFilter[EDGE_HOR].byte_size() );

      const UnitArea ctuArea( pcv.chrFormat, Area( x << pcv.maxCUWidthLog2, y << pcv.maxCUHeightLog2, pcv.maxCUWidth, pcv.maxCUWidth ) );

      // CU-based deblocking
      for( auto &currCU : cs.traverseCUs( CS::getArea( cs, ctuArea, CH_L ), CH_L ) )
      {
        xDeblockCU( currCU, EDGE_HOR );
      }

      if( CS::isDualITree( cs ) )
      {
        memset( m_aapucBS       [EDGE_HOR].data(), 0,     m_aapucBS       [EDGE_HOR].byte_size() );
        memset( m_aapbEdgeFilter[EDGE_HOR].data(), false, m_aapbEdgeFilter[EDGE_HOR].byte_size() );

        for( auto &currCU : cs.traverseCUs( CS::getArea( cs, ctuArea, CH_C ), CH_C ) )
        {
          xDeblockCU( currCU, EDGE_HOR );
        }
      }
    }
  }

  DTRACE_PIC_COMP(D_REC_CB_LUMA_LF,   cs, cs.getRecoBuf(), COMPONENT_Y);
  DTRACE_PIC_COMP(D_REC_CB_CHROMA_LF, cs, cs.getRecoBuf(), COMPONENT_Cb);
  DTRACE_PIC_COMP(D_REC_CB_CHROMA_LF, cs, cs.getRecoBuf(), COMPONENT_Cr);

  DTRACE    ( g_trace_ctx, D_CRC, "LoopFilter" );
  DTRACE_CRC( g_trace_ctx, D_CRC, cs, cs.getRecoBuf() );
}


// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

/**
 Deblocking filter process in CU-based (the same function as conventional's)

 \param cu               the CU to be deblocked
 \param edgeDir          the direction of the edge in block boundary (horizontal/vertical), which is added newly
*/
void LoopFilter::xDeblockCU( CodingUnit& cu, const DeblockEdgeDir edgeDir )
{
  const PreCalcValues& pcv = *cu.cs->pcv;
  const Area area          = cu.Y().valid() ? cu.Y() : Area( recalcPosition( cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, cu.blocks[cu.chType].pos() ), recalcSize( cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, cu.blocks[cu.chType].size() ) );

  xSetLoopfilterParam( cu );

  for( auto &currTU : CU::traverseTUs( cu ) )
  {
    const Area& areaTu    = cu.Y().valid() ? currTU.block( COMPONENT_Y ) : area;
    xSetEdgefilterMultiple( cu, EDGE_VER, areaTu, m_stLFCUParam.internalEdge );
    xSetEdgefilterMultiple( cu, EDGE_HOR, areaTu, m_stLFCUParam.internalEdge );
  }

  for( auto &currPU : CU::traversePUs( cu ) )
  {
    const Area& areaPu = cu.Y().valid() ? currPU.block( COMPONENT_Y ) : area;
    const bool xOff    = currPU.blocks[cu.chType].x != cu.blocks[cu.chType].x;
    const bool yOff    = currPU.blocks[cu.chType].y != cu.blocks[cu.chType].y;

    xSetEdgefilterMultiple( cu, EDGE_VER, areaPu, (xOff ? m_stLFCUParam.internalEdge : m_stLFCUParam.leftEdge), xOff );
    xSetEdgefilterMultiple( cu, EDGE_HOR, areaPu, (yOff ? m_stLFCUParam.internalEdge : m_stLFCUParam.topEdge),  yOff );

  }

#if JVET_K_AFFINE
  if ( cu.affine )
  {
    const int widthInBaseUnits = cu.Y().width >> pcv.minCUWidthLog2;
    for( uint32_t edgeIdx = 1 ; edgeIdx < widthInBaseUnits ; edgeIdx++ )
    {
      const Area affiBlockV( cu.Y().x + edgeIdx * pcv.minCUWidth, cu.Y().y, pcv.minCUWidth, cu.Y().height );
      xSetEdgefilterMultiple( cu, EDGE_VER, affiBlockV, m_stLFCUParam.internalEdge, 1 );
    }
    const int heightInBaseUnits = cu.Y().height >> pcv.minCUHeightLog2;
    for( uint32_t edgeIdx = 1 ; edgeIdx < heightInBaseUnits ; edgeIdx++ )
    {
      const Area affiBlockH( cu.Y().x, cu.Y().y + edgeIdx * pcv.minCUHeight, cu.Y().width, pcv.minCUHeight );
      xSetEdgefilterMultiple( cu, EDGE_HOR, affiBlockH, m_stLFCUParam.internalEdge, 1 );
    }
  }
#endif
  const unsigned uiPelsInPart = pcv.minCUWidth;

  for( int y = 0; y < area.height; y += uiPelsInPart )
  {
    for( int x = 0; x < area.width; x += uiPelsInPart )
    {
      unsigned uiBSCheck = 1;
      if( !cu.cs->pcv->noRQT && uiPelsInPart == 4 )
      {
        uiBSCheck = ( ( edgeDir == EDGE_VER ) && ( x % 8 == 0 ) ) || ( ( edgeDir == EDGE_HOR ) && ( y % 8 == 0 ) );
      }
      const Position localPos  { area.x + x, area.y + y };
      const unsigned rasterIdx = getRasterIdx( localPos, pcv );

      if( m_aapbEdgeFilter[edgeDir][rasterIdx] && uiBSCheck )
      {
        m_aapucBS[edgeDir][rasterIdx] = xGetBoundaryStrengthSingle( cu, edgeDir, localPos );
      }
    }
  }

#if DB_TU_FIX==0
  const unsigned PartIdxIncr  = ( cu.cs->pcv->noRQT && cu.cs->pcv->only2Nx2N ? 1 : ( DEBLOCK_SMALLEST_BLOCK / uiPelsInPart ? DEBLOCK_SMALLEST_BLOCK / uiPelsInPart : 1 ) );
  const unsigned uiSizeInPU   = ( cu.cs->pcv->noRQT && cu.cs->pcv->only2Nx2N ? 1 : pcv.partsInCtuWidth >> cu.qtDepth );
#endif
  const unsigned shiftFactor  = edgeDir == EDGE_VER ? ::getComponentScaleX( COMPONENT_Cb, pcv.chrFormat ) : ::getComponentScaleY( COMPONENT_Cb, pcv.chrFormat );
  const bool bAlwaysDoChroma  = pcv.chrFormat == CHROMA_444 || pcv.noRQT;

#if DEBLOCKING_GRID_8x8
  if (edgeDir == EDGE_HOR)
  {
    if (!((cu.block(COMPONENT_Y).y % 8) == 0))
      return;
  }
  else
  {
    if (!((cu.block(COMPONENT_Y).x % 8) == 0))
      return;
  }
#endif

#if DB_TU_FIX
  unsigned int orthogonalLength = 1;
  unsigned int orthogonalIncrement = 1;

  if (cu.blocks[COMPONENT_Y].valid())
  {
    if ((cu.blocks[COMPONENT_Y].height > 64) && (edgeDir == EDGE_HOR))
    {
      orthogonalIncrement = 64 / 4;
      orthogonalLength = cu.blocks[COMPONENT_Y].height / 4;
    }
    if ((cu.blocks[COMPONENT_Y].width > 64) && (edgeDir == EDGE_VER))
    {
      orthogonalIncrement = 64 / 4;
      orthogonalLength = cu.blocks[COMPONENT_Y].width / 4;

    }
  }
  for (int edge = 0; edge < orthogonalLength; edge += orthogonalIncrement)
  {
    if (cu.blocks[COMPONENT_Y].valid())
    {
      xEdgeFilterLuma(cu, edgeDir, edge);
    }
    if (cu.blocks[COMPONENT_Cb].valid() && pcv.chrFormat != CHROMA_400 && (bAlwaysDoChroma || (uiPelsInPart > DEBLOCK_SMALLEST_BLOCK) || (edge % ((DEBLOCK_SMALLEST_BLOCK << shiftFactor) / uiPelsInPart)) == 0))
    {
      xEdgeFilterChroma(cu, edgeDir, edge);
    }
  }
#else
  for( int iEdge = 0; iEdge < uiSizeInPU; iEdge += PartIdxIncr )
  {
    if( cu.blocks[COMPONENT_Y].valid() )
    {
      xEdgeFilterLuma  ( cu, edgeDir, iEdge );
    }

    if( cu.blocks[COMPONENT_Cb].valid() && pcv.chrFormat != CHROMA_400 && ( bAlwaysDoChroma || ( uiPelsInPart > DEBLOCK_SMALLEST_BLOCK ) || ( iEdge % ( ( DEBLOCK_SMALLEST_BLOCK << shiftFactor ) / uiPelsInPart ) ) == 0 ) )
    {
      xEdgeFilterChroma( cu, edgeDir, iEdge );
    }
  }
#endif
}


void LoopFilter::xSetEdgefilterMultiple( const CodingUnit&    cu,
                                         const DeblockEdgeDir edgeDir,
                                         const Area&          area,
                                         const bool           bValue,
                                         const bool           EdgeIdx )
{
  const PreCalcValues& pcv = *cu.cs->pcv;

  const unsigned uiAdd     = ( edgeDir == EDGE_VER ) ? pcv.partsInCtuWidth : 1;
  const unsigned uiNumElem = ( edgeDir == EDGE_VER ) ? ( area.height / pcv.minCUHeight ) : ( area.width / pcv.minCUWidth );
  unsigned uiBsIdx         = getRasterIdx( area, pcv );

  for( int ui = 0; ui < uiNumElem; ui++ )
  {
    m_aapbEdgeFilter[edgeDir][uiBsIdx] = bValue;
    if( ! EdgeIdx )
    {
      m_aapucBS[edgeDir][uiBsIdx] = bValue;
    }
    uiBsIdx += uiAdd;
  }
}
void LoopFilter::xSetLoopfilterParam( const CodingUnit& cu )
{
  const Slice& slice = *cu.slice;
#if HEVC_TILES_WPP
  const PPS&   pps   = *cu.cs->pps;
#endif

  if( slice.getDeblockingFilterDisable() )
  {
    m_stLFCUParam.leftEdge = m_stLFCUParam.topEdge = m_stLFCUParam.internalEdge = false;
    return;
  }

  const Position& pos = cu.blocks[cu.chType].pos();

  m_stLFCUParam.internalEdge = true;
#if HEVC_TILES_WPP
  m_stLFCUParam.leftEdge     = ( 0 < pos.x ) && isAvailableLeft ( cu, *cu.cs->getCU( pos.offset( -1,  0 ), cu.chType ), !slice.getLFCrossSliceBoundaryFlag(), !pps.getLoopFilterAcrossTilesEnabledFlag() );
  m_stLFCUParam.topEdge      = ( 0 < pos.y ) && isAvailableAbove( cu, *cu.cs->getCU( pos.offset(  0, -1 ), cu.chType ), !slice.getLFCrossSliceBoundaryFlag(), !pps.getLoopFilterAcrossTilesEnabledFlag() );
#else
  m_stLFCUParam.leftEdge     = ( 0 < pos.x ) && isAvailable ( cu, *cu.cs->getCU( pos.offset( -1,  0 ), cu.chType ), !slice.getLFCrossSliceBoundaryFlag());
  m_stLFCUParam.topEdge      = ( 0 < pos.y ) && isAvailable ( cu, *cu.cs->getCU( pos.offset(  0, -1 ), cu.chType ), !slice.getLFCrossSliceBoundaryFlag());
#endif
}

unsigned LoopFilter::xGetBoundaryStrengthSingle ( const CodingUnit& cu, const DeblockEdgeDir edgeDir, const Position& localPos ) const
{
  const Slice& sliceQ = *cu.slice;

  const Position& cuPosLuma = cu.lumaPos();

  const Position& posQ  = localPos;
  const Position  posP  = ( edgeDir == EDGE_VER ) ? posQ.offset( -1, 0 ) : posQ.offset( 0, -1 );

  const bool sameCU     = posP.x >= cuPosLuma.x && posP.y >= cuPosLuma.y;

  const CodingUnit& cuQ = cu;
  const CodingUnit& cuP = sameCU ? cu : *cu.cs->getCU( posP, cu.chType );

  //-- Set BS for Intra MB : BS = 4 or 3
  if( ( MODE_INTRA == cuP.predMode ) || ( MODE_INTRA == cuQ.predMode ) )
  {
    return 2;
  }

  const TransformUnit& tuQ = posQ == cuQ.lumaPos() ? *cuQ.firstTU : *cuQ.cs->getTU(posQ, cuQ.chType);
  const TransformUnit& tuP = posP == cuP.lumaPos() ? *cuP.firstTU : *cuP.cs->getTU(posP, cuP.chType);
  const PreCalcValues& pcv = *cu.cs->pcv;
  const unsigned rasterIdx = getRasterIdx( posQ, pcv );

  //-- Set BS for not Intra MB : BS = 2 or 1 or 0
  if (m_aapucBS[edgeDir][rasterIdx] && (TU::getCbf(tuQ, COMPONENT_Y) || TU::getCbf(tuP, COMPONENT_Y)))
  {
    return 1;
  }

  // and now the pred
  const MotionInfo&     miQ = cuQ.cs->getMotionInfo( posQ );
  const MotionInfo&     miP = cuP.cs->getMotionInfo( posP );
  const Slice&       sliceP = *cuP.slice;

  if (sliceQ.isInterB() || sliceP.isInterB())
  {
    const Picture *piRefP0 = ( 0 > miP.refIdx[0] ) ? NULL : sliceP.getRefPic( REF_PIC_LIST_0, miP.refIdx[0] );
    const Picture *piRefP1 = ( 0 > miP.refIdx[1] ) ? NULL : sliceP.getRefPic( REF_PIC_LIST_1, miP.refIdx[1] );
    const Picture *piRefQ0 = ( 0 > miQ.refIdx[0] ) ? NULL : sliceQ.getRefPic( REF_PIC_LIST_0, miQ.refIdx[0] );
    const Picture *piRefQ1 = ( 0 > miQ.refIdx[1] ) ? NULL : sliceQ.getRefPic( REF_PIC_LIST_1, miQ.refIdx[1] );

    Mv mvP0, mvP1, mvQ0, mvQ1;

    if( 0 <= miP.refIdx[0] ) { mvP0 = miP.mv[0]; }
    if( 0 <= miP.refIdx[1] ) { mvP1 = miP.mv[1]; }
    if( 0 <= miQ.refIdx[0] ) { mvQ0 = miQ.mv[0]; }
    if( 0 <= miQ.refIdx[1] ) { mvQ1 = miQ.mv[1]; }

    int nThreshold = 4;
#if JVET_K0346 || JVET_K_AFFINE
    if( cu.cs->sps->getSpsNext().getUseHighPrecMv() )
    {
      mvP0.setHighPrec();
      mvP1.setHighPrec();
      mvQ0.setHighPrec();
      mvQ1.setHighPrec();
      nThreshold = 4 << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;
    }
#endif
    unsigned uiBs = 0;

    //th can be optimized
    if ( ((piRefP0==piRefQ0)&&(piRefP1==piRefQ1)) || ((piRefP0==piRefQ1)&&(piRefP1==piRefQ0)) )
    {
      if ( piRefP0 != piRefP1 )   // Different L0 & L1
      {
        if ( piRefP0 == piRefQ0 )
        {
          uiBs  = ((abs(mvQ0.getHor() - mvP0.getHor()) >= nThreshold) || (abs(mvQ0.getVer() - mvP0.getVer()) >= nThreshold) ||
                   (abs(mvQ1.getHor() - mvP1.getHor()) >= nThreshold) || (abs(mvQ1.getVer() - mvP1.getVer()) >= nThreshold))
                  ? 1 : 0;
        }
        else
        {
          uiBs  = ((abs(mvQ1.getHor() - mvP0.getHor()) >= nThreshold) || (abs(mvQ1.getVer() - mvP0.getVer()) >= nThreshold) ||
                   (abs(mvQ0.getHor() - mvP1.getHor()) >= nThreshold) || (abs(mvQ0.getVer() - mvP1.getVer()) >= nThreshold))
                  ? 1 : 0;
        }
      }
      else    // Same L0 & L1
      {
        uiBs  = ((abs(mvQ0.getHor() - mvP0.getHor()) >= nThreshold) || (abs(mvQ0.getVer() - mvP0.getVer()) >= nThreshold) ||
                 (abs(mvQ1.getHor() - mvP1.getHor()) >= nThreshold) || (abs(mvQ1.getVer() - mvP1.getVer()) >= nThreshold))
              &&
                ((abs(mvQ1.getHor() - mvP0.getHor()) >= nThreshold) || (abs(mvQ1.getVer() - mvP0.getVer()) >= nThreshold) ||
                 (abs(mvQ0.getHor() - mvP1.getHor()) >= nThreshold) || (abs(mvQ0.getVer() - mvP1.getVer()) >= nThreshold))
              ? 1 : 0;
      }
    }
    else // for all different Ref_Idx
    {
      uiBs = 1;
    }
    return uiBs;
  }


  // pcSlice->isInterP()
  CHECK(0 > miP.refIdx[0], "Invalid reference picture list index");
  CHECK(0 > miQ.refIdx[0], "Invalid reference picture list index");
  const Picture *piRefP0 = sliceP.getRefPic(REF_PIC_LIST_0, miP.refIdx[0]);
  const Picture *piRefQ0 = sliceQ.getRefPic(REF_PIC_LIST_0, miQ.refIdx[0]);

  if (piRefP0 != piRefQ0)
  {
    return 1;
  }

  Mv mvP0 = miP.mv[0];
  Mv mvQ0 = miQ.mv[0];

  int nThreshold = 4;
#if JVET_K0346 || JVET_K_AFFINE
  if( cu.cs->sps->getSpsNext().getUseHighPrecMv() )
  {
    mvP0.setHighPrec();
    mvQ0.setHighPrec();
    nThreshold = 4 << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;
  }
#endif
  return ( ( abs( mvQ0.getHor() - mvP0.getHor() ) >= nThreshold ) || ( abs( mvQ0.getVer() - mvP0.getVer() ) >= nThreshold ) ) ? 1 : 0;
}

void LoopFilter::xEdgeFilterLuma(const CodingUnit& cu, const DeblockEdgeDir edgeDir, const int iEdge)
{
  const CompArea&  lumaArea = cu.block(COMPONENT_Y);
  const PreCalcValues& pcv = *cu.cs->pcv;

  PelBuf        picYuvRec = cu.cs->getRecoBuf( lumaArea );
  Pel           *piSrc    = picYuvRec.buf;
  const int     iStride   = picYuvRec.stride;
  Pel           *piTmpSrc = piSrc;
  const PPS     &pps      = *(cu.cs->pps);
  const SPS     &sps      = *(cu.cs->sps);
  const Slice   &slice    = *(cu.slice);
  const bool    ppsTransquantBypassEnabledFlag = pps.getTransquantBypassEnabledFlag();
  const int     bitDepthLuma                   = sps.getBitDepth(CHANNEL_TYPE_LUMA);
  const ClpRng& clpRng( cu.cs->slice->clpRng(COMPONENT_Y) );

  int          iQP          = 0;
  unsigned     uiNumParts   = ( pcv.rectCUs ? ( ( edgeDir == EDGE_VER ) ? lumaArea.height / pcv.minCUHeight : lumaArea.width / pcv.minCUWidth ) : pcv.partsInCtuWidth >> cu.qtDepth );
  int          pelsInPart   = pcv.minCUWidth;
  unsigned     uiBsAbsIdx   = 0, uiBs = 0;
  int          iOffset, iSrcStep;

  bool  bPCMFilter      = (sps.getUsePCM() && sps.getPCMFilterDisableFlag()) ? true : false;
  bool  bPartPNoFilter  = false;
  bool  bPartQNoFilter  = false;
  int   betaOffsetDiv2  = slice.getDeblockingFilterBetaOffsetDiv2();
  int   tcOffsetDiv2    = slice.getDeblockingFilterTcOffsetDiv2();
  int   xoffset, yoffset;

  Position pos;

  if (edgeDir == EDGE_VER)
  {
    xoffset   = 0;
    yoffset   = pelsInPart;
    iOffset   = 1;
    iSrcStep  = iStride;
    piTmpSrc += iEdge * pelsInPart;
    pos       = Position{ lumaArea.x + iEdge * pelsInPart, lumaArea.y - yoffset };
  }
  else  // (edgeDir == EDGE_HOR)
  {
    xoffset   = pelsInPart;
    yoffset   = 0;
    iOffset   = iStride;
    iSrcStep  = 1;
    piTmpSrc += iEdge*pelsInPart*iStride;
    pos       = Position{ lumaArea.x - xoffset, lumaArea.y + iEdge * pelsInPart };
  }

  const int iBitdepthScale = 1 << (bitDepthLuma - 8);

  // dec pos since within the loop we first calc the pos
  for( int iIdx = 0; iIdx < uiNumParts; iIdx++ )
  {
    pos.x += xoffset;
    pos.y += yoffset;

    uiBsAbsIdx = getRasterIdx( pos, pcv );
    uiBs       = m_aapucBS[edgeDir][uiBsAbsIdx];

    if( uiBs )
    {
      const CodingUnit& cuQ =  cu;
      const CodingUnit& cuP = *cu.cs->getCU(pos.offset(xoffset - pelsInPart, yoffset - pelsInPart), cu.chType);
      // Derive neighboring PU index
      if (edgeDir == EDGE_VER)
      {
#if HEVC_TILES_WPP
        CHECK( !isAvailableLeft( cu, cuP, !slice.getLFCrossSliceBoundaryFlag(), !pps.getLoopFilterAcrossTilesEnabledFlag() ), "Neighbour not available" );
#else
        CHECK( !isAvailable( cu, cuP, !slice.getLFCrossSliceBoundaryFlag() ), "Neighbour not available" );
#endif
      }
      else  // (iDir == EDGE_HOR)
      {
#if HEVC_TILES_WPP
        CHECK( !isAvailableAbove( cu, cuP, !slice.getLFCrossSliceBoundaryFlag(), !pps.getLoopFilterAcrossTilesEnabledFlag() ), "Neighbour not available" );
#else
        CHECK( !isAvailable( cu, cuP, !slice.getLFCrossSliceBoundaryFlag() ), "Neighbour not available" );
#endif
      }

      iQP = (cuP.qp + cuQ.qp + 1) >> 1;

      const int iIndexTC  = Clip3(0, MAX_QP + DEFAULT_INTRA_TC_OFFSET, int(iQP + DEFAULT_INTRA_TC_OFFSET*(uiBs - 1) + (tcOffsetDiv2 << 1)));
      const int iIndexB   = Clip3(0, MAX_QP, iQP + (betaOffsetDiv2 << 1));

      const int iTc       = sm_tcTable  [iIndexTC] * iBitdepthScale;
      const int iBeta     = sm_betaTable[iIndexB ] * iBitdepthScale;
      const int iSideThreshold = ( iBeta + ( iBeta >> 1 ) ) >> 3;
      const int iThrCut   = iTc * 10;

      const unsigned uiBlocksInPart = pelsInPart / 4 ? pelsInPart / 4 : 1;

      for( int iBlkIdx = 0; iBlkIdx < uiBlocksInPart; iBlkIdx++ )
      {
        const int dp0 = xCalcDP(piTmpSrc + iSrcStep*(iIdx*pelsInPart + iBlkIdx * 4 + 0), iOffset);
        const int dq0 = xCalcDQ(piTmpSrc + iSrcStep*(iIdx*pelsInPart + iBlkIdx * 4 + 0), iOffset);
        const int dp3 = xCalcDP(piTmpSrc + iSrcStep*(iIdx*pelsInPart + iBlkIdx * 4 + 3), iOffset);
        const int dq3 = xCalcDQ(piTmpSrc + iSrcStep*(iIdx*pelsInPart + iBlkIdx * 4 + 3), iOffset);
        const int d0 = dp0 + dq0;
        const int d3 = dp3 + dq3;

        const int dp = dp0 + dp3;
        const int dq = dq0 + dq3;
        const int d  = d0  + d3;

        bPartPNoFilter = bPartQNoFilter = false;
        if( bPCMFilter )
        {
          // Check if each of PUs is I_PCM with LF disabling
          bPartPNoFilter = cuP.ipcm;
          bPartQNoFilter = cuQ.ipcm;
        }
        if( ppsTransquantBypassEnabledFlag )
        {
          // check if each of PUs is lossless coded
          bPartPNoFilter = bPartPNoFilter || cuP.transQuantBypass;
          bPartQNoFilter = bPartQNoFilter || cuQ.transQuantBypass;
        }

        if( d < iBeta )
        {
          const bool bFilterP = (dp < iSideThreshold);
          const bool bFilterQ = (dq < iSideThreshold);

          const bool sw = xUseStrongFiltering( piTmpSrc + iSrcStep * ( iIdx*pelsInPart + iBlkIdx * 4 + 0 ), iOffset, 2 * d0, iBeta, iTc )
                       && xUseStrongFiltering( piTmpSrc + iSrcStep * ( iIdx*pelsInPart + iBlkIdx * 4 + 3 ), iOffset, 2 * d3, iBeta, iTc );

          for( int i = 0; i < DEBLOCK_SMALLEST_BLOCK / 2; i++ )
          {
            xPelFilterLuma( piTmpSrc + iSrcStep*( iIdx*pelsInPart + iBlkIdx * 4 + i ), iOffset, iTc, sw, bPartPNoFilter, bPartQNoFilter, iThrCut, bFilterP, bFilterQ, clpRng );
          }
        }
      }
    }
  }
}


void LoopFilter::xEdgeFilterChroma(const CodingUnit& cu, const DeblockEdgeDir edgeDir, const int iEdge)
{
  const Position lumaPos   = cu.Y().valid() ? cu.Y().pos() : recalcPosition( cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, cu.blocks[cu.chType].pos() );
  const Size     lumaSize  = cu.Y().valid() ? cu.Y().size() : recalcSize( cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, cu.blocks[cu.chType].size() );

  const PreCalcValues& pcv = *cu.cs->pcv;
  unsigned  rasterIdx      = getRasterIdx( lumaPos, pcv );

  PelBuf     picYuvRecCb   = cu.cs->getRecoBuf( cu.block(COMPONENT_Cb) );
  PelBuf     picYuvRecCr   = cu.cs->getRecoBuf( cu.block(COMPONENT_Cr) );
  Pel       *piSrcCb       = picYuvRecCb.buf;
  Pel       *piSrcCr       = picYuvRecCr.buf;
  const int  iStride       = picYuvRecCb.stride;
  const SPS &sps           = *cu.cs->sps;
  const PPS &pps           = *cu.cs->pps;
  const Slice  &slice      = *cu.slice;
  const ChromaFormat nChromaFormat   = sps.getChromaFormatIdc();
  const unsigned uiPelsInPartChromaH = pcv.minCUWidth  >> ::getComponentScaleX(COMPONENT_Cb, nChromaFormat);
  const unsigned uiPelsInPartChromaV = pcv.minCUHeight >> ::getComponentScaleY(COMPONENT_Cb, nChromaFormat);

  int       iOffset, iSrcStep;
  unsigned  uiLoopLength;

  bool      bPCMFilter      = (sps.getUsePCM() && sps.getPCMFilterDisableFlag()) ? true : false;
  bool      bPartPNoFilter  = false;
  bool      bPartQNoFilter  = false;
  const int tcOffsetDiv2    = slice.getDeblockingFilterTcOffsetDiv2();

  // Vertical Position
  unsigned uiEdgeNumInCtuVert = rasterIdx % pcv.partsInCtuWidth + iEdge;
  unsigned uiEdgeNumInCtuHor  = rasterIdx / pcv.partsInCtuWidth + iEdge;

  if( ( uiPelsInPartChromaH < DEBLOCK_SMALLEST_BLOCK ) && ( uiPelsInPartChromaV < DEBLOCK_SMALLEST_BLOCK ) &&
      (
        ( ( uiEdgeNumInCtuVert % ( DEBLOCK_SMALLEST_BLOCK / uiPelsInPartChromaH ) ) && ( edgeDir == EDGE_VER ) ) ||
        ( ( uiEdgeNumInCtuHor  % ( DEBLOCK_SMALLEST_BLOCK / uiPelsInPartChromaV ) ) && ( edgeDir == EDGE_HOR ) )
      )
    )
  {
    return;
  }

  unsigned uiNumParts = ( pcv.rectCUs ? ( ( edgeDir == EDGE_VER ) ? lumaSize.height / pcv.minCUHeight : lumaSize.width / pcv.minCUWidth ) : pcv.partsInCtuWidth >> cu.qtDepth );
  int   uiNumPelsLuma = pcv.minCUWidth;
  unsigned uiBsAbsIdx;
  unsigned ucBs;

  Pel* piTmpSrcCb = piSrcCb;
  Pel* piTmpSrcCr = piSrcCr;
  int xoffset, yoffset;
  Position pos( lumaPos.x, lumaPos.y );

  if( edgeDir == EDGE_VER )
  {
    xoffset      = 0;
    yoffset      = uiNumPelsLuma;
    iOffset      = 1;
    iSrcStep     = iStride;
    piTmpSrcCb  += iEdge*uiPelsInPartChromaH;
    piTmpSrcCr  += iEdge*uiPelsInPartChromaH;
    uiLoopLength = uiPelsInPartChromaV;
    pos          = Position{ lumaPos.x + iEdge*uiNumPelsLuma, lumaPos.y - yoffset };
  }
  else  // (edgeDir == EDGE_HOR)
  {
    xoffset      = uiNumPelsLuma;
    yoffset      = 0;
    iOffset      = iStride;
    iSrcStep     = 1;
    piTmpSrcCb  += iEdge*iStride*uiPelsInPartChromaV;
    piTmpSrcCr  += iEdge*iStride*uiPelsInPartChromaV;
    uiLoopLength = uiPelsInPartChromaH;
    pos          = Position{ lumaPos.x - xoffset, lumaPos.y + iEdge*uiNumPelsLuma };
  }

  const int iBitdepthScale = 1 << (sps.getBitDepth(CHANNEL_TYPE_CHROMA) - 8);

  for( int iIdx = 0; iIdx < uiNumParts; iIdx++ )
  {
    pos.x += xoffset;
    pos.y += yoffset;

    uiBsAbsIdx = getRasterIdx( pos, pcv );
    ucBs       = m_aapucBS[edgeDir][uiBsAbsIdx];

    if (ucBs > 1)
    {
      const CodingUnit& cuQ =  cu;
      const CodingUnit& cuP = *cu.cs->getCU( recalcPosition( cu.chromaFormat, CHANNEL_TYPE_LUMA, cu.chType, pos.offset( xoffset - uiNumPelsLuma, yoffset - uiNumPelsLuma ) ), cu.chType );

      if (edgeDir == EDGE_VER)
      {
#if HEVC_TILES_WPP
        CHECK(!isAvailableLeft(cu, cuP, !slice.getLFCrossSliceBoundaryFlag(), !pps.getLoopFilterAcrossTilesEnabledFlag()), "Neighbour not available");
#else
        CHECK(!isAvailable(cu, cuP, !slice.getLFCrossSliceBoundaryFlag()), "Neighbour not available");
#endif
      }
      else  // (iDir == EDGE_HOR)
      {
#if HEVC_TILES_WPP
        CHECK(!isAvailableAbove(cu, cuP, !slice.getLFCrossSliceBoundaryFlag(), !pps.getLoopFilterAcrossTilesEnabledFlag()), "Neighbour not available");
#else
        CHECK(!isAvailable(cu, cuP, !slice.getLFCrossSliceBoundaryFlag()), "Neighbour not available");
#endif
      }

      bPartPNoFilter = bPartQNoFilter = false;
      if (bPCMFilter)
      {
        // Check if each of PUs is I_PCM with LF disabling
        bPartPNoFilter = cuP.ipcm;
        bPartQNoFilter = cuQ.ipcm;
      }
      if( pps.getTransquantBypassEnabledFlag() )
      {
        // check if each of PUs is lossless coded
        bPartPNoFilter = bPartPNoFilter || cuP.transQuantBypass;
        bPartQNoFilter = bPartQNoFilter || cuQ.transQuantBypass;
      }

      for( int chromaIdx = 0; chromaIdx < 2; chromaIdx++ )
      {
        const ClpRng& clpRng( cu.cs->slice->clpRng( ComponentID( chromaIdx + 1 )) );
        const int chromaQPOffset = pps.getQpOffset( ComponentID( chromaIdx + 1 ) );
        Pel* piTmpSrcChroma = (chromaIdx == 0) ? piTmpSrcCb : piTmpSrcCr;

        int iQP = ( ( cuP.qp + cuQ.qp + 1 ) >> 1 ) + chromaQPOffset;
        if (iQP >= chromaQPMappingTableSize)
        {
          if( sps.getChromaFormatIdc() == CHROMA_420 )
          {
            iQP -= 6;
          }
          else if( iQP > MAX_QP )
          {
            iQP = MAX_QP;
          }
        }
        else if( iQP >= 0 )
        {
          iQP = getScaledChromaQP(iQP, sps.getChromaFormatIdc());
        }

        const int iIndexTC = Clip3<int>( 0, MAX_QP + DEFAULT_INTRA_TC_OFFSET, iQP + DEFAULT_INTRA_TC_OFFSET*( ucBs - 1 ) + ( tcOffsetDiv2 << 1 ) );
        const int iTc      = sm_tcTable[iIndexTC] * iBitdepthScale;

        for( unsigned uiStep = 0; uiStep < uiLoopLength; uiStep++ )
        {
          xPelFilterChroma( piTmpSrcChroma + iSrcStep*( uiStep + iIdx*uiLoopLength ), iOffset, iTc, bPartPNoFilter, bPartQNoFilter, clpRng );
        }
      }
    }
  }
}



/**
 - Deblocking for the luminance component with strong or weak filter
 .
 \param piSrc           pointer to picture data
 \param iOffset         offset value for picture data
 \param tc              tc value
 \param sw              decision strong/weak filter
 \param bPartPNoFilter  indicator to disable filtering on partP
 \param bPartQNoFilter  indicator to disable filtering on partQ
 \param iThrCut         threshold value for weak filter decision
 \param bFilterSecondP  decision weak filter/no filter for partP
 \param bFilterSecondQ  decision weak filter/no filter for partQ
 \param bitDepthLuma    luma bit depth
*/
inline void LoopFilter::xPelFilterLuma( Pel* piSrc, const int iOffset, const int tc, const bool sw, const bool bPartPNoFilter, const bool bPartQNoFilter, const int iThrCut, const bool bFilterSecondP, const bool bFilterSecondQ, const ClpRng& clpRng ) const
{
  int delta;

  const Pel m4  = piSrc[ 0          ];
  const Pel m3  = piSrc[-iOffset    ];
  const Pel m5  = piSrc[ iOffset    ];
  const Pel m2  = piSrc[-iOffset * 2];
  const Pel m6  = piSrc[ iOffset * 2];
  const Pel m1  = piSrc[-iOffset * 3];
  const Pel m7  = piSrc[ iOffset * 3];
  const Pel m0  = piSrc[-iOffset * 4];

  if (sw)
  {
    piSrc[-iOffset]     = Clip3( m3 - 2 * tc, m3 + 2 * tc, ( (     m1 + 2 * m2 + 2 * m3 + 2 * m4 +     m5 + 4 ) >> 3 ) );
    piSrc[ 0]           = Clip3( m4 - 2 * tc, m4 + 2 * tc, ( (     m2 + 2 * m3 + 2 * m4 + 2 * m5 +     m6 + 4 ) >> 3 ) );
    piSrc[-iOffset * 2] = Clip3( m2 - 2 * tc, m2 + 2 * tc, ( (     m1 +     m2 +     m3 +     m4 +          2 ) >> 2 ) );
    piSrc[ iOffset]     = Clip3( m5 - 2 * tc, m5 + 2 * tc, ( (     m3 +     m4 +     m5 +     m6 +          2 ) >> 2 ) );
    piSrc[-iOffset * 3] = Clip3( m1 - 2 * tc, m1 + 2 * tc, ( ( 2 * m0 + 3 * m1 +     m2 +     m3 +     m4 + 4 ) >> 3 ) );
    piSrc[ iOffset * 2] = Clip3( m6 - 2 * tc, m6 + 2 * tc, ( (     m3 +     m4 +     m5 + 3 * m6 + 2 * m7 + 4 ) >> 3 ) );
  }
  else
  {
    /* Weak filter */
    delta = ( 9 * ( m4 - m3 ) - 3 * ( m5 - m2 ) + 8 ) >> 4;

    if ( abs(delta) < iThrCut )
    {
      delta = Clip3( -tc, tc, delta );
      piSrc[-iOffset] = ClipPel( m3 + delta, clpRng);
      piSrc[0]        = ClipPel( m4 - delta, clpRng);

      const int tc2 = tc >> 1;
      if( bFilterSecondP )
      {
        const int delta1 = Clip3( -tc2, tc2, ( ( ( ( m1 + m3 + 1 ) >> 1 ) - m2 + delta ) >> 1 ) );
        piSrc[-iOffset * 2] = ClipPel( m2 + delta1, clpRng);
      }
      if( bFilterSecondQ )
      {
        const int delta2 = Clip3( -tc2, tc2, ( ( ( ( m6 + m4 + 1 ) >> 1 ) - m5 - delta ) >> 1 ) );
        piSrc[iOffset] = ClipPel( m5 + delta2, clpRng);
      }
    }
  }

  if(bPartPNoFilter)
  {
    piSrc[-iOffset    ] = m3;
    piSrc[-iOffset * 2] = m2;
    piSrc[-iOffset * 3] = m1;
  }

  if(bPartQNoFilter)
  {
    piSrc[ 0          ] = m4;
    piSrc[ iOffset    ] = m5;
    piSrc[ iOffset * 2] = m6;
  }
}

/**
 - Deblocking of one line/column for the chrominance component
 .
 \param piSrc           pointer to picture data
 \param iOffset         offset value for picture data
 \param tc              tc value
 \param bPartPNoFilter  indicator to disable filtering on partP
 \param bPartQNoFilter  indicator to disable filtering on partQ
 \param bitDepthChroma  chroma bit depth
 */
inline void LoopFilter::xPelFilterChroma( Pel* piSrc, const int iOffset, const int tc, const bool bPartPNoFilter, const bool bPartQNoFilter, const ClpRng& clpRng ) const
{
  int delta;

  const Pel m4  = piSrc[ 0          ];
  const Pel m3  = piSrc[-iOffset    ];
  const Pel m5  = piSrc[ iOffset    ];
  const Pel m2  = piSrc[-iOffset * 2];

  delta           = Clip3( -tc, tc, ( ( ( ( m4 - m3 ) << 2 ) + m2 - m5 + 4 ) >> 3 ) );
  piSrc[-iOffset] = ClipPel( m3 + delta, clpRng );
  piSrc[ 0      ] = ClipPel( m4 - delta, clpRng );

  if( bPartPNoFilter )
  {
    piSrc[-iOffset] = m3;
  }
  if( bPartQNoFilter )
  {
    piSrc[ 0      ] = m4;
  }
}

/**
 - Decision between strong and weak filter
 .
 \param offset         offset value for picture data
 \param d               d value
 \param beta            beta value
 \param tc              tc value
 \param piSrc           pointer to picture data
 */
inline bool LoopFilter::xUseStrongFiltering( Pel* piSrc, const int iOffset, const int d, const int beta, const int tc ) const
{
  const Pel m4 = piSrc[ 0          ];
  const Pel m3 = piSrc[-iOffset    ];
  const Pel m7 = piSrc[ iOffset * 3];
  const Pel m0 = piSrc[-iOffset * 4];

  const int d_strong = abs( m0 - m3 ) + abs( m7 - m4 );

  return ( ( d_strong < ( beta >> 3 ) ) && ( d < ( beta >> 2 ) ) && ( abs( m3 - m4 ) < ( ( tc * 5 + 1 ) >> 1 ) ) );
}

inline int LoopFilter::xCalcDP( Pel* piSrc, const int iOffset ) const
{
  return abs( piSrc[-iOffset * 3] - 2 * piSrc[-iOffset * 2] + piSrc[-iOffset] );
}

inline int LoopFilter::xCalcDQ( Pel* piSrc, const int iOffset ) const
{
  return abs( piSrc[0] - 2 * piSrc[iOffset] + piSrc[iOffset * 2] );
}

//! \}

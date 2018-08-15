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

/** \file     UnitTool.cpp
 *  \brief    defines operations for basic units
 */

#include "UnitTools.h"

#include "dtrace_next.h"

#include "Unit.h"
#include "Slice.h"
#include "Picture.h"

#include <utility>
#include <algorithm>

// CS tools


uint64_t CS::getEstBits(const CodingStructure &cs)
{
  return cs.fracBits >> SCALE_BITS;
}



bool CS::isDualITree( const CodingStructure &cs )
{
  return cs.slice->isIntra() && !cs.pcv->ISingleTree;
}

UnitArea CS::getArea( const CodingStructure &cs, const UnitArea &area, const ChannelType chType )
{
  return isDualITree( cs ) ? area.singleChan( chType ) : area;
}

// CU tools

bool CU::isIntra(const CodingUnit &cu)
{
  return cu.predMode == MODE_INTRA;
}

bool CU::isInter(const CodingUnit &cu)
{
  return cu.predMode == MODE_INTER;
}

bool CU::isRDPCMEnabled(const CodingUnit& cu)
{
  return cu.cs->sps->getSpsRangeExtension().getRdpcmEnabledFlag(cu.predMode == MODE_INTRA ? RDPCM_SIGNAL_IMPLICIT : RDPCM_SIGNAL_EXPLICIT);
}

bool CU::isLosslessCoded(const CodingUnit &cu)
{
  return cu.cs->pps->getTransquantBypassEnabledFlag() && cu.transQuantBypass;
}

bool CU::isSameSlice(const CodingUnit& cu, const CodingUnit& cu2)
{
  return cu.slice->getIndependentSliceIdx() == cu2.slice->getIndependentSliceIdx();
}

#if HEVC_TILES_WPP
bool CU::isSameTile(const CodingUnit& cu, const CodingUnit& cu2)
{
  return cu.tileIdx == cu2.tileIdx;
}

bool CU::isSameSliceAndTile(const CodingUnit& cu, const CodingUnit& cu2)
{
  return ( cu.slice->getIndependentSliceIdx() == cu2.slice->getIndependentSliceIdx() ) && ( cu.tileIdx == cu2.tileIdx );
}
#endif

bool CU::isSameCtu(const CodingUnit& cu, const CodingUnit& cu2)
{
  uint32_t ctuSizeBit = g_aucLog2[cu.cs->sps->getMaxCUWidth()];

  Position pos1Ctu(cu.lumaPos().x  >> ctuSizeBit, cu.lumaPos().y  >> ctuSizeBit);
  Position pos2Ctu(cu2.lumaPos().x >> ctuSizeBit, cu2.lumaPos().y >> ctuSizeBit);

  return pos1Ctu.x == pos2Ctu.x && pos1Ctu.y == pos2Ctu.y;
}

uint32_t CU::getIntraSizeIdx(const CodingUnit &cu)
{
  uint8_t uiWidth = cu.lumaSize().width;

  uint32_t  uiCnt   = 0;

  while (uiWidth)
  {
    uiCnt++;
    uiWidth >>= 1;
  }

  uiCnt -= 2;

  return uiCnt > 6 ? 6 : uiCnt;
}

bool CU::isLastSubCUOfCtu( const CodingUnit &cu )
{
  const SPS &sps      = *cu.cs->sps;
  const Area cuAreaY = CS::isDualITree( *cu.cs ) ? Area( recalcPosition( cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, cu.blocks[cu.chType].pos() ), recalcSize( cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, cu.blocks[cu.chType].size() ) ) : ( const Area& ) cu.Y();

  return ( ( ( ( cuAreaY.x + cuAreaY.width  ) & cu.cs->pcv->maxCUWidthMask  ) == 0 || cuAreaY.x + cuAreaY.width  == sps.getPicWidthInLumaSamples()  ) &&
           ( ( ( cuAreaY.y + cuAreaY.height ) & cu.cs->pcv->maxCUHeightMask ) == 0 || cuAreaY.y + cuAreaY.height == sps.getPicHeightInLumaSamples() ) );
}

uint32_t CU::getCtuAddr( const CodingUnit &cu )
{
  return getCtuAddr( cu.blocks[cu.chType].lumaPos(), *cu.cs->pcv );
}

int CU::predictQP( const CodingUnit& cu, const int prevQP )
{
  const CodingStructure &cs = *cu.cs;

#if ENABLE_WPP_PARALLELISM
  if( cs.sps->getSpsNext().getUseNextDQP() )
  {
    // Inter-CTU 2D "planar"   c(orner)  a(bove)
    // predictor arrangement:  b(efore)  p(rediction)

    // restrict the lookup, as it might cross CTU/slice/tile boundaries
    const CodingUnit *cuA = cs.getCURestricted( cu.blocks[cu.chType].pos().offset(  0, -1 ), cu, cu.chType );
    const CodingUnit *cuB = cs.getCURestricted( cu.blocks[cu.chType].pos().offset( -1,  0 ), cu, cu.chType );
    const CodingUnit *cuC = cs.getCURestricted( cu.blocks[cu.chType].pos().offset( -1, -1 ), cu, cu.chType );

    const int a = cuA ? cuA->qp : cs.slice->getSliceQpBase();
    const int b = cuB ? cuB->qp : cs.slice->getSliceQpBase();
    const int c = cuC ? cuC->qp : cs.slice->getSliceQpBase();

    return Clip3( ( a < b ? a : b ), ( a > b ? a : b ), a + b - c ); // derived from Martucci's Median Adaptive Prediction, 1990
  }

#endif
  // only predict within the same CTU, use HEVC's above+left prediction
  const int a = ( cu.blocks[cu.chType].y & ( cs.pcv->maxCUHeightMask >> getChannelTypeScaleY( cu.chType, cu.chromaFormat ) ) ) ? ( cs.getCU( cu.blocks[cu.chType].pos().offset( 0, -1 ), cu.chType ) )->qp : prevQP;
  const int b = ( cu.blocks[cu.chType].x & ( cs.pcv->maxCUWidthMask  >> getChannelTypeScaleX( cu.chType, cu.chromaFormat ) ) ) ? ( cs.getCU( cu.blocks[cu.chType].pos().offset( -1, 0 ), cu.chType ) )->qp : prevQP;

  return ( a + b + 1 ) >> 1;
}

bool CU::isQGStart( const CodingUnit& cu )
{
  const SPS &sps = *cu.cs->sps;
  const PPS &pps = *cu.cs->pps;

  return ( cu.blocks[cu.chType].x % ( ( 1 << ( g_aucLog2[sps.getMaxCUWidth()]  - pps.getMaxCuDQPDepth() ) ) >> getChannelTypeScaleX( cu.chType, cu.chromaFormat ) ) ) == 0 &&
         ( cu.blocks[cu.chType].y % ( ( 1 << ( g_aucLog2[sps.getMaxCUHeight()] - pps.getMaxCuDQPDepth() ) ) >> getChannelTypeScaleY( cu.chType, cu.chromaFormat ) ) ) == 0;
}

uint32_t CU::getNumPUs( const CodingUnit& cu )
{
  uint32_t cnt = 0;
  PredictionUnit *pu = cu.firstPU;

  do
  {
    cnt++;
  } while( ( pu != cu.lastPU ) && ( pu = pu->next ) );

  return cnt;
}

void CU::addPUs( CodingUnit& cu )
{
  cu.cs->addPU( CS::getArea( *cu.cs, cu, cu.chType ), cu.chType );
}


PartSplit CU::getSplitAtDepth( const CodingUnit& cu, const unsigned depth )
{
  if( depth >= cu.depth ) return CU_DONT_SPLIT;

  const PartSplit cuSplitType = PartSplit( ( cu.splitSeries >> ( depth * SPLIT_DMULT ) ) & SPLIT_MASK );

  if     ( cuSplitType == CU_QUAD_SPLIT    ) return CU_QUAD_SPLIT;

  else if( cuSplitType == CU_HORZ_SPLIT    ) return CU_HORZ_SPLIT;

  else if( cuSplitType == CU_VERT_SPLIT    ) return CU_VERT_SPLIT;

  else if( cuSplitType == CU_TRIH_SPLIT    ) return CU_TRIH_SPLIT;
  else if( cuSplitType == CU_TRIV_SPLIT    ) return CU_TRIV_SPLIT;
  else   { THROW( "Unknown split mode"    ); return CU_QUAD_SPLIT; }
}

bool CU::hasNonTsCodedBlock( const CodingUnit& cu )
{
  bool hasAnyNonTSCoded = false;

  for( auto &currTU : traverseTUs( cu ) )
  {
    for( uint32_t i = 0; i < ::getNumberValidTBlocks( *cu.cs->pcv ); i++ )
    {
      hasAnyNonTSCoded |= ( currTU.blocks[i].valid() && !currTU.transformSkip[i] && TU::getCbf( currTU, ComponentID( i ) ) );
    }
  }

  return hasAnyNonTSCoded;
}

uint32_t CU::getNumNonZeroCoeffNonTs( const CodingUnit& cu )
{
  uint32_t count = 0;
  for( auto &currTU : traverseTUs( cu ) )
  {
    count += TU::getNumNonZeroCoeffsNonTS( currTU );
  }

  return count;
}




PUTraverser CU::traversePUs( CodingUnit& cu )
{
  return PUTraverser( cu.firstPU, cu.lastPU->next );
}

TUTraverser CU::traverseTUs( CodingUnit& cu )
{
  return TUTraverser( cu.firstTU, cu.lastTU->next );
}

cPUTraverser CU::traversePUs( const CodingUnit& cu )
{
  return cPUTraverser( cu.firstPU, cu.lastPU->next );
}

cTUTraverser CU::traverseTUs( const CodingUnit& cu )
{
  return cTUTraverser( cu.firstTU, cu.lastTU->next );
}

// PU tools

int PU::getIntraMPMs( const PredictionUnit &pu, unsigned* mpm, const ChannelType &channelType /*= CHANNEL_TYPE_LUMA*/ )
{
  const unsigned numMPMs = pu.cs->pcv->numMPMs;
#if INTRA67_3MPM
  {
    int numCand      = -1;
    int leftIntraDir = DC_IDX, aboveIntraDir = DC_IDX;

    const CompArea &area = pu.block(getFirstComponentOfChannel(channelType));
    const Position &pos  = area.pos();

    // Get intra direction of left PU
    const PredictionUnit *puLeft = pu.cs->getPURestricted(pos.offset(-1, 0), pu, channelType);

    if (puLeft && CU::isIntra(*puLeft->cu))
    {
      leftIntraDir = puLeft->intraDir[channelType];

      if (isChroma(channelType) && leftIntraDir == DM_CHROMA_IDX)
      {
        leftIntraDir = puLeft->intraDir[0];
      }
    }

    // Get intra direction of above PU
    const PredictionUnit *puAbove = pu.cs->getPURestricted(pos.offset(0, -1), pu, channelType);

    if (puAbove && CU::isIntra(*puAbove->cu) && CU::isSameCtu(*pu.cu, *puAbove->cu))
    {
      aboveIntraDir = puAbove->intraDir[channelType];

      if (isChroma(channelType) && aboveIntraDir == DM_CHROMA_IDX)
      {
        aboveIntraDir = puAbove->intraDir[0];
      }
    }

    CHECK(2 >= numMPMs, "Invalid number of most probable modes");

    const int offset = (int) NUM_LUMA_MODE - 5;
    const int mod    = offset + 3;

    if (leftIntraDir == aboveIntraDir)
    {
      numCand = 1;

      if (leftIntraDir > DC_IDX)   // angular modes
      {
        mpm[0] = leftIntraDir;
        mpm[1] = ((leftIntraDir + offset) % mod) + 2;
        mpm[2] = ((leftIntraDir - 1) % mod) + 2;
      }
      else   // non-angular
      {
        mpm[0] = PLANAR_IDX;
        mpm[1] = DC_IDX;
        mpm[2] = VER_IDX;
      }
    }
    else
    {
      numCand = 2;

      mpm[0] = leftIntraDir;
      mpm[1] = aboveIntraDir;

      if (leftIntraDir && aboveIntraDir)   // both modes are non-planar
      {
        mpm[2] = PLANAR_IDX;
      }
      else
      {
        mpm[2] = (leftIntraDir + aboveIntraDir) < 2 ? VER_IDX : DC_IDX;
      }
    }
    for (int i = 0; i < numMPMs; i++)
    {
      CHECK(mpm[i] >= NUM_LUMA_MODE, "Invalid MPM");
    }
    CHECK(numCand == 0, "No candidates found");
    return numCand;
  }
#else
  {
    int numCand = -1;
    int leftIntraDir = DC_IDX, aboveIntraDir = DC_IDX;

    const CompArea& area = pu.block( getFirstComponentOfChannel( channelType ) );
    const Position& pos  = area.pos();

    // Get intra direction of left PU
    const PredictionUnit *puLeft = pu.cs->getPURestricted( pos.offset( -1, 0 ), pu, channelType );

    if( puLeft && CU::isIntra( *puLeft->cu ) )
    {
      leftIntraDir = puLeft->intraDir[channelType];

      if( isChroma( channelType ) && leftIntraDir == DM_CHROMA_IDX )
      {
        leftIntraDir = puLeft->intraDir[0];
      }
    }

    // Get intra direction of above PU
    const PredictionUnit* puAbove = pu.cs->getPURestricted( pos.offset( 0, -1 ), pu, channelType );

    if( puAbove && CU::isIntra( *puAbove->cu ) && CU::isSameCtu( *pu.cu, *puAbove->cu ) )
    {
      aboveIntraDir = puAbove->intraDir[channelType];

      if( isChroma( channelType ) && aboveIntraDir == DM_CHROMA_IDX )
      {
        aboveIntraDir = puAbove->intraDir[0];
      }
    }

    CHECK( 2 >= numMPMs, "Invalid number of most probable modes" );

    const int offset = 29;

    const int mod    = offset + 3;

    if( leftIntraDir == aboveIntraDir )
    {
      numCand = 1;

      if( leftIntraDir > DC_IDX ) // angular modes
      {
        mpm[0] =   g_intraMode65to33AngMapping[leftIntraDir];
        mpm[1] = ((g_intraMode65to33AngMapping[leftIntraDir] + offset) % mod) + 2;
        mpm[2] = ((g_intraMode65to33AngMapping[leftIntraDir] - 1)      % mod) + 2;
      }
      else //non-angular
      {
        mpm[0] = g_intraMode65to33AngMapping[PLANAR_IDX];
        mpm[1] = g_intraMode65to33AngMapping[DC_IDX];
        mpm[2] = g_intraMode65to33AngMapping[VER_IDX];
      }
    }
    else
    {
      numCand = 2;

      mpm[0] = g_intraMode65to33AngMapping[leftIntraDir];
      mpm[1] = g_intraMode65to33AngMapping[aboveIntraDir];

      if( leftIntraDir && aboveIntraDir ) //both modes are non-planar
      {
        mpm[2] = g_intraMode65to33AngMapping[PLANAR_IDX];
      }
      else
      {
        mpm[2] = g_intraMode65to33AngMapping[(leftIntraDir + aboveIntraDir) < 2 ? VER_IDX : DC_IDX];
      }
    }
    for( uint32_t i = 0; i < numMPMs; i++ )
    {
      mpm[i] = g_intraMode33to65AngMapping[mpm[i]];
      CHECK( mpm[i] >= NUM_LUMA_MODE, "Invalid MPM" );
    }
    CHECK( numCand == 0, "No candidates found" );
    return numCand;
  }
#endif
}


void PU::getIntraChromaCandModes( const PredictionUnit &pu, unsigned modeList[NUM_CHROMA_MODE] )
{
  {
    modeList[  0 ] = PLANAR_IDX;
    modeList[  1 ] = VER_IDX;
    modeList[  2 ] = HOR_IDX;
    modeList[  3 ] = DC_IDX;
#if JVET_K0190
    modeList[4] = LM_CHROMA_IDX;
    modeList[5] = DM_CHROMA_IDX;
#else
    modeList[  4 ] = DM_CHROMA_IDX;
#endif

    const PredictionUnit *lumaPU = CS::isDualITree( *pu.cs ) ? pu.cs->picture->cs->getPU( pu.blocks[pu.chType].lumaPos(), CHANNEL_TYPE_LUMA ) : &pu;
    const uint32_t lumaMode = lumaPU->intraDir[CHANNEL_TYPE_LUMA];
    for( int i = 0; i < 4; i++ )
    {
      if( lumaMode == modeList[i] )
      {
        modeList[i] = VDIA_IDX;
        break;
      }
    }
  }
}


#if JVET_K0190
bool PU::isLMCMode(unsigned mode)
{
#if JVET_K0190
  return (mode == LM_CHROMA_IDX);
#else
  return (mode >= LM_CHROMA_IDX && mode <= LM_CHROMA_F4_IDX);
#endif
}
#endif
#if JVET_K0190
bool PU::isLMCModeEnabled(const PredictionUnit &pu, unsigned mode)
{
  if ( pu.cs->sps->getSpsNext().getUseLMChroma() )
  {
#if JVET_K0190
    return true;
#else
    if ( mode == LM_CHROMA_IDX )
    {
      return true;
    }
    else if ( ( mode == MMLM_CHROMA_IDX && PU::isMMLMEnabled( pu ) ) || ( mode >= LM_CHROMA_F1_IDX && mode <= LM_CHROMA_F4_IDX && PU::isMFLMEnabled( pu ) ) )
    {
      return true;
    }
#endif
  }
  return false;
}

int PU::getLMSymbolList(const PredictionUnit &pu, int *pModeList)
{
  const int iNeighbors = 5;
  const PredictionUnit* neighboringPUs[ iNeighbors ];

  const CompArea& area = pu.Cb();
  const Position posLT = area.topLeft();
  const Position posRT = area.topRight();
  const Position posLB = area.bottomLeft();

  neighboringPUs[ 0 ] = pu.cs->getPURestricted( posLB.offset(-1,  0), pu, CHANNEL_TYPE_CHROMA ); //left
  neighboringPUs[ 1 ] = pu.cs->getPURestricted( posRT.offset( 0, -1), pu, CHANNEL_TYPE_CHROMA ); //above
  neighboringPUs[ 2 ] = pu.cs->getPURestricted( posRT.offset( 1, -1), pu, CHANNEL_TYPE_CHROMA ); //aboveRight
  neighboringPUs[ 3 ] = pu.cs->getPURestricted( posLB.offset(-1,  1), pu, CHANNEL_TYPE_CHROMA ); //BelowLeft
  neighboringPUs[ 4 ] = pu.cs->getPURestricted( posLT.offset(-1, -1), pu, CHANNEL_TYPE_CHROMA ); //AboveLeft

  int iCount = 0;
  for ( int i = 0; i < iNeighbors; i++ )
  {
    if ( neighboringPUs[i] && CU::isIntra( *(neighboringPUs[i]->cu) ) )
    {
      int iMode = neighboringPUs[i]->intraDir[CHANNEL_TYPE_CHROMA];
      if ( ! PU::isLMCMode( iMode ) )
      {
        iCount++;
      }
    }
  }

  bool bNonLMInsert = false;
  int iIdx = 0;

  pModeList[ iIdx++ ] = LM_CHROMA_IDX;

  if ( iCount >= g_aiNonLMPosThrs[0] && ! bNonLMInsert )
  {
    pModeList[ iIdx++ ] = -1;
    bNonLMInsert = true;
  }
#if !JVET_K0190
  if ( PU::isMMLMEnabled( pu ) )
  {
    pModeList[ iIdx++ ] = MMLM_CHROMA_IDX;
  }
#endif

  if ( iCount >= g_aiNonLMPosThrs[1] && ! bNonLMInsert )
  {
    pModeList[ iIdx++ ] = -1;
    bNonLMInsert = true;
  }
#if !JVET_K0190
  if ( PU::isMFLMEnabled( pu ) )
  {
    pModeList[ iIdx++ ] = LM_CHROMA_F1_IDX;
    pModeList[ iIdx++ ] = LM_CHROMA_F2_IDX;
    if ( iCount >= g_aiNonLMPosThrs[2] && ! bNonLMInsert )
    {
      pModeList[ iIdx++ ] = -1;
      bNonLMInsert = true;
    }
    pModeList[ iIdx++ ] = LM_CHROMA_F3_IDX;
    pModeList[ iIdx++ ] = LM_CHROMA_F4_IDX;
  }
#endif
  if ( ! bNonLMInsert )
  {
    pModeList[ iIdx++ ] = -1;
    bNonLMInsert = true;
  }

  return iIdx;
}

#endif


bool PU::isChromaIntraModeCrossCheckMode( const PredictionUnit &pu )
{
  return pu.intraDir[CHANNEL_TYPE_CHROMA] == DM_CHROMA_IDX;
}

uint32_t PU::getFinalIntraMode( const PredictionUnit &pu, const ChannelType &chType )
{
  uint32_t uiIntraMode = pu.intraDir[chType];

  if( uiIntraMode == DM_CHROMA_IDX && !isLuma( chType ) )
  {
    const PredictionUnit &lumaPU = CS::isDualITree( *pu.cs ) ? *pu.cs->picture->cs->getPU( pu.blocks[chType].lumaPos(), CHANNEL_TYPE_LUMA ) : *pu.cs->getPU( pu.blocks[chType].lumaPos(), CHANNEL_TYPE_LUMA );
    uiIntraMode = lumaPU.intraDir[0];
  }
  if( pu.chromaFormat == CHROMA_422 && !isLuma( chType ) )
  {
    uiIntraMode = g_chroma422IntraAngleMappingTable[uiIntraMode];
  }
  return uiIntraMode;
}


void PU::getInterMergeCandidates( const PredictionUnit &pu, MergeCtx& mrgCtx, const int& mrgCandIdx )
{
  const CodingStructure &cs  = *pu.cs;
  const Slice &slice         = *pu.cs->slice;
  const uint32_t maxNumMergeCand = slice.getMaxNumMergeCand();
  const bool canFastExit     = pu.cs->pps->getLog2ParallelMergeLevelMinus2() == 0;

  bool isCandInter[MRG_MAX_NUM_CANDS];

  for (uint32_t ui = 0; ui < maxNumMergeCand; ++ui)
  {
    isCandInter[ui] = false;
    mrgCtx.interDirNeighbours[ui] = 0;
    mrgCtx.mrgTypeNeighbours [ui] = MRG_TYPE_DEFAULT_N;
    mrgCtx.mvFieldNeighbours[(ui << 1)    ].refIdx = NOT_VALID;
    mrgCtx.mvFieldNeighbours[(ui << 1) + 1].refIdx = NOT_VALID;
  }

  mrgCtx.numValidMergeCand = maxNumMergeCand;
  // compute the location of the current PU

  int cnt = 0;

  const Position posLT = pu.Y().topLeft();
  const Position posRT = pu.Y().topRight();
  const Position posLB = pu.Y().bottomLeft();

  MotionInfo miAbove, miLeft, miAboveLeft, miAboveRight, miBelowLeft;

  //left
  const PredictionUnit* puLeft = cs.getPURestricted( posLB.offset( -1, 0 ), pu, pu.chType );

  const bool isAvailableA1 = puLeft && isDiffMER( pu, *puLeft ) && pu.cu != puLeft->cu && CU::isInter( *puLeft->cu );

  if( isAvailableA1 )
  {
    miLeft = puLeft->getMotionInfo( posLB.offset(-1, 0) );

    isCandInter[cnt] = true;

    // get Inter Dir
    mrgCtx.interDirNeighbours[cnt] = miLeft.interDir;

    // get Mv from Left
    mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miLeft.mv[0], miLeft.refIdx[0]);

    if (slice.isInterB())
    {
      mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miLeft.mv[1], miLeft.refIdx[1]);
    }

    if( mrgCandIdx == cnt && canFastExit )
    {
      return;
    }

    cnt++;
  }

  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }


  // above
  const PredictionUnit *puAbove = cs.getPURestricted( posRT.offset( 0, -1 ), pu, pu.chType );

  bool isAvailableB1 = puAbove && isDiffMER( pu, *puAbove ) && pu.cu != puAbove->cu && CU::isInter( *puAbove->cu );

  if( isAvailableB1 )
  {
    miAbove = puAbove->getMotionInfo( posRT.offset( 0, -1 ) );

    if( !isAvailableA1 || ( miAbove != miLeft ) )
    {
      isCandInter[cnt] = true;

      // get Inter Dir
      mrgCtx.interDirNeighbours[cnt] = miAbove.interDir;
      // get Mv from Left
      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField( miAbove.mv[0], miAbove.refIdx[0] );

      if( slice.isInterB() )
      {
        mrgCtx.mvFieldNeighbours[( cnt << 1 ) + 1].setMvField( miAbove.mv[1], miAbove.refIdx[1] );
      }

      if( mrgCandIdx == cnt && canFastExit )
      {
        return;
      }

      cnt++;
    }
  }

  // early termination
  if( cnt == maxNumMergeCand )
  {
    return;
  }

  // above right
  const PredictionUnit *puAboveRight = cs.getPURestricted( posRT.offset( 1, -1 ), pu, pu.chType );

  bool isAvailableB0 = puAboveRight && isDiffMER( pu, *puAboveRight ) && CU::isInter( *puAboveRight->cu );

  if( isAvailableB0 )
  {
    miAboveRight = puAboveRight->getMotionInfo( posRT.offset( 1, -1 ) );

#if HM_JEM_MERGE_CANDS
    if( ( !isAvailableB1 || ( miAbove != miAboveRight ) ) && ( !isAvailableA1 || ( miLeft != miAboveRight ) ) )
#else
    if( !isAvailableB1 || ( miAbove != miAboveRight ) )
#endif
    {
      isCandInter[cnt] = true;

      // get Inter Dir
      mrgCtx.interDirNeighbours[cnt] = miAboveRight.interDir;
      // get Mv from Left
      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField( miAboveRight.mv[0], miAboveRight.refIdx[0] );

      if( slice.isInterB() )
      {
        mrgCtx.mvFieldNeighbours[( cnt << 1 ) + 1].setMvField( miAboveRight.mv[1], miAboveRight.refIdx[1] );
      }

      if( mrgCandIdx == cnt && canFastExit )
      {
        return;
      }

      cnt++;
    }
  }
  // early termination
  if( cnt == maxNumMergeCand )
  {
    return;
  }

  //left bottom
  const PredictionUnit *puLeftBottom = cs.getPURestricted( posLB.offset( -1, 1 ), pu, pu.chType );

  bool isAvailableA0 = puLeftBottom && isDiffMER( pu, *puLeftBottom ) && CU::isInter( *puLeftBottom->cu );

  if( isAvailableA0 )
  {
    miBelowLeft = puLeftBottom->getMotionInfo( posLB.offset( -1, 1 ) );

#if HM_JEM_MERGE_CANDS
    if( ( !isAvailableA1 || ( miBelowLeft != miLeft ) ) && ( !isAvailableB1 || ( miBelowLeft != miAbove ) ) && ( !isAvailableB0 || ( miBelowLeft != miAboveRight ) ) )
#else
    if( !isAvailableA1 || ( miBelowLeft != miLeft ) )
#endif
    {
      isCandInter[cnt] = true;

      // get Inter Dir
      mrgCtx.interDirNeighbours[cnt] = miBelowLeft.interDir;
      // get Mv from Bottom-Left
      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField( miBelowLeft.mv[0], miBelowLeft.refIdx[0] );

      if( slice.isInterB() )
      {
        mrgCtx.mvFieldNeighbours[( cnt << 1 ) + 1].setMvField( miBelowLeft.mv[1], miBelowLeft.refIdx[1] );
      }

      if( mrgCandIdx == cnt && canFastExit )
      {
        return;
      }

      cnt++;
    }
  }
  // early termination
  if( cnt == maxNumMergeCand )
  {
    return;
  }

#if JVET_K0346
  bool enableSubPuMvp = slice.getSPS()->getSpsNext().getUseSubPuMvp();
  bool isAvailableSubPu = false;
  unsigned subPuMvpPos = 0;

  if( enableSubPuMvp )
  {
    CHECK( mrgCtx.subPuMvpMiBuf   .area() == 0 || !mrgCtx.subPuMvpMiBuf   .buf, "Buffer not initialized" );

    mrgCtx.subPuMvpMiBuf   .fill( MotionInfo() );
  }

  if( enableSubPuMvp && slice.getEnableTMVPFlag() )
  {
    bool bMrgIdxMatchATMVPCan = ( mrgCandIdx == cnt );
    bool tmpLICFlag           = false;

    isAvailableSubPu = cs.sps->getSpsNext().getUseATMVP() && getInterMergeSubPuMvpCand( pu, mrgCtx, tmpLICFlag, cnt );

    if( isAvailableSubPu )
    {
      isCandInter[cnt] = true;

      mrgCtx.mrgTypeNeighbours[cnt] = MRG_TYPE_SUBPU_ATMVP;

      if( bMrgIdxMatchATMVPCan )
      {
        return;
      }
      subPuMvpPos = cnt;
      cnt++;

      if( cnt == maxNumMergeCand )
      {
        return;
      }
    }

  }
#endif

  // above left
#if JVET_K0346
  if( cnt < ( enableSubPuMvp ? 6 : 4 ) )
#else
  if( cnt < 4 )
#endif
  {
    const PredictionUnit *puAboveLeft = cs.getPURestricted( posLT.offset( -1, -1 ), pu, pu.chType );

    bool isAvailableB2 = puAboveLeft && isDiffMER( pu, *puAboveLeft ) && CU::isInter( *puAboveLeft->cu );

    if( isAvailableB2 )
    {
      miAboveLeft = puAboveLeft->getMotionInfo( posLT.offset( -1, -1 ) );

#if HM_JEM_MERGE_CANDS
      if( ( !isAvailableA1 || ( miLeft != miAboveLeft ) ) && ( !isAvailableB1 || ( miAbove != miAboveLeft ) ) && ( !isAvailableA0 || ( miBelowLeft != miAboveLeft ) ) && ( !isAvailableB0 || ( miAboveRight != miAboveLeft ) ) )
#else
      if( ( !isAvailableA1 || ( miLeft != miAboveLeft ) ) && ( !isAvailableB1 || ( miAbove != miAboveLeft ) ) )
#endif
      {
        isCandInter[cnt] = true;

        // get Inter Dir
        mrgCtx.interDirNeighbours[cnt] = miAboveLeft.interDir;
        // get Mv from Above-Left
        mrgCtx.mvFieldNeighbours[cnt << 1].setMvField( miAboveLeft.mv[0], miAboveLeft.refIdx[0] );

        if( slice.isInterB() )
        {
          mrgCtx.mvFieldNeighbours[( cnt << 1 ) + 1].setMvField( miAboveLeft.mv[1], miAboveLeft.refIdx[1] );
        }

        if( mrgCandIdx == cnt && canFastExit )
        {
          return;
        }

        cnt++;
      }
    }
  }
  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }

  if (slice.getEnableTMVPFlag())
  {
    //>> MTK colocated-RightBottom
    // offset the pos to be sure to "point" to the same position the uiAbsPartIdx would've pointed to
    Position posRB = pu.Y().bottomRight().offset(-3, -3);

    const PreCalcValues& pcv = *cs.pcv;

    Position posC0;
    Position posC1 = pu.Y().center();
    bool C0Avail = false;

    if (((posRB.x + pcv.minCUWidth) < pcv.lumaWidth) && ((posRB.y + pcv.minCUHeight) < pcv.lumaHeight))
    {
      {
        Position posInCtu( posRB.x & pcv.maxCUWidthMask, posRB.y & pcv.maxCUHeightMask );

        if( ( posInCtu.x + 4 < pcv.maxCUWidth ) &&           // is not at the last column of CTU
            ( posInCtu.y + 4 < pcv.maxCUHeight ) )           // is not at the last row    of CTU
        {
          posC0 = posRB.offset( 4, 4 );
          C0Avail = true;
        }
        else if( posInCtu.x + 4 < pcv.maxCUWidth )           // is not at the last column of CTU But is last row of CTU
        {
          posC0 = posRB.offset( 4, 4 );
          // in the reference the CTU address is not set - thus probably resulting in no using this C0 possibility
        }
        else if( posInCtu.y + 4 < pcv.maxCUHeight )          // is not at the last row of CTU But is last column of CTU
        {
          posC0 = posRB.offset( 4, 4 );
          C0Avail = true;
        }
        else //is the right bottom corner of CTU
        {
          posC0 = posRB.offset( 4, 4 );
          // same as for last column but not last row
        }
      }
    }

    Mv        cColMv;
    int       iRefIdx     = 0;
    int       dir         = 0;
    unsigned  uiArrayAddr = cnt;
    bool      bExistMV    = ( C0Avail && getColocatedMVP(pu, REF_PIC_LIST_0, posC0, cColMv, iRefIdx ) )
                                      || getColocatedMVP(pu, REF_PIC_LIST_0, posC1, cColMv, iRefIdx );

    if (bExistMV)
    {
      dir     |= 1;
      mrgCtx.mvFieldNeighbours[2 * uiArrayAddr].setMvField(cColMv, iRefIdx);
    }

    if (slice.isInterB())
    {
      bExistMV = ( C0Avail && getColocatedMVP(pu, REF_PIC_LIST_1, posC0, cColMv, iRefIdx ) )
                           || getColocatedMVP(pu, REF_PIC_LIST_1, posC1, cColMv, iRefIdx );
      if (bExistMV)
      {
        dir     |= 2;
        mrgCtx.mvFieldNeighbours[2 * uiArrayAddr + 1].setMvField(cColMv, iRefIdx);
      }
    }

    if( dir != 0 )
    {
#if JVET_K0346
      bool addTMvp = !( cs.sps->getSpsNext().getUseSubPuMvp() && isAvailableSubPu );
      if( !addTMvp )
      {
        if ( dir != mrgCtx.interDirNeighbours[subPuMvpPos] )
        {
          addTMvp = true;
        }
        else
        {
          for( unsigned refList = 0; refList < NUM_REF_PIC_LIST_01; refList++ )
          {
            if( dir & ( 1 << refList ) )
            {
              if( mrgCtx.mvFieldNeighbours[( cnt << 1 ) + refList] != mrgCtx.mvFieldNeighbours[(subPuMvpPos << 1) + refList] )
              {
                addTMvp = true;
                break;
              }
            }
          }
        }
      }
#else
      bool addTMvp = true;
#endif
#if HM_JEM_MERGE_CANDS
#if JVET_K0346
      int iSpanCand = isAvailableSubPu ? cnt - 1 : cnt;
#else
      int iSpanCand = cnt;
#endif
      for( int i = 0; i < iSpanCand; i++ )
      {
        if( mrgCtx.interDirNeighbours[  i           ] == dir &&
            mrgCtx.mvFieldNeighbours [  i << 1      ] == mrgCtx.mvFieldNeighbours[  uiArrayAddr << 1      ] &&
            mrgCtx.mvFieldNeighbours [( i << 1 ) + 1] == mrgCtx.mvFieldNeighbours[( uiArrayAddr << 1 ) + 1] )
        {
          addTMvp = false;
        }
      }
#endif
      if( addTMvp )
      {
        mrgCtx.interDirNeighbours[uiArrayAddr] = dir;
        isCandInter              [uiArrayAddr] = true;

        if( mrgCandIdx == cnt && canFastExit )
        {
          return;
        }

        cnt++;
      }
    }
  }

  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }

  uint32_t uiArrayAddr = cnt;
  uint32_t uiCutoff    = std::min( uiArrayAddr, 4u );

  if (slice.isInterB())
  {
    static const uint32_t NUM_PRIORITY_LIST = 12;
    static const uint32_t uiPriorityList0[NUM_PRIORITY_LIST] = { 0 , 1, 0, 2, 1, 2, 0, 3, 1, 3, 2, 3 };
    static const uint32_t uiPriorityList1[NUM_PRIORITY_LIST] = { 1 , 0, 2, 0, 2, 1, 3, 0, 3, 1, 3, 2 };

    for (int idx = 0; idx < uiCutoff * (uiCutoff - 1) && uiArrayAddr != maxNumMergeCand; idx++)
    {
      CHECK( idx >= NUM_PRIORITY_LIST, "Invalid priority list number" );
      int i = uiPriorityList0[idx];
      int j = uiPriorityList1[idx];
      if (isCandInter[i] && isCandInter[j] && (mrgCtx.interDirNeighbours[i] & 0x1) && (mrgCtx.interDirNeighbours[j] & 0x2))
      {
        isCandInter[uiArrayAddr] = true;
        mrgCtx.interDirNeighbours[uiArrayAddr] = 3;

        // get Mv from cand[i] and cand[j]
        mrgCtx.mvFieldNeighbours[ uiArrayAddr << 1     ].setMvField(mrgCtx.mvFieldNeighbours[ i << 1     ].mv, mrgCtx.mvFieldNeighbours[ i << 1     ].refIdx);
        mrgCtx.mvFieldNeighbours[(uiArrayAddr << 1) + 1].setMvField(mrgCtx.mvFieldNeighbours[(j << 1) + 1].mv, mrgCtx.mvFieldNeighbours[(j << 1) + 1].refIdx);

        int iRefPOCL0 = slice.getRefPOC(REF_PIC_LIST_0, mrgCtx.mvFieldNeighbours[(uiArrayAddr << 1)    ].refIdx);
        int iRefPOCL1 = slice.getRefPOC(REF_PIC_LIST_1, mrgCtx.mvFieldNeighbours[(uiArrayAddr << 1) + 1].refIdx);

        if( iRefPOCL0 == iRefPOCL1 && mrgCtx.mvFieldNeighbours[( uiArrayAddr << 1 )].mv == mrgCtx.mvFieldNeighbours[( uiArrayAddr << 1 ) + 1].mv )
        {
          isCandInter[uiArrayAddr] = false;
        }
        else
        {
          uiArrayAddr++;
        }
      }
    }
  }

  // early termination
  if (uiArrayAddr == maxNumMergeCand)
  {
    return;
  }

  int iNumRefIdx = slice.isInterB() ? std::min(slice.getNumRefIdx(REF_PIC_LIST_0), slice.getNumRefIdx(REF_PIC_LIST_1)) : slice.getNumRefIdx(REF_PIC_LIST_0);

  int r = 0;
  int refcnt = 0;
  while (uiArrayAddr < maxNumMergeCand)
  {
    isCandInter               [uiArrayAddr     ] = true;
    mrgCtx.interDirNeighbours [uiArrayAddr     ] = 1;
    mrgCtx.mvFieldNeighbours  [uiArrayAddr << 1].setMvField(Mv(0, 0), r);

    if (slice.isInterB())
    {
      mrgCtx.interDirNeighbours [ uiArrayAddr          ] = 3;
      mrgCtx.mvFieldNeighbours  [(uiArrayAddr << 1) + 1].setMvField(Mv(0, 0), r);
    }

    uiArrayAddr++;

    if (refcnt == iNumRefIdx - 1)
    {
      r = 0;
    }
    else
    {
      ++r;
      ++refcnt;
    }
  }
  mrgCtx.numValidMergeCand = uiArrayAddr;
}



static int xGetDistScaleFactor(const int &iCurrPOC, const int &iCurrRefPOC, const int &iColPOC, const int &iColRefPOC)
{
  int iDiffPocD = iColPOC - iColRefPOC;
  int iDiffPocB = iCurrPOC - iCurrRefPOC;

  if (iDiffPocD == iDiffPocB)
  {
    return 4096;
  }
  else
  {
    int iTDB = Clip3(-128, 127, iDiffPocB);
    int iTDD = Clip3(-128, 127, iDiffPocD);
    int iX = (0x4000 + abs(iTDD / 2)) / iTDD;
    int iScale = Clip3(-4096, 4095, (iTDB * iX + 32) >> 6);
    return iScale;
  }
}

bool PU::getColocatedMVP(const PredictionUnit &pu, const RefPicList &eRefPicList, const Position &_pos, Mv& rcMv, const int &refIdx )
{
  // don't perform MV compression when generally disabled or subPuMvp is used
  const unsigned scale = ( pu.cs->pcv->noMotComp ? 1 : 4 * std::max<int>(1, 4 * AMVP_DECIMATION_FACTOR / 4) );
  const unsigned mask  = ~( scale - 1 );

  const Position pos = Position{ PosType( _pos.x & mask ), PosType( _pos.y & mask ) };

  const Slice &slice = *pu.cs->slice;

  // use coldir.
  const Picture* const pColPic = slice.getRefPic(RefPicList(slice.isInterB() ? 1 - slice.getColFromL0Flag() : 0), slice.getColRefIdx());

  if( !pColPic )
  {
    return false;
  }

  RefPicList eColRefPicList = slice.getCheckLDC() ? eRefPicList : RefPicList(slice.getColFromL0Flag());

  const MotionInfo& mi = pColPic->cs->getMotionInfo( pos );

  if( !mi.isInter )
  {
    return false;
  }

  int iColRefIdx = mi.refIdx[eColRefPicList];

  if (iColRefIdx < 0)
  {
    eColRefPicList = RefPicList(1 - eColRefPicList);
    iColRefIdx = mi.refIdx[eColRefPicList];

    if (iColRefIdx < 0)
    {
      return false;
    }
  }

  const Slice *pColSlice = nullptr;

  for( const auto s : pColPic->slices )
  {
    if( s->getIndependentSliceIdx() == mi.sliceIdx )
    {
      pColSlice = s;
      break;
    }
  }

  CHECK( pColSlice == nullptr, "Slice segment not found" );

  const Slice &colSlice = *pColSlice;

  const bool bIsCurrRefLongTerm = slice.getRefPic(eRefPicList, refIdx)->longTerm;
  const bool bIsColRefLongTerm  = colSlice.getIsUsedAsLongTerm(eColRefPicList, iColRefIdx);

  if (bIsCurrRefLongTerm != bIsColRefLongTerm)
  {
    return false;
  }


  // Scale the vector.
  Mv cColMv = mi.mv[eColRefPicList];

  if (bIsCurrRefLongTerm /*|| bIsColRefLongTerm*/)
  {
    rcMv = cColMv;
  }
  else
  {
    const int currPOC    = slice.getPOC();
    const int colPOC     = colSlice.getPOC();
    const int colRefPOC  = colSlice.getRefPOC(eColRefPicList, iColRefIdx);
    const int currRefPOC = slice.getRefPic(eRefPicList, refIdx)->getPOC();
    const int distscale  = xGetDistScaleFactor(currPOC, currRefPOC, colPOC, colRefPOC);

    if (distscale == 4096)
    {
      rcMv = cColMv;
    }
    else
    {
#if JVET_K0346 || JVET_K_AFFINE
      if( pu.cs->sps->getSpsNext().getUseHighPrecMv() )
      {
        // allow extended precision for temporal scaling
        cColMv.setHighPrec();
      }
#endif
      rcMv = cColMv.scaleMv(distscale);
    }
  }

  return true;
}

bool PU::isDiffMER(const PredictionUnit &pu1, const PredictionUnit &pu2)
{
  const unsigned xN = pu1.lumaPos().x;
  const unsigned yN = pu1.lumaPos().y;
  const unsigned xP = pu2.lumaPos().x;
  const unsigned yP = pu2.lumaPos().y;

  unsigned plevel = pu1.cs->pps->getLog2ParallelMergeLevelMinus2() + 2;

  if ((xN >> plevel) != (xP >> plevel))
  {
    return true;
  }

  if ((yN >> plevel) != (yP >> plevel))
  {
    return true;
  }

  return false;
}

/** Constructs a list of candidates for AMVP (See specification, section "Derivation process for motion vector predictor candidates")
* \param uiPartIdx
* \param uiPartAddr
* \param eRefPicList
* \param iRefIdx
* \param pInfo
*/
void PU::fillMvpCand(PredictionUnit &pu, const RefPicList &eRefPicList, const int &refIdx, AMVPInfo &amvpInfo)
{
  CodingStructure &cs = *pu.cs;

  AMVPInfo *pInfo = &amvpInfo;

  pInfo->numCand = 0;

  if (refIdx < 0)
  {
    return;
  }

  //-- Get Spatial MV
  Position posLT = pu.Y().topLeft();
  Position posRT = pu.Y().topRight();
  Position posLB = pu.Y().bottomLeft();

  bool isScaledFlagLX = false; /// variable name from specification; true when the PUs below left or left are available (availableA0 || availableA1).

  {
    const PredictionUnit* tmpPU = cs.getPURestricted( posLB.offset( -1, 1 ), pu, pu.chType ); // getPUBelowLeft(idx, partIdxLB);
    isScaledFlagLX = tmpPU != NULL && CU::isInter( *tmpPU->cu );

    if( !isScaledFlagLX )
    {
      tmpPU = cs.getPURestricted( posLB.offset( -1, 0 ), pu, pu.chType );
      isScaledFlagLX = tmpPU != NULL && CU::isInter( *tmpPU->cu );
    }
  }

  // Left predictor search
  if( isScaledFlagLX )
  {
    bool bAdded = addMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_BELOW_LEFT, *pInfo );

    if( !bAdded )
    {
      bAdded = addMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_LEFT, *pInfo );

      if( !bAdded )
      {
        bAdded = addMVPCandWithScaling( pu, eRefPicList, refIdx, posLB, MD_BELOW_LEFT, *pInfo );

        if( !bAdded )
        {
          addMVPCandWithScaling( pu, eRefPicList, refIdx, posLB, MD_LEFT, *pInfo );
        }
      }
    }
  }

  // Above predictor search
  {
    bool bAdded = addMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE_RIGHT, *pInfo );

    if( !bAdded )
    {
      bAdded = addMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE, *pInfo );

      if( !bAdded )
      {
        addMVPCandUnscaled( pu, eRefPicList, refIdx, posLT, MD_ABOVE_LEFT, *pInfo );
      }
    }
  }

  if( !isScaledFlagLX )
  {
    bool bAdded = addMVPCandWithScaling( pu, eRefPicList, refIdx, posRT, MD_ABOVE_RIGHT, *pInfo );

    if( !bAdded )
    {
      bAdded = addMVPCandWithScaling( pu, eRefPicList, refIdx, posRT, MD_ABOVE, *pInfo );

      if( !bAdded )
      {
        addMVPCandWithScaling( pu, eRefPicList, refIdx, posLT, MD_ABOVE_LEFT, *pInfo );
      }
    }
  }

#if JVET_K0357_AMVR
  if( pu.cu->imv != 0)
  {
    unsigned imvShift = pu.cu->imv << 1;
    for( int i = 0; i < pInfo->numCand; i++ )
    {
      roundMV( pInfo->mvCand[i], imvShift );
    }
  }
#endif

  if( pInfo->numCand == 2 )
  {
    if( pInfo->mvCand[0] == pInfo->mvCand[1] )
    {
      pInfo->numCand = 1;
    }
  }

  if( cs.slice->getEnableTMVPFlag() )
  {
    // Get Temporal Motion Predictor
    const int refIdx_Col = refIdx;

    Position posRB = pu.Y().bottomRight().offset(-3, -3);

    const PreCalcValues& pcv = *cs.pcv;

    Position posC0;
    bool C0Avail = false;
    Position posC1 = pu.Y().center();

    Mv cColMv;

    if( ( ( posRB.x + pcv.minCUWidth ) < pcv.lumaWidth ) && ( ( posRB.y + pcv.minCUHeight ) < pcv.lumaHeight ) )
    {
      Position posInCtu( posRB.x & pcv.maxCUWidthMask, posRB.y & pcv.maxCUHeightMask );

      if ((posInCtu.x + 4 < pcv.maxCUWidth) &&           // is not at the last column of CTU
          (posInCtu.y + 4 < pcv.maxCUHeight))             // is not at the last row    of CTU
      {
        posC0 = posRB.offset(4, 4);
        C0Avail = true;
      }
      else if (posInCtu.x + 4 < pcv.maxCUWidth)           // is not at the last column of CTU But is last row of CTU
      {
        // in the reference the CTU address is not set - thus probably resulting in no using this C0 possibility
        posC0 = posRB.offset(4, 4);
      }
      else if (posInCtu.y + 4 < pcv.maxCUHeight)          // is not at the last row of CTU But is last column of CTU
      {
        posC0 = posRB.offset(4, 4);
        C0Avail = true;
      }
      else //is the right bottom corner of CTU
      {
        // same as for last column but not last row
        posC0 = posRB.offset(4, 4);
      }
    }

    if ((C0Avail && getColocatedMVP(pu, eRefPicList, posC0, cColMv, refIdx_Col)) || getColocatedMVP(pu, eRefPicList, posC1, cColMv, refIdx_Col))
    {
      pInfo->mvCand[pInfo->numCand++] = cColMv;
    }
  }
  if (pInfo->numCand > AMVP_MAX_NUM_CANDS)
  {
    pInfo->numCand = AMVP_MAX_NUM_CANDS;
  }

  while (pInfo->numCand < AMVP_MAX_NUM_CANDS)
  {
#if JVET_K0346 || JVET_K_AFFINE
    const bool prec = pInfo->mvCand[pInfo->numCand].highPrec;
    pInfo->mvCand[pInfo->numCand] = Mv( 0, 0, prec );
#else
    pInfo->mvCand[pInfo->numCand] = Mv( 0, 0 );
#endif
    pInfo->numCand++;
  }

  if( pu.cs->sps->getSpsNext().getUseHighPrecMv() )
  {
    for( Mv &mv : pInfo->mvCand )
    {
      if( mv.highPrec ) mv.setLowPrec();
    }
  }
#if JVET_K0357_AMVR
  if (pu.cu->imv != 0)
  {
    unsigned imvShift = pu.cu->imv << 1;
    for (int i = 0; i < pInfo->numCand; i++)
    {
      roundMV(pInfo->mvCand[i], imvShift);
    }
  }
#endif
#if JVET_K0346 || JVET_K_AFFINE
  if (pu.cs->sps->getSpsNext().getUseHighPrecMv())
  {
    for (Mv &mv : pInfo->mvCand)
    {
      if (mv.highPrec) mv.setLowPrec();
    }
  }
#endif
}


#if JVET_K_AFFINE
#if JVET_K0337_AFFINE_MVP_IMPROVE
const int getAvailableAffineNeighbours( const PredictionUnit &pu, const PredictionUnit* npu[] )
{
  const Position posLT = pu.Y().topLeft();
  const Position posRT = pu.Y().topRight();
  const Position posLB = pu.Y().bottomLeft();

  int num = 0;
  const PredictionUnit* puLeft = pu.cs->getPURestricted( posLB.offset( -1, 0 ), pu, pu.chType );
  if ( puLeft && puLeft->cu->affine )
  {
    npu[num++] = puLeft;
  }

  const PredictionUnit* puAbove = pu.cs->getPURestricted( posRT.offset( 0, -1 ), pu, pu.chType );
  if ( puAbove && puAbove->cu->affine )
  {
    npu[num++] = puAbove;
  }

  const PredictionUnit* puAboveRight = pu.cs->getPURestricted( posRT.offset( 1, -1 ), pu, pu.chType );
  if ( puAboveRight && puAboveRight->cu->affine )
  {
    npu[num++] = puAboveRight;
  }

  const PredictionUnit *puLeftBottom = pu.cs->getPURestricted( posLB.offset( -1, 1 ), pu, pu.chType );
  if ( puLeftBottom && puLeftBottom->cu->affine )
  {
    npu[num++] = puLeftBottom;
  }

  const PredictionUnit *puAboveLeft = pu.cs->getPURestricted( posLT.offset( -1, -1 ), pu, pu.chType );
  if ( puAboveLeft && puAboveLeft->cu->affine )
  {
    npu[num++] = puAboveLeft;
  }

  return num;
}
#endif

void PU::xInheritedAffineMv( const PredictionUnit &pu, const PredictionUnit* puNeighbour, RefPicList eRefPicList, Mv rcMv[3] )
{
  int posNeiX = puNeighbour->Y().pos().x;
  int posNeiY = puNeighbour->Y().pos().y;
  int posCurX = pu.Y().pos().x;
  int posCurY = pu.Y().pos().y;

  int neiW = puNeighbour->Y().width;
  int curW = pu.Y().width;
#if JVET_K0337_AFFINE_6PARA || !JVET_K_AFFINE_BUG_FIXES
  int neiH = puNeighbour->Y().height;
  int curH = pu.Y().height;
#endif
  
  Mv mvLT, mvRT, mvLB;
  const Position posLT = puNeighbour->Y().topLeft();
  const Position posRT = puNeighbour->Y().topRight();
  const Position posLB = puNeighbour->Y().bottomLeft();
  mvLT = puNeighbour->getMotionInfo( posLT ).mv[eRefPicList];
  mvRT = puNeighbour->getMotionInfo( posRT ).mv[eRefPicList];
  mvLB = puNeighbour->getMotionInfo( posLB ).mv[eRefPicList];

#if JVET_K_AFFINE_BUG_FIXES
  int shift = MAX_CU_DEPTH;
  int iDMvHorX, iDMvHorY, iDMvVerX, iDMvVerY;

  iDMvHorX = (mvRT - mvLT).getHor() << (shift - g_aucLog2[neiW]);
  iDMvHorY = (mvRT - mvLT).getVer() << (shift - g_aucLog2[neiW]);
#if JVET_K0337_AFFINE_6PARA
  if ( puNeighbour->cu->affineType == AFFINEMODEL_6PARAM )
  {
    iDMvVerX = (mvLB - mvLT).getHor() << (shift - g_aucLog2[neiH]);
    iDMvVerY = (mvLB - mvLT).getVer() << (shift - g_aucLog2[neiH]);
  }
  else
#endif
  {
    iDMvVerX = -iDMvHorY;
    iDMvVerY = iDMvHorX;
  }

  int iMvScaleHor = mvLT.getHor() << shift;
  int iMvScaleVer = mvLT.getVer() << shift;
  int horTmp, verTmp;

  // v0
  horTmp = iMvScaleHor + iDMvHorX * (posCurX - posNeiX) + iDMvVerX * (posCurY - posNeiY);
  verTmp = iMvScaleVer + iDMvHorY * (posCurX - posNeiX) + iDMvVerY * (posCurY - posNeiY);
  roundAffineMv( horTmp, verTmp, shift );
  rcMv[0] = Mv( horTmp, verTmp, true );

  // v1
  horTmp = iMvScaleHor + iDMvHorX * (posCurX + curW - posNeiX) + iDMvVerX * (posCurY - posNeiY);
  verTmp = iMvScaleVer + iDMvHorY * (posCurX + curW - posNeiX) + iDMvVerY * (posCurY - posNeiY);
  roundAffineMv( horTmp, verTmp, shift );
  rcMv[1] = Mv( horTmp, verTmp, true );

  // v2
#if JVET_K0337_AFFINE_6PARA
  if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
  {
    horTmp = iMvScaleHor + iDMvHorX * (posCurX - posNeiX) + iDMvVerX * (posCurY + curH - posNeiY);
    verTmp = iMvScaleVer + iDMvHorY * (posCurX - posNeiX) + iDMvVerY * (posCurY + curH - posNeiY);
    roundAffineMv( horTmp, verTmp, shift );
    rcMv[2] = Mv( horTmp, verTmp, true );
  }
#endif
#else
  rcMv[0].hor = int( mvLT.hor + 1.0 * (mvRT.hor - mvLT.hor) * (posCurX - posNeiX) / neiW + 1.0 * (mvLB.hor - mvLT.hor) * (posCurY - posNeiY) / neiH );
  rcMv[0].ver = int( mvLT.ver + 1.0 * (mvRT.ver - mvLT.ver) * (posCurX - posNeiX) / neiW + 1.0 * (mvLB.ver - mvLT.ver) * (posCurY - posNeiY) / neiH );
  rcMv[1].hor = int( rcMv[0].hor + 1.0 * (mvRT.hor - mvLT.hor) * curW / neiW );
  rcMv[1].ver = int( rcMv[0].ver + 1.0 * (mvRT.ver - mvLT.ver) * curW / neiW );
  rcMv[2].hor = int( rcMv[0].hor + 1.0 * (mvLB.hor - mvLT.hor) * curH / neiH );
  rcMv[2].ver = int( rcMv[0].ver + 1.0 * (mvLB.ver - mvLT.ver) * curH / neiH );

  rcMv[0].highPrec = true;
  rcMv[1].highPrec = true;
  rcMv[2].highPrec = true;
#endif
}

#if !JVET_K0337_AFFINE_MVP_IMPROVE
bool isValidAffineCandidate( const PredictionUnit &pu, Mv cMv0, Mv cMv1, Mv cMv2, int& riDV )
{
  Mv zeroMv(0, 0);
  Mv deltaHor = cMv1 - cMv0;
  Mv deltaVer = cMv2 - cMv0;

  // same motion vector, translation model
  if ( deltaHor == zeroMv )
    return false;

  deltaHor.setHighPrec();
  deltaVer.setHighPrec();

  // S/8, but the Mv is 4 precision, so change to S/2
  int width = pu.Y().width;
  int height = pu.Y().height;
  int iDiffHor = width>>1;
  int iDiffVer = height>>1;

  if ( deltaHor.getAbsHor() > iDiffHor || deltaHor.getAbsVer() > iDiffVer || deltaVer.getAbsHor() > iDiffHor || deltaVer.getAbsVer() > iDiffVer )
  {
    return false;
  }
  // Calculate DV
  riDV = abs( deltaHor.getHor() * height - deltaVer.getVer() * width ) + abs( deltaHor.getVer() * height + deltaVer.getHor() * width );
  return true;
}
#endif

void PU::fillAffineMvpCand(PredictionUnit &pu, const RefPicList &eRefPicList, const int &refIdx, AffineAMVPInfo &affiAMVPInfo)
{
  affiAMVPInfo.numCand = 0;

  if (refIdx < 0)
  {
    return;
  }

#if JVET_K0337_AFFINE_MVP_IMPROVE
  const int curWidth = pu.Y().width;
  const int curHeight = pu.Y().height;

  // insert inherited affine candidates
  Mv outputAffineMv[3];
  const int maxNei = 5;
  const PredictionUnit* npu[maxNei];
  int numAffNeigh = getAvailableAffineNeighbours( pu, npu );
  int targetRefPOC = pu.cu->slice->getRefPOC( eRefPicList, refIdx );

  for ( int refPicList = 0; refPicList < 2 && affiAMVPInfo.numCand < AMVP_MAX_NUM_CANDS; refPicList++ )
  {
    RefPicList eTestRefPicList = (refPicList == 0) ? eRefPicList : RefPicList( 1 - eRefPicList );

    for ( int neighIdx = 0; neighIdx < numAffNeigh && affiAMVPInfo.numCand < AMVP_MAX_NUM_CANDS; neighIdx++ )
    {
      const PredictionUnit* puNeighbour = npu[neighIdx];

      if ( ((puNeighbour->interDir & (eTestRefPicList + 1)) == 0) || pu.cu->slice->getRefPOC( eTestRefPicList, puNeighbour->refIdx[eTestRefPicList] ) != targetRefPOC )
      {
        continue;
      }

      xInheritedAffineMv( pu, puNeighbour, eTestRefPicList, outputAffineMv );

      outputAffineMv[0].roundMV2SignalPrecision();
      outputAffineMv[1].roundMV2SignalPrecision();
#if JVET_K0337_AFFINE_6PARA
      if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
      {
        outputAffineMv[2].roundMV2SignalPrecision();
      }

      if ( affiAMVPInfo.numCand == 0
        || (pu.cu->affineType == AFFINEMODEL_4PARAM && (outputAffineMv[0] != affiAMVPInfo.mvCandLT[0] || outputAffineMv[1] != affiAMVPInfo.mvCandRT[0]))
        || (pu.cu->affineType == AFFINEMODEL_6PARAM && (outputAffineMv[0] != affiAMVPInfo.mvCandLT[0] || outputAffineMv[1] != affiAMVPInfo.mvCandRT[0] || outputAffineMv[2] != affiAMVPInfo.mvCandLB[0]))
        )
#else
      if ( affiAMVPInfo.numCand == 0 || outputAffineMv[0] != affiAMVPInfo.mvCandLT[0] || outputAffineMv[1] != affiAMVPInfo.mvCandRT[0] )
#endif
      {
        affiAMVPInfo.mvCandLT[affiAMVPInfo.numCand] = outputAffineMv[0];
        affiAMVPInfo.mvCandRT[affiAMVPInfo.numCand] = outputAffineMv[1];
        affiAMVPInfo.mvCandLB[affiAMVPInfo.numCand] = outputAffineMv[2];
        affiAMVPInfo.numCand++;
      }
    }
  }

  if ( affiAMVPInfo.numCand >= AMVP_MAX_NUM_CANDS )
  {
    return;
  }

  // insert constructed affine candidates
  int cornerMVPattern = 0;
  Position posLT = pu.Y().topLeft();
  Position posRT = pu.Y().topRight();
  Position posLB = pu.Y().bottomLeft();

  //-------------------  V0 (START) -------------------//
  AMVPInfo amvpInfo0;
  amvpInfo0.numCand = 0;

  // A->C: Above Left, Above, Left
  addMVPCandUnscaled( pu, eRefPicList, refIdx, posLT, MD_ABOVE_LEFT, amvpInfo0, true );
  if ( amvpInfo0.numCand < 1 )
  {
    addMVPCandUnscaled( pu, eRefPicList, refIdx, posLT, MD_ABOVE, amvpInfo0, true );
  }
  if ( amvpInfo0.numCand < 1 )
  {
    addMVPCandUnscaled( pu, eRefPicList, refIdx, posLT, MD_LEFT, amvpInfo0, true );
  }
  cornerMVPattern = cornerMVPattern | amvpInfo0.numCand;

  //-------------------  V1 (START) -------------------//
  AMVPInfo amvpInfo1;
  amvpInfo1.numCand = 0;

  // D->E: Above, Above Right
  addMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE, amvpInfo1, true );
  if ( amvpInfo1.numCand < 1 )
  {
    addMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE_RIGHT, amvpInfo1, true );
  }
  cornerMVPattern = cornerMVPattern | (amvpInfo1.numCand << 1);

  //-------------------  V2 (START) -------------------//
  AMVPInfo amvpInfo2;
  amvpInfo2.numCand = 0;

  // F->G: Left, Below Left
  addMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_LEFT, amvpInfo2, true );
  if ( amvpInfo2.numCand < 1 )
  {
    addMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_BELOW_LEFT, amvpInfo2, true );
  }
  cornerMVPattern = cornerMVPattern | (amvpInfo2.numCand << 2);

  outputAffineMv[0] = amvpInfo0.mvCand[0];
  outputAffineMv[1] = amvpInfo1.mvCand[0];
  outputAffineMv[2] = amvpInfo2.mvCand[0];

  outputAffineMv[0].setHighPrec();
  outputAffineMv[1].setHighPrec();
  outputAffineMv[2].setHighPrec();

  outputAffineMv[0].roundMV2SignalPrecision();
  outputAffineMv[1].roundMV2SignalPrecision();
  outputAffineMv[2].roundMV2SignalPrecision();

  if ( cornerMVPattern == 7 || cornerMVPattern == 3 || cornerMVPattern == 5 )
  {
#if JVET_K0337_AFFINE_6PARA
    if ( cornerMVPattern == 3 && pu.cu->affineType == AFFINEMODEL_6PARAM ) // V0 V1 are available, derived V2 for 6-para
    {
      int shift = MAX_CU_DEPTH;
      int vx2 = (outputAffineMv[0].getHor() << shift) - ((outputAffineMv[1].getVer() - outputAffineMv[0].getVer()) << (shift + g_aucLog2[curHeight] - g_aucLog2[curWidth]));
      int vy2 = (outputAffineMv[0].getVer() << shift) + ((outputAffineMv[1].getHor() - outputAffineMv[0].getHor()) << (shift + g_aucLog2[curHeight] - g_aucLog2[curWidth]));
      roundAffineMv( vx2, vy2, shift );
      outputAffineMv[2].set( vx2, vy2 );
      outputAffineMv[2].roundMV2SignalPrecision();
    }
#endif

    if ( cornerMVPattern == 5 ) // V0 V2 are available, derived V1
    {
      int shift = MAX_CU_DEPTH;
      int vx1 = (outputAffineMv[0].getHor() << shift) + ((outputAffineMv[2].getVer() - outputAffineMv[0].getVer()) << (shift + g_aucLog2[curWidth] - g_aucLog2[curHeight]));
      int vy1 = (outputAffineMv[0].getVer() << shift) - ((outputAffineMv[2].getHor() - outputAffineMv[0].getHor()) << (shift + g_aucLog2[curWidth] - g_aucLog2[curHeight]));
      roundAffineMv( vx1, vy1, shift );
      outputAffineMv[1].set( vx1, vy1 );
      outputAffineMv[1].roundMV2SignalPrecision();
    }

#if JVET_K0337_AFFINE_6PARA
    if ( affiAMVPInfo.numCand == 0
      || (pu.cu->affineType == AFFINEMODEL_4PARAM && (outputAffineMv[0] != affiAMVPInfo.mvCandLT[0] || outputAffineMv[1] != affiAMVPInfo.mvCandRT[0]))
      || (pu.cu->affineType == AFFINEMODEL_6PARAM && (outputAffineMv[0] != affiAMVPInfo.mvCandLT[0] || outputAffineMv[1] != affiAMVPInfo.mvCandRT[0] || outputAffineMv[2] != affiAMVPInfo.mvCandLB[0]))
      )
#else
    if ( affiAMVPInfo.numCand == 0 || outputAffineMv[0] != affiAMVPInfo.mvCandLT[0] || outputAffineMv[1] != affiAMVPInfo.mvCandRT[0] )
#endif
    {
      affiAMVPInfo.mvCandLT[affiAMVPInfo.numCand] = outputAffineMv[0];
      affiAMVPInfo.mvCandRT[affiAMVPInfo.numCand] = outputAffineMv[1];
      affiAMVPInfo.mvCandLB[affiAMVPInfo.numCand] = outputAffineMv[2];
      affiAMVPInfo.numCand++;
    }
  }
#else
  //-- Get Spatial MV
  Position posLT = pu.Y().topLeft();
  Position posRT = pu.Y().topRight();
  Position posLB = pu.Y().bottomLeft();


  //-------------------  V0 (START) -------------------//
  AMVPInfo amvpInfo0;
  amvpInfo0.numCand = 0;

  // A->C: Above Left, Above, Left
  addMVPCandUnscaled( pu, eRefPicList, refIdx, posLT, MD_ABOVE_LEFT, amvpInfo0, true );
  addMVPCandUnscaled( pu, eRefPicList, refIdx, posLT, MD_ABOVE,      amvpInfo0, true );
  addMVPCandUnscaled( pu, eRefPicList, refIdx, posLT, MD_LEFT,       amvpInfo0, true );

  if( amvpInfo0.numCand < AFFINE_MAX_NUM_V0 )
  {
    addMVPCandWithScaling( pu, eRefPicList, refIdx, posLT, MD_ABOVE_LEFT, amvpInfo0, true );
    if ( amvpInfo0.numCand < AFFINE_MAX_NUM_V0 )
    {
      addMVPCandWithScaling( pu, eRefPicList, refIdx, posLT, MD_ABOVE, amvpInfo0, true );
      if ( amvpInfo0.numCand < AFFINE_MAX_NUM_V0 )
      {
        addMVPCandWithScaling( pu, eRefPicList, refIdx, posLT, MD_LEFT, amvpInfo0, true );
      }
    }
  }

  //-------------------  V1 (START) -------------------//
  AMVPInfo amvpInfo1;
  amvpInfo1.numCand = 0;

  // D->E: Above, Above Right
  addMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE,       amvpInfo1, true );
  addMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE_RIGHT, amvpInfo1, true );
  if( amvpInfo1.numCand < AFFINE_MAX_NUM_V1 )
  {
    addMVPCandWithScaling( pu, eRefPicList, refIdx, posRT, MD_ABOVE, amvpInfo1, true );
    if( amvpInfo1.numCand < AFFINE_MAX_NUM_V1 )
    {
      addMVPCandWithScaling( pu, eRefPicList, refIdx, posRT, MD_ABOVE_RIGHT, amvpInfo1, true );
    }
  }

  //-------------------  V2 (START) -------------------//
  AMVPInfo amvpInfo2;
  amvpInfo2.numCand = 0;

  // F->G: Left, Below Left
  addMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_LEFT,       amvpInfo2, true );
  addMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_BELOW_LEFT, amvpInfo2, true );
  if( amvpInfo2.numCand < AFFINE_MAX_NUM_V2 )
  {
    addMVPCandWithScaling( pu, eRefPicList, refIdx, posLB, MD_LEFT, amvpInfo2, true );
    if( amvpInfo2.numCand < AFFINE_MAX_NUM_V2 )
    {
      addMVPCandWithScaling( pu, eRefPicList, refIdx, posLB, MD_BELOW_LEFT, amvpInfo2, true );
    }
  }

  for ( int i = 0; i < amvpInfo0.numCand; i++ )
  {
    amvpInfo0.mvCand[i].setHighPrec();
  }
  for ( int i = 0; i < amvpInfo1.numCand; i++ )
  {
    amvpInfo1.mvCand[i].setHighPrec();
  }
  for ( int i = 0; i < amvpInfo2.numCand; i++ )
  {
    amvpInfo2.mvCand[i].setHighPrec();
  }

  // Check Valid Candidates and Sort through DV
  int   iRecord[AFFINE_MAX_NUM_COMB][3];
  int   iDV[AFFINE_MAX_NUM_COMB];
  int   iTempDV;
  int   iCount = 0;
  for ( int i = 0; i < amvpInfo0.numCand; i++ )
  {
    for ( int j = 0; j < amvpInfo1.numCand; j++ )
    {
      for ( int k = 0; k < amvpInfo2.numCand; k++ )
      {
        bool bValid = isValidAffineCandidate( pu, amvpInfo0.mvCand[i], amvpInfo1.mvCand[j], amvpInfo2.mvCand[k], iDV[iCount] );
        if ( bValid )
        {
          // Sort
          if ( iCount==0 || iDV[iCount]>=iDV[iCount-1] )
          {
            iRecord[iCount][0] = i;
            iRecord[iCount][1] = j;
            iRecord[iCount][2] = k;
          }
          else
          {
            // save last element
            iTempDV = iDV[iCount];
            // find position and move back record
            int m = 0;
            for ( m = iCount - 1; m >= 0 && iTempDV < iDV[m]; m-- )
            {
              iDV[m+1] = iDV[m];
              memcpy( iRecord[m+1], iRecord[m], sizeof(int) * 3 );
            }
            // insert
            iDV[m+1] = iTempDV;
            iRecord[m+1][0] = i;
            iRecord[m+1][1] = j;
            iRecord[m+1][2] = k;
          }
          iCount++;
        }
      }
    }
  }

  affiAMVPInfo.numCand = std::min<int>(iCount, AMVP_MAX_NUM_CANDS);

  int iWidth = pu.Y().width;
  int iHeight = pu.Y().height;

  for ( int i = 0; i < affiAMVPInfo.numCand; i++ )
  {
    affiAMVPInfo.mvCandLT[i] = amvpInfo0.mvCand[ iRecord[i][0] ];
    affiAMVPInfo.mvCandRT[i] = amvpInfo1.mvCand[ iRecord[i][1] ];
    affiAMVPInfo.mvCandLB[i] = amvpInfo2.mvCand[ iRecord[i][2] ];

    affiAMVPInfo.mvCandLT[i].roundMV2SignalPrecision();
    affiAMVPInfo.mvCandRT[i].roundMV2SignalPrecision();

    clipMv( affiAMVPInfo.mvCandLT[i], pu.cu->lumaPos(), *pu.cs->sps );
    clipMv( affiAMVPInfo.mvCandRT[i], pu.cu->lumaPos(), *pu.cs->sps );

    int vx2 =  - ( affiAMVPInfo.mvCandRT[i].getVer() - affiAMVPInfo.mvCandLT[i].getVer() ) * iHeight / iWidth + affiAMVPInfo.mvCandLT[i].getHor();
    int vy2 =    ( affiAMVPInfo.mvCandRT[i].getHor() - affiAMVPInfo.mvCandLT[i].getHor() ) * iHeight / iWidth + affiAMVPInfo.mvCandLT[i].getVer();

    affiAMVPInfo.mvCandLB[i] = Mv( vx2, vy2, true );
    if( !pu.cu->cs->pcv->only2Nx2N )
    {
      affiAMVPInfo.mvCandLB[i].roundMV2SignalPrecision();
    }

    clipMv( affiAMVPInfo.mvCandLB[i], pu.cu->lumaPos(), *pu.cs->sps );
  }
#endif

  if ( affiAMVPInfo.numCand < 2 )
  {
    AMVPInfo amvpInfo;
    PU::fillMvpCand( pu, eRefPicList, refIdx, amvpInfo );

    int iAdd = amvpInfo.numCand - affiAMVPInfo.numCand;
    for ( int i = 0; i < iAdd; i++ )
    {
      amvpInfo.mvCand[i].setHighPrec();
#if !JVET_K_AFFINE_BUG_FIXES
      clipMv( amvpInfo.mvCand[i], pu.cu->lumaPos(), *pu.cs->sps );
#endif
      affiAMVPInfo.mvCandLT[affiAMVPInfo.numCand] = amvpInfo.mvCand[i];
      affiAMVPInfo.mvCandRT[affiAMVPInfo.numCand] = amvpInfo.mvCand[i];
      affiAMVPInfo.mvCandLB[affiAMVPInfo.numCand] = amvpInfo.mvCand[i];
      affiAMVPInfo.numCand++;
    }
  }
}
#endif

#if JVET_K_AFFINE
bool PU::addMVPCandUnscaled( const PredictionUnit &pu, const RefPicList &eRefPicList, const int &iRefIdx, const Position &pos, const MvpDir &eDir, AMVPInfo &info, bool affine )
#else
bool PU::addMVPCandUnscaled( const PredictionUnit &pu, const RefPicList &eRefPicList, const int &iRefIdx, const Position &pos, const MvpDir &eDir, AMVPInfo &info )
#endif
{
        CodingStructure &cs    = *pu.cs;
  const PredictionUnit *neibPU = NULL;
        Position neibPos;

  switch (eDir)
  {
  case MD_LEFT:
    neibPos = pos.offset( -1,  0 );
    break;
  case MD_ABOVE:
    neibPos = pos.offset(  0, -1 );
    break;
  case MD_ABOVE_RIGHT:
    neibPos = pos.offset(  1, -1 );
    break;
  case MD_BELOW_LEFT:
    neibPos = pos.offset( -1,  1 );
    break;
  case MD_ABOVE_LEFT:
    neibPos = pos.offset( -1, -1 );
    break;
  default:
    break;
  }

  neibPU = cs.getPURestricted( neibPos, pu, pu.chType );

  if( neibPU == NULL || !CU::isInter( *neibPU->cu ) )
  {
    return false;
  }

  const MotionInfo& neibMi        = neibPU->getMotionInfo( neibPos );

  const int        currRefPOC     = cs.slice->getRefPic( eRefPicList, iRefIdx )->getPOC();
  const RefPicList eRefPicList2nd = ( eRefPicList == REF_PIC_LIST_0 ) ? REF_PIC_LIST_1 : REF_PIC_LIST_0;

  for( int predictorSource = 0; predictorSource < 2; predictorSource++ ) // examine the indicated reference picture list, then if not available, examine the other list.
  {
    const RefPicList eRefPicListIndex = ( predictorSource == 0 ) ? eRefPicList : eRefPicList2nd;
    const int        neibRefIdx       = neibMi.refIdx[eRefPicListIndex];

    if( neibRefIdx >= 0 && currRefPOC == cs.slice->getRefPOC( eRefPicListIndex, neibRefIdx ) )
    {
#if JVET_K_AFFINE
      if( affine )
      {
        int i = 0;
        for( i = 0; i < info.numCand; i++ )
        {
          if( info.mvCand[i] == neibMi.mv[eRefPicListIndex] )
          {
            break;
          }
        }
        if( i == info.numCand )
        {
          info.mvCand[info.numCand++] = neibMi.mv[eRefPicListIndex];
          Mv cMvHigh = neibMi.mv[eRefPicListIndex];
          cMvHigh.setHighPrec();
//          CHECK( !neibMi.mv[eRefPicListIndex].highPrec, "Unexpected low precision mv.");
          return true;
        }
      }
      else
#endif
      {
        info.mvCand[info.numCand++] = neibMi.mv[eRefPicListIndex];
        return true;
      }
    }
  }


  return false;
}

/**
* \param pInfo
* \param eRefPicList
* \param iRefIdx
* \param uiPartUnitIdx
* \param eDir
* \returns bool
*/
#if JVET_K_AFFINE
bool PU::addMVPCandWithScaling( const PredictionUnit &pu, const RefPicList &eRefPicList, const int &iRefIdx, const Position &pos, const MvpDir &eDir, AMVPInfo &info, bool affine )
#else
bool PU::addMVPCandWithScaling( const PredictionUnit &pu, const RefPicList &eRefPicList, const int &iRefIdx, const Position &pos, const MvpDir &eDir, AMVPInfo &info )
#endif
{
        CodingStructure &cs    = *pu.cs;
  const Slice &slice           = *cs.slice;
  const PredictionUnit *neibPU = NULL;
        Position neibPos;

  switch( eDir )
  {
  case MD_LEFT:
    neibPos = pos.offset( -1,  0 );
    break;
  case MD_ABOVE:
    neibPos = pos.offset(  0, -1 );
    break;
  case MD_ABOVE_RIGHT:
    neibPos = pos.offset(  1, -1 );
    break;
  case MD_BELOW_LEFT:
    neibPos = pos.offset( -1,  1 );
    break;
  case MD_ABOVE_LEFT:
    neibPos = pos.offset( -1, -1 );
    break;
  default:
    break;
  }

  neibPU = cs.getPURestricted( neibPos, pu, pu.chType );

  if( neibPU == NULL || !CU::isInter( *neibPU->cu ) )
  {
    return false;
  }

  const MotionInfo& neibMi        = neibPU->getMotionInfo( neibPos );

  const RefPicList eRefPicList2nd = ( eRefPicList == REF_PIC_LIST_0 ) ? REF_PIC_LIST_1 : REF_PIC_LIST_0;

  const int  currPOC            = slice.getPOC();
  const int  currRefPOC         = slice.getRefPic( eRefPicList, iRefIdx )->poc;
  const bool bIsCurrRefLongTerm = slice.getRefPic( eRefPicList, iRefIdx )->longTerm;
  const int  neibPOC            = currPOC;

  for( int predictorSource = 0; predictorSource < 2; predictorSource++ ) // examine the indicated reference picture list, then if not available, examine the other list.
  {
    const RefPicList eRefPicListIndex = (predictorSource == 0) ? eRefPicList : eRefPicList2nd;
    const int        neibRefIdx       = neibMi.refIdx[eRefPicListIndex];
    if( neibRefIdx >= 0 )
    {
      const bool bIsNeibRefLongTerm = slice.getRefPic(eRefPicListIndex, neibRefIdx)->longTerm;

      if (bIsCurrRefLongTerm == bIsNeibRefLongTerm)
      {
        Mv cMv = neibMi.mv[eRefPicListIndex];

        if( !( bIsCurrRefLongTerm /* || bIsNeibRefLongTerm*/) )
        {
          const int neibRefPOC = slice.getRefPOC( eRefPicListIndex, neibRefIdx );
          const int scale      = xGetDistScaleFactor( currPOC, currRefPOC, neibPOC, neibRefPOC );

          if( scale != 4096 )
          {
#if JVET_K0346 || JVET_K_AFFINE
            if( slice.getSPS()->getSpsNext().getUseHighPrecMv() )
            {
              cMv.setHighPrec();
            }
#endif
            cMv = cMv.scaleMv( scale );
          }
        }

#if JVET_K_AFFINE
        if( affine )
        {
          int i;
          for( i = 0; i < info.numCand; i++ )
          {
            if( info.mvCand[i] == cMv )
            {
              break;
            }
          }
          if( i == info.numCand )
          {
            info.mvCand[info.numCand++] = cMv;
//            CHECK( !cMv.highPrec, "Unexpected low precision mv.");
            return true;
          }
        }
        else
#endif
        {
          info.mvCand[info.numCand++] = cMv;
          return true;
        }
      }
    }
  }


  return false;
}

bool PU::isBipredRestriction(const PredictionUnit &pu)
{
#if JVET_K0346
  const SPSNext &spsNext = pu.cs->sps->getSpsNext();
  if( !pu.cs->pcv->only2Nx2N && !spsNext.getUseSubPuMvp() && pu.cu->lumaSize().width == 8 && ( pu.lumaSize().width < 8 || pu.lumaSize().height < 8 ) )
#else
  if( !pu.cs->pcv->only2Nx2N && pu.cu->lumaSize().width == 8 && ( pu.lumaSize().width < 8 || pu.lumaSize().height < 8 ) )
#endif
  {
    return true;
  }
  return false;
}


#if JVET_K_AFFINE
const PredictionUnit* getFirstAvailableAffineNeighbour( const PredictionUnit &pu )
{
  const Position posLT = pu.Y().topLeft();
  const Position posRT = pu.Y().topRight();
  const Position posLB = pu.Y().bottomLeft();

  const PredictionUnit* puLeft = pu.cs->getPURestricted( posLB.offset( -1, 0 ), pu, pu.chType );
  if( puLeft && puLeft->cu->affine )
  {
    return puLeft;
  }
  const PredictionUnit* puAbove = pu.cs->getPURestricted( posRT.offset( 0, -1 ), pu, pu.chType );
  if( puAbove && puAbove->cu->affine )
  {
    return puAbove;
  }
  const PredictionUnit* puAboveRight = pu.cs->getPURestricted( posRT.offset( 1, -1 ), pu, pu.chType );
  if( puAboveRight && puAboveRight->cu->affine )
  {
    return puAboveRight;
  }
  const PredictionUnit *puLeftBottom = pu.cs->getPURestricted( posLB.offset( -1, 1 ), pu, pu.chType );
  if( puLeftBottom && puLeftBottom->cu->affine )
  {
    return puLeftBottom;
  }
  const PredictionUnit *puAboveLeft = pu.cs->getPURestricted( posLT.offset( -1, -1 ), pu, pu.chType );
  if( puAboveLeft && puAboveLeft->cu->affine )
  {
    return puAboveLeft;
  }
  return nullptr;
}

bool PU::isAffineMrgFlagCoded( const PredictionUnit &pu )
{
#if JVET_K_AFFINE_BUG_FIXES
  if ( pu.cu->lumaSize().width < 8 || pu.cu->lumaSize().height < 8 )
#else
  if( ( pu.cs->sps->getSpsNext().getUseQTBT() ) && pu.cu->lumaSize().area() < 64 )
#endif
  {
    return false;
  }
  return getFirstAvailableAffineNeighbour( pu ) != nullptr;
}

void PU::getAffineMergeCand( const PredictionUnit &pu, MvField (*mvFieldNeighbours)[3], unsigned char &interDirNeighbours, int &numValidMergeCand )
{
  for ( int mvNum = 0; mvNum < 3; mvNum++ )
  {
    mvFieldNeighbours[0][mvNum].setMvField( Mv(), -1 );
    mvFieldNeighbours[1][mvNum].setMvField( Mv(), -1 );
  }

  const PredictionUnit* puFirstNeighbour = getFirstAvailableAffineNeighbour( pu );
  if( puFirstNeighbour == nullptr )
  {
    numValidMergeCand = -1;
    return;
  }
  else
  {
    numValidMergeCand = 1;
  }

  // get Inter Dir
  interDirNeighbours = puFirstNeighbour->getMotionInfo().interDir;

#if JVET_K0337_AFFINE_6PARA // inherit affine type
  pu.cu->affineType = puFirstNeighbour->cu->affineType;
#endif

  // derive Mv from neighbor affine block
  Mv cMv[3];
  if ( interDirNeighbours != 2 )
  {
    xInheritedAffineMv( pu, puFirstNeighbour, REF_PIC_LIST_0, cMv );
    for ( int mvNum = 0; mvNum < 3; mvNum++ )
    {
      mvFieldNeighbours[0][mvNum].setMvField( cMv[mvNum], puFirstNeighbour->refIdx[0] );
    }
  }

  if ( pu.cs->slice->isInterB() )
  {
    if ( interDirNeighbours != 1 )
    {
      xInheritedAffineMv( pu, puFirstNeighbour, REF_PIC_LIST_1, cMv );
      for ( int mvNum = 0; mvNum < 3; mvNum++ )
      {
        mvFieldNeighbours[1][mvNum].setMvField( cMv[mvNum], puFirstNeighbour->refIdx[1] );
      }
    }
  }
}

void PU::setAllAffineMvField( PredictionUnit &pu, MvField *mvField, RefPicList eRefList )
{
  // Set Mv
  Mv mv[3];
  for ( int i = 0; i < 3; i++ )
  {
    mv[i] = mvField[i].mv;
  }
  setAllAffineMv( pu, mv[0], mv[1], mv[2], eRefList );

  // Set RefIdx
  CHECK( mvField[0].refIdx != mvField[1].refIdx || mvField[0].refIdx != mvField[2].refIdx, "Affine mv corners don't have the same refIdx." );
  pu.refIdx[eRefList] = mvField[0].refIdx;
}

void PU::setAllAffineMv( PredictionUnit& pu, Mv affLT, Mv affRT, Mv affLB, RefPicList eRefList )
{
  int width  = pu.Y().width;
  int shift = MAX_CU_DEPTH;

  affLT.setHighPrec();
  affRT.setHighPrec();
  affLB.setHighPrec();
  int deltaMvHorX, deltaMvHorY, deltaMvVerX, deltaMvVerY;
  deltaMvHorX = (affRT - affLT).getHor() << (shift - g_aucLog2[width]);
  deltaMvHorY = (affRT - affLT).getVer() << (shift - g_aucLog2[width]);
#if JVET_K0337_AFFINE_6PARA
  int height = pu.Y().height;
  if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
  {
    deltaMvVerX = (affLB - affLT).getHor() << (shift - g_aucLog2[height]);
    deltaMvVerY = (affLB - affLT).getVer() << (shift - g_aucLog2[height]);
  }
  else
  {
    deltaMvVerX = -deltaMvHorY;
    deltaMvVerY = deltaMvHorX;
  }
#else
  deltaMvVerX = -deltaMvHorY;
  deltaMvVerY = deltaMvHorX;
#endif

  int mvScaleHor = affLT.getHor() << shift;
  int mvScaleVer = affLT.getVer() << shift;

  int blockWidth = AFFINE_MIN_BLOCK_SIZE;
  int blockHeight = AFFINE_MIN_BLOCK_SIZE;
  const int halfBW = blockWidth >> 1;
  const int halfBH = blockHeight >> 1;

  MotionBuf mb = pu.getMotionBuf();
  int mvScaleTmpHor, mvScaleTmpVer;
  for ( int h = 0; h < pu.Y().height; h += blockHeight )
  {
    for ( int w = 0; w < pu.Y().width; w += blockWidth )
    {
      mvScaleTmpHor = mvScaleHor + deltaMvHorX * (halfBW + w) + deltaMvVerX * (halfBH + h);
      mvScaleTmpVer = mvScaleVer + deltaMvHorY * (halfBW + w) + deltaMvVerY * (halfBH + h);
#if JVET_K_AFFINE_BUG_FIXES
      roundAffineMv( mvScaleTmpHor, mvScaleTmpVer, shift );
#else
      mvScaleTmpHor >>= shift;
      mvScaleTmpVer >>= shift;
#endif

      for ( int y = (h >> MIN_CU_LOG2); y < ((h + blockHeight) >> MIN_CU_LOG2); y++ )
      {
        for ( int x = (w >> MIN_CU_LOG2); x < ((w + blockHeight) >> MIN_CU_LOG2); x++ )
        {
          mb.at( x, y ).mv[eRefList] = Mv( mvScaleTmpHor, mvScaleTmpVer, true );
        }
      }
    }
  }

  // Set AffineMvField for affine motion compensation LT, RT, LB and RB
#if !JVET_K_AFFINE_BUG_FIXES
  Mv mv = affRT + affLB - affLT;
#endif
  mb.at(            0,             0 ).mv[eRefList] = affLT;
  mb.at( mb.width - 1,             0 ).mv[eRefList] = affRT;
#if !JVET_K_AFFINE_BUG_FIXES
  mb.at(            0, mb.height - 1 ).mv[eRefList] = affLB;
  mb.at( mb.width - 1, mb.height - 1 ).mv[eRefList] = mv;
#endif

#if JVET_K0337_AFFINE_6PARA
  if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
  {
    mb.at( 0, mb.height - 1 ).mv[eRefList] = affLB;
  }
#endif
}
#endif

#if JVET_K0346
static bool deriveScaledMotionTemporal( const Slice&      slice,
                                        const Position&   colPos,
                                        const Picture*    pColPic,
                                        const RefPicList  eCurrRefPicList,
                                        Mv&         cColMv,
                                        const RefPicList  eFetchRefPicList)
{
  const MotionInfo &mi = pColPic->cs->getMotionInfo(colPos);
  const Slice *pColSlice = nullptr;

  for (const auto &pSlice : pColPic->slices)
  {
    if (pSlice->getIndependentSliceIdx() == mi.sliceIdx)
    {
      pColSlice = pSlice;
      break;
    }
  }

  CHECK(pColSlice == nullptr, "Couldn't find the colocated slice");

  int iColPOC, iColRefPOC, iCurrPOC, iCurrRefPOC, iScale;
  bool bAllowMirrorMV = true;
  RefPicList eColRefPicList = slice.getCheckLDC() ? eCurrRefPicList : RefPicList(1 - eFetchRefPicList);
  if (pColPic == slice.getRefPic(RefPicList(slice.isInterB() ? 1 - slice.getColFromL0Flag() : 0), slice.getColRefIdx()))
  {
    eColRefPicList = eCurrRefPicList;   //67 -> disable, 64 -> enable
    bAllowMirrorMV = false;
  }

  // Although it might make sense to keep the unavailable motion field per direction still be unavailable, I made the MV prediction the same way as in TMVP
  // So there is an interaction between MV0 and MV1 of the corresponding blocks identified by TV.

  // Grab motion and do necessary scaling.{{
  iCurrPOC = slice.getPOC();

  int iColRefIdx = mi.refIdx[eColRefPicList];

  if (iColRefIdx < 0 && (slice.getCheckLDC() || bAllowMirrorMV))
  {
    eColRefPicList = RefPicList(1 - eColRefPicList);
    iColRefIdx = mi.refIdx[eColRefPicList];

    if (iColRefIdx < 0)
    {
      return false;
    }
  }

  if (iColRefIdx >= 0 && slice.getNumRefIdx(eCurrRefPicList) > 0)
  {
    iColPOC = pColSlice->getPOC();
    iColRefPOC = pColSlice->getRefPOC(eColRefPicList, iColRefIdx);
    ///////////////////////////////////////////////////////////////
    // Set the target reference index to 0, may be changed later //
    ///////////////////////////////////////////////////////////////
    iCurrRefPOC = slice.getRefPic(eCurrRefPicList, 0)->getPOC();
    // Scale the vector.
    cColMv = mi.mv[eColRefPicList];
    //pcMvFieldSP[2*iPartition + eCurrRefPicList].getMv();
    // Assume always short-term for now
    iScale = xGetDistScaleFactor(iCurrPOC, iCurrRefPOC, iColPOC, iColRefPOC);

    if (iScale != 4096)
    {
      if (slice.getSPS()->getSpsNext().getUseHighPrecMv())
      {
        cColMv.setHighPrec();
      }

      cColMv = cColMv.scaleMv(iScale);
    }

    return true;
  }
  return false;
}

#if JVET_K0346
void clipColBlkMv(int& mvX, int& mvY, const PredictionUnit& pu)
{
  Position puPos = pu.lumaPos();
  Size     puSize = pu.lumaSize();

  int ctuSize = pu.cs->sps->getSpsNext().getCTUSize();
  int ctuX = puPos.x / ctuSize*ctuSize;
  int ctuY = puPos.y / ctuSize*ctuSize;

  int horMax = std::min((int)pu.cs->sps->getPicWidthInLumaSamples(), ctuX + ctuSize + 4) - puSize.width;
  int horMin = std::max((int)0, ctuX);
  int verMax = std::min((int)pu.cs->sps->getPicHeightInLumaSamples(), ctuY + ctuSize) - puSize.height;
  int verMin = std::min((int)0, ctuY);

  horMax = horMax - puPos.x;
  horMin = horMin - puPos.x;
  verMax = verMax - puPos.y;
  verMin = verMin - puPos.y;

  mvX = std::min(horMax, std::max(horMin, mvX));
  mvY = std::min(verMax, std::max(verMin, mvY));
}
#endif

bool PU::getInterMergeSubPuMvpCand(const PredictionUnit &pu, MergeCtx& mrgCtx, bool& LICFlag, const int count)
{
  const Slice   &slice = *pu.cs->slice;
#if JVET_K0346
  const unsigned scale = 4 * std::max<int>(1, 4 * AMVP_DECIMATION_FACTOR / 4);
  const unsigned mask = ~(scale - 1);
#else
  const SPSNext &spsNext = pu.cs->sps->getSpsNext();
#endif

  const Picture *pColPic = slice.getRefPic(RefPicList(slice.isInterB() ? 1 - slice.getColFromL0Flag() : 0), slice.getColRefIdx());
#if JVET_K0346
  Mv cTMv;
  RefPicList fetchRefPicList = RefPicList(slice.isInterB() ? 1 - slice.getColFromL0Flag() : 0);

  bool terminate = false;
  for (unsigned currRefListId = 0; currRefListId < (slice.getSliceType() == B_SLICE ? 2 : 1) && !terminate; currRefListId++)
  {
    for (int uiN = 0; uiN < count && !terminate; uiN++)
    {
      RefPicList currRefPicList = RefPicList(slice.getCheckLDC() ? (slice.getColFromL0Flag() ? currRefListId : 1 - currRefListId) : currRefListId);

      if ((mrgCtx.interDirNeighbours[uiN] & (1 << currRefPicList)) && slice.getRefPic(currRefPicList, mrgCtx.mvFieldNeighbours[uiN * 2 + currRefPicList].refIdx) == pColPic)
      {
        cTMv = mrgCtx.mvFieldNeighbours[uiN * 2 + currRefPicList].mv;
        terminate = true;
        fetchRefPicList = currRefPicList;
        break;
      }
    }
  }
#else
  int iPocColPic = pColPic->getPOC();
  Mv cTMv;

  RefPicList eFetchRefPicList = RefPicList(slice.isInterB() ? 1 - slice.getColFromL0Flag() : 0);
  if (count)
  {
    const unsigned uiN = 0;
    for (unsigned uiCurrRefListId = 0; uiCurrRefListId < (slice.getSliceType() == B_SLICE ? 2 : 1); uiCurrRefListId++)
    {
      RefPicList  eCurrRefPicList = RefPicList(RefPicList(slice.isInterB() ? (slice.getColFromL0Flag() ? uiCurrRefListId : 1 - uiCurrRefListId) : uiCurrRefListId));
      if (mrgCtx.interDirNeighbours[uiN] & (1 << eCurrRefPicList))
      {
        pColPic = slice.getRefPic(eCurrRefPicList, mrgCtx.mvFieldNeighbours[uiN * 2 + eCurrRefPicList].refIdx);
        iPocColPic = pColPic->poc;
        cTMv = mrgCtx.mvFieldNeighbours[uiN * 2 + eCurrRefPicList].mv;
        eFetchRefPicList = eCurrRefPicList;
        break;
      }
    }
  }
#endif

  ///////////////////////////////////////////////////////////////////////
  ////////          GET Initial Temporal Vector                  ////////
  ///////////////////////////////////////////////////////////////////////
  int mvPrec = 2;
  if (pu.cs->sps->getSpsNext().getUseHighPrecMv())
  {
    cTMv.setHighPrec();
    mvPrec += VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;
  }
  int mvRndOffs = (1 << mvPrec) >> 1;

  Mv cTempVector = cTMv;
  bool  tempLICFlag = false;

  // compute the location of the current PU
  Position puPos = pu.lumaPos();
  Size puSize = pu.lumaSize();
#if JVET_K0346
  int numPartLine = std::max(puSize.width >> slice.getSubPuMvpSubblkLog2Size(), 1u);
  int numPartCol = std::max(puSize.height >> slice.getSubPuMvpSubblkLog2Size(), 1u);
  int puHeight = numPartCol == 1 ? puSize.height : 1 << slice.getSubPuMvpSubblkLog2Size();
  int puWidth = numPartLine == 1 ? puSize.width : 1 << slice.getSubPuMvpSubblkLog2Size();
#else
  int iNumPartLine = std::max(puSize.width >> spsNext.getSubPuMvpLog2Size(), 1u);
  int iNumPartCol = std::max(puSize.height >> spsNext.getSubPuMvpLog2Size(), 1u);
  int iPUHeight = iNumPartCol == 1 ? puSize.height : 1 << spsNext.getSubPuMvpLog2Size();
  int iPUWidth = iNumPartLine == 1 ? puSize.width : 1 << spsNext.getSubPuMvpLog2Size();
#endif

  Mv cColMv;
  // use coldir.
  bool     bBSlice = slice.isInterB();
#if !JVET_K0346
  unsigned bColL0 = slice.getColFromL0Flag();
#endif

  Position centerPos;

  bool found = false;
#if JVET_K0346
  cTempVector = cTMv;
  int tempX = ((cTempVector.getHor() + mvRndOffs) >> mvPrec);
  int tempY = ((cTempVector.getVer() + mvRndOffs) >> mvPrec);
  clipColBlkMv(tempX, tempY, pu);

  if (puSize.width == puWidth && puSize.height == puHeight)
  {
    centerPos.x = puPos.x + (puSize.width >> 1) + tempX;
    centerPos.y = puPos.y + (puSize.height >> 1) + tempY;
  }
  else
  {
    centerPos.x = puPos.x + ((puSize.width / puWidth) >> 1)   * puWidth + (puWidth >> 1) + tempX;
    centerPos.y = puPos.y + ((puSize.height / puHeight) >> 1) * puHeight + (puHeight >> 1) + tempY;
  }

  centerPos.x = Clip3(0, (int)pColPic->lwidth() - 1, centerPos.x);
  centerPos.y = Clip3(0, (int)pColPic->lheight() - 1, centerPos.y);

  centerPos = Position{ PosType(centerPos.x & mask), PosType(centerPos.y & mask) };

  // derivation of center motion parameters from the collocated CU
  const MotionInfo &mi = pColPic->cs->getMotionInfo(centerPos);

  if (mi.isInter)
  {
    for (unsigned currRefListId = 0; currRefListId < (bBSlice ? 2 : 1); currRefListId++)
    {
      RefPicList  currRefPicList = RefPicList(currRefListId);

      if (deriveScaledMotionTemporal(slice, centerPos, pColPic, currRefPicList, cColMv, fetchRefPicList))
      {
        // set as default, for further motion vector field spanning
        mrgCtx.mvFieldNeighbours[(count << 1) + currRefListId].setMvField(cColMv, 0);
        mrgCtx.interDirNeighbours[count] |= (1 << currRefListId);
        LICFlag = tempLICFlag;
        found = true;
      }
      else
      {
        mrgCtx.mvFieldNeighbours[(count << 1) + currRefListId].setMvField(Mv(), NOT_VALID);
        mrgCtx.interDirNeighbours[count] &= ~(1 << currRefListId);
      }
    }
  }
#else
  bool bInit = false;
  for (unsigned uiLX = 0; uiLX < (bBSlice ? 2 : 1) && !found; uiLX++)
  {
    RefPicList eListY = RefPicList(bBSlice ? (bColL0 ? uiLX : 1 - uiLX) : uiLX);

    for (int refIdxY = (bInit ? 0 : -1); refIdxY < slice.getNumRefIdx(eListY) && !found; refIdxY++)
    {
      if (!bInit)
      {
        bInit = true;
      }
      else
      {
        pColPic = slice.getRefPic(eListY, refIdxY);
        eFetchRefPicList = eListY;
      }
      int iNewColPicPOC = pColPic->getPOC();
      if (iNewColPicPOC != iPocColPic)
      {
        //////////////// POC based scaling of the temporal vector /////////////
        int iScale = xGetDistScaleFactor(slice.getPOC(), iNewColPicPOC, slice.getPOC(), iPocColPic);
        if (iScale != 4096)
        {
          cTempVector = cTMv.scaleMv(iScale);
        }
      }
      else
      {
        cTempVector = cTMv;
      }

      if (puSize.width == iPUWidth && puSize.height == iPUHeight)
      {
        centerPos.x = puPos.x + (puSize.width >> 1) + ((cTempVector.getHor() + mvRndOffs) >> mvPrec);
        centerPos.y = puPos.y + (puSize.height >> 1) + ((cTempVector.getVer() + mvRndOffs) >> mvPrec);
      }
      else
      {
        centerPos.x = puPos.x + ((puSize.width / iPUWidth) >> 1) * iPUWidth + (iPUWidth >> 1) + ((cTempVector.getHor() + mvRndOffs) >> mvPrec);
        centerPos.y = puPos.y + ((puSize.height / iPUHeight) >> 1) * iPUHeight + (iPUHeight >> 1) + ((cTempVector.getVer() + mvRndOffs) >> mvPrec);
      }

      centerPos.x = Clip3(0, (int)pColPic->lwidth() - 1, centerPos.x);
      centerPos.y = Clip3(0, (int)pColPic->lheight() - 1, centerPos.y);

      // derivation of center motion parameters from the collocated CU
      const MotionInfo &mi = pColPic->cs->getMotionInfo(centerPos);

      if (mi.isInter)
      {
        for (uint32_t uiCurrRefListId = 0; uiCurrRefListId < (bBSlice ? 2 : 1); uiCurrRefListId++)
        {
          RefPicList  eCurrRefPicList = RefPicList(uiCurrRefListId);

          if (deriveScaledMotionTemporal(slice, centerPos, pColPic, eCurrRefPicList, cColMv, eFetchRefPicList))
          {
            // set as default, for further motion vector field spanning
            mrgCtx.mvFieldNeighbours[(count << 1) + uiCurrRefListId].setMvField(cColMv, 0);
            mrgCtx.interDirNeighbours[count] |= (1 << uiCurrRefListId);
            LICFlag = tempLICFlag;
            found = true;
          }
          else
          {
            mrgCtx.mvFieldNeighbours[(count << 1) + uiCurrRefListId].setMvField(Mv(), NOT_VALID);
            mrgCtx.interDirNeighbours[count] &= ~(1 << uiCurrRefListId);
          }
        }
      }
    }
  }
#endif

  if (!found)
  {
    return false;
  }
  
#if JVET_K0346
  int xOff = puWidth / 2;
  int yOff = puHeight / 2;

  // compute the location of the current PU
  xOff += tempX;
  yOff += tempY;
#else
  int xOff = iPUWidth / 2;
  int yOff = iPUHeight / 2;

  // compute the location of the current PU
  xOff += ((cTempVector.getHor() + mvRndOffs) >> mvPrec);
  yOff += ((cTempVector.getVer() + mvRndOffs) >> mvPrec);
#endif

  int iPicWidth = pColPic->lwidth() - 1;
  int iPicHeight = pColPic->lheight() - 1;

  MotionBuf& mb = mrgCtx.subPuMvpMiBuf;

  const bool isBiPred = isBipredRestriction(pu);

#if JVET_K0346
  for (int y = puPos.y; y < puPos.y + puSize.height; y += puHeight)
  {
    for (int x = puPos.x; x < puPos.x + puSize.width; x += puWidth)
#else
  for (int y = puPos.y; y < puPos.y + puSize.height; y += iPUHeight)
  {
    for (int x = puPos.x; x < puPos.x + puSize.width; x += iPUWidth)
#endif
    {
      Position colPos{ x + xOff, y + yOff };

      colPos.x = Clip3(0, iPicWidth, colPos.x);
      colPos.y = Clip3(0, iPicHeight, colPos.y);

#if JVET_K0346 
      colPos = Position{ PosType(colPos.x & mask), PosType(colPos.y & mask) };
#endif

      const MotionInfo &colMi = pColPic->cs->getMotionInfo(colPos);

      MotionInfo mi;

      mi.isInter = true;
      mi.sliceIdx = slice.getIndependentSliceIdx();

      if (colMi.isInter)
      {
#if JVET_K0346
        for (unsigned currRefListId = 0; currRefListId < (bBSlice ? 2 : 1); currRefListId++)
        {
          RefPicList currRefPicList = RefPicList(currRefListId);
          if (deriveScaledMotionTemporal(slice, colPos, pColPic, currRefPicList, cColMv, fetchRefPicList))
          {
            mi.refIdx[currRefListId] = 0;
            mi.mv[currRefListId] = cColMv;
          }
#else
        for (uint32_t uiCurrRefListId = 0; uiCurrRefListId < (bBSlice ? 2 : 1); uiCurrRefListId++)
        {
          RefPicList eCurrRefPicList = RefPicList(uiCurrRefListId);
          if (deriveScaledMotionTemporal(slice, colPos, pColPic, eCurrRefPicList, cColMv, eFetchRefPicList))
          {
            mi.refIdx[uiCurrRefListId] = 0;
            mi.mv[uiCurrRefListId] = cColMv;
          }
#endif
        }
        }
      else
      {
        // intra coded, in this case, no motion vector is available for list 0 or list 1, so use default
        mi.mv[0] = mrgCtx.mvFieldNeighbours[(count << 1) + 0].mv;
        mi.mv[1] = mrgCtx.mvFieldNeighbours[(count << 1) + 1].mv;
        mi.refIdx[0] = mrgCtx.mvFieldNeighbours[(count << 1) + 0].refIdx;
        mi.refIdx[1] = mrgCtx.mvFieldNeighbours[(count << 1) + 1].refIdx;
      }

      mi.interDir = (mi.refIdx[0] != -1 ? 1 : 0) + (mi.refIdx[1] != -1 ? 2 : 0);

      if (isBiPred && mi.interDir == 3)
      {
        mi.interDir = 1;
        mi.mv[1] = Mv();
        mi.refIdx[1] = NOT_VALID;
      }

#if JVET_K0346
      mb.subBuf(g_miScaling.scale(Position{ x, y } -pu.lumaPos()), g_miScaling.scale(Size(puWidth, puHeight))).fill(mi);
#else
      mb.subBuf(g_miScaling.scale(Position{ x, y } -pu.lumaPos()), g_miScaling.scale(Size(iPUWidth, iPUHeight))).fill(mi);
#endif
      }
    }

  return true;
  }
#endif

void PU::spanMotionInfo( PredictionUnit &pu, const MergeCtx &mrgCtx )
{
  MotionBuf mb = pu.getMotionBuf();

  if( !pu.mergeFlag || pu.mergeType == MRG_TYPE_DEFAULT_N )
  {
    MotionInfo mi;

    mi.isInter  = CU::isInter( *pu.cu );
    mi.sliceIdx = pu.cu->slice->getIndependentSliceIdx();

    if( mi.isInter )
    {
      mi.interDir = pu.interDir;

      for( int i = 0; i < NUM_REF_PIC_LIST_01; i++ )
      {
        mi.mv[i]     = pu.mv[i];
        mi.refIdx[i] = pu.refIdx[i];
      }
    }

#if JVET_K_AFFINE
    if( pu.cu->affine )
    {
      for( int y = 0; y < mb.height; y++ )
      {
        for( int x = 0; x < mb.width; x++ )
        {
          MotionInfo &dest = mb.at( x, y );
          dest.isInter  = mi.isInter;
          dest.interDir = mi.interDir;
          dest.sliceIdx = mi.sliceIdx;
          for( int i = 0; i < NUM_REF_PIC_LIST_01; i++ )
          {
            if( mi.refIdx[i] == -1 )
            {
              dest.mv[i] = Mv();
            }
            dest.refIdx[i] = mi.refIdx[i];
          }
        }
      }
    }
    else
#endif
    {
      mb.fill( mi );
    }
  }
#if JVET_K0346
  else if (pu.mergeType == MRG_TYPE_SUBPU_ATMVP)
  {
    CHECK(mrgCtx.subPuMvpMiBuf.area() == 0 || !mrgCtx.subPuMvpMiBuf.buf, "Buffer not initialized");
    mb.copyFrom(mrgCtx.subPuMvpMiBuf);
  }
#endif
  else
  {

    if( isBipredRestriction( pu ) )
    {
      for( int y = 0; y < mb.height; y++ )
      {
        for( int x = 0; x < mb.width; x++ )
        {
          MotionInfo &mi = mb.at( x, y );
          if( mi.interDir == 3 )
          {
            mi.interDir  = 1;
            mi.mv    [1] = Mv();
            mi.refIdx[1] = NOT_VALID;
          }
        }
      }
    }
  }
}

#if JVET_K0357_AMVR
void PU::applyImv( PredictionUnit& pu, MergeCtx &mrgCtx, InterPrediction *interPred )
{
  if( !pu.mergeFlag )
  {
    unsigned imvShift = pu.cu->imv << 1;
    if( pu.interDir != 2 /* PRED_L1 */ )
    {
      if (pu.cu->imv)
      {
        CHECK( pu.mvd[0].highPrec, "Motion vector difference should never be high precision" );
        pu.mvd[0] = Mv( pu.mvd[0].hor << imvShift, pu.mvd[0].ver << imvShift );
      }
      unsigned mvp_idx = pu.mvpIdx[0];
      AMVPInfo amvpInfo;
      PU::fillMvpCand(pu, REF_PIC_LIST_0, pu.refIdx[0], amvpInfo);
      pu.mvpNum[0] = amvpInfo.numCand;
      pu.mvpIdx[0] = mvp_idx;
      pu.mv    [0] = amvpInfo.mvCand[mvp_idx] + pu.mvd[0];
    }

    if (pu.interDir != 1 /* PRED_L0 */)
    {
      if( !( pu.cu->cs->slice->getMvdL1ZeroFlag() && pu.interDir == 3 ) && pu.cu->imv )/* PRED_BI */
      {
        CHECK( pu.mvd[1].highPrec, "Motion vector difference should never be high precision" );
        pu.mvd[1] = Mv( pu.mvd[1].hor << imvShift, pu.mvd[1].ver << imvShift );
      }
      unsigned mvp_idx = pu.mvpIdx[1];
      AMVPInfo amvpInfo;
      PU::fillMvpCand(pu, REF_PIC_LIST_1, pu.refIdx[1], amvpInfo);
      pu.mvpNum[1] = amvpInfo.numCand;
      pu.mvpIdx[1] = mvp_idx;
      pu.mv    [1] = amvpInfo.mvCand[mvp_idx] + pu.mvd[1];
    }
  }
  else
  {
    // this function is never called for merge
    THROW("unexpected");
    PU::getInterMergeCandidates ( pu, mrgCtx );
    PU::restrictBiPredMergeCands( pu, mrgCtx );

    mrgCtx.setMergeInfo( pu, pu.mergeIdx );
  }

  PU::spanMotionInfo( pu, mrgCtx );
}
#endif

bool PU::isBiPredFromDifferentDir( const PredictionUnit& pu )
{
  if ( pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0 )
  {
    const int iPOC0 = pu.cu->slice->getRefPOC( REF_PIC_LIST_0, pu.refIdx[0] );
    const int iPOC1 = pu.cu->slice->getRefPOC( REF_PIC_LIST_1, pu.refIdx[1] );
    const int iPOC  = pu.cu->slice->getPOC();
    if ( (iPOC - iPOC0)*(iPOC - iPOC1) < 0 )
    {
      return true;
    }
  }

  return false;
}

void PU::restrictBiPredMergeCands( const PredictionUnit &pu, MergeCtx& mergeCtx )
{
  if( PU::isBipredRestriction( pu ) )
  {
    for( uint32_t mergeCand = 0; mergeCand < mergeCtx.numValidMergeCand; ++mergeCand )
    {
      if( mergeCtx.interDirNeighbours[ mergeCand ] == 3 )
      {
        mergeCtx.interDirNeighbours[ mergeCand ] = 1;
        mergeCtx.mvFieldNeighbours[( mergeCand << 1 ) + 1].setMvField( Mv( 0, 0 ), -1 );
      }
    }
  }
}

#if JVET_K0357_AMVR
void CU::resetMVDandMV2Int( CodingUnit& cu, InterPrediction *interPred )
{
  for( auto &pu : CU::traversePUs( cu ) )
  {
    MergeCtx mrgCtx;

    if( !pu.mergeFlag )
    {
      unsigned imvShift = cu.imv << 1;
      if( pu.interDir != 2 /* PRED_L1 */ )
      {
        Mv mv        = pu.mv[0];
        Mv mvPred;
        AMVPInfo amvpInfo;
        PU::fillMvpCand(pu, REF_PIC_LIST_0, pu.refIdx[0], amvpInfo);
        pu.mvpNum[0] = amvpInfo.numCand;

        mvPred       = amvpInfo.mvCand[pu.mvpIdx[0]];
        roundMV      ( mv, imvShift );
        pu.mv[0]     = mv;
        Mv mvDiff    = mv - mvPred;
        pu.mvd[0]    = mvDiff;
      }
      if( pu.interDir != 1 /* PRED_L0 */ )
      {
        Mv mv        = pu.mv[1];
        Mv mvPred;
        AMVPInfo amvpInfo;
        PU::fillMvpCand(pu, REF_PIC_LIST_1, pu.refIdx[1], amvpInfo);
        pu.mvpNum[1] = amvpInfo.numCand;

        mvPred       = amvpInfo.mvCand[pu.mvpIdx[1]];
        roundMV      ( mv, imvShift );
        Mv mvDiff    = mv - mvPred;

        if( pu.cu->cs->slice->getMvdL1ZeroFlag() && pu.interDir == 3 /* PRED_BI */ )
        {
          pu.mvd[1] = Mv();
          mv = mvPred;
        }
        else
        {
          pu.mvd[1] = mvDiff;
        }
        pu.mv[1] = mv;
      }

    }
    else
    {
        PU::getInterMergeCandidates ( pu, mrgCtx );
        PU::restrictBiPredMergeCands( pu, mrgCtx );

        mrgCtx.setMergeInfo( pu, pu.mergeIdx );
    }

    PU::spanMotionInfo( pu, mrgCtx );
  }
}

bool CU::hasSubCUNonZeroMVd( const CodingUnit& cu )
{
  bool bNonZeroMvd = false;

  for( const auto &pu : CU::traversePUs( cu ) )
  {
    if( ( !pu.mergeFlag ) && ( !cu.skip ) )
    {
      if( pu.interDir != 2 /* PRED_L1 */ )
      {
        bNonZeroMvd |= pu.mvd[REF_PIC_LIST_0].getHor() != 0;
        bNonZeroMvd |= pu.mvd[REF_PIC_LIST_0].getVer() != 0;
      }
      if( pu.interDir != 1 /* PRED_L0 */ )
      {
        if( !pu.cu->cs->slice->getMvdL1ZeroFlag() || pu.interDir != 3 /* PRED_BI */ )
        {
          bNonZeroMvd |= pu.mvd[REF_PIC_LIST_1].getHor() != 0;
          bNonZeroMvd |= pu.mvd[REF_PIC_LIST_1].getVer() != 0;
        }
      }
    }
  }

  return bNonZeroMvd;
}

int CU::getMaxNeighboriMVCandNum( const CodingStructure& cs, const Position& pos )
{
  const int  numDefault     = 0;
  int        maxImvNumCand  = 0;

  // Get BCBP of left PU
#if HEVC_TILES_WPP
  const CodingUnit *cuLeft  = cs.getCURestricted( pos.offset( -1, 0 ), cs.slice->getIndependentSliceIdx(), cs.picture->tileMap->getTileIdxMap( pos ), CH_L );
#else
  const CodingUnit *cuLeft  = cs.getCURestricted( pos.offset( -1, 0 ), cs.slice->getIndependentSliceIdx(), CH_L );
#endif
  maxImvNumCand = ( cuLeft ) ? cuLeft->imvNumCand : numDefault;

  // Get BCBP of above PU
#if HEVC_TILES_WPP
  const CodingUnit *cuAbove = cs.getCURestricted( pos.offset( 0, -1 ), cs.slice->getIndependentSliceIdx(), cs.picture->tileMap->getTileIdxMap( pos ), CH_L );
#else
  const CodingUnit *cuAbove = cs.getCURestricted( pos.offset( 0, -1 ), cs.slice->getIndependentSliceIdx(), CH_L );
#endif
  maxImvNumCand = std::max( maxImvNumCand, ( cuAbove ) ? cuAbove->imvNumCand : numDefault );

  return maxImvNumCand;
}
#endif





// TU tools

#if HEVC_USE_4x4_DSTVII
bool TU::useDST(const TransformUnit &tu, const ComponentID &compID)
{
  return isLuma(compID) && tu.cu->predMode == MODE_INTRA;
}

#endif

bool TU::isNonTransformedResidualRotated(const TransformUnit &tu, const ComponentID &compID)
{
  return tu.cs->sps->getSpsRangeExtension().getTransformSkipRotationEnabledFlag() && tu.blocks[compID].width == 4 && tu.cu->predMode == MODE_INTRA;
}

bool TU::getCbf( const TransformUnit &tu, const ComponentID &compID )
{
#if ENABLE_BMS
  return getCbfAtDepth( tu, compID, tu.depth );
#else
  return tu.cbf[compID];
#endif
}

#if ENABLE_BMS
bool TU::getCbfAtDepth(const TransformUnit &tu, const ComponentID &compID, const unsigned &depth)
{
  return ((tu.cbf[compID] >> depth) & 1) == 1;
}

void TU::setCbfAtDepth(TransformUnit &tu, const ComponentID &compID, const unsigned &depth, const bool &cbf)
{
  // first clear the CBF at the depth
  tu.cbf[compID] &= ~(1  << depth);
  // then set the CBF
  tu.cbf[compID] |= ((cbf ? 1 : 0) << depth);
}
#else
void TU::setCbf( TransformUnit &tu, const ComponentID &compID, const bool &cbf )
{
  tu.cbf[compID] = cbf;
}
#endif

bool TU::hasTransformSkipFlag(const CodingStructure& cs, const CompArea& area)
{
  uint32_t transformSkipLog2MaxSize = cs.pps->getPpsRangeExtension().getLog2MaxTransformSkipBlockSize();

  if( cs.pcv->rectCUs )
  {
    return ( area.width * area.height <= (1 << ( transformSkipLog2MaxSize << 1 )) );
  }
  return ( area.width <= (1 << transformSkipLog2MaxSize) );
}

uint32_t TU::getGolombRiceStatisticsIndex(const TransformUnit &tu, const ComponentID &compID)
{
  const bool transformSkip    = tu.transformSkip[compID];
  const bool transquantBypass = tu.cu->transQuantBypass;

  //--------

  const uint32_t channelTypeOffset = isChroma(compID) ? 2 : 0;
  const uint32_t nonTransformedOffset = (transformSkip || transquantBypass) ? 1 : 0;

  //--------

  const uint32_t selectedIndex = channelTypeOffset + nonTransformedOffset;
  CHECK( selectedIndex >= RExt__GOLOMB_RICE_ADAPTATION_STATISTICS_SETS, "Invalid golomb rice adaptation statistics set" );

  return selectedIndex;
}

#if HEVC_USE_MDCS
uint32_t TU::getCoefScanIdx(const TransformUnit &tu, const ComponentID &compID)
{
  //------------------------------------------------

  //this mechanism is available for intra only

  if( !CU::isIntra( *tu.cu ) )
  {
    return SCAN_DIAG;
  }

  //------------------------------------------------

  //check that MDCS can be used for this TU


  const CompArea &area      = tu.blocks[compID];
  const SPS &sps            = *tu.cs->sps;
  const ChromaFormat format = sps.getChromaFormatIdc();


  const uint32_t maximumWidth  = MDCS_MAXIMUM_WIDTH  >> getComponentScaleX(compID, format);
  const uint32_t maximumHeight = MDCS_MAXIMUM_HEIGHT >> getComponentScaleY(compID, format);

  if ((area.width > maximumWidth) || (area.height > maximumHeight))
  {
    return SCAN_DIAG;
  }

  //------------------------------------------------

  //otherwise, select the appropriate mode

  const PredictionUnit &pu = *tu.cs->getPU( area.pos(), toChannelType( compID ) );

  uint32_t uiDirMode = PU::getFinalIntraMode(pu, toChannelType(compID));

  //------------------

       if (abs((int) uiDirMode - VER_IDX) <= MDCS_ANGLE_LIMIT)
  {
    return SCAN_HOR;
  }
  else if (abs((int) uiDirMode - HOR_IDX) <= MDCS_ANGLE_LIMIT)
  {
    return SCAN_VER;
  }
  else
  {
    return SCAN_DIAG;
  }
}

#endif
bool TU::hasCrossCompPredInfo( const TransformUnit &tu, const ComponentID &compID )
{
  return ( isChroma(compID) && tu.cs->pps->getPpsRangeExtension().getCrossComponentPredictionEnabledFlag() && TU::getCbf( tu, COMPONENT_Y ) &&
         ( CU::isInter(*tu.cu) || PU::isChromaIntraModeCrossCheckMode( *tu.cs->getPU( tu.blocks[compID].pos(), toChannelType( compID ) ) ) ) );
}

uint32_t TU::getNumNonZeroCoeffsNonTS( const TransformUnit& tu, const bool bLuma, const bool bChroma )
{
  uint32_t count = 0;
  for( uint32_t i = 0; i < ::getNumberValidTBlocks( *tu.cs->pcv ); i++ )
  {
    if( tu.blocks[i].valid() && !tu.transformSkip[i] && TU::getCbf( tu, ComponentID( i ) ) )
    {
      if( isLuma  ( tu.blocks[i].compID ) && !bLuma   ) continue;
      if( isChroma( tu.blocks[i].compID ) && !bChroma ) continue;

      uint32_t area = tu.blocks[i].area();
      const TCoeff* coeff = tu.getCoeffs( ComponentID( i ) ).buf;
      for( uint32_t j = 0; j < area; j++ )
      {
        count += coeff[j] != 0;
      }
    }
  }
  return count;
}

bool TU::needsSqrt2Scale( const Size& size )
{
  return (((g_aucLog2[size.width] + g_aucLog2[size.height]) & 1) == 1);
}

#if HM_QTBT_AS_IN_JEM_QUANT

bool TU::needsBlockSizeTrafoScale( const Size& size )
{
  return needsSqrt2Scale( size ) || isNonLog2BlockSize( size );
}
#else
bool TU::needsQP3Offset(const TransformUnit &tu, const ComponentID &compID)
{
  if( tu.cs->pcv->rectCUs && !tu.transformSkip[compID] )
  {
    return ( ( ( g_aucLog2[tu.blocks[compID].width] + g_aucLog2[tu.blocks[compID].height] ) & 1 ) == 1 );
  }
  return false;
}
#endif





// other tools

uint32_t getCtuAddr( const Position& pos, const PreCalcValues& pcv )
{
  return ( pos.x >> pcv.maxCUWidthLog2 ) + ( pos.y >> pcv.maxCUHeightLog2 ) * pcv.widthInCtus;
}




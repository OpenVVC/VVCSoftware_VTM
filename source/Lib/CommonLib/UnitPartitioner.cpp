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

/** \file     UnitPartitioner.h
 *  \brief    Provides a class for partitioning management
 */

#include "UnitPartitioner.h"

#include "CodingStructure.h"
#include "Unit.h"
#include "Slice.h"
#include "UnitTools.h"
#include "Picture.h"


PartLevel::PartLevel()
: split               ( CU_DONT_SPLIT )
, parts               (               )
, idx                 ( 0u            )
, checkdIfImplicit    ( false         )
, isImplicit          ( false         )
, implicitSplit       ( CU_DONT_SPLIT )
, firstSubPartSplit   ( CU_DONT_SPLIT )
, canQtSplit          ( true          )
{
}

PartLevel::PartLevel( const PartSplit _split, const Partitioning& _parts )
: split               ( _split        )
, parts               ( _parts        )
, idx                 ( 0u            )
, checkdIfImplicit    ( false         )
, isImplicit          ( false         )
, implicitSplit       ( CU_DONT_SPLIT )
, firstSubPartSplit   ( CU_DONT_SPLIT )
, canQtSplit          ( true          )
{
}

PartLevel::PartLevel( const PartSplit _split, Partitioning&& _parts )
: split               ( _split                               )
, parts               ( std::forward<Partitioning>( _parts ) )
, idx                 ( 0u                                   )
, checkdIfImplicit    ( false                                )
, isImplicit          ( false                                )
, implicitSplit       ( CU_DONT_SPLIT                        )
, firstSubPartSplit   ( CU_DONT_SPLIT                        )
, canQtSplit          ( true                                 )
{
}

//////////////////////////////////////////////////////////////////////////
// Partitioner class
//////////////////////////////////////////////////////////////////////////

SplitSeries Partitioner::getSplitSeries() const
{
  SplitSeries splitSeries = 0;
  SplitSeries depth = 0;

  for( const auto &level : m_partStack )
  {
    if( level.split == CTU_LEVEL ) continue;
    else splitSeries += static_cast< SplitSeries >( level.split ) << ( depth * SPLIT_DMULT );

    depth++;
  }

  return splitSeries;
}

void Partitioner::setCUData( CodingUnit& cu )
{
  cu.depth       = currDepth;
  cu.btDepth     = currBtDepth;
  cu.mtDepth     = currMtDepth;
  cu.qtDepth     = currQtDepth;
  cu.splitSeries = getSplitSeries();
}

void Partitioner::copyState( const Partitioner& other )
{
  m_partStack = other.m_partStack;
  currBtDepth = other.currBtDepth;
  currQtDepth = other.currQtDepth;
  currDepth   = other.currDepth;
  currMtDepth = other.currMtDepth;
#if ENABLE_BMS
  currTrDepth = other.currTrDepth;
#endif
#if !HM_QTBT_ONLY_QT_IMPLICIT || JVET_K0554
  currImplicitBtDepth
              = other.currImplicitBtDepth;
#endif
  chType      = other.chType;
#ifdef _DEBUG
  m_currArea  = other.m_currArea;
#endif
}

//////////////////////////////////////////////////////////////////////////
// AdaptiveDepthPartitioner class
//////////////////////////////////////////////////////////////////////////

void AdaptiveDepthPartitioner::setMaxMinDepth( unsigned& minDepth, unsigned& maxDepth, const CodingStructure& cs ) const
{
  unsigned          stdMinDepth = 0;
  unsigned          stdMaxDepth = ( ( cs.sps->getSpsNext().getUseQTBT() )
                                        ? g_aucLog2[cs.sps->getSpsNext().getCTUSize()] - g_aucLog2[cs.sps->getSpsNext().getMinQTSize( cs.slice->getSliceType(), chType )]
                                        : cs.sps->getLog2DiffMaxMinCodingBlockSize() );
  const Position    pos         = currArea().blocks[chType].pos();
  const unsigned    curSliceIdx = cs.slice->getIndependentSliceIdx();
#if HEVC_TILES_WPP
  const unsigned    curTileIdx  = cs.picture->tileMap->getTileIdxMap( currArea().lumaPos() );

  const CodingUnit* cuLeft        = cs.getCURestricted( pos.offset( -1,                                                                        0 ), curSliceIdx, curTileIdx, chType );
  const CodingUnit* cuBelowLeft   = cs.getCURestricted( pos.offset( -1, cs.pcv->minCUHeight >> getChannelTypeScaleY( chType, cs.pcv->chrFormat ) ), curSliceIdx, curTileIdx, chType );  // should use actual block size instead of minCU size
  const CodingUnit* cuAbove       = cs.getCURestricted( pos.offset(  0,                                                                       -1 ), curSliceIdx, curTileIdx, chType );
  const CodingUnit* cuAboveRight  = cs.getCURestricted( pos.offset( cs.pcv->minCUWidth >> getChannelTypeScaleX( chType, cs.pcv->chrFormat ),  -1 ), curSliceIdx, curTileIdx, chType );  // should use actual block size instead of minCU size
#else
  const CodingUnit* cuLeft        = cs.getCURestricted( pos.offset( -1,                                                                        0 ), curSliceIdx, chType );
  const CodingUnit* cuBelowLeft   = cs.getCURestricted( pos.offset( -1, cs.pcv->minCUHeight >> getChannelTypeScaleY( chType, cs.pcv->chrFormat ) ), curSliceIdx, chType );  // should use actual block size instead of minCU size
  const CodingUnit* cuAbove       = cs.getCURestricted( pos.offset(  0,                                                                       -1 ), curSliceIdx, chType );
  const CodingUnit* cuAboveRight  = cs.getCURestricted( pos.offset( cs.pcv->minCUWidth >> getChannelTypeScaleX( chType, cs.pcv->chrFormat ),  -1 ), curSliceIdx, chType );  // should use actual block size instead of minCU size
#endif

  minDepth = stdMaxDepth;
  maxDepth = stdMinDepth;

  if( cuLeft )
  {
    minDepth = std::min<unsigned>( minDepth, cuLeft->qtDepth );
    maxDepth = std::max<unsigned>( maxDepth, cuLeft->qtDepth );
  }
  else
  {
    minDepth = stdMinDepth;
    maxDepth = stdMaxDepth;
  }

  if( cuBelowLeft )
  {
    minDepth = std::min<unsigned>( minDepth, cuBelowLeft->qtDepth );
    maxDepth = std::max<unsigned>( maxDepth, cuBelowLeft->qtDepth );
  }
  else
  {
    minDepth = stdMinDepth;
    maxDepth = stdMaxDepth;
  }

  if( cuAbove )
  {
    minDepth = std::min<unsigned>( minDepth, cuAbove->qtDepth );
    maxDepth = std::max<unsigned>( maxDepth, cuAbove->qtDepth );
  }
  else
  {
    minDepth = stdMinDepth;
    maxDepth = stdMaxDepth;
  }

  if( cuAboveRight )
  {
    minDepth = std::min<unsigned>( minDepth, cuAboveRight->qtDepth );
    maxDepth = std::max<unsigned>( maxDepth, cuAboveRight->qtDepth );
  }
  else
  {
    minDepth = stdMinDepth;
    maxDepth = stdMaxDepth;
  }

  minDepth = ( minDepth >= 1 ? minDepth - 1 : 0 );
  maxDepth = std::min<unsigned>( stdMaxDepth, maxDepth + 1 );
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
// QTBTPartitioner
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

void QTBTPartitioner::initCtu( const UnitArea& ctuArea, const ChannelType _chType, const Slice& slice )
{
#if _DEBUG
  m_currArea = ctuArea;
#endif
  currDepth   = 0;
#if ENABLE_BMS
  currTrDepth = 0;
#endif
  currBtDepth = 0;
  currMtDepth = 0;
  currQtDepth = 0;
#if !HM_QTBT_ONLY_QT_IMPLICIT || JVET_K0554
  currImplicitBtDepth = 0;
#endif
  chType      = _chType;

  m_partStack.clear();
  m_partStack.push_back( PartLevel( CTU_LEVEL, Partitioning{ ctuArea } ) );
}

void QTBTPartitioner::splitCurrArea( const PartSplit split, const CodingStructure& cs )
{
  CHECKD( !canSplit( split, cs ), "Trying to apply a prohibited split!" );

#if !HM_QTBT_ONLY_QT_IMPLICIT || JVET_K0554
  bool isImplicit = isSplitImplicit( split, cs );
#endif
  bool canQtSplit = canSplit( CU_QUAD_SPLIT, cs );

  switch( split )
  {
  case CU_QUAD_SPLIT:
    m_partStack.push_back( PartLevel( split, PartitionerImpl::getCUSubPartitions( currArea(), cs ) ) );
    break;
  case CU_HORZ_SPLIT:
  case CU_VERT_SPLIT:
    CHECK( !cs.sps->getSpsNext().getUseQTBT(), "QTBT disabled" );
    m_partStack.push_back( PartLevel( split, PartitionerImpl::getCUSubPartitions( currArea(), cs, split ) ) );
    break;
  case CU_TRIH_SPLIT:
  case CU_TRIV_SPLIT:
    CHECK( ( cs.sps->getSpsNext().getMTTMode() & 1 ) != 1, "Triple splits are not allowed" );
    m_partStack.push_back( PartLevel( split, PartitionerImpl::getCUSubPartitions( currArea(), cs, split ) ) );
    break;
#if ENABLE_BMS
  case TU_MAX_TR_SPLIT:
    m_partStack.push_back( PartLevel( split, PartitionerImpl::getMaxTuTiling( currArea(), cs ) ) );
    break;
#endif
  default:
    THROW( "Unknown split mode" );
    break;
  }

  currDepth++;
#if _DEBUG
  m_currArea = m_partStack.back().parts.front();
#endif

#if ENABLE_BMS
  if( split == TU_MAX_TR_SPLIT )
  {
    currTrDepth++;
  }
#endif
#if ENABLE_BMS
  else
  {
    currTrDepth = 0;
  }
#endif

  if( split == CU_HORZ_SPLIT || split == CU_VERT_SPLIT || split == CU_TRIH_SPLIT || split == CU_TRIV_SPLIT )
  {
    currBtDepth++;
#if !HM_QTBT_ONLY_QT_IMPLICIT || JVET_K0554
    if( isImplicit ) currImplicitBtDepth++;
#endif
    currMtDepth++;

    if( split == CU_TRIH_SPLIT || split == CU_TRIV_SPLIT )
    {
      // first and last part of triple split are equivalent to double bt split
      currBtDepth++;
    }
    m_partStack.back().canQtSplit = canQtSplit;
  }
  else if( split == CU_QUAD_SPLIT )
  {
    CHECK( currBtDepth > 0, "Cannot split a non-square area other than with a binary split" );
    CHECK( currMtDepth > 0, "Cannot split a non-square area other than with a binary split" );
    currMtDepth = 0;
    currBtDepth = 0;
    currQtDepth++;
  }
}

bool QTBTPartitioner::canSplit( const PartSplit split, const CodingStructure &cs )
{
  const PartSplit implicitSplit = getImplicitSplit( cs );

  // the minimal and maximal sizes are given in luma samples
  const CompArea area           = currArea().Y();

#if !HM_QTBT_ONLY_QT_IMPLICIT || JVET_K0554
  const unsigned maxBTD         = cs.pcv->getMaxBtDepth( *cs.slice, chType ) + currImplicitBtDepth;
#else
  const unsigned maxBTD         = cs.pcv->getMaxBtDepth( *cs.slice, chType );
#endif
  const unsigned maxBtSize      = cs.pcv->getMaxBtSize( *cs.slice, chType );
  const unsigned minBtSize      = cs.pcv->getMinBtSize( *cs.slice, chType );
  const unsigned maxTtSize      = cs.pcv->getMaxTtSize( *cs.slice, chType );
  const unsigned minTtSize      = cs.pcv->getMinTtSize( *cs.slice, chType );
#if ENABLE_BMS
  const unsigned maxTrSize      = cs.sps->getMaxTrSize();
#endif

  const PartSplit lastSplit = m_partStack.back().split;
  const PartSplit parlSplit = lastSplit == CU_TRIH_SPLIT ? CU_HORZ_SPLIT : CU_VERT_SPLIT;
#if JVET_K0351_LESS_CONSTRAINT == 0
  const PartSplit prevSplit     = m_partStack.back().firstSubPartSplit;
  const PartSplit perpSplit     = lastSplit == CU_HORZ_SPLIT ? CU_VERT_SPLIT : CU_HORZ_SPLIT;
  const PartSplit perpTriSp     = lastSplit == CU_HORZ_SPLIT ? CU_TRIV_SPLIT : CU_TRIH_SPLIT;
#endif

  if( isNonLog2BlockSize( currArea().Y() ) )
  {
    return false;
  }

  switch( split )
  {
  case CTU_LEVEL:
    THROW( "Checking if top level split is possible" );
    return true;
    break;
#if ENABLE_BMS
  case TU_MAX_TR_SPLIT:
    return area.width > maxTrSize || area.height > maxTrSize;
    break;
#endif
  case CU_QUAD_SPLIT:
  {
    // don't allow QT-splitting below a BT split
    PartSplit lastSplit = m_partStack.back().split;
    if( lastSplit != CTU_LEVEL && lastSplit != CU_QUAD_SPLIT )                  return false;

#if !JVET_K0554
    // allowing QT split even if a BT split is implied
    if( implicitSplit != CU_DONT_SPLIT )                                        return true;

#endif
    unsigned minQtSize = cs.pcv->getMinQtSize( *cs.slice, chType );
    if( currArea().lwidth() <= minQtSize || currArea().lheight() <= minQtSize ) return false;

#if JVET_K0554
    // allowing QT split even if a BT split is implied
    if( implicitSplit != CU_DONT_SPLIT )                                        return true;

#endif
    return true;
  }
  break;
#if JVET_K0554
  case CU_DONT_SPLIT:
    return implicitSplit == CU_DONT_SPLIT;
    break;
#endif
  // general check for BT split, specific checks are done in a separate switch
  case CU_HORZ_SPLIT:
  case CU_VERT_SPLIT:
  {
#if JVET_K0351_LESS_CONSTRAINT == 0
    // don't remove redundancy for intra, as it changes the processing order, which might cause intra gains
    if( !cs.slice->isIntra() && m_partStack.back().idx == 1 && implicitSplit == CU_DONT_SPLIT && ( lastSplit == CU_HORZ_SPLIT || lastSplit == CU_VERT_SPLIT ) )
    {
      if( split == perpSplit && prevSplit == perpSplit && ( ( lastSplit == CU_VERT_SPLIT && m_partStack.back().canQtSplit ) || lastSplit == CU_HORZ_SPLIT ) )
      {
        return false;
      }
    }
#endif
    if( ( lastSplit == CU_TRIH_SPLIT || lastSplit == CU_TRIV_SPLIT ) && currPartIdx() == 1 && split == parlSplit )
    {
      return false;
    }
#if JVET_K0230_DUAL_CODING_TREE_UNDER_64x64_BLOCK
    if (CS::isDualITree(cs) && (area.width > 64 || area.height > 64))
    {
      return false;
    }
#endif
  }
  case CU_TRIH_SPLIT:
  case CU_TRIV_SPLIT:
  {
#if JVET_K0351_LESS_CONSTRAINT == 0
    // don't remove redundancy for intra, as it changes the processing order, which might cause intra gains
    if( !cs.slice->isIntra() && m_partStack.back().idx == 1 && implicitSplit == CU_DONT_SPLIT && ( lastSplit == CU_HORZ_SPLIT || lastSplit == CU_VERT_SPLIT ) )
    {
      if( split == perpTriSp && prevSplit == perpTriSp )
      {
        return false;
      }
    }
#endif
#if JVET_K0230_DUAL_CODING_TREE_UNDER_64x64_BLOCK
    if (CS::isDualITree(cs) && (area.width > 64 || area.height > 64))
    {
      return false;
    }
#endif
  }
#if !HM_QTBT_ONLY_QT_IMPLICIT || JVET_K0554
    if( implicitSplit == split )                                   return true;
    if( implicitSplit != CU_DONT_SPLIT && implicitSplit != split ) return false;
#endif
  case CU_MT_SPLIT:
  case CU_BT_SPLIT:
  {
    if( !cs.sps->getSpsNext().getUseQTBT() )                  return false;
    if( currMtDepth >= maxBTD )                               return false;
    if(      ( area.width <= minBtSize && area.height <= minBtSize )
        && ( ( area.width <= minTtSize && area.height <= minTtSize ) || cs.sps->getSpsNext().getMTTMode() == 0 ) ) return false;
    if(      ( area.width > maxBtSize || area.height > maxBtSize )
        && ( ( area.width > maxTtSize || area.height > maxTtSize ) || cs.sps->getSpsNext().getMTTMode() == 0 ) ) return false;
#if JVET_K0230_DUAL_CODING_TREE_UNDER_64x64_BLOCK
    if (CS::isDualITree(cs) && (area.width > 64 || area.height > 64))
    {
      return false;
    }
#endif
  }
  break;
  default:
    THROW( "Unknown split mode" );
    return false;
    break;
  }

  // specific check for BT splits
  switch( split )
  {
  case CU_HORZ_SPLIT:
    if( area.height <= minBtSize || area.height > maxBtSize )     return false;
    break;
  case CU_VERT_SPLIT:
    if( area.width <= minBtSize || area.width > maxBtSize )       return false;
    break;
  case CU_TRIH_SPLIT:
    if( ( cs.sps->getSpsNext().getMTTMode() & 1 ) != 1 )          return false;
    if( area.height <= 2 * minTtSize || area.height > maxTtSize ) return false;
    break;
  case CU_TRIV_SPLIT:
    if( ( cs.sps->getSpsNext().getMTTMode() & 1 ) != 1 )          return false;
    if( area.width <= 2 * minTtSize || area.width > maxTtSize )   return false;
    break;
  default:
    break;
  }

  return true;
}

bool QTBTPartitioner::isSplitImplicit( const PartSplit split, const CodingStructure &cs )
{
  return split == getImplicitSplit( cs );
}

PartSplit QTBTPartitioner::getImplicitSplit( const CodingStructure &cs )
{
  if( m_partStack.back().checkdIfImplicit )
  {
    return m_partStack.back().implicitSplit;
  }

  PartSplit split = CU_DONT_SPLIT;

#if !ENABLE_BMS
  if( currArea().lwidth() > cs.sps->getMaxTrSize() || currArea().lheight() > cs.sps->getMaxTrSize() )
  {
    split = CU_QUAD_SPLIT;
  }

#endif
  if( split == CU_DONT_SPLIT )
  {
    const bool isBlInPic = cs.picture->Y().contains( currArea().Y().bottomLeft() );
    const bool isTrInPic = cs.picture->Y().contains( currArea().Y().topRight() );

#if HM_QTBT_ONLY_QT_IMPLICIT && !JVET_K0554
    if( !isBlInPic || !isTrInPic )
    {
      split = CU_QUAD_SPLIT;
    }
#else
    const CompArea& area      = currArea().Y();
    const unsigned maxBtSize  = cs.pcv->getMaxBtSize( *cs.slice, chType );
    const bool isBtAllowed    = area.width <= maxBtSize && area.height <= maxBtSize;
#if JVET_K0554
    const unsigned minQtSize  = cs.pcv->getMinQtSize( *cs.slice, chType );
    const bool isQtAllowed    = area.width >  minQtSize && area.height >  minQtSize && currBtDepth == 0;

    if( !isBlInPic && !isTrInPic && isQtAllowed )
#else
    if( !isBlInPic && !isTrInPic )
#endif
    {
      split = CU_QUAD_SPLIT;
    }
    else if( !isBlInPic && isBtAllowed )
    {
      split = CU_HORZ_SPLIT;
    }
    else if( !isTrInPic && isBtAllowed )
    {
      split = CU_VERT_SPLIT;
    }
    else if( !isBlInPic || !isTrInPic )
    {
      split = CU_QUAD_SPLIT;
    }
#endif
#if JVET_K0230_DUAL_CODING_TREE_UNDER_64x64_BLOCK
    if (CS::isDualITree(cs) && (currArea().Y().width > 64 || currArea().Y().height > 64))
    {
      split = CU_QUAD_SPLIT;
    }
#endif
  }

  m_partStack.back().checkdIfImplicit = true;
  m_partStack.back().isImplicit = split != CU_DONT_SPLIT;
  m_partStack.back().implicitSplit = split;

  return split;
}

void QTBTPartitioner::exitCurrSplit()
{
  PartSplit currSplit = m_partStack.back().split;
  unsigned  currIdx = m_partStack.back().idx;

  m_partStack.pop_back();

  CHECK( currDepth == 0, "depth is '0', although a split was performed" );
  currDepth--;
#if _DEBUG
  m_currArea = m_partStack.back().parts[m_partStack.back().idx];
#endif

  if( currSplit == CU_HORZ_SPLIT || currSplit == CU_VERT_SPLIT || currSplit == CU_TRIH_SPLIT || currSplit == CU_TRIV_SPLIT )
  {
    CHECK( !m_partStack.back().checkdIfImplicit, "Didn't check if the current split is implicit" );
    CHECK( currBtDepth == 0, "BT depth is '0', athough a BT split was performed" );
    CHECK( currMtDepth == 0, "MT depth is '0', athough a BT split was performed" );
    currMtDepth--;
#if !HM_QTBT_ONLY_QT_IMPLICIT || JVET_K0554
    if( m_partStack.back().isImplicit ) currImplicitBtDepth--;
#endif
    currBtDepth--;
    if( ( currSplit == CU_TRIH_SPLIT || currSplit == CU_TRIV_SPLIT ) && currIdx != 1 )
    {
      CHECK( currBtDepth == 0, "BT depth is '0', athough a TT split was performed" );
      currBtDepth--;
    }
  }
#if ENABLE_BMS
  else if( currSplit == TU_MAX_TR_SPLIT )
  {
    CHECK( currTrDepth == 0, "TR depth is '0', although a TU split was performed" );
    currTrDepth--;
  }
#endif
  else
  {
#if ENABLE_BMS
    CHECK( currTrDepth > 0, "RQT found with QTBT partitioner" );

#endif
    CHECK( currQtDepth == 0, "QT depth is '0', although a QT split was performed" );
    currQtDepth--;
  }
}

bool QTBTPartitioner::nextPart( const CodingStructure &cs, bool autoPop /*= false*/ )
{
  const Position &prevPos = currArea().blocks[chType].pos();

  unsigned currIdx = ++m_partStack.back().idx;

  m_partStack.back().checkdIfImplicit = false;
  m_partStack.back().isImplicit = false;

  if( currIdx == 1 )
  {
    const CodingUnit* prevCU = cs.getCU( prevPos, chType );
    m_partStack.back().firstSubPartSplit = prevCU ? CU::getSplitAtDepth( *prevCU, currDepth ) : CU_DONT_SPLIT;
  }

  if( currIdx < m_partStack.back().parts.size() )
  {
    if( m_partStack.back().split == CU_TRIH_SPLIT || m_partStack.back().split == CU_TRIV_SPLIT )
    {
      // adapt the current bt depth
      if( currIdx == 1 ) currBtDepth--;
      else               currBtDepth++;
    }
#if _DEBUG
    m_currArea = m_partStack.back().parts[currIdx];
#endif
    return true;
  }
  else
  {
    if( autoPop ) exitCurrSplit();
    return false;
  }
}

bool QTBTPartitioner::hasNextPart()
{
  return ( ( m_partStack.back().idx + 1 ) < m_partStack.back().parts.size() );
}



//////////////////////////////////////////////////////////////////////////
// PartitionerFactory
//////////////////////////////////////////////////////////////////////////

Partitioner* PartitionerFactory::get( const Slice& slice )
{
  if( slice.getSPS()->getSpsNext().getUseQTBT() )
  {
    return new QTBTPartitioner;
  }
  else
  {
    THROW( "Unknown partitioner!" );
  }
}

//////////////////////////////////////////////////////////////////////////
// Partitioner methods describing the actual partitioning logic
//////////////////////////////////////////////////////////////////////////

Partitioning PartitionerImpl::getCUSubPartitions( const UnitArea &cuArea, const CodingStructure &cs, const PartSplit _splitType /*= CU_QUAD_SPLIT*/ )
{
  const PartSplit splitType = _splitType;

  if( splitType == CU_QUAD_SPLIT )
  {
    if( !cs.pcv->noChroma2x2 )
    {
      Partitioning sub;

      sub.resize( 4, cuArea );

      for( uint32_t i = 0; i < 4; i++ )
      {
        for( auto &blk : sub[i].blocks )
        {
          blk.height >>= 1;
          blk.width  >>= 1;
          if( i >= 2 ) blk.y += blk.height;
          if( i &  1 ) blk.x += blk.width;
        }

        CHECK( sub[i].lumaSize().height < MIN_TU_SIZE, "the split causes the block to be smaller than the minimal TU size" );
      }

      return sub;
    }
    else
    {
      const uint32_t minCUSize = ( cs.sps->getMaxCUWidth() >> cs.sps->getMaxCodingDepth() );

      bool canSplit = cuArea.lumaSize().width > minCUSize && cuArea.lumaSize().height > minCUSize;

      Partitioning ret;

      if( cs.slice->getSliceType() == I_SLICE )
      {
        canSplit &= cuArea.lumaSize().width > cs.pcv->minCUWidth && cuArea.lumaSize().height > cs.pcv->minCUHeight;
      }

      if( canSplit )
      {
        ret.resize( 4 );

        if( cuArea.chromaFormat == CHROMA_400 )
        {
          CompArea  blkY = cuArea.Y();
          blkY.width >>= 1;
          blkY.height >>= 1;
          ret[0]  = UnitArea( cuArea.chromaFormat, blkY );
          blkY.x += blkY.width;
          ret[1]  = UnitArea( cuArea.chromaFormat, blkY );
          blkY.x -= blkY.width;
          blkY.y += blkY.height;
          ret[2]  = UnitArea( cuArea.chromaFormat, blkY );
          blkY.x += blkY.width;
          ret[3]  = UnitArea( cuArea.chromaFormat, blkY );
        }
        else
        {
          for( uint32_t i = 0; i < 4; i++ )
          {
            ret[i] = cuArea;

            CompArea &blkY  = ret[i].Y();
            CompArea &blkCb = ret[i].Cb();
            CompArea &blkCr = ret[i].Cr();

            blkY.width  /= 2;
            blkY.height /= 2;

            // TODO: get those params from SPS
            if( blkCb.width > 4 )
            {
              blkCb.width  /= 2;
              blkCb.height /= 2;
              blkCr.width  /= 2;
              blkCr.height /= 2;
            }
            else if( i > 0 )
            {
              blkCb = CompArea();
              blkCr = CompArea();
            }

            if( ( i & 1 ) == 1 )
            {
              blkY.x  += blkY .width;
              blkCb.x += blkCb.width;
              blkCr.x += blkCr.width;
            }

            if( i > 1 )
            {
              blkY.y  += blkY .height;
              blkCb.y += blkCb.height;
              blkCr.y += blkCr.height;
            }
          }
        }
      }

      return ret;
    }
  }
  else if( splitType == CU_HORZ_SPLIT )
  {
    Partitioning sub;

    sub.resize(2, cuArea);

    for (uint32_t i = 0; i < 2; i++)
    {
      for (auto &blk : sub[i].blocks)
      {
        blk.height >>= 1;
        if (i == 1) blk.y += blk.height;
      }

      CHECK(sub[i].lumaSize().height < MIN_TU_SIZE, "the cs split causes the block to be smaller than the minimal TU size");
    }

    return sub;
  }
  else if( splitType == CU_VERT_SPLIT )
  {
    Partitioning sub;

    sub.resize( 2, cuArea );

    for( uint32_t i = 0; i < 2; i++ )
    {
      for( auto &blk : sub[i].blocks )
      {
        blk.width >>= 1;
        if( i == 1 ) blk.x += blk.width;
      }

      CHECK( sub[i].lumaSize().width < MIN_TU_SIZE, "the split causes the block to be smaller than the minimal TU size" );
    }

    return sub;
  }
  else if( splitType == CU_TRIH_SPLIT )
  {
    Partitioning sub;

    sub.resize( 3, cuArea );

    for( int i = 0; i < 3; i++ )
    {
      for( auto &blk : sub[i].blocks )
      {
        blk.height >>= 1;
        if( ( i + 1 ) & 1 ) blk.height >>= 1;
        if( i == 1 )        blk.y       +=     blk.height / 2;
        if( i == 2 )        blk.y       += 3 * blk.height;
      }

      CHECK( sub[i].lumaSize().height < MIN_TU_SIZE, "the cs split causes the block to be smaller than the minimal TU size" );
    }

    return sub;
  }
  else if( splitType == CU_TRIV_SPLIT )
  {
    Partitioning sub;

    sub.resize( 3, cuArea );

    for( int i = 0; i < 3; i++ )
    {
      for( auto &blk : sub[i].blocks )
      {
        blk.width >>= 1;

        if( ( i + 1 ) & 1 ) blk.width >>= 1;
        if( i == 1 )        blk.x      +=     blk.width / 2;
        if( i == 2 )        blk.x      += 3 * blk.width;
      }

      CHECK( sub[i].lumaSize().width < MIN_TU_SIZE, "the cs split causes the block to be smaller than the minimal TU size" );
    }

    return sub;
  }
  else
  {
    THROW( "Unknown CU sub-partitioning" );
    return Partitioning();
  }
}

#if ENABLE_BMS
static const int g_maxRtGridSize = 3;

static const int g_zScanToX[1 << ( g_maxRtGridSize << 1 )] =
{
   0,  1,  0,  1,  2,  3,  2,  3,
   0,  1,  0,  1,  2,  3,  2,  3,
   4,  5,  4,  5,  6,  7,  6,  7,
   4,  5,  4,  5,  6,  7,  6,  7,
   0,  1,  0,  1,  2,  3,  2,  3,
   0,  1,  0,  1,  2,  3,  2,  3,
   4,  5,  4,  5,  6,  7,  6,  7,
   4,  5,  4,  5,  6,  7,  6,  7,
};
static const int g_zScanToY[1 << ( g_maxRtGridSize << 1 )] =
{
   0,  0,  1,  1,  0,  0,  1,  1,
   2,  2,  3,  3,  2,  2,  3,  3,
   0,  0,  1,  1,  0,  0,  1,  1,
   2,  2,  3,  3,  2,  2,  3,  3,
   4,  4,  5,  5,  4,  4,  5,  5,
   6,  6,  7,  7,  6,  5,  7,  7,
   4,  4,  5,  5,  4,  4,  5,  5,
   6,  6,  7,  7,  6,  5,  7,  7,
};
static const int g_rsScanToZ[1 << ( g_maxRtGridSize << 1 )] =
{
   0,  1,  4,  5, 16, 17, 20, 21,
   2,  3,  6,  7, 18, 19, 22, 23,
   8,  9, 12, 13, 24, 25, 28, 29,
  10, 11, 14, 15, 26, 27, 30, 31,
  32, 33, 36, 37, 48, 49, 52, 53,
  34, 35, 38, 39, 50, 51, 54, 55,
  40, 41, 44, 45, 56, 57, 60, 61,
  42, 43, 46, 47, 58, 59, 62, 63,
};

Partitioning PartitionerImpl::getMaxTuTiling( const UnitArea &cuArea, const CodingStructure &cs )
{
  static_assert( MAX_LOG2_DIFF_CU_TR_SIZE <= g_maxRtGridSize, "Z-scan tables are only provided for MAX_LOG2_DIFF_CU_TR_SIZE for up to 3 (8x8 tiling)!" );

  const CompArea area = cuArea.Y().valid() ? cuArea.Y() : cuArea.Cb();
  const int maxTrSize = cs.sps->getMaxTrSize() >> ( isLuma( area.compID ) ? 0 : 1 );
  const int numTilesH = std::max<int>( 1, area.width  / maxTrSize );
  const int numTilesV = std::max<int>( 1, area.height / maxTrSize );
  const int numTiles  = numTilesH * numTilesV;

  CHECK( numTiles > MAX_CU_TILING_PARTITIONS, "CU partitioning requires more partitions than available" );

  Partitioning ret;
  ret.resize( numTiles, cuArea );

  for( int i = 0; i < numTiles; i++ )
  {
    const int rsy = i / numTilesH;
    const int rsx = i % numTilesH;

    const int x = g_zScanToX[g_rsScanToZ[( rsy << g_maxRtGridSize ) + rsx]];
    const int y = g_zScanToY[g_rsScanToZ[( rsy << g_maxRtGridSize ) + rsx]];

    UnitArea& tile = ret[i];

    for( CompArea &comp : tile.blocks )
    {
      if( !comp.valid() ) continue;

      comp.width  /= numTilesH;
      comp.height /= numTilesV;

      comp.x += comp.width  * x;
      comp.y += comp.height * y;
    }
  }

  return ret;
}
#endif

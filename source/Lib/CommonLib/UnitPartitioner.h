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

#ifndef __UNITPARTITIONER__
#define __UNITPARTITIONER__

#include "Unit.h"

#include "CommonDef.h"

#if ENABLE_BMS
static_assert( MAX_CU_TILING_PARTITIONS >= 4, "Minimum required number of partitions for the Partitioning type is 4!" );
typedef static_vector<UnitArea, MAX_CU_TILING_PARTITIONS> Partitioning;
#else
typedef static_vector<UnitArea, 4> Partitioning;
#endif

//////////////////////////////////////////////////////////////////////////
// PartManager class - manages the partitioning tree
//
// contains the currently processed partitioning area (currArea)
// as well as the all partitioning decisions that led to this area
// being processed (in m_partStack).
//////////////////////////////////////////////////////////////////////////

enum PartSplit
{
  CTU_LEVEL        = 0,
  CU_QUAD_SPLIT,

  CU_HORZ_SPLIT,
  CU_VERT_SPLIT,
  CU_TRIH_SPLIT,
  CU_TRIV_SPLIT,
#if ENABLE_BMS
  TU_MAX_TR_SPLIT,
#endif
  NUM_PART_SPLIT,
  CU_MT_SPLIT             = 1000, ///< dummy element to indicate the MT (multi-type-tree) split
  CU_BT_SPLIT             = 1001, ///< dummy element to indicate the BT split
  CU_DONT_SPLIT           = 2000  ///< dummy element to indicate no splitting
};



struct PartLevel
{
  PartSplit    split;
  Partitioning parts;
  unsigned     idx;
  bool         checkdIfImplicit;
  bool         isImplicit;
  PartSplit    implicitSplit;
  PartSplit    firstSubPartSplit;
  bool         canQtSplit;

  PartLevel();
  PartLevel( const PartSplit _split, const Partitioning&  _parts );
  PartLevel( const PartSplit _split,       Partitioning&& _parts );
};

// set depending on max QT / BT possibilities
typedef static_vector<PartLevel, 2 * MAX_CU_DEPTH + 1> PartitioningStack;

class Partitioner
{
protected:
  PartitioningStack m_partStack;
#if _DEBUG
  UnitArea          m_currArea;
#endif

public:
  unsigned currDepth;
  unsigned currQtDepth;
#if ENABLE_BMS
  unsigned currTrDepth;
#endif
  unsigned currBtDepth;
  unsigned currMtDepth;

#if !HM_QTBT_ONLY_QT_IMPLICIT || JVET_K0554
  unsigned currImplicitBtDepth;
#endif
  ChannelType chType;

  virtual ~Partitioner                    () { }

  const PartLevel& currPartLevel          () const { return m_partStack.back(); }
  const UnitArea&  currArea               () const { return currPartLevel().parts[currPartIdx()]; }
  const unsigned   currPartIdx            () const { return currPartLevel().idx; }
  const PartitioningStack& getPartStack   () const { return m_partStack; }

  SplitSeries getSplitSeries              () const;

  virtual void initCtu                    ( const UnitArea& ctuArea, const ChannelType _chType, const Slice& slice )    = 0;
  virtual void splitCurrArea              ( const PartSplit split, const CodingStructure &cs )                          = 0;
  virtual void exitCurrSplit              ()                                                                            = 0;
  virtual bool nextPart                   ( const CodingStructure &cs, bool autoPop = false )                           = 0;
  virtual bool hasNextPart                ()                                                                            = 0;

  virtual void setCUData                  ( CodingUnit& cu );

  virtual void copyState                  ( const Partitioner& other );

public:
  virtual bool canSplit                   ( const PartSplit split,                          const CodingStructure &cs ) = 0;
  virtual bool isSplitImplicit            ( const PartSplit split,                          const CodingStructure &cs ) = 0;
  virtual PartSplit getImplicitSplit      (                                                 const CodingStructure &cs ) = 0;
};

class AdaptiveDepthPartitioner : public Partitioner
{
public:
  void setMaxMinDepth( unsigned& minDepth, unsigned& maxDepth, const CodingStructure& cs ) const;
};

class QTBTPartitioner : public AdaptiveDepthPartitioner
{
public:
  void initCtu                    ( const UnitArea& ctuArea, const ChannelType _chType, const Slice& slice );
  void splitCurrArea              ( const PartSplit split, const CodingStructure &cs );
  void exitCurrSplit              ();
  bool nextPart                   ( const CodingStructure &cs, bool autoPop = false );
  bool hasNextPart                ();

  bool canSplit                   ( const PartSplit split,                          const CodingStructure &cs );
  bool isSplitImplicit            ( const PartSplit split,                          const CodingStructure &cs );
  PartSplit getImplicitSplit      (                                                 const CodingStructure &cs );
};





namespace PartitionerFactory
{
  Partitioner* get( const Slice& slice );
};

//////////////////////////////////////////////////////////////////////////
// Partitioner namespace - contains methods calculating the actual splits
//////////////////////////////////////////////////////////////////////////

namespace PartitionerImpl
{
  Partitioning getCUSubPartitions( const UnitArea   &cuArea, const CodingStructure &cs, const PartSplit splitType = CU_QUAD_SPLIT );
#if ENABLE_BMS
  Partitioning getMaxTuTiling    ( const UnitArea& curArea, const CodingStructure &cs );
#endif
};

#endif

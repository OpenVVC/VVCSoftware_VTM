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

/** \file     EncModeCtrl.cpp
    \brief    Encoder controller for trying out specific modes
*/

#include "EncModeCtrl.h"

#include "AQp.h"
#include "RateCtrl.h"

#include "CommonLib/RdCost.h"
#include "CommonLib/CodingStructure.h"
#include "CommonLib/Picture.h"
#include "CommonLib/UnitTools.h"

#include "CommonLib/dtrace_next.h"

#include <cmath>

void EncModeCtrl::init( EncCfg *pCfg, RateCtrl *pRateCtrl, RdCost* pRdCost )
{
  m_pcEncCfg      = pCfg;
  m_pcRateCtrl    = pRateCtrl;
  m_pcRdCost      = pRdCost;
  m_fastDeltaQP   = false;
#if SHARP_LUMA_DELTA_QP
  m_lumaQPOffset  = 0;

  initLumaDeltaQpLUT();
#endif
}

bool EncModeCtrl::tryModeMaster( const EncTestMode& encTestmode, const CodingStructure &cs, Partitioner& partitioner )
{
#if ENABLE_SPLIT_PARALLELISM
  if( m_ComprCUCtxList.back().isLevelSplitParallel )
  {
    if( !parallelJobSelector( encTestmode, cs, partitioner ) )
    {
      return false;
    }
  }
#endif
  return tryMode( encTestmode, cs, partitioner );
}

void EncModeCtrl::setEarlySkipDetected()
{
  m_ComprCUCtxList.back().earlySkip = true;
}

void EncModeCtrl::xExtractFeatures( const EncTestMode encTestmode, CodingStructure& cs )
{
  CHECK( cs.features.size() < NUM_ENC_FEATURES, "Features vector is not initialized" );

  cs.features[ENC_FT_DISTORTION     ] = double( cs.dist              );
  cs.features[ENC_FT_FRAC_BITS      ] = double( cs.fracBits          );
  cs.features[ENC_FT_RD_COST        ] = double( cs.cost              );
  cs.features[ENC_FT_ENC_MODE_TYPE  ] = double( encTestmode.type     );
  cs.features[ENC_FT_ENC_MODE_OPTS  ] = double( encTestmode.opts     );
  cs.features[ENC_FT_ENC_MODE_PART  ] = double( encTestmode.partSize );
}

bool EncModeCtrl::nextMode( const CodingStructure &cs, Partitioner &partitioner )
{
  m_ComprCUCtxList.back().lastTestMode = m_ComprCUCtxList.back().testModes.back();

  m_ComprCUCtxList.back().testModes.pop_back();

  while( !m_ComprCUCtxList.back().testModes.empty() && !tryModeMaster( currTestMode(), cs, partitioner ) )
  {
    m_ComprCUCtxList.back().testModes.pop_back();
  }

  return !m_ComprCUCtxList.back().testModes.empty();
}

EncTestMode EncModeCtrl::currTestMode() const
{
  return m_ComprCUCtxList.back().testModes.back();
}

EncTestMode EncModeCtrl::lastTestMode() const
{
  return m_ComprCUCtxList.back().lastTestMode;
}

bool EncModeCtrl::anyMode() const
{
  return !m_ComprCUCtxList.back().testModes.empty();
}

void EncModeCtrl::setBest( CodingStructure& cs )
{
  if( cs.cost != MAX_DOUBLE && !cs.cus.empty() )
  {
    m_ComprCUCtxList.back().bestCS = &cs;
    m_ComprCUCtxList.back().bestCU = cs.cus[0];
    m_ComprCUCtxList.back().bestTU = cs.cus[0]->firstTU;
    m_ComprCUCtxList.back().lastTestMode = getCSEncMode( cs );
  }
}

#if !JVET_K0220_ENC_CTRL
bool EncModeCtrl::hasOnlySplitModes() const
{
  for( const auto& mode : m_ComprCUCtxList.back().testModes )
  {
    if( !isModeSplit( mode ) )
    {
      return false;
    }
  }

  return true;
}

#endif
void EncModeCtrl::xGetMinMaxQP( int& minQP, int& maxQP, const CodingStructure& cs, const Partitioner &partitioner, const int baseQP, const SPS& sps, const PPS& pps, const bool splitMode )
{
  if( m_pcEncCfg->getUseRateCtrl() )
  {
    minQP = m_pcRateCtrl->getRCQP();
    maxQP = m_pcRateCtrl->getRCQP();
    return;
  }

  const uint32_t currDepth = partitioner.currDepth;

  if( !splitMode )
  {
    if( currDepth <= pps.getMaxCuDQPDepth() )
    {
      int deltaQP = m_pcEncCfg->getMaxDeltaQP();
      minQP = Clip3( -sps.getQpBDOffset( CHANNEL_TYPE_LUMA ), MAX_QP, baseQP - deltaQP );
      maxQP = Clip3( -sps.getQpBDOffset( CHANNEL_TYPE_LUMA ), MAX_QP, baseQP + deltaQP );
    }
    else
    {
      minQP = cs.currQP[partitioner.chType];
      maxQP = cs.currQP[partitioner.chType];
    }

#if SHARP_LUMA_DELTA_QP
    if( m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() )
    {
      minQP = Clip3( -sps.getQpBDOffset( CHANNEL_TYPE_LUMA ), MAX_QP, baseQP - m_lumaQPOffset );
      maxQP = minQP; // force encode choose the modified QO
    }
#endif
  }
  else
  {
    if( currDepth == pps.getMaxCuDQPDepth() )
    {
      int deltaQP = m_pcEncCfg->getMaxDeltaQP();
      minQP = Clip3( -sps.getQpBDOffset( CHANNEL_TYPE_LUMA ), MAX_QP, baseQP - deltaQP );
      maxQP = Clip3( -sps.getQpBDOffset( CHANNEL_TYPE_LUMA ), MAX_QP, baseQP + deltaQP );
    }
    else if( currDepth < pps.getMaxCuDQPDepth() )
    {
      minQP = baseQP;
      maxQP = baseQP;
    }
    else
    {
      minQP = cs.currQP[partitioner.chType];
      maxQP = cs.currQP[partitioner.chType];
    }

#if SHARP_LUMA_DELTA_QP
    if( m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() )
    {
      minQP = Clip3( -sps.getQpBDOffset( CHANNEL_TYPE_LUMA ), MAX_QP, baseQP - m_lumaQPOffset );
      maxQP = minQP;
    }
#endif
  }
}


int EncModeCtrl::xComputeDQP( const CodingStructure &cs, const Partitioner &partitioner )
{
  Picture* picture    = cs.picture;
  unsigned uiAQDepth  = std::min( partitioner.currDepth, ( uint32_t ) picture->aqlayer.size() - 1 );
  AQpLayer* pcAQLayer = picture->aqlayer[uiAQDepth];

  double dMaxQScale   = pow( 2.0, m_pcEncCfg->getQPAdaptationRange() / 6.0 );
  double dAvgAct      = pcAQLayer->getAvgActivity();
  double dCUAct       = pcAQLayer->getActivity( cs.area.Y().topLeft() );
  double dNormAct     = ( dMaxQScale*dCUAct + dAvgAct ) / ( dCUAct + dMaxQScale*dAvgAct );
  double dQpOffset    = log( dNormAct ) / log( 2.0 ) * 6.0;
  int    iQpOffset    = int( floor( dQpOffset + 0.49999 ) );
  return iQpOffset;
}


#if SHARP_LUMA_DELTA_QP
void EncModeCtrl::initLumaDeltaQpLUT()
{
  const LumaLevelToDeltaQPMapping &mapping = m_pcEncCfg->getLumaLevelToDeltaQPMapping();

  if( !mapping.isEnabled() )
  {
    return;
  }

  // map the sparse LumaLevelToDeltaQPMapping.mapping to a fully populated linear table.

  int         lastDeltaQPValue = 0;
  std::size_t nextSparseIndex = 0;
  for( int index = 0; index < LUMA_LEVEL_TO_DQP_LUT_MAXSIZE; index++ )
  {
    while( nextSparseIndex < mapping.mapping.size() && index >= mapping.mapping[nextSparseIndex].first )
    {
      lastDeltaQPValue = mapping.mapping[nextSparseIndex].second;
      nextSparseIndex++;
    }
    m_lumaLevelToDeltaQPLUT[index] = lastDeltaQPValue;
  }
}

int EncModeCtrl::calculateLumaDQP( const CPelBuf& rcOrg )
{
  double avg = 0;

  // Get QP offset derived from Luma level
#if !WCG_EXT
  if( m_pcEncCfg->getLumaLevelToDeltaQPMapping().mode == LUMALVL_TO_DQP_AVG_METHOD )
#else
  CHECK( m_pcEncCfg->getLumaLevelToDeltaQPMapping().mode != LUMALVL_TO_DQP_AVG_METHOD, "invalid delta qp mode" );
#endif
  {
    // Use avg method
    int sum = 0;
    for( uint32_t y = 0; y < rcOrg.height; y++ )
    {
      for( uint32_t x = 0; x < rcOrg.width; x++ )
      {
        sum += rcOrg.at( x, y );
      }
    }
    avg = ( double ) sum / rcOrg.area();
  }
#if !WCG_EXT
  else
  {
    // Use maximum luma value
    int maxVal = 0;
    for( uint32_t y = 0; y < rcOrg.height; y++ )
    {
      for( uint32_t x = 0; x < rcOrg.width; x++ )
      {
        const Pel& v = rcOrg.at( x, y );
        if( v > maxVal )
        {
          maxVal = v;
        }
      }
    }
    // use a percentage of the maxVal
    avg = ( double ) maxVal * m_pcEncCfg->getLumaLevelToDeltaQPMapping().maxMethodWeight;
  }
#endif
  int lumaIdx = Clip3<int>( 0, int( LUMA_LEVEL_TO_DQP_LUT_MAXSIZE ) - 1, int( avg + 0.5 ) );
  int QP = m_lumaLevelToDeltaQPLUT[lumaIdx];
  return QP;
}
#endif

#if ENABLE_SPLIT_PARALLELISM
void EncModeCtrl::copyState( const EncModeCtrl& other, const UnitArea& area )
{
  m_slice          = other.m_slice;
  m_fastDeltaQP    = other.m_fastDeltaQP;
  m_lumaQPOffset   = other.m_lumaQPOffset;
  m_runNextInParallel
                   = other.m_runNextInParallel;
  m_ComprCUCtxList = other.m_ComprCUCtxList;
}

#endif
void CacheBlkInfoCtrl::create()
{
  const unsigned numPos = MAX_CU_SIZE >> MIN_CU_LOG2;

  m_numWidths  = gp_sizeIdxInfo->numWidths();
  m_numHeights = gp_sizeIdxInfo->numHeights();

  for( unsigned x = 0; x < numPos; x++ )
  {
    for( unsigned y = 0; y < numPos; y++ )
    {
      m_codedCUInfo[x][y] = new CodedCUInfo**[m_numWidths];

      for( int wIdx = 0; wIdx < gp_sizeIdxInfo->numWidths(); wIdx++ )
      {
        if( gp_sizeIdxInfo->isCuSize( gp_sizeIdxInfo->sizeFrom( wIdx ) ) && x + ( gp_sizeIdxInfo->sizeFrom( wIdx ) >> MIN_CU_LOG2 ) <= ( MAX_CU_SIZE >> MIN_CU_LOG2 ) )
        {
          m_codedCUInfo[x][y][wIdx] = new CodedCUInfo*[gp_sizeIdxInfo->numHeights()];

          for( int hIdx = 0; hIdx < gp_sizeIdxInfo->numHeights(); hIdx++ )
          {
            if( gp_sizeIdxInfo->isCuSize( gp_sizeIdxInfo->sizeFrom( hIdx ) ) && y + ( gp_sizeIdxInfo->sizeFrom( hIdx ) >> MIN_CU_LOG2 ) <= ( MAX_CU_SIZE >> MIN_CU_LOG2 ) )
            {
              m_codedCUInfo[x][y][wIdx][hIdx] = new CodedCUInfo;
            }
            else
            {
              m_codedCUInfo[x][y][wIdx][hIdx] = nullptr;
            }
          }
        }
        else
        {
          m_codedCUInfo[x][y][wIdx] = nullptr;
        }
      }
    }
  }
}

void CacheBlkInfoCtrl::destroy()
{
  const unsigned numPos = MAX_CU_SIZE >> MIN_CU_LOG2;

  for( unsigned x = 0; x < numPos; x++ )
  {
    for( unsigned y = 0; y < numPos; y++ )
    {
      for( int wIdx = 0; wIdx < gp_sizeIdxInfo->numWidths(); wIdx++ )
      {
        if( m_codedCUInfo[x][y][wIdx] )
        {
          for( int hIdx = 0; hIdx < gp_sizeIdxInfo->numHeights(); hIdx++ )
          {
            if( m_codedCUInfo[x][y][wIdx][hIdx] )
            {
              delete m_codedCUInfo[x][y][wIdx][hIdx];
            }
          }

          delete[] m_codedCUInfo[x][y][wIdx];
        }
      }

      delete[] m_codedCUInfo[x][y];
    }
  }
}

void CacheBlkInfoCtrl::init( const Slice &slice )
{
  const unsigned numPos = MAX_CU_SIZE >> MIN_CU_LOG2;

  for( unsigned x = 0; x < numPos; x++ )
  {
    for( unsigned y = 0; y < numPos; y++ )
    {
      for( int wIdx = 0; wIdx < gp_sizeIdxInfo->numWidths(); wIdx++ )
      {
        if( m_codedCUInfo[x][y][wIdx] )
        {
          for( int hIdx = 0; hIdx < gp_sizeIdxInfo->numHeights(); hIdx++ )
          {
            if( m_codedCUInfo[x][y][wIdx][hIdx] )
            {
              memset( m_codedCUInfo[x][y][wIdx][hIdx], 0, sizeof( CodedCUInfo ) );
            }
          }
        }
      }
    }
  }

  m_slice_chblk = &slice;
#if ENABLE_SPLIT_PARALLELISM

  m_currTemporalId = 0;
#endif
}
#if ENABLE_SPLIT_PARALLELISM

void CacheBlkInfoCtrl::touch( const UnitArea& area )
{
  CodedCUInfo& cuInfo = getBlkInfo( area );
  cuInfo.temporalId = m_currTemporalId;
}

void CacheBlkInfoCtrl::copyState( const CacheBlkInfoCtrl &other, const UnitArea& area )
{
  m_slice_chblk = other.m_slice_chblk;

  m_currTemporalId = other.m_currTemporalId;

  if( m_slice_chblk->isIntra() ) return;

  const int cuSizeMask = m_slice_chblk->getSPS()->getMaxCUWidth() - 1;

  const int minPosX = ( area.lx() & cuSizeMask ) >> MIN_CU_LOG2;
  const int minPosY = ( area.ly() & cuSizeMask ) >> MIN_CU_LOG2;
  const int maxPosX = ( area.Y().bottomRight().x & cuSizeMask ) >> MIN_CU_LOG2;
  const int maxPosY = ( area.Y().bottomRight().y & cuSizeMask ) >> MIN_CU_LOG2;

  for( unsigned x = minPosX; x <= maxPosX; x++ )
  {
    for( unsigned y = minPosY; y <= maxPosY; y++ )
    {
      for( int wIdx = 0; wIdx < gp_sizeIdxInfo->numWidths(); wIdx++ )
      {
        const int width = gp_sizeIdxInfo->sizeFrom( wIdx );

        if( m_codedCUInfo[x][y][wIdx] && width <= area.lwidth() && x + ( width >> MIN_CU_LOG2 ) <= ( maxPosX + 1 ) )
        {
          for( int hIdx = 0; hIdx < gp_sizeIdxInfo->numHeights(); hIdx++ )
          {
            const int height = gp_sizeIdxInfo->sizeFrom( hIdx );

            if( gp_sizeIdxInfo->isCuSize( height ) && height <= area.lheight() && y + ( height >> MIN_CU_LOG2 ) <= ( maxPosY + 1 ) )
            {
              if( other.m_codedCUInfo[x][y][wIdx][hIdx]->temporalId > m_codedCUInfo[x][y][wIdx][hIdx]->temporalId )
              {
                *m_codedCUInfo[x][y][wIdx][hIdx] = *other.m_codedCUInfo[x][y][wIdx][hIdx];
                m_codedCUInfo[x][y][wIdx][hIdx]->temporalId = m_currTemporalId;
              }
            }
            else if( y + ( height >> MIN_CU_LOG2 ) > maxPosY + 1 )
            {
              break;;
            }
          }
        }
        else if( x + ( width >> MIN_CU_LOG2 ) > maxPosX + 1 )
        {
          break;
        }
      }
    }
  }
}
#endif

CodedCUInfo& CacheBlkInfoCtrl::getBlkInfo( const UnitArea& area )
{
  unsigned idx1, idx2, idx3, idx4;
  getAreaIdx( area.Y(), *m_slice_chblk->getPPS()->pcv, idx1, idx2, idx3, idx4 );

  return *m_codedCUInfo[idx1][idx2][idx3][idx4];
}

bool CacheBlkInfoCtrl::isSkip( const UnitArea& area )
{
  unsigned idx1, idx2, idx3, idx4;
  getAreaIdx( area.Y(), *m_slice_chblk->getPPS()->pcv, idx1, idx2, idx3, idx4 );

  return m_codedCUInfo[idx1][idx2][idx3][idx4]->isSkip;
}

void CacheBlkInfoCtrl::setMv( const UnitArea& area, const RefPicList refPicList, const int iRefIdx, const Mv& rMv )
{
  if( iRefIdx >= MAX_STORED_CU_INFO_REFS ) return;

  unsigned idx1, idx2, idx3, idx4;
  getAreaIdx( area.Y(), *m_slice_chblk->getPPS()->pcv, idx1, idx2, idx3, idx4 );

  m_codedCUInfo[idx1][idx2][idx3][idx4]->saveMv [refPicList][iRefIdx] = rMv;
  m_codedCUInfo[idx1][idx2][idx3][idx4]->validMv[refPicList][iRefIdx] = true;
#if ENABLE_SPLIT_PARALLELISM

  touch( area );
#endif
}

bool CacheBlkInfoCtrl::getMv( const UnitArea& area, const RefPicList refPicList, const int iRefIdx, Mv& rMv ) const
{
  unsigned idx1, idx2, idx3, idx4;
  getAreaIdx( area.Y(), *m_slice_chblk->getPPS()->pcv, idx1, idx2, idx3, idx4 );

  if( iRefIdx >= MAX_STORED_CU_INFO_REFS )
  {
    rMv = m_codedCUInfo[idx1][idx2][idx3][idx4]->saveMv[refPicList][0];
    return false;
  }

  rMv = m_codedCUInfo[idx1][idx2][idx3][idx4]->saveMv[refPicList][iRefIdx];
  return m_codedCUInfo[idx1][idx2][idx3][idx4]->validMv[refPicList][iRefIdx];
}


#if REUSE_CU_RESULTS
static bool isTheSameNbHood( const CodingUnit &cu, const Partitioner &partitioner )
{
  if( cu.chType != partitioner.chType )
  {
    return false;
  }

  const PartitioningStack &ps = partitioner.getPartStack();

  int i = 1;

  for( ; i < ps.size(); i++ )
  {
    if( ps[i].split != CU::getSplitAtDepth( cu, i - 1 ) )
    {
      break;
    }
  }

  const UnitArea &cmnAnc = ps[i - 1].parts[ps[i - 1].idx];

  for( int i = 0; i < cmnAnc.blocks.size(); i++ )
  {
    if( i < cu.blocks.size() && cu.blocks[i].valid() && cu.blocks[i].pos() != cmnAnc.blocks[i].pos() )
    {
      return false;
    }
  }

  return true;
}

void BestEncInfoCache::create( const ChromaFormat chFmt )
{
  const unsigned numPos = MAX_CU_SIZE >> MIN_CU_LOG2;

  m_numWidths  = gp_sizeIdxInfo->numWidths();
  m_numHeights = gp_sizeIdxInfo->numHeights();

  for( unsigned x = 0; x < numPos; x++ )
  {
    for( unsigned y = 0; y < numPos; y++ )
    {
      m_bestEncInfo[x][y] = new BestEncodingInfo**[m_numWidths];

      for( int wIdx = 0; wIdx < gp_sizeIdxInfo->numWidths(); wIdx++ )
      {
        if( gp_sizeIdxInfo->isCuSize( gp_sizeIdxInfo->sizeFrom( wIdx ) ) && x + ( gp_sizeIdxInfo->sizeFrom( wIdx ) >> MIN_CU_LOG2 ) <= ( MAX_CU_SIZE >> MIN_CU_LOG2 ) )
        {
          m_bestEncInfo[x][y][wIdx] = new BestEncodingInfo*[gp_sizeIdxInfo->numHeights()];

          for( int hIdx = 0; hIdx < gp_sizeIdxInfo->numHeights(); hIdx++ )
          {
            if( gp_sizeIdxInfo->isCuSize( gp_sizeIdxInfo->sizeFrom( hIdx ) ) && y + ( gp_sizeIdxInfo->sizeFrom( hIdx ) >> MIN_CU_LOG2 ) <= ( MAX_CU_SIZE >> MIN_CU_LOG2 ) )
            {
              m_bestEncInfo[x][y][wIdx][hIdx] = new BestEncodingInfo;

              int w = gp_sizeIdxInfo->sizeFrom( wIdx );
              int h = gp_sizeIdxInfo->sizeFrom( hIdx );

              const UnitArea area( chFmt, Area( 0, 0, w, h ) );

              m_bestEncInfo[x][y][wIdx][hIdx]->cu.UnitArea::operator=( area );
              m_bestEncInfo[x][y][wIdx][hIdx]->pu.UnitArea::operator=( area );
              m_bestEncInfo[x][y][wIdx][hIdx]->tu.UnitArea::operator=( area );

              m_bestEncInfo[x][y][wIdx][hIdx]->poc      = -1;
              m_bestEncInfo[x][y][wIdx][hIdx]->testMode = EncTestMode();
            }
            else
            {
              m_bestEncInfo[x][y][wIdx][hIdx] = nullptr;
            }
          }
        }
        else
        {
          m_bestEncInfo[x][y][wIdx] = nullptr;
        }
      }
    }
  }
}

void BestEncInfoCache::destroy()
{
  const unsigned numPos = MAX_CU_SIZE >> MIN_CU_LOG2;

  for( unsigned x = 0; x < numPos; x++ )
  {
    for( unsigned y = 0; y < numPos; y++ )
    {
      for( int wIdx = 0; wIdx < gp_sizeIdxInfo->numWidths(); wIdx++ )
      {
        if( m_bestEncInfo[x][y][wIdx] )
        {
          for( int hIdx = 0; hIdx < gp_sizeIdxInfo->numHeights(); hIdx++ )
          {
            if( m_bestEncInfo[x][y][wIdx][hIdx] )
            {
              delete m_bestEncInfo[x][y][wIdx][hIdx];
            }
          }

          delete[] m_bestEncInfo[x][y][wIdx];
        }
      }

      delete[] m_bestEncInfo[x][y];
    }
  }

  delete[] m_pCoeff;
  delete[] m_pPcmBuf;
}

void BestEncInfoCache::init( const Slice &slice )
{
  bool isInitialized = m_slice_bencinf;

  m_slice_bencinf = &slice;

  if( isInitialized ) return;
  
  const unsigned numPos = MAX_CU_SIZE >> MIN_CU_LOG2;

  m_numWidths  = gp_sizeIdxInfo->numWidths();
  m_numHeights = gp_sizeIdxInfo->numHeights();

  size_t numCoeff = 0;
  
  for( unsigned x = 0; x < numPos; x++ )
  {
    for( unsigned y = 0; y < numPos; y++ )
    {
      for( int wIdx = 0; wIdx < gp_sizeIdxInfo->numWidths(); wIdx++ )
      {
        if( m_bestEncInfo[x][y][wIdx] ) for( int hIdx = 0; hIdx < gp_sizeIdxInfo->numHeights(); hIdx++ )
        {
          if( m_bestEncInfo[x][y][wIdx][hIdx] )
          {
            for( const CompArea& blk : m_bestEncInfo[x][y][wIdx][hIdx]->cu.blocks )
            {
              numCoeff += blk.area();
            }
          }
        }
      }
    }
  }

  m_pCoeff  = new TCoeff[numCoeff];
  m_pPcmBuf = new Pel   [numCoeff];

  TCoeff *coeffPtr = m_pCoeff;
  Pel    *pcmPtr   = m_pPcmBuf;

  m_dummyCS.pcv = m_slice_bencinf->getPPS()->pcv;

  for( unsigned x = 0; x < numPos; x++ )
  {
    for( unsigned y = 0; y < numPos; y++ )
    {
      for( int wIdx = 0; wIdx < gp_sizeIdxInfo->numWidths(); wIdx++ )
      {
        if( m_bestEncInfo[x][y][wIdx] ) for( int hIdx = 0; hIdx < gp_sizeIdxInfo->numHeights(); hIdx++ )
        {
          if( m_bestEncInfo[x][y][wIdx][hIdx] )
          {
            TCoeff *coeff[MAX_NUM_TBLOCKS] = { 0, };
            Pel    *pcmbf[MAX_NUM_TBLOCKS] = { 0, };

            const UnitArea &area = m_bestEncInfo[x][y][wIdx][hIdx]->tu;

            for( int i = 0; i < area.blocks.size(); i++ )
            {
              coeff[i] = coeffPtr; coeffPtr += area.blocks[i].area();
              pcmbf[i] =   pcmPtr;   pcmPtr += area.blocks[i].area();
            }

            m_bestEncInfo[x][y][wIdx][hIdx]->tu.cs = &m_dummyCS;
            m_bestEncInfo[x][y][wIdx][hIdx]->tu.init( coeff, pcmbf );
          }
        }
      }
    }
  }
}

bool BestEncInfoCache::setFromCs( const CodingStructure& cs, const Partitioner& partitioner )
{
  if( cs.cus.size() != 1 || cs.tus.size() != 1 || cs.pus.size() != 1 )
  {
    return false;
  }

  unsigned idx1, idx2, idx3, idx4;
  getAreaIdx( cs.area.Y(), *m_slice_bencinf->getPPS()->pcv, idx1, idx2, idx3, idx4 );

  BestEncodingInfo& encInfo = *m_bestEncInfo[idx1][idx2][idx3][idx4];

  encInfo.poc            =  cs.picture->poc;
  encInfo.cu.repositionTo( *cs.cus.front() );
  encInfo.pu.repositionTo( *cs.pus.front() );
  encInfo.tu.repositionTo( *cs.tus.front() );
  encInfo.cu             = *cs.cus.front();
  encInfo.pu             = *cs.pus.front();
  for( auto &blk : cs.tus.front()->blocks )
  {
    if( blk.valid() ) encInfo.tu.copyComponentFrom( *cs.tus.front(), blk.compID );
  }
  encInfo.testMode       = getCSEncMode( cs );

  return true;
}

bool BestEncInfoCache::isValid( const CodingStructure& cs, const Partitioner& partitioner )
{
  unsigned idx1, idx2, idx3, idx4;
  getAreaIdx( cs.area.Y(), *m_slice_bencinf->getPPS()->pcv, idx1, idx2, idx3, idx4 );

  BestEncodingInfo& encInfo = *m_bestEncInfo[idx1][idx2][idx3][idx4];

  if( cs.picture->poc != encInfo.poc || CS::getArea( cs, cs.area, partitioner.chType ) != encInfo.cu || !isTheSameNbHood( encInfo.cu, partitioner ) )
  {
    return false;
  }
  else
  {
    return true;
  }
}

bool BestEncInfoCache::setCsFrom( CodingStructure& cs, EncTestMode& testMode, const Partitioner& partitioner ) const
{
  unsigned idx1, idx2, idx3, idx4;
  getAreaIdx( cs.area.Y(), *m_slice_bencinf->getPPS()->pcv, idx1, idx2, idx3, idx4 );

  BestEncodingInfo& encInfo = *m_bestEncInfo[idx1][idx2][idx3][idx4];

  if( cs.picture->poc != encInfo.poc || CS::getArea( cs, cs.area, partitioner.chType ) != encInfo.cu || !isTheSameNbHood( encInfo.cu, partitioner ) )
  {
    return false;
  }

  CodingUnit     &cu = cs.addCU( CS::getArea( cs, cs.area, partitioner.chType ), partitioner.chType );
  PredictionUnit &pu = cs.addPU( CS::getArea( cs, cs.area, partitioner.chType ), partitioner.chType );
  TransformUnit  &tu = cs.addTU( CS::getArea( cs, cs.area, partitioner.chType ), partitioner.chType );

  cu          .repositionTo( encInfo.cu );
  pu          .repositionTo( encInfo.pu );
  tu          .repositionTo( encInfo.tu );

  cu          = encInfo.cu;
  pu          = encInfo.pu;
  for( auto &blk : tu.blocks )
  {
    if( blk.valid() ) tu.copyComponentFrom( encInfo.tu, blk.compID );
  }

  testMode    = encInfo.testMode;

  return true;
}

#endif
#if !JVET_K0220_ENC_CTRL
void SaveLoadEncInfoCtrl::create()
{
  m_saveLoadInfo = new SaveLoadStruct*[gp_sizeIdxInfo->numWidths()];

  for( int wIdx = 0; wIdx < gp_sizeIdxInfo->numWidths(); wIdx++ )
  {
    m_saveLoadInfo[wIdx] = new SaveLoadStruct[gp_sizeIdxInfo->numHeights()];
  }
}

void SaveLoadEncInfoCtrl::destroy()
{
  for( int wIdx = 0; wIdx < gp_sizeIdxInfo->numWidths(); wIdx++ )
  {
    delete[] m_saveLoadInfo[wIdx];
  }

  delete[] m_saveLoadInfo;
}

void SaveLoadEncInfoCtrl::init( const Slice &slice )
{
  for( int wIdx = 0; wIdx < gp_sizeIdxInfo->numWidths(); wIdx++ )
  {
    memset( m_saveLoadInfo[wIdx], 0, gp_sizeIdxInfo->numHeights() * sizeof( SaveLoadStruct ) );
  }

  m_slice_sls = &slice;
}
#if ENABLE_SPLIT_PARALLELISM

void SaveLoadEncInfoCtrl::copyState( const SaveLoadEncInfoCtrl &other, const UnitArea& area )
{
  for( int wIdx = 0; wIdx < gp_sizeIdxInfo->numWidths(); wIdx++ )
  {
    memcpy( m_saveLoadInfo[wIdx], other.m_saveLoadInfo[wIdx], gp_sizeIdxInfo->numHeights() * sizeof( SaveLoadStruct ) );
  }

  m_slice_sls = other.m_slice_sls;
}
#endif

SaveLoadStruct& SaveLoadEncInfoCtrl::getSaveLoadStruct( const UnitArea& area )
{
  unsigned idx1, idx2, idx3, idx4;
  getAreaIdx( area.Y(), *m_slice_sls->getPPS()->pcv, idx1, idx2, idx3, idx4 );

  return m_saveLoadInfo[idx3][idx4];
}

SaveLoadStruct& SaveLoadEncInfoCtrl::getSaveLoadStructQuad( const UnitArea& area )
{
  unsigned idx1, idx2, idx3, idx4;
  getAreaIdx( Area( area.lx(), area.ly(), area.lwidth() / 2, area.lheight() / 2 ), *m_slice_sls->getPPS()->pcv, idx1, idx2, idx3, idx4 );

  return m_saveLoadInfo[idx3][idx4];
}

SaveLoadTag SaveLoadEncInfoCtrl::getSaveLoadTag( const UnitArea& area )
{
  unsigned idx1, idx2, idx3, idx4;
  getAreaIdx( area.Y(), *m_slice_sls->getPPS()->pcv, idx1, idx2, idx3, idx4 );

  unsigned PartIdx = ( ( idx1 << 8 ) | idx2 );
  SaveLoadTag uc   = ( PartIdx == m_saveLoadInfo[idx3][idx4].partIdx ) ? m_saveLoadInfo[idx3][idx4].tag : SAVE_LOAD_INIT;
  return uc;
}
unsigned SaveLoadEncInfoCtrl::getSaveLoadInterDir( const UnitArea& area )
{
  unsigned idx1, idx2, idx3, idx4;
  getAreaIdx( area.Y(), *m_slice_sls->getPPS()->pcv, idx1, idx2, idx3, idx4 );

  return m_saveLoadInfo[idx3][idx4].interDir;
}


#if JVET_K_AFFINE
unsigned SaveLoadEncInfoCtrl::getSaveLoadAffineFlag( const UnitArea& area )
{
  unsigned idx1, idx2, idx3, idx4;
  getAreaIdx( area.Y(), *m_slice_sls->getPPS()->pcv, idx1, idx2, idx3, idx4 );

  return m_saveLoadInfo[idx3][idx4].affineFlag;
}
#endif
#endif

static bool interHadActive( const ComprCUCtx& ctx )
{
  return ctx.interHad != 0;
}

//////////////////////////////////////////////////////////////////////////
// EncModeCtrlQTBT
//////////////////////////////////////////////////////////////////////////

EncModeCtrlMTnoRQT::EncModeCtrlMTnoRQT()
{
#if !REUSE_CU_RESULTS
  CacheBlkInfoCtrl::create();
#if !JVET_K0220_ENC_CTRL
  SaveLoadEncInfoCtrl::create();
#endif
#endif
}

EncModeCtrlMTnoRQT::~EncModeCtrlMTnoRQT()
{
#if !REUSE_CU_RESULTS
  CacheBlkInfoCtrl::destroy();
#if !JVET_K0220_ENC_CTRL
  SaveLoadEncInfoCtrl::destroy();
#endif
#endif
}

#if REUSE_CU_RESULTS
void EncModeCtrlMTnoRQT::create( const EncCfg& cfg )
{
  CacheBlkInfoCtrl::create();
  BestEncInfoCache::create( cfg.getChromaFormatIdc() );
#if !JVET_K0220_ENC_CTRL
  SaveLoadEncInfoCtrl::create();
#endif
}

void EncModeCtrlMTnoRQT::destroy()
{
  CacheBlkInfoCtrl::destroy();
  BestEncInfoCache::destroy();
#if !JVET_K0220_ENC_CTRL
  SaveLoadEncInfoCtrl::destroy();
#endif
}

#endif
void EncModeCtrlMTnoRQT::initCTUEncoding( const Slice &slice )
{
  CacheBlkInfoCtrl::init( slice );
#if REUSE_CU_RESULTS
  BestEncInfoCache::init( slice );
#endif
#if !JVET_K0220_ENC_CTRL
  SaveLoadEncInfoCtrl::init( slice );
#endif

  CHECK( !m_ComprCUCtxList.empty(), "Mode list is not empty at the beginning of a CTU" );

  m_slice             = &slice;
#if ENABLE_SPLIT_PARALLELISM
  m_runNextInParallel      = false;
#endif

  if( m_pcEncCfg->getUseE0023FastEnc() )
  {
    m_skipThreshold = ( ( slice.getMinPictureDistance() <= PICTURE_DISTANCE_TH ) ? FAST_SKIP_DEPTH : SKIP_DEPTH );
  }
  else
  {
    m_skipThreshold = SKIP_DEPTH;
  }
}

#if ENABLE_TRACING && !JVET_K0220_ENC_CTRL
static unsigned getHalvedIdx( unsigned idx )
{
  return gp_sizeIdxInfo->idxFrom( gp_sizeIdxInfo->sizeFrom( idx ) >> 1 );
}
#endif

void EncModeCtrlMTnoRQT::initCULevel( Partitioner &partitioner, const CodingStructure& cs )
{
  // Min/max depth
  unsigned minDepth = 0;
  unsigned maxDepth = g_aucLog2[cs.sps->getSpsNext().getCTUSize()] - g_aucLog2[cs.sps->getSpsNext().getMinQTSize( m_slice->getSliceType(), partitioner.chType )];
  if( m_pcEncCfg->getUseFastLCTU() )
  {
    if( auto adPartitioner = dynamic_cast<AdaptiveDepthPartitioner*>( &partitioner ) )
    {
      // LARGE CTU
      adPartitioner->setMaxMinDepth( minDepth, maxDepth, cs );
    }
  }

  m_ComprCUCtxList.push_back( ComprCUCtx( cs, minDepth, maxDepth, NUM_EXTRA_FEATURES ) );

#if ENABLE_SPLIT_PARALLELISM
  if( m_runNextInParallel )
  {
    for( auto &level : m_ComprCUCtxList )
    {
      CHECK( level.isLevelSplitParallel, "Tring to parallelize a level within parallel execution!" );
    }
    CHECK( cs.picture->scheduler.getSplitJobId() == 0, "Trying to run a parallel level although jobId is 0!" );
    m_runNextInParallel                          = false;
    m_ComprCUCtxList.back().isLevelSplitParallel = true;
  }

#endif
#if !HM_NO_ADDITIONAL_SPEEDUPS || JVET_K0220_ENC_CTRL
  const CodingUnit* cuLeft  = cs.getCU( cs.area.blocks[partitioner.chType].pos().offset( -1, 0 ), partitioner.chType );
  const CodingUnit* cuAbove = cs.getCU( cs.area.blocks[partitioner.chType].pos().offset( 0, -1 ), partitioner.chType );

  const bool qtBeforeBt = ( (  cuLeft  &&  cuAbove  && cuLeft ->qtDepth > partitioner.currQtDepth && cuAbove->qtDepth > partitioner.currQtDepth )
                         || (  cuLeft  && !cuAbove  && cuLeft ->qtDepth > partitioner.currQtDepth )
                         || ( !cuLeft  &&  cuAbove  && cuAbove->qtDepth > partitioner.currQtDepth )
                         || ( !cuAbove && !cuLeft   && cs.area.lwidth() >= ( 32 << cs.slice->getDepth() ) ) )
                         && ( cs.area.lwidth() > ( cs.pcv->getMinQtSize( *cs.slice, partitioner.chType ) << 1 ) );
#endif

  // set features
  ComprCUCtx &cuECtx  = m_ComprCUCtxList.back();
  cuECtx.set( BEST_NON_SPLIT_COST,  MAX_DOUBLE );
  cuECtx.set( BEST_VERT_SPLIT_COST, MAX_DOUBLE );
  cuECtx.set( BEST_HORZ_SPLIT_COST, MAX_DOUBLE );
  cuECtx.set( BEST_TRIH_SPLIT_COST, MAX_DOUBLE );
  cuECtx.set( BEST_TRIV_SPLIT_COST, MAX_DOUBLE );
  cuECtx.set( DO_TRIH_SPLIT,        cs.sps->getSpsNext().getMTTMode() & 1 );
  cuECtx.set( DO_TRIV_SPLIT,        cs.sps->getSpsNext().getMTTMode() & 1 );
#if !JVET_K0220_ENC_CTRL
  SaveLoadStruct &sls = getSaveLoadStruct( partitioner.currArea() );
  cuECtx.set( HISTORY_DO_SAVE,      sls.partIdx == cuECtx.partIdx && sls.tag == SAVE_ENC_INFO );
  cuECtx.set( SAVE_LOAD_TAG,        sls.partIdx == cuECtx.partIdx  ? sls.tag  : SAVE_LOAD_INIT );
  cuECtx.set( HISTORY_NEED_TO_SAVE, m_pcEncCfg->getUseSaveLoadEncInfo() && cs.area.lwidth() > ( 1 << MIN_CU_LOG2 ) && cs.area.lheight() > ( 1 << MIN_CU_LOG2 ) );
#endif
#if JVET_K0357_AMVR
  cuECtx.set( BEST_IMV_COST,        MAX_DOUBLE * .5 );
  cuECtx.set( BEST_NO_IMV_COST,     MAX_DOUBLE * .5 );
#endif
#if !HM_NO_ADDITIONAL_SPEEDUPS || JVET_K0220_ENC_CTRL
  cuECtx.set( QT_BEFORE_BT,         qtBeforeBt );
  cuECtx.set( DID_QUAD_SPLIT,       false );
  cuECtx.set( IS_BEST_NOSPLIT_SKIP, false );
  cuECtx.set( MAX_QT_SUB_DEPTH,     0 );
#endif
#if REUSE_CU_RESULTS
  const bool isReusingCu = isValid( cs, partitioner );
  cuECtx.set( IS_REUSING_CU,        isReusingCu );
#endif

#if !JVET_K0220_ENC_CTRL
  DTRACE( g_trace_ctx, D_SAVE_LOAD, "SaveLoadTag at %d,%d (%dx%d): %d, Split: %d\n",
          cs.area.lx(), cs.area.ly(),
          cs.area.lwidth(), cs.area.lheight(),
          cuECtx.get<SaveLoadTag>( SAVE_LOAD_TAG ),
          sls.partIdx == cuECtx.partIdx && sls.tag == LOAD_ENC_INFO && m_pcEncCfg->getUseSaveLoadSplitDecision() ? sls.split : 0 );

#endif
  // QP
  int baseQP = cs.baseQP;
  if( m_pcEncCfg->getUseAdaptiveQP() )
  {
    baseQP = Clip3( -cs.sps->getQpBDOffset( CHANNEL_TYPE_LUMA ), MAX_QP, baseQP + xComputeDQP( cs, partitioner ) );
  }
  int minQP = baseQP;
  int maxQP = baseQP;

#if SHARP_LUMA_DELTA_QP
  if( m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() )
  {
    if( partitioner.currDepth <= cs.pps->getMaxCuDQPDepth() )
    {
      CompArea clipedArea = clipArea( cs.area.Y(), cs.picture->Y() );
      // keep using the same m_QP_LUMA_OFFSET in the same CTU
      m_lumaQPOffset = calculateLumaDQP( cs.getOrgBuf( clipedArea ) );
    }
  }
#endif

  xGetMinMaxQP( minQP, maxQP, cs, partitioner, baseQP, *cs.sps, *cs.pps, true );

  // Add coding modes here
  // NOTE: Working back to front, as a stack, which is more efficient with the container
  // NOTE: First added modes will be processed at the end.

  //////////////////////////////////////////////////////////////////////////
  // Add unit split modes

#if !HM_NO_ADDITIONAL_SPEEDUPS || JVET_K0220_ENC_CTRL
  if( !cuECtx.get<bool>( QT_BEFORE_BT ) )
  {
    for( int qp = maxQP; qp >= minQP; qp-- )
    {
      m_ComprCUCtxList.back().testModes.push_back( { ETM_SPLIT_QT, SIZE_2Nx2N, ETO_STANDARD, qp, false } );
    }
  }
#else
  for( int qp = maxQP; qp >= minQP; qp-- )
  {
    m_ComprCUCtxList.back().testModes.push_back( { ETM_SPLIT_QT, SIZE_2Nx2N, ETO_STANDARD, qp, false } );
  }
#endif

  if( partitioner.canSplit( CU_TRIV_SPLIT, cs ) )
  {
    // add split modes
    for( int qp = maxQP; qp >= minQP; qp-- )
    {
      m_ComprCUCtxList.back().testModes.push_back( { ETM_SPLIT_TT_V, SIZE_2Nx2N, ETO_STANDARD, qp, false } );
    }
  }

  if( partitioner.canSplit( CU_TRIH_SPLIT, cs ) )
  {
    // add split modes
    for( int qp = maxQP; qp >= minQP; qp-- )
    {
      m_ComprCUCtxList.back().testModes.push_back( { ETM_SPLIT_TT_H, SIZE_2Nx2N, ETO_STANDARD, qp, false } );
    }
  }

  if( partitioner.canSplit( CU_VERT_SPLIT, cs ) )
  {
    // add split modes
    for( int qp = maxQP; qp >= minQP; qp-- )
    {
      m_ComprCUCtxList.back().testModes.push_back( { ETM_SPLIT_BT_V, SIZE_2Nx2N, ETO_STANDARD, qp, false } );
    }
    m_ComprCUCtxList.back().set( DID_VERT_SPLIT, true );
  }
  else
  {
    m_ComprCUCtxList.back().set( DID_VERT_SPLIT, false );
  }

  if( partitioner.canSplit( CU_HORZ_SPLIT, cs ) )
  {
    // add split modes
    for( int qp = maxQP; qp >= minQP; qp-- )
    {
      m_ComprCUCtxList.back().testModes.push_back( { ETM_SPLIT_BT_H, SIZE_2Nx2N, ETO_STANDARD, qp, false } );
    }
    m_ComprCUCtxList.back().set( DID_HORZ_SPLIT, true );
  }
  else
  {
    m_ComprCUCtxList.back().set( DID_HORZ_SPLIT, false );
  }

#if !HM_NO_ADDITIONAL_SPEEDUPS || JVET_K0220_ENC_CTRL
  if( cuECtx.get<bool>( QT_BEFORE_BT ) )
  {
    for( int qp = maxQP; qp >= minQP; qp-- )
    {
      m_ComprCUCtxList.back().testModes.push_back( { ETM_SPLIT_QT, SIZE_2Nx2N, ETO_STANDARD, qp, false } );
    }
  }

#endif
  m_ComprCUCtxList.back().testModes.push_back( { ETM_POST_DONT_SPLIT } );

#if REUSE_CU_RESULTS
  if( isReusingCu )
  {
    m_ComprCUCtxList.back().testModes.push_back( { ETM_RECO_CACHED } );
  }

#endif
  xGetMinMaxQP( minQP, maxQP, cs, partitioner, baseQP, *cs.sps, *cs.pps, false );

  bool useLossless = false;
  int  lowestQP = minQP;
  if( cs.pps->getTransquantBypassEnabledFlag() )
  {
    useLossless = true; // mark that the first iteration is to cost TQB mode.
    minQP = minQP - 1;  // increase loop variable range by 1, to allow testing of TQB mode along with other QPs

    if( m_pcEncCfg->getCUTransquantBypassFlagForceValue() )
    {
      maxQP = minQP;
    }
  }

  //////////////////////////////////////////////////////////////////////////
  // Add unit coding modes: Intra, InterME, InterMerge ...

  for( int qpLoop = maxQP; qpLoop >= minQP; qpLoop-- )
  {
    const int  qp       = std::max( qpLoop, lowestQP );
    const bool lossless = useLossless && qpLoop == minQP;
    // add intra modes
    m_ComprCUCtxList.back().testModes.push_back( { ETM_IPCM,  SIZE_2Nx2N, ETO_STANDARD, qp, lossless } );
    m_ComprCUCtxList.back().testModes.push_back( { ETM_INTRA, SIZE_2Nx2N, ETO_STANDARD, qp, lossless } );
  }

  // add first pass modes
  if( !m_slice->isIntra() )
  {
    for( int qpLoop = maxQP; qpLoop >= minQP; qpLoop-- )
    {
      const int  qp       = std::max( qpLoop, lowestQP );
      const bool lossless = useLossless && qpLoop == minQP;

#if JVET_K0357_AMVR
      if( m_pcEncCfg->getIMV() )
      {
        if( m_pcEncCfg->getIMV() == IMV_4PEL )
        {
          int imv = m_pcEncCfg->getIMV4PelFast() ? 3 : 2;
          m_ComprCUCtxList.back().testModes.push_back( { ETM_INTER_ME, SIZE_2Nx2N, EncTestModeOpts( imv << ETO_IMV_SHIFT ), qp, lossless } );
        }
        m_ComprCUCtxList.back().testModes.push_back( { ETM_INTER_ME, SIZE_2Nx2N, EncTestModeOpts( 1 << ETO_IMV_SHIFT ), qp, lossless } );
      }
#endif
      // add inter modes
      if( m_pcEncCfg->getUseEarlySkipDetection() )
      {
        m_ComprCUCtxList.back().testModes.push_back( { ETM_MERGE_SKIP,  SIZE_2Nx2N, ETO_STANDARD, qp, lossless } );
#if JVET_K_AFFINE
        if( cs.sps->getSpsNext().getUseAffine() )
        {
          m_ComprCUCtxList.back().testModes.push_back( { ETM_AFFINE,      SIZE_2Nx2N, ETO_STANDARD, qp, lossless } );
        }
#endif
        m_ComprCUCtxList.back().testModes.push_back( { ETM_INTER_ME,    SIZE_2Nx2N, ETO_STANDARD, qp, lossless } );
      }
      else
      {
        m_ComprCUCtxList.back().testModes.push_back( { ETM_INTER_ME,    SIZE_2Nx2N, ETO_STANDARD, qp, lossless } );

        m_ComprCUCtxList.back().testModes.push_back( { ETM_MERGE_SKIP,  SIZE_2Nx2N, ETO_STANDARD, qp, lossless } );
#if JVET_K_AFFINE
        if( cs.sps->getSpsNext().getUseAffine() )
        {
          m_ComprCUCtxList.back().testModes.push_back( { ETM_AFFINE,      SIZE_2Nx2N, ETO_STANDARD, qp, lossless } );
        }
#endif
      }
    }
  }

  // ensure to skip unprobable modes
  if( !tryModeMaster( m_ComprCUCtxList.back().testModes.back(), cs, partitioner ) )
  {
    nextMode( cs, partitioner );
  }

  m_ComprCUCtxList.back().lastTestMode = EncTestMode();
}

void EncModeCtrlMTnoRQT::finishCULevel( Partitioner &partitioner )
{
#if !JVET_K0220_ENC_CTRL
  ComprCUCtx &cuECtx   = m_ComprCUCtxList.back();
  const CompArea &area = partitioner.currArea().Y();
  SaveLoadStruct &sls  = getSaveLoadStruct( partitioner.currArea() );

  if( cuECtx.get<bool>( HISTORY_DO_SAVE ) && cuECtx.bestCS )
  {
    sls.partIdx = cuECtx.partIdx;
    sls.tag     = LOAD_ENC_INFO;
    cuECtx.set( HISTORY_DO_SAVE, false );
  }

  if( m_pcEncCfg->getUseSaveLoadEncInfo() && area.width > ( 1 << MIN_CU_LOG2 ) && area.height > ( 1 << MIN_CU_LOG2 ) )
  {
    SaveLoadStruct &subSls = getSaveLoadStructQuad( partitioner.currArea() );

    subSls.tag     = SAVE_LOAD_INIT;
    subSls.partIdx = 0xff;

    DTRACE( g_trace_ctx, D_SAVE_LOAD, "SaveLoad %d (%d %d) [%d %d] \n", SAVE_LOAD_INIT, -1, -1, getHalvedIdx( cuECtx.cuW ), getHalvedIdx( cuECtx.cuH ) );
    DTRACE( g_trace_ctx, D_SAVE_LOAD, "saving tag at %d,%d (%dx%d) -> %d\n", area.x, area.y, area.width >> 1, area.height >> 1, SAVE_LOAD_INIT );
  }

  if( m_pcEncCfg->getUseSaveLoadSplitDecision() && m_pcEncCfg->getUseSaveLoadEncInfo() && area.width > ( 1 << MIN_CU_LOG2 ) && area.height > ( 1 << MIN_CU_LOG2 ) && cuECtx.get<uint8_t>( SAVE_LOAD_TAG ) == SAVE_ENC_INFO )
  {
    uint8_t c = 0;
    double bestNonSplit = cuECtx.get<double>( BEST_NON_SPLIT_COST  );
    double horzCost     = cuECtx.get<double>( BEST_HORZ_SPLIT_COST );
    double vertCost     = cuECtx.get<double>( BEST_VERT_SPLIT_COST );
    double cshorzCost   = cuECtx.get<double>( BEST_TRIH_SPLIT_COST );
    double csvertCost   = cuECtx.get<double>( BEST_TRIV_SPLIT_COST );
    double threshold    = JVET_D0077_SPLIT_DECISION_COST_SCALE * std::min( bestNonSplit, std::min( horzCost, vertCost ) );

    if( bestNonSplit > threshold )
    {
      c |= 0x01;
    }
    if( horzCost > threshold )
    {
      c |= 0x02;
    }
    if( vertCost > threshold )
    {
      c |= 0x04;
    }
    if( cshorzCost > threshold )
    {
      c |= 0x08;
    }
    if( csvertCost > threshold )
    {
      c |= 0x10;
    }
    sls.split = c;

    DTRACE( g_trace_ctx, D_SAVE_LOAD, "saving split info at %d,%d (%dx%d) thres = %f, cans (%f, %f, %f) -> %d\n", area.x, area.y, area.width, area.height, threshold, bestNonSplit, horzCost, vertCost, c & 0x7 );
  }

#endif
  m_ComprCUCtxList.pop_back();
}


bool EncModeCtrlMTnoRQT::tryMode( const EncTestMode& encTestmode, const CodingStructure &cs, Partitioner& partitioner )
{
  CHECK( encTestmode.partSize != SIZE_2Nx2N, "Only 2Nx2N supported with QTBT" );

  ComprCUCtx& cuECtx = m_ComprCUCtxList.back();

  // Fast checks, partitioning depended

  // if early skip detected, skip all modes checking but the splits
  if( cuECtx.earlySkip && m_pcEncCfg->getUseEarlySkipDetection() && !isModeSplit( encTestmode ) && !( isModeInter( encTestmode ) && encTestmode.partSize == SIZE_2Nx2N ) )
  {
    return false;
  }

  const PartSplit implicitSplit = partitioner.getImplicitSplit( cs );
  const bool isBoundary         = implicitSplit != CU_DONT_SPLIT;

  if( isBoundary && encTestmode.type != ETM_SPLIT_QT )
  {
#if !HM_QTBT_ONLY_QT_IMPLICIT || JVET_K0554
    return getPartSplit( encTestmode ) == implicitSplit;
#else
    return false;
#endif
  }
  else if( isBoundary && encTestmode.type == ETM_SPLIT_QT )
  {
    return partitioner.canSplit( CU_QUAD_SPLIT, cs );
  }

#if !ENABLE_BMS
  if( partitioner.currArea().lwidth() > m_slice->getSPS()->getMaxTrSize() && encTestmode.type != ETM_SPLIT_QT )
  {
    return false;
  }

#endif
#if REUSE_CU_RESULTS
  if( cuECtx.get<bool>( IS_REUSING_CU ) )
  {
    if( encTestmode.type == ETM_RECO_CACHED )
    {
      return true;
    }

    if( isModeNoSplit( encTestmode ) )
    {
      return false;
    }
  }

#endif
  const Slice&           slice       = *m_slice;
  const SPS&             sps         = *slice.getSPS();
  const uint32_t             numComp     = getNumberValidComponents( slice.getSPS()->getChromaFormatIdc() );
  const uint32_t             width       = partitioner.currArea().lumaSize().width;
  const CodingStructure *bestCS      = cuECtx.bestCS;
  const CodingUnit      *bestCU      = cuECtx.bestCU;
  const EncTestMode      bestMode    = bestCS ? getCSEncMode( *bestCS ) : EncTestMode();

#if !JVET_K0220_ENC_CTRL
  uint8_t       saveLoadSplit          = 0;
  SaveLoadTag saveLoadTag            = cuECtx.get<SaveLoadTag>( SAVE_LOAD_TAG );
  SaveLoadStruct &sls                = getSaveLoadStruct( partitioner.currArea() );
#endif
  CodedCUInfo    &relatedCU          = getBlkInfo( partitioner.currArea() );

  if( cuECtx.minDepth > partitioner.currQtDepth && partitioner.canSplit( CU_QUAD_SPLIT, cs ) )
  {
    // enforce QT
    return encTestmode.type == ETM_SPLIT_QT;
  }
  else if( encTestmode.type == ETM_SPLIT_QT && cuECtx.maxDepth <= partitioner.currQtDepth )
  {
    // don't check this QT depth
    return false;
  }

#if !JVET_K0220_ENC_CTRL
  if( m_pcEncCfg->getUseSaveLoadEncInfo() && m_pcEncCfg->getUseSaveLoadSplitDecision() && saveLoadTag == LOAD_ENC_INFO )
  {
    saveLoadSplit = sls.split;
    if( ( saveLoadSplit & 0x01 ) && isModeNoSplit( encTestmode ) )
    {
      return false;
    }
  }
#endif
  if( bestCS && bestCS->cus.size() == 1 )
  {
    // update the best non-split cost
    cuECtx.set( BEST_NON_SPLIT_COST, bestCS->cost );
  }

  if( encTestmode.type == ETM_INTRA )
  {
    if( getFastDeltaQp() )
    {
      if( cs.area.lumaSize().width > cs.pcv->fastDeltaQPCuMaxSize )
      {
        return false; // only check necessary 2Nx2N Intra in fast delta-QP mode
      }
    }

    if( m_pcEncCfg->getUseFastLCTU() && partitioner.currArea().lumaSize().area() > 4096 )
    {
      return false;
    }

#if JVET_K0230_DUAL_CODING_TREE_UNDER_64x64_BLOCK
    if (CS::isDualITree(cs) && (partitioner.currArea().lumaSize().width > 64 || partitioner.currArea().lumaSize().height > 64))
    {
      return false;
    }
#endif

    if( m_pcEncCfg->getUsePbIntraFast() && !cs.slice->isIntra() && !interHadActive( cuECtx ) && cuECtx.bestCU && CU::isInter( *cuECtx.bestCU ) )
    {
      return false;
    }

    // INTRA MODES
    CHECK( !slice.isIntra() && !cuECtx.bestTU, "No possible non-intra encoding for a P- or B-slice found" );

    if( !( slice.isIntra() || bestMode.type == ETM_INTRA ||
         ( ( !m_pcEncCfg->getDisableIntraPUsInInterSlices() ) && !relatedCU.isInter && (
                                         ( cuECtx.bestTU->cbf[0] != 0 ) ||
           ( ( numComp > COMPONENT_Cb ) && cuECtx.bestTU->cbf[1] != 0 ) ||
           ( ( numComp > COMPONENT_Cr ) && cuECtx.bestTU->cbf[2] != 0 )  // avoid very complex intra if it is unlikely
         ) ) ) )
    {
      return false;
    }

    if( lastTestMode().type != ETM_INTRA && cuECtx.bestCS && cuECtx.bestCU && interHadActive( cuECtx ) )
    {
      // Get SATD threshold from best Inter-CU
      if( !cs.slice->isIntra() && m_pcEncCfg->getUsePbIntraFast() )
      {
        CodingUnit* bestCU = cuECtx.bestCU;
        if( bestCU && CU::isInter( *bestCU ) )
        {
          DistParam distParam;
          const bool useHad = !bestCU->transQuantBypass;
          m_pcRdCost->setDistParam( distParam, cs.getOrgBuf( COMPONENT_Y ), cuECtx.bestCS->getPredBuf( COMPONENT_Y ), cs.sps->getBitDepth( CHANNEL_TYPE_LUMA ), COMPONENT_Y, useHad );
          cuECtx.interHad = distParam.distFunc( distParam );
        }
      }
    }

    return true;
  }
  else if( encTestmode.type == ETM_IPCM )
  {
    if( getFastDeltaQp() )
    {
      const SPS &sps = *cs.sps;
      const uint32_t fastDeltaQPCuMaxPCMSize = Clip3( ( uint32_t ) 1 << sps.getPCMLog2MinSize(), ( uint32_t ) 1 << sps.getPCMLog2MaxSize(), 32u );

      if( cs.area.lumaSize().width > fastDeltaQPCuMaxPCMSize )
      {
        return false;   // only check necessary PCM in fast deltaqp mode
      }
    }

    // PCM MODES
    return sps.getUsePCM() && width <= ( 1 << sps.getPCMLog2MaxSize() ) && width >= ( 1 << sps.getPCMLog2MinSize() );
  }
  else if( isModeInter( encTestmode ) )
  {
    // INTER MODES (ME + MERGE/SKIP)
    CHECK( slice.isIntra(), "Inter-mode should not be in the I-Slice mode list!" );

    if( getFastDeltaQp() )
    {
      if( encTestmode.type == ETM_MERGE_SKIP )
      {
        return false;
      }
      if( encTestmode.partSize != SIZE_2Nx2N || cs.area.lumaSize().width > cs.pcv->fastDeltaQPCuMaxSize )
      {
        return false; // only check necessary 2Nx2N Inter in fast deltaqp mode
      }
    }

    // --- Check if we can quit current mode using SAVE/LOAD coding history

    if( encTestmode.type == ETM_INTER_ME )
    {
      if( encTestmode.opts == ETO_STANDARD )
      {
        // NOTE: ETO_STANDARD is always done when early SKIP mode detection is enabled
        if( !m_pcEncCfg->getUseEarlySkipDetection() )
        {
#if JVET_K0220_ENC_CTRL
          if( relatedCU.isSkip || relatedCU.isIntra )
#else
          if( ( saveLoadTag == LOAD_ENC_INFO && ( sls.mergeFlag ) ) || relatedCU.isSkip || relatedCU.isIntra )
#endif
          {
            return false;
          }
        }
      }
#if !JVET_K0220_ENC_CTRL
      else if( ( saveLoadTag == LOAD_ENC_INFO ) && sls.mergeFlag )
      {
        return false;
      }
#endif
#if JVET_K0357_AMVR
      else if ((encTestmode.opts & ETO_IMV) != 0)
      {
        int imvOpt = (encTestmode.opts & ETO_IMV) >> ETO_IMV_SHIFT;

        if (imvOpt == 3 && cuECtx.get<double>(BEST_NO_IMV_COST) * 1.06 < cuECtx.get<double>(BEST_IMV_COST))
        {
          return false;
        }
      }
#endif
    }

#if JVET_K_AFFINE
#if JVET_K0220_ENC_CTRL
    if ( encTestmode.type == ETM_AFFINE && relatedCU.isIntra )
#else
    if ( encTestmode.type == ETM_AFFINE && ((saveLoadTag == LOAD_ENC_INFO && !sls.affineFlag) || relatedCU.isIntra) )
#endif
    {
      return false;
    }
#endif
    return true;
  }
  else if( isModeSplit( encTestmode ) )
  {
#if !JVET_K0220_ENC_CTRL
    if( cuECtx.get<bool>( HISTORY_NEED_TO_SAVE ) )
    {
      SaveLoadStruct &slsNext = getSaveLoadStructQuad( partitioner.currArea() );

      slsNext.tag     = SAVE_ENC_INFO;
      slsNext.partIdx = cuECtx.partIdx;

      // don't save saveLoadTag at this level anymore
      cuECtx.set( HISTORY_NEED_TO_SAVE, false );

      DTRACE( g_trace_ctx, D_SAVE_LOAD, "SaveLoad %d (%d %d) [%d %d] \n", SAVE_ENC_INFO, ( cuECtx.partIdx >> 8 ), ( cuECtx.partIdx & 0xff ), getHalvedIdx( cuECtx.cuW ), getHalvedIdx( cuECtx.cuH ) );

#if ENABLE_TRACING
      const Area& area = partitioner.currArea().Y();
      DTRACE( g_trace_ctx, D_SAVE_LOAD, "saving tag at %d,%d (%dx%d) -> %d\n", area.x, area.y, area.width >> 1, area.height >> 1, SAVE_ENC_INFO );
#endif
    }

#endif
#if !HM_NO_ADDITIONAL_SPEEDUPS || JVET_K0220_ENC_CTRL
    //////////////////////////////////////////////////////////////////////////
    // skip-history rule - don't split further if at least for three past levels
    //                     in the split tree it was found that skip is the best mode
    //////////////////////////////////////////////////////////////////////////
    int skipScore = 0;

    if( !slice.isIntra() && cuECtx.get<bool>( IS_BEST_NOSPLIT_SKIP ) )
    {
      for( int i = 2; i < m_ComprCUCtxList.size(); i++ )
      {
        if( ( m_ComprCUCtxList.end() - i )->get<bool>( IS_BEST_NOSPLIT_SKIP ) )
        {
          skipScore += 1;
        }
        else
        {
          break;
        }
      }
    }

    const PartSplit split = getPartSplit( encTestmode );
    if( !partitioner.canSplit( split, cs ) || skipScore >= 2 )
#else
    const PartSplit split = getPartSplit( encTestmode );
    if( !partitioner.canSplit( split, cs ) )
#endif
    {
      if( split == CU_HORZ_SPLIT ) cuECtx.set( DID_HORZ_SPLIT, false );
      if( split == CU_VERT_SPLIT ) cuECtx.set( DID_VERT_SPLIT, false );
#if !HM_NO_ADDITIONAL_SPEEDUPS || JVET_K0220_ENC_CTRL
      if( split == CU_QUAD_SPLIT ) cuECtx.set( DID_QUAD_SPLIT, false );
#endif

      return false;
    }

    if( m_pcEncCfg->getUseContentBasedFastQtbt() )
    {
      const CompArea& currArea = partitioner.currArea().Y();
      int cuHeight  = currArea.height;
      int cuWidth   = currArea.width;

      const bool condIntraInter = m_pcEncCfg->getIntraPeriod() == 1 ? ( partitioner.currBtDepth == 0 ) : ( cuHeight > 32 && cuWidth > 32 );

      if( cuWidth == cuHeight && condIntraInter && getPartSplit( encTestmode ) != CU_QUAD_SPLIT )
      {
        const CPelBuf bufCurrArea = cs.getOrgBuf( partitioner.currArea().block( COMPONENT_Y ) );

        double horVal = 0;
        double verVal = 0;
        double dupVal = 0;
        double dowVal = 0;

        const double th = m_pcEncCfg->getIntraPeriod() == 1 ? 1.2 : 1.0;

        unsigned j, k;

        for( j = 0; j < cuWidth - 1; j++ )
        {
          for( k = 0; k < cuHeight - 1; k++ )
          {
            horVal += abs( bufCurrArea.at( j + 1, k     ) - bufCurrArea.at( j, k ) );
            verVal += abs( bufCurrArea.at( j    , k + 1 ) - bufCurrArea.at( j, k ) );
            dowVal += abs( bufCurrArea.at( j + 1, k )     - bufCurrArea.at( j, k + 1 ) );
            dupVal += abs( bufCurrArea.at( j + 1, k + 1 ) - bufCurrArea.at( j, k ) );
          }
        }
        if( horVal > th * verVal && sqrt( 2 ) * horVal > th * dowVal && sqrt( 2 ) * horVal > th * dupVal && ( getPartSplit( encTestmode ) == CU_HORZ_SPLIT || getPartSplit( encTestmode ) == CU_TRIH_SPLIT ) )
        {
          return false;
        }
        if( th * dupVal < sqrt( 2 ) * verVal && th * dowVal < sqrt( 2 ) * verVal && th * horVal < verVal && ( getPartSplit( encTestmode ) == CU_VERT_SPLIT || getPartSplit( encTestmode ) == CU_TRIV_SPLIT ) )
        {
          return false;
        }
      }

      if( m_pcEncCfg->getIntraPeriod() == 1 && cuWidth <= 32 && cuHeight <= 32 && bestCS && bestCS->tus.size() == 1 && bestCU && bestCU->depth == partitioner.currDepth && partitioner.currBtDepth > 1 && isLuma( partitioner.chType ) )
      {
        if( !bestCU->rootCbf )
        {
          return false;
        }
      }
    }

    if( bestCU && bestCU->skip && bestCU->mtDepth >= m_skipThreshold && !isModeSplit( cuECtx.lastTestMode ) )
    {
      return false;
    }

#if !HM_NO_ADDITIONAL_SPEEDUPS || JVET_K0220_ENC_CTRL
    int featureToSet = -1;
#endif

    switch( getPartSplit( encTestmode ) )
    {
      case CU_QUAD_SPLIT:
        {
#if ENABLE_SPLIT_PARALLELISM
          if( !cuECtx.isLevelSplitParallel )
#endif
#if !HM_NO_ADDITIONAL_SPEEDUPS || JVET_K0220_ENC_CTRL
          if( !cuECtx.get<bool>( QT_BEFORE_BT ) && bestCU )
#else
          if( bestCU )
#endif
          {
            unsigned maxBTD        = cs.pcv->getMaxBtDepth( slice, partitioner.chType );
            const CodingUnit *cuBR = bestCS->cus.back();

            if( bestCU && ( ( bestCU->btDepth == 0 &&                               maxBTD >= ( slice.isIntra() ? 3 : 2 ) )
                         || ( bestCU->btDepth == 1 && cuBR && cuBR->btDepth == 1 && maxBTD >= ( slice.isIntra() ? 4 : 3 ) ) )
                       && cuECtx.get<bool>( DID_HORZ_SPLIT ) && cuECtx.get<bool>( DID_VERT_SPLIT ) )
            {
              return false;
            }
          }
          if( m_pcEncCfg->getUseEarlyCU() && bestCS->cost != MAX_DOUBLE && bestCU && bestCU->skip )
          {
            return false;
          }
          if( getFastDeltaQp() && width <= slice.getPPS()->pcv->fastDeltaQPCuMaxSize )
          {
            return false;
          }
        }
        break;
      case CU_HORZ_SPLIT:
#if !JVET_K0220_ENC_CTRL
        if( saveLoadSplit & 0x02 )
        {
          cuECtx.set( DID_HORZ_SPLIT, false );
          return false;
        }

#endif
#if !HM_NO_ADDITIONAL_SPEEDUPS || JVET_K0220_ENC_CTRL
        featureToSet = DID_HORZ_SPLIT;
#endif
        break;
      case CU_VERT_SPLIT:
#if !JVET_K0220_ENC_CTRL
        if( cuECtx.get<bool>( DID_HORZ_SPLIT ) && bestCU && bestCU->skip && bestCU->btDepth == partitioner.currBtDepth && partitioner.currBtDepth >= SKIPHORNOVERQT_DEPTH_TH )
        {
          cuECtx.set( DID_VERT_SPLIT, false );
          return false;
        }

        if( saveLoadSplit & 0x04 )
        {
          cuECtx.set( DID_VERT_SPLIT, false );
          return false;
        }

#endif
#if !HM_NO_ADDITIONAL_SPEEDUPS || JVET_K0220_ENC_CTRL
        featureToSet = DID_VERT_SPLIT;
#endif
        break;
      case CU_TRIH_SPLIT:
        if( cuECtx.get<bool>( DID_HORZ_SPLIT ) && bestCU && bestCU->btDepth == partitioner.currBtDepth && !bestCU->rootCbf )
        {
          return false;
        }

#if !JVET_K0220_ENC_CTRL
        if( saveLoadSplit & 0x08 )
        {
          return false;
        }

#endif
        if( !cuECtx.get<bool>( DO_TRIH_SPLIT ) )
        {
          return false;
        }
        break;
      case CU_TRIV_SPLIT:
#if !JVET_K0220_ENC_CTRL
        if( cuECtx.get<bool>( DID_HORZ_SPLIT ) && bestCU && bestCU->skip && bestCU->btDepth == partitioner.currBtDepth && partitioner.currBtDepth >= ( SKIPHORNOVERQT_DEPTH_TH - 1 ) )
        {
          return false;
        }

#endif
        if( cuECtx.get<bool>( DID_VERT_SPLIT ) && bestCU && bestCU->btDepth == partitioner.currBtDepth && !bestCU->rootCbf )
        {
          return false;
        }

#if !JVET_K0220_ENC_CTRL
        if( saveLoadSplit & 0x10 )
        {
          return false;
        }

#endif
        if( !cuECtx.get<bool>( DO_TRIV_SPLIT ) )
        {
          return false;
        }
        break;
      default:
        THROW( "Only CU split modes are governed by the EncModeCtrl" );
        return false;
        break;
    }

#if !HM_NO_ADDITIONAL_SPEEDUPS || JVET_K0220_ENC_CTRL
    switch( split )
    {
      case CU_HORZ_SPLIT:
      case CU_TRIH_SPLIT:
        if( cuECtx.get<bool>( QT_BEFORE_BT ) && cuECtx.get<bool>( DID_QUAD_SPLIT ) )
        {
          if( cuECtx.get<int>( MAX_QT_SUB_DEPTH ) > partitioner.currQtDepth + 1 )
          {
            if( featureToSet >= 0 ) cuECtx.set( featureToSet, false );
            return false;
          }
        }
        break;
      case CU_VERT_SPLIT:
      case CU_TRIV_SPLIT:
        if( cuECtx.get<bool>( QT_BEFORE_BT ) && cuECtx.get<bool>( DID_QUAD_SPLIT ) )
        {
          if( cuECtx.get<int>( MAX_QT_SUB_DEPTH ) > partitioner.currQtDepth + 1 )
          {
            if( featureToSet >= 0 ) cuECtx.set( featureToSet, false );
            return false;
          }
        }
        break;
      default:
        break;
    }
#endif

#if !HM_NO_ADDITIONAL_SPEEDUPS || JVET_K0220_ENC_CTRL
    if( split == CU_QUAD_SPLIT ) cuECtx.set( DID_QUAD_SPLIT, true );
#endif
    return true;
  }
  else
  {
    CHECK( encTestmode.type != ETM_POST_DONT_SPLIT, "Unknown mode" );

    if( !bestCS || ( bestCS && isModeSplit( bestMode ) ) )
    {
      return false;
    }
    else
    {
#if REUSE_CU_RESULTS
      setFromCs( *bestCS, partitioner );

#endif
      // assume the non-split modes are done and set the marks for the best found mode
      if( bestCS && bestCU )
      {
        if( CU::isInter( *bestCU ) )
        {
          relatedCU.isInter   = true;
#if HM_CODED_CU_INFO
          relatedCU.isSkip   |= bestCU->skip;
#else
          relatedCU.isSkip    = bestCU->skip;
#endif
        }
        else if( CU::isIntra( *bestCU ) )
        {
          relatedCU.isIntra   = true;
        }
#if ENABLE_SPLIT_PARALLELISM
        touch( partitioner.currArea() );
#endif
#if !HM_NO_ADDITIONAL_SPEEDUPS || JVET_K0220_ENC_CTRL
        cuECtx.set( IS_BEST_NOSPLIT_SKIP, bestCU->skip );
#endif
      }
#if !JVET_K0220_ENC_CTRL

      if( cuECtx.get<bool>( HISTORY_DO_SAVE ) && bestCS->cost != MAX_DOUBLE )
      {
        if( bestCU->predMode != MODE_INTRA )
        {
          sls.mergeFlag  = bestCU->firstPU->mergeFlag;
          sls.interDir   = bestCU->firstPU->interDir;
#if JVET_K0357_AMVR
          sls.imv        = bestCU->imv;
#endif
#if JVET_K_AFFINE
          sls.affineFlag = bestCU->affine;
#endif
        }
        else
        {
        }

#if JVET_K1000_SIMPLIFIED_EMT
        sls.emtCuFlag  = bestCU->emtFlag;
        sls.emtTuIndex = bestCU->firstTU->emtIdx; //since this is the QTBT path, there is only one TU
#endif

        sls.tag = LOAD_ENC_INFO;
        CHECK( sls.partIdx != cuECtx.partIdx, "partidx is not consistent" );

#if ENABLE_TRACING
        const Area& area = partitioner.currArea().Y();
        DTRACE( g_trace_ctx, D_SAVE_LOAD, "saving tag at %d,%d (%dx%d) -> %d\n", area.x, area.y, area.width, area.height, LOAD_ENC_INFO );
#endif
      }
#endif
    }

    return false;
  }
}

bool EncModeCtrlMTnoRQT::useModeResult( const EncTestMode& encTestmode, CodingStructure*& tempCS, Partitioner& partitioner )
{
  xExtractFeatures( encTestmode, *tempCS );

  ComprCUCtx& cuECtx = m_ComprCUCtxList.back();


  if(      encTestmode.type == ETM_SPLIT_BT_H )
  {
    cuECtx.set( BEST_HORZ_SPLIT_COST, tempCS->cost );
  }
  else if( encTestmode.type == ETM_SPLIT_BT_V )
  {
    cuECtx.set( BEST_VERT_SPLIT_COST, tempCS->cost );
  }
  else if( encTestmode.type == ETM_SPLIT_TT_H )
  {
    cuECtx.set( BEST_TRIH_SPLIT_COST, tempCS->cost );
  }
  else if( encTestmode.type == ETM_SPLIT_TT_V )
  {
    cuECtx.set( BEST_TRIV_SPLIT_COST, tempCS->cost );
  }
#if JVET_K1000_SIMPLIFIED_EMT
  else if( encTestmode.type == ETM_INTRA && encTestmode.partSize == SIZE_2Nx2N )
  {
    const CodingUnit cu = *tempCS->getCU( partitioner.chType );

    if( !cu.emtFlag )
    {
      cuECtx.bestEmtSize2Nx2N1stPass = tempCS->cost;
    }
  }
#endif

#if JVET_K0357_AMVR
  if( m_pcEncCfg->getIMV4PelFast() && m_pcEncCfg->getIMV() && encTestmode.type == ETM_INTER_ME )
  {
    int imvMode = ( encTestmode.opts & ETO_IMV ) >> ETO_IMV_SHIFT;

    if( imvMode == 1 )
    {
      if( tempCS->cost < cuECtx.get<double>( BEST_IMV_COST ) )
      {
        cuECtx.set( BEST_IMV_COST, tempCS->cost );
      }
    }
    else if( imvMode == 0 )
    {
      if( tempCS->cost < cuECtx.get<double>( BEST_NO_IMV_COST ) )
      {
        cuECtx.set( BEST_NO_IMV_COST, tempCS->cost );
      }
    }
  }
#endif

#if !HM_NO_ADDITIONAL_SPEEDUPS || JVET_K0220_ENC_CTRL
  if( encTestmode.type == ETM_SPLIT_QT )
  {
    int maxQtD = 0;
    for( const auto& cu : tempCS->cus )
    {
      maxQtD = std::max<int>( maxQtD, cu->qtDepth );
    }
    cuECtx.set( MAX_QT_SUB_DEPTH, maxQtD );
  }
#endif

  if( ( tempCS->sps->getSpsNext().getMTTMode() & 1 ) == 1 )
  {
#if !HM_QTBT_ONLY_QT_IMPLICIT || JVET_K0554
    int maxMtD = tempCS->pcv->getMaxBtDepth( *tempCS->slice, partitioner.chType ) + partitioner.currImplicitBtDepth;
#else
    int maxMtD = tempCS->pcv->getMaxBtDepth( *tempCS->slice, partitioner.chType );
#endif

    if( encTestmode.type == ETM_SPLIT_BT_H )
    {
      if( tempCS->cus.size() > 2 )
      {
        int h_2   = tempCS->area.blocks[partitioner.chType].height / 2;
        int cu1_h = tempCS->cus.front()->blocks[partitioner.chType].height;
        int cu2_h = tempCS->cus.back() ->blocks[partitioner.chType].height;

        cuECtx.set( DO_TRIH_SPLIT, cu1_h < h_2 || cu2_h < h_2 || partitioner.currMtDepth + 1 == maxMtD );
      }
    }
    else if( encTestmode.type == ETM_SPLIT_BT_V )
    {
      if( tempCS->cus.size() > 2 )
      {
        int w_2   = tempCS->area.blocks[partitioner.chType].width / 2;
        int cu1_w = tempCS->cus.front()->blocks[partitioner.chType].width;
        int cu2_w = tempCS->cus.back() ->blocks[partitioner.chType].width;

        cuECtx.set( DO_TRIV_SPLIT, cu1_w < w_2 || cu2_w < w_2 || partitioner.currMtDepth + 1 == maxMtD );
      }
    }
  }

  // for now just a simple decision based on RD-cost or choose tempCS if bestCS is not yet coded
  if( !cuECtx.bestCS || tempCS->features[ENC_FT_RD_COST] < cuECtx.bestCS->features[ENC_FT_RD_COST] )
  {
    cuECtx.bestCS = tempCS;
    cuECtx.bestCU = tempCS->cus[0];
    cuECtx.bestTU = cuECtx.bestCU->firstTU;

    if( isModeInter( encTestmode ) )
    {
      //Here we take the best cost of both inter modes. We are assuming only the inter modes (and all of them) have come before the intra modes!!!
      cuECtx.bestInterCost = cuECtx.bestCS->cost;
    }

    return true;
  }
  else
  {
    return false;
  }
}

#if ENABLE_SPLIT_PARALLELISM
void EncModeCtrlMTnoRQT::copyState( const EncModeCtrl& other, const UnitArea& area )
{
  const EncModeCtrlMTnoRQT* pOther = dynamic_cast<const EncModeCtrlMTnoRQT*>( &other );

  CHECK( !pOther, "Trying to copy state from a different type of controller" );

  this->EncModeCtrl        ::copyState( *pOther, area );
#if !JVET_K0220_ENC_CTRL
  this->SaveLoadEncInfoCtrl::copyState( *pOther, area );
#endif
  this->CacheBlkInfoCtrl   ::copyState( *pOther, area );

  m_skipThreshold = pOther->m_skipThreshold;
}

int EncModeCtrlMTnoRQT::getNumParallelJobs( const CodingStructure &cs, Partitioner& partitioner ) const
{
  int numJobs = 1; // for no-split coding

  if( partitioner.canSplit( CU_QUAD_SPLIT, cs ) )
  {
    numJobs = 2;
  }

  if( partitioner.canSplit( CU_VERT_SPLIT, cs ) )
  {
    numJobs = 3;
  }

  if( partitioner.canSplit( CU_HORZ_SPLIT, cs ) )
  {
    numJobs = 4;
  }

  if( partitioner.canSplit( CU_TRIV_SPLIT, cs ) )
  {
    numJobs = 5;
  }

  if( partitioner.canSplit( CU_TRIH_SPLIT, cs ) )
  {
    numJobs = 6;
  }

  CHECK( numJobs >= NUM_RESERVERD_SPLIT_JOBS, "More jobs specified than allowed" );

  return numJobs;
}

bool EncModeCtrlMTnoRQT::isParallelSplit( const CodingStructure &cs, Partitioner& partitioner ) const
{
  if( partitioner.getImplicitSplit( cs ) != CU_DONT_SPLIT || cs.picture->scheduler.getSplitJobId() != 0 ) return false;
  const int numJobs = getNumParallelJobs( cs, partitioner );
  const int numPxl  = partitioner.currArea().Y().area();
  const int parlAt  = m_pcEncCfg->getNumSplitThreads() <= 3 ? 1024 : 256;
  if(  cs.slice->isIntra() && numJobs > 2 && ( numPxl == parlAt || !partitioner.canSplit( CU_QUAD_SPLIT, cs ) ) ) return true;
  if( !cs.slice->isIntra() && numJobs > 1 && ( numPxl == parlAt || !partitioner.canSplit( CU_QUAD_SPLIT, cs ) ) ) return true;
  return false;
}

bool EncModeCtrlMTnoRQT::parallelJobSelector( const EncTestMode& encTestmode, const CodingStructure &cs, Partitioner& partitioner ) const
{
  // Job descriptors
  //  - 1: all non-split modes
  //  - 2: QT-split
  //  - 3: all vertical modes but TT_V
  //  - 4: all horizontal modes but TT_H
  //  - 5: TT_V
  //  - 6: TT_H
  switch( cs.picture->scheduler.getSplitJobId() )
  {
  case 1:
    // be sure to execute post dont split
    return !isModeSplit( encTestmode );
    break;
  case 2:
    return encTestmode.type == ETM_SPLIT_QT;
    break;
  case 3:
    switch( encTestmode.type )
    {
    case ETM_SPLIT_BT_V:
      return true;
      break;
    default:
      return false;
      break;
    }
    break;
  case 4:
    switch( encTestmode.type )
    {
    case ETM_SPLIT_BT_H:
      return true;
      break;
    default:
      return false;
      break;
    }
    break;
  case 5:
    return encTestmode.type == ETM_SPLIT_TT_V;
    break;
  case 6:
    return encTestmode.type == ETM_SPLIT_TT_H;
    break;
  default:
    THROW( "Unknown job-ID for parallelization of EncModeCtrlMTnoRQT: " << cs.picture->scheduler.getSplitJobId() );
    break;
  }
}

#endif



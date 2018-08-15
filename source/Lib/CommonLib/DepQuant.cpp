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

#include "DepQuant.h"
#include "TrQuant.h"
#include "CodingStructure.h"
#include "UnitTools.h"

#include <bitset>




#if JVET_K0072


namespace DQIntern
{
  /*================================================================================*/
  /*=====                                                                      =====*/
  /*=====   R A T E   E S T I M A T O R                                        =====*/
  /*=====                                                                      =====*/
  /*================================================================================*/

  struct NbInfoSbb
  {
    uint8_t   num;
    uint8_t   inPos[5];
  };
  struct NbInfoOut
  {
    uint16_t  maxDist;
    uint16_t  num;
    uint16_t  outPos[5];
  };
  struct CoeffFracBits
  {
    int32_t   bits[7];
  };


  class Rom
  {
  public:
    Rom() : m_scansInitialized(false) {}
    ~Rom() { xUninitScanArrays(); }
    void              init        ()                                  { xInitScanArrays(); }
    const NbInfoSbb*  getNbInfoSbb( int sId, int hId, int vId ) const { return m_scanId2NbInfoSbbArray[sId][hId][vId]; }
    const NbInfoOut*  getNbInfoOut( int sId, int hId, int vId ) const { return m_scanId2NbInfoOutArray[sId][hId][vId]; }
  private:
    void  xInitScanArrays   ();
    void  xUninitScanArrays ();
  private:
    bool       m_scansInitialized;
    NbInfoSbb* m_scanId2NbInfoSbbArray[ SCAN_NUMBER_OF_TYPES ][ MAX_CU_SIZE/2+1 ][ MAX_CU_SIZE/2+1 ];
    NbInfoOut* m_scanId2NbInfoOutArray[ SCAN_NUMBER_OF_TYPES ][ MAX_CU_SIZE/2+1 ][ MAX_CU_SIZE/2+1 ];
  };

  void Rom::xInitScanArrays()
  {
    if( m_scansInitialized )
    {
      return;
    }
    ::memset( m_scanId2NbInfoSbbArray, 0, sizeof(m_scanId2NbInfoSbbArray) );
    ::memset( m_scanId2NbInfoOutArray, 0, sizeof(m_scanId2NbInfoOutArray) );

    SizeIndexInfoLog2 sizeInfo;
    sizeInfo.init ( MAX_CU_SIZE );
    uint32_t raster2id[ MAX_CU_SIZE * MAX_CU_SIZE ];

    for( uint32_t blockHeightIdx = 0; blockHeightIdx < sizeInfo.numHeights(); blockHeightIdx++ )
    {
      for( uint32_t blockWidthIdx = 0; blockWidthIdx < sizeInfo.numWidths(); blockWidthIdx++ )
      {
        const uint32_t blockWidth   = sizeInfo.sizeFrom( blockWidthIdx  );
        const uint32_t blockHeight  = sizeInfo.sizeFrom( blockHeightIdx );
        const uint32_t totalValues  = blockWidth * blockHeight;
        const uint32_t log2CGWidth  = (blockWidth & 3) + (blockHeight & 3) > 0 ? 1 : 2;
        const uint32_t log2CGHeight = (blockWidth & 3) + (blockHeight & 3) > 0 ? 1 : 2;
        const uint32_t groupWidth   = 1 << log2CGWidth;
        const uint32_t groupHeight  = 1 << log2CGHeight;
        const uint32_t groupSize    = groupWidth * groupHeight;
        if( ((blockWidth>>log2CGWidth)<<log2CGWidth)!=blockWidth || ((blockHeight>>log2CGHeight)<<log2CGHeight)!=blockHeight )
        {
          continue;
        }
        for( uint32_t scanTypeIdx = 0; scanTypeIdx < SCAN_NUMBER_OF_TYPES; scanTypeIdx++ )
        {
          const CoeffScanType scanType  = CoeffScanType(scanTypeIdx);
          const uint32_t*         scanId2RP = g_scanOrder     [SCAN_GROUPED_4x4][scanType][blockWidthIdx][blockHeightIdx];
          const uint32_t*         scanId2X  = g_scanOrderPosXY[SCAN_GROUPED_4x4][scanType][blockWidthIdx][blockHeightIdx][0];
          const uint32_t*         scanId2Y  = g_scanOrderPosXY[SCAN_GROUPED_4x4][scanType][blockWidthIdx][blockHeightIdx][1];
          NbInfoSbb*&         sId2NbSbb = m_scanId2NbInfoSbbArray           [scanType][blockWidthIdx][blockHeightIdx];
          NbInfoOut*&         sId2NbOut = m_scanId2NbInfoOutArray           [scanType][blockWidthIdx][blockHeightIdx];

          sId2NbSbb = new NbInfoSbb[ totalValues ];
          sId2NbOut = new NbInfoOut[ totalValues ];

          for( uint32_t scanId = 0; scanId < totalValues; scanId++ )
          {
            raster2id[ scanId2RP[ scanId ] ] = scanId;
          }

          for( unsigned scanId = 0; scanId < totalValues; scanId++ )
          {
            const int posX = scanId2X [ scanId ];
            const int posY = scanId2Y [ scanId ];
            const int rpos = scanId2RP[ scanId ];
            {
              //===== inside subband neighbours =====
              NbInfoSbb&     nbSbb  = sId2NbSbb[ scanId ];
              const int      begSbb = scanId - ( scanId & (groupSize-1) ); // first pos in current subblock
              int            cpos[5];
              cpos[0] = ( posX < blockWidth -1                         ? ( raster2id[rpos+1           ] - begSbb < groupSize ? raster2id[rpos+1           ] - begSbb : 0 ) : 0 );
              cpos[1] = ( posX < blockWidth -2                         ? ( raster2id[rpos+2           ] - begSbb < groupSize ? raster2id[rpos+2           ] - begSbb : 0 ) : 0 );
              cpos[2] = ( posX < blockWidth -1 && posY < blockHeight-1 ? ( raster2id[rpos+1+blockWidth] - begSbb < groupSize ? raster2id[rpos+1+blockWidth] - begSbb : 0 ) : 0 );
              cpos[3] = ( posY < blockHeight-1                         ? ( raster2id[rpos+  blockWidth] - begSbb < groupSize ? raster2id[rpos+  blockWidth] - begSbb : 0 ) : 0 );
              cpos[4] = ( posY < blockHeight-2                         ? ( raster2id[rpos+2*blockWidth] - begSbb < groupSize ? raster2id[rpos+2*blockWidth] - begSbb : 0 ) : 0 );
              for( nbSbb.num = 0; true; )
              {
                int nk = -1;
                for( int k = 0; k < 5; k++ )
                {
                  if( cpos[k] != 0 && ( nk < 0 || cpos[k] < cpos[nk] ) )
                  {
                    nk = k;
                  }
                }
                if( nk < 0 )
                {
                  break;
                }
                nbSbb.inPos[ nbSbb.num++ ] = uint8_t( cpos[nk] );
                cpos[nk] = 0;
              }
              for( int k = nbSbb.num; k < 5; k++ )
              {
                nbSbb.inPos[k] = 0;
              }
            }
            {
              //===== outside subband neighbours =====
              NbInfoOut&     nbOut  = sId2NbOut[ scanId ];
              const int      begSbb = scanId - ( scanId & (groupSize-1) ); // first pos in current subblock
              int            cpos[5];
              cpos[0] = ( posX < blockWidth -1                         ? ( raster2id[rpos+1           ] - begSbb >= groupSize ? raster2id[rpos+1           ] : 0 ) : 0 );
              cpos[1] = ( posX < blockWidth -2                         ? ( raster2id[rpos+2           ] - begSbb >= groupSize ? raster2id[rpos+2           ] : 0 ) : 0 );
              cpos[2] = ( posX < blockWidth -1 && posY < blockHeight-1 ? ( raster2id[rpos+1+blockWidth] - begSbb >= groupSize ? raster2id[rpos+1+blockWidth] : 0 ) : 0 );
              cpos[3] = ( posY < blockHeight-1                         ? ( raster2id[rpos+  blockWidth] - begSbb >= groupSize ? raster2id[rpos+  blockWidth] : 0 ) : 0 );
              cpos[4] = ( posY < blockHeight-2                         ? ( raster2id[rpos+2*blockWidth] - begSbb >= groupSize ? raster2id[rpos+2*blockWidth] : 0 ) : 0 );
              for( nbOut.num = 0; true; )
              {
                int nk = -1;
                for( int k = 0; k < 5; k++ )
                {
                  if( cpos[k] != 0 && ( nk < 0 || cpos[k] < cpos[nk] ) )
                  {
                    nk = k;
                  }
                }
                if( nk < 0 )
                {
                  break;
                }
                nbOut.outPos[ nbOut.num++ ] = uint16_t( cpos[nk] );
                cpos[nk] = 0;
              }
              for( int k = nbOut.num; k < 5; k++ )
              {
                nbOut.outPos[k] = 0;
              }
              nbOut.maxDist = ( scanId == 0 ? 0 : sId2NbOut[scanId-1].maxDist );
              for( int k = 0; k < nbOut.num; k++ )
              {
                if( nbOut.outPos[k] > nbOut.maxDist )
                {
                  nbOut.maxDist = nbOut.outPos[k];
                }
              }
            }
          }

          // make it relative
          for( unsigned scanId = 0; scanId < totalValues; scanId++ )
          {
            NbInfoOut& nbOut  = sId2NbOut[scanId];
            const int  begSbb = scanId - ( scanId & (groupSize-1) ); // first pos in current subblock
            for( int k = 0; k < nbOut.num; k++ )
            {
              nbOut.outPos[k] -= begSbb;
            }
            nbOut.maxDist -= scanId;
          }
        }
      }
    }
    m_scansInitialized = true;
  }

  void Rom::xUninitScanArrays()
  {
    if( !m_scansInitialized )
    {
      return;
    }
    for( uint32_t blockHeightIdx = 0; blockHeightIdx <= MAX_CU_SIZE/2; blockHeightIdx++ )
    {
      for( uint32_t blockWidthIdx = 0; blockWidthIdx <= MAX_CU_SIZE/2; blockWidthIdx++ )
      {
        for( uint32_t scanTypeIdx = 0; scanTypeIdx < SCAN_NUMBER_OF_TYPES; scanTypeIdx++ )
        {
          NbInfoSbb*& sId2NbSbb = m_scanId2NbInfoSbbArray[scanTypeIdx][blockWidthIdx][blockHeightIdx];
          NbInfoOut*& sId2NbOut = m_scanId2NbInfoOutArray[scanTypeIdx][blockWidthIdx][blockHeightIdx];
          if( sId2NbSbb )
          {
            delete [] sId2NbSbb;
          }
          if( sId2NbOut )
          {
            delete [] sId2NbOut;
          }
        }
      }
    }
    m_scansInitialized = false;
  }


  static Rom g_Rom;


  class RateEstimator
  {
  public:
    RateEstimator () {}
    ~RateEstimator() {}
    void initBlock( const TransformUnit& tu, const ComponentID     compID );
    void initCtx  ( const TransformUnit& tu, const FracBitsAccess& fracBitsAccess );

    inline bool               luma() const { return m_compID == COMPONENT_Y; }
    inline int32_t            widthInSbb() const { return m_widthInSbb; }
    inline int32_t            heightInSbb() const { return m_heightInSbb; }
    inline int32_t            numCoeff() const { return m_numCoeff; }
    inline int32_t            numSbb() const { return m_numSbb; }
    inline int32_t            sbbSize() const { return m_sbbSize; }
    inline int32_t            sbbPos(unsigned scanIdx) const { return m_scanSbbId2SbbPos[scanIdx >> m_log2SbbSize]; }
    inline int32_t            rasterPos(unsigned scanIdx) const { return m_scanId2BlkPos[scanIdx]; }
    inline int32_t            posX(unsigned scanIdx) const { return m_scanId2PosX[scanIdx]; }
    inline int32_t            posY(unsigned scanIdx) const { return m_scanId2PosY[scanIdx]; }
    inline const NbInfoSbb &  nbInfoSbb(unsigned scanIdx) const { return m_scanId2NbInfoSbb[scanIdx]; }
    inline const NbInfoOut *  nbInfoOut() const { return m_scanId2NbInfoOut; }
    inline const BinFracBits *sigSbbFracBits() const { return m_sigSbbFracBits; }
    inline const BinFracBits *sigFlagBits(unsigned stateId) const
    {
      return m_sigFracBits[std::max(((int) stateId) - 1, 0)];
    }
    inline const CoeffFracBits *gtxFracBits(unsigned stateId) const { return m_gtxFracBits; }
    inline int32_t              lastOffset(unsigned scanIdx) const
    {
      return m_lastBitsX[m_scanId2PosX[scanIdx]] + m_lastBitsY[m_scanId2PosY[scanIdx]];
    }

  private:
    void  xSetLastCoeffOffset ( const FracBitsAccess& fracBitsAccess, const TransformUnit& tu );
    void  xSetSigSbbFracBits  ( const FracBitsAccess& fracBitsAccess );
    void  xSetSigFlagBits     ( const FracBitsAccess& fracBitsAccess );
    void  xSetGtxFlagBits     ( const FracBitsAccess& fracBitsAccess );

  private:
    static const unsigned sm_numCtxSetsSig    = 3;
    static const unsigned sm_numCtxSetsGtx    = 2;
    static const unsigned sm_maxNumSigSbbCtx  = 2;
    static const unsigned sm_maxNumSigCtx     = 18;
    static const unsigned sm_maxNumGtxCtx     = 21;

  private:
    ComponentID       m_compID;
    ChannelType       m_chType;
    unsigned          m_width;
    unsigned          m_height;
    unsigned          m_numCoeff;
    unsigned          m_numSbb;
    unsigned          m_log2SbbWidth;
    unsigned          m_log2SbbHeight;
    unsigned          m_log2SbbSize;
    unsigned          m_sbbSize;
    unsigned          m_sbbMask;
    unsigned          m_widthInSbb;
    unsigned          m_heightInSbb;
    CoeffScanType     m_scanType;
    const unsigned*   m_scanSbbId2SbbPos;
    const unsigned*   m_scanId2BlkPos;
    const unsigned*   m_scanId2PosX;
    const unsigned*   m_scanId2PosY;
    const NbInfoSbb*  m_scanId2NbInfoSbb;
    const NbInfoOut*  m_scanId2NbInfoOut;
    int32_t           m_lastBitsX      [ MAX_TU_SIZE ];
    int32_t           m_lastBitsY      [ MAX_TU_SIZE ];
    BinFracBits       m_sigSbbFracBits [ sm_maxNumSigSbbCtx ];
    BinFracBits       m_sigFracBits    [ sm_numCtxSetsSig   ][ sm_maxNumSigCtx ];
    CoeffFracBits     m_gtxFracBits                          [ sm_maxNumGtxCtx ];
  };

  void RateEstimator::initBlock( const TransformUnit& tu, const ComponentID compID )
  {
    CHECKD( tu.cs->sps->getSpsRangeExtension().getExtendedPrecisionProcessingFlag(), "ext precision is not supported" );

    const CompArea& area  = tu.blocks[ compID ];
    m_compID              = compID;
    m_chType              = toChannelType( m_compID );
    m_width               = area.width;
    m_height              = area.height;
    m_numCoeff            = m_width * m_height;
    const bool      no4x4 = ( ( m_width & 3 ) != 0 || ( m_height & 3 ) != 0 );
    m_log2SbbWidth        = ( no4x4 ? 1 : 2 );
    m_log2SbbHeight       = ( no4x4 ? 1 : 2 );
    m_log2SbbSize         = m_log2SbbWidth + m_log2SbbHeight;
    m_sbbSize             = ( 1 << m_log2SbbSize );
    m_sbbMask             = m_sbbSize - 1;
    m_widthInSbb          = m_width  >> m_log2SbbWidth;
    m_heightInSbb         = m_height >> m_log2SbbHeight;
    m_numSbb              = m_widthInSbb * m_heightInSbb;
#if HEVC_USE_MDCS
    m_scanType            = CoeffScanType( TU::getCoefScanIdx( tu, m_compID ) );
#else
    m_scanType            = SCAN_DIAG;
#endif
    SizeType        hsbb  = gp_sizeIdxInfo->idxFrom( m_widthInSbb  );
    SizeType        vsbb  = gp_sizeIdxInfo->idxFrom( m_heightInSbb );
    SizeType        hsId  = gp_sizeIdxInfo->idxFrom( m_width  );
    SizeType        vsId  = gp_sizeIdxInfo->idxFrom( m_height );
    m_scanSbbId2SbbPos    = g_scanOrder     [ SCAN_UNGROUPED   ][ m_scanType ][ hsbb ][ vsbb ];
    m_scanId2BlkPos       = g_scanOrder     [ SCAN_GROUPED_4x4 ][ m_scanType ][ hsId ][ vsId ];
    m_scanId2PosX         = g_scanOrderPosXY[ SCAN_GROUPED_4x4 ][ m_scanType ][ hsId ][ vsId ][ 0 ];
    m_scanId2PosY         = g_scanOrderPosXY[ SCAN_GROUPED_4x4 ][ m_scanType ][ hsId ][ vsId ][ 1 ];
    m_scanId2NbInfoSbb    = g_Rom.getNbInfoSbb( m_scanType, hsId, vsId );
    m_scanId2NbInfoOut    = g_Rom.getNbInfoOut( m_scanType, hsId, vsId );
  }

  void RateEstimator::initCtx( const TransformUnit& tu, const FracBitsAccess& fracBitsAccess )
  {
    xSetSigSbbFracBits  ( fracBitsAccess );
    xSetSigFlagBits     ( fracBitsAccess );
    xSetGtxFlagBits     ( fracBitsAccess );
    xSetLastCoeffOffset ( fracBitsAccess, tu );
  }

  void RateEstimator::xSetLastCoeffOffset( const FracBitsAccess& fracBitsAccess, const TransformUnit& tu )
  {
    int32_t cbfDeltaBits = 0;
    if( m_compID == COMPONENT_Y && !CU::isIntra(*tu.cu) && !tu.depth )
    {
      const BinFracBits bits  = fracBitsAccess.getFracBitsArray( Ctx::QtRootCbf() );
      cbfDeltaBits            = int32_t( bits.intBits[1] ) - int32_t( bits.intBits[0] );
    }
    else
    {
#if ENABLE_BMS
      BinFracBits bits = fracBitsAccess.getFracBitsArray( Ctx::QtCbf[m_compID]( DeriveCtx::CtxQtCbf( m_compID, tu.depth, tu.cbf[COMPONENT_Cb] ) ) );
#else
      BinFracBits bits = fracBitsAccess.getFracBitsArray( Ctx::QtCbf[m_compID]( DeriveCtx::CtxQtCbf( m_compID, tu.cbf[COMPONENT_Cb] ) ) );
#endif
      cbfDeltaBits = int32_t( bits.intBits[1] ) - int32_t( bits.intBits[0] );
    }

    static const unsigned prefixCtx[] = { 0, 0, 0, 3, 6, 10, 15, 21 };
    uint32_t              ctxBits  [ LAST_SIGNIFICANT_GROUPS ];
    for( unsigned xy = 0; xy < 2; xy++ )
    {
      int32_t             bitOffset   = ( xy ? cbfDeltaBits : 0 );
      int32_t*            lastBits    = ( xy ? m_lastBitsY : m_lastBitsX );
      const unsigned      size        = ( xy ? m_height : m_width );
      const unsigned      log2Size    = g_aucNextLog2[ size ];
#if HEVC_USE_MDCS
      const bool          useYCtx     = ( m_scanType == SCAN_VER ? ( xy == 0 ) : ( xy != 0 ) );
#else
      const bool          useYCtx     = ( xy != 0 );
#endif
      const CtxSet&       ctxSetLast  = ( useYCtx ? Ctx::LastY : Ctx::LastX )[ m_chType ];
      const unsigned      lastShift   = ( m_compID == COMPONENT_Y ? (log2Size+1)>>2 : ( tu.cs->pcv->rectCUs ? Clip3<unsigned>(0,2,size>>3) : log2Size-2 ) );
      const unsigned      lastOffset  = ( m_compID == COMPONENT_Y ? ( tu.cs->pcv->rectCUs ? prefixCtx[log2Size] : 3*(log2Size-2)+((log2Size-1)>>2) ) : 0 );
      uint32_t            sumFBits    = 0;
      unsigned            maxCtxId    = g_uiGroupIdx[ size - 1 ];
      for( unsigned ctxId = 0; ctxId < maxCtxId; ctxId++ )
      {
        const BinFracBits bits  = fracBitsAccess.getFracBitsArray( ctxSetLast( lastOffset + ( ctxId >> lastShift ) ) );
        ctxBits[ ctxId ]        = sumFBits + bits.intBits[0] + ( ctxId>3 ? ((ctxId-2)>>1)<<SCALE_BITS : 0 ) + bitOffset;
        sumFBits               +=            bits.intBits[1];
      }
      ctxBits  [ maxCtxId ]     = sumFBits + ( maxCtxId>3 ? ((maxCtxId-2)>>1)<<SCALE_BITS : 0 ) + bitOffset;
      for( unsigned pos = 0; pos < size; pos++ )
      {
        lastBits[ pos ]         = ctxBits[ g_uiGroupIdx[ pos ] ];
      }
    }
  }

  void RateEstimator::xSetSigSbbFracBits( const FracBitsAccess& fracBitsAccess )
  {
    const CtxSet& ctxSet = Ctx::SigCoeffGroup[ m_chType ];
    for( unsigned ctxId = 0; ctxId < sm_maxNumSigSbbCtx; ctxId++ )
    {
      m_sigSbbFracBits[ ctxId ] = fracBitsAccess.getFracBitsArray( ctxSet( ctxId ) );
    }
  }

  void RateEstimator::xSetSigFlagBits( const FracBitsAccess& fracBitsAccess )
  {
    for( unsigned ctxSetId = 0; ctxSetId < sm_numCtxSetsSig; ctxSetId++ )
    {
      BinFracBits*    bits    = m_sigFracBits [ ctxSetId ];
      const CtxSet&   ctxSet  = Ctx::SigFlag  [ m_chType + 2*ctxSetId ];
      const unsigned  numCtx  = ( m_compID == COMPONENT_Y ? 18 : 12 );
      for( unsigned ctxId = 0; ctxId < numCtx; ctxId++ )
      {
        bits[ ctxId ] = fracBitsAccess.getFracBitsArray( ctxSet( ctxId ) );
      }
    }
  }

  void RateEstimator::xSetGtxFlagBits( const FracBitsAccess& fracBitsAccess )
  {
    const CtxSet&   ctxSetPar   = Ctx::ParFlag [     m_chType ];
    const CtxSet&   ctxSetGt1   = Ctx::GtxFlag [ 2 + m_chType ];
    const CtxSet&   ctxSetGt2   = Ctx::GtxFlag [     m_chType ];
    const unsigned  numCtx      = ( m_compID == COMPONENT_Y ? 21 : 11 );
    for( unsigned ctxId = 0; ctxId < numCtx; ctxId++ )
    {
      BinFracBits     fbPar = fracBitsAccess.getFracBitsArray( ctxSetPar( ctxId ) );
      BinFracBits     fbGt1 = fracBitsAccess.getFracBitsArray( ctxSetGt1( ctxId ) );
      BinFracBits     fbGt2 = fracBitsAccess.getFracBitsArray( ctxSetGt2( ctxId ) );
      CoeffFracBits&  cb    = m_gtxFracBits[ ctxId ];
      int32_t         par0  = (1<<SCALE_BITS) + int32_t(fbPar.intBits[0]);
      int32_t         par1  = (1<<SCALE_BITS) + int32_t(fbPar.intBits[1]);
      cb.bits[0]  = 0;
      cb.bits[1]  = par0 + fbGt1.intBits[0];
      cb.bits[2]  = par1 + fbGt1.intBits[0];
      cb.bits[3]  = par0 + fbGt1.intBits[1] + fbGt2.intBits[0];
      cb.bits[4]  = par1 + fbGt1.intBits[1] + fbGt2.intBits[0];
      cb.bits[5]  = par0 + fbGt1.intBits[1] + fbGt2.intBits[1];
      cb.bits[6]  = par1 + fbGt1.intBits[1] + fbGt2.intBits[1];
    }
  }





  /*================================================================================*/
  /*=====                                                                      =====*/
  /*=====   D A T A   S T R U C T U R E S                                      =====*/
  /*=====                                                                      =====*/
  /*================================================================================*/

  enum ScanPosType { SCAN_ISCSBB = 0, SCAN_SOCSBB = 1, SCAN_EOCSBB = 2 };

  struct ScanInfo
  {
    const int     sbbSize;
    const int     numSbb;
    int           scanIdx;
    int           rasterPos;
    int           lastOffset;
    unsigned      sigCtxOffsetNext;
    unsigned      gtxCtxOffsetNext;
    int           insidePos;
    int           nextInsidePos;
    NbInfoSbb     nextNbInfoSbb;
    bool          sosbb;
    bool          eosbb;
    bool          socsbb;
    bool          eocsbb;
    int           sbbPos;
    int           nextSbbRight;
    int           nextSbbBelow;
  protected:
    ScanInfo( int _sbbSize, int _numSbb ) : sbbSize( _sbbSize ), numSbb( _numSbb ) {}
  };

  class ScanData : public ScanInfo
  {
  public:
    ScanData( const RateEstimator& rateEst, int firstPos )
      : ScanInfo          ( rateEst.sbbSize(), rateEst.numSbb() )
      , m_rateEst         ( rateEst )
      , m_luma            ( m_rateEst.luma() )
      , m_sbbMask         ( sbbSize - 1 )
      , m_widthInSbb      ( m_rateEst.widthInSbb() )
      , m_heightInSbb     ( m_rateEst.heightInSbb() )
      , m_numCoeffMinus1  ( m_rateEst.numCoeff() - 1 )
      , m_numCoeffMinusSbb( m_rateEst.numCoeff() - sbbSize )
    {
      xSet( firstPos );
    }
    inline bool    valid() const { return scanIdx >= 0; }
    void           next  ()               { xSet( scanIdx-1 ); }
    void           set   ( int id )       { xSet(id); }

  private:
    inline void xSet(int _scanIdx)
    {
      scanIdx = _scanIdx;
      if( scanIdx >= 0 )
      {
        rasterPos               = m_rateEst.rasterPos   ( scanIdx );
        sbbPos                  = m_rateEst.sbbPos      ( scanIdx );
        lastOffset              = m_rateEst.lastOffset  ( scanIdx );
        insidePos               = scanIdx & m_sbbMask;
        sosbb                   = ( insidePos == m_sbbMask );
        eosbb                   = ( insidePos == 0 );
        socsbb                  = ( sosbb && scanIdx > sbbSize && scanIdx < m_numCoeffMinus1   );
        eocsbb                  = ( eosbb && scanIdx > 0       && scanIdx < m_numCoeffMinusSbb );
        if( scanIdx )
        {
          const int nextScanIdx = scanIdx - 1;
          const int diag        = m_rateEst.posX( nextScanIdx ) + m_rateEst.posY( nextScanIdx );
          if( m_luma )
          {
            sigCtxOffsetNext    = ( diag < 2 ? 12 : diag < 5 ?  6 : 0 );
            gtxCtxOffsetNext    = ( diag < 1 ? 16 : diag < 3 ? 11 : diag < 10 ? 6 : 1 );
          }
          else
          {
            sigCtxOffsetNext    = ( diag < 2 ? 6 : 0 );
            gtxCtxOffsetNext    = ( diag < 1 ? 6 : 1 );
          }
          nextInsidePos         = nextScanIdx & m_sbbMask;
          nextNbInfoSbb         = m_rateEst.nbInfoSbb( nextScanIdx );
          if( eosbb )
          {
            const int nextSbbPos  = m_rateEst.sbbPos( nextScanIdx );
            const int nextSbbPosY = nextSbbPos               / m_widthInSbb;
            const int nextSbbPosX = nextSbbPos - nextSbbPosY * m_widthInSbb;
            nextSbbRight          = ( nextSbbPosX < m_widthInSbb  - 1 ? nextSbbPos + 1            : 0 );
            nextSbbBelow          = ( nextSbbPosY < m_heightInSbb - 1 ? nextSbbPos + m_widthInSbb : 0 );
          }
        }
      }
    }
  private:
    const RateEstimator& m_rateEst;
    const bool           m_luma;
    const int            m_sbbMask;
    const int            m_widthInSbb;
    const int            m_heightInSbb;
    const int            m_numCoeffMinus1;
    const int            m_numCoeffMinusSbb;
  };


  struct PQData
  {
    TCoeff  absLevel;
    int64_t deltaDist;
  };


  struct Decision
  {
    int64_t rdCost;
    TCoeff  absLevel;
    int     prevId;
  };




  /*================================================================================*/
  /*=====                                                                      =====*/
  /*=====   P R E - Q U A N T I Z E R                                          =====*/
  /*=====                                                                      =====*/
  /*================================================================================*/

  class Quantizer
  {
  public:
    Quantizer() {}

    void  dequantBlock  ( const TransformUnit& tu, const ComponentID compID, const QpParam& cQP, CoeffBuf& recCoeff   ) const;
    void  initQuantBlock( const TransformUnit& tu, const ComponentID compID, const QpParam& cQP, const double lambda  );

    inline void   preQuantCoeff(const TCoeff absCoeff, PQData *pqData) const;
    inline TCoeff getLastThreshold() const { return m_thresLast; }
    inline TCoeff getSSbbThreshold() const { return m_thresSSbb; }

  private:
    // quantization
    int               m_QShift;
    int64_t           m_QAdd;
    int64_t           m_QScale;
    TCoeff            m_maxQIdx;
    TCoeff            m_thresLast;
    TCoeff            m_thresSSbb;
    // distortion normalization
    int               m_DistShift;
    int64_t           m_DistAdd;
    int64_t           m_DistStepAdd;
    int64_t           m_DistOrgFact;
  };

  inline int ceil_log2(uint64_t x)
  {
    static const uint64_t t[6] = { 0xFFFFFFFF00000000ull, 0x00000000FFFF0000ull, 0x000000000000FF00ull, 0x00000000000000F0ull, 0x000000000000000Cull, 0x0000000000000002ull };
    int y = (((x & (x - 1)) == 0) ? 0 : 1);
    int j = 32;
    for( int i = 0; i < 6; i++)
    {
      int k = (((x & t[i]) == 0) ? 0 : j);
      y += k;
      x >>= k;
      j >>= 1;
    }
    return y;
  }

  void Quantizer::initQuantBlock( const TransformUnit& tu, const ComponentID compID, const QpParam& cQP, const double lambda )
  {
#if HEVC_USE_SCALING_LISTS
    CHECK ( tu.cs->sps->getScalingListFlag(), "Scaling lists not supported" );
#endif
    CHECKD( lambda <= 0.0, "Lambda must be greater than 0" );

    const int         qpDQ                  = cQP.Qp + 1;
    const int         qpPer                 = qpDQ / 6;
    const int         qpRem                 = qpDQ - 6 * qpPer;
    const SPS&        sps                   = *tu.cs->sps;
    const CompArea&   area                  = tu.blocks[ compID ];
    const ChannelType chType                = toChannelType( compID );
    const int         channelBitDepth       = sps.getBitDepth( chType );
    const int         maxLog2TrDynamicRange = sps.getMaxLog2TrDynamicRange( chType );
    const int         nomTransformShift     = getTransformShift( channelBitDepth, area.size(), maxLog2TrDynamicRange );
    const bool        clipTransformShift    = ( tu.transformSkip[ compID ] && sps.getSpsRangeExtension().getExtendedPrecisionProcessingFlag() );
    const int         transformShift        = ( clipTransformShift ? std::max<int>( 0, nomTransformShift ) : nomTransformShift );

    // quant parameters
    m_QShift                    = QUANT_SHIFT  - 1 + qpPer + transformShift;
    m_QAdd                      = -( ( 3 << m_QShift ) >> 1 );
#if HM_QTBT_AS_IN_JEM_QUANT
    Intermediate_Int  invShift  = IQUANT_SHIFT + 1 - qpPer - transformShift + ( TU::needsBlockSizeTrafoScale( area ) ? ADJ_DEQUANT_SHIFT : 0 );
    m_QScale                    = ( TU::needsSqrt2Scale( area ) ? ( g_quantScales[ qpRem ] * 181 ) >> 7 : g_quantScales[ qpRem ] );
#else
    Intermediate_Int  invShift  = IQUANT_SHIFT + 1 - qpPer - transformShift;
    m_QScale                    = g_quantScales   [ qpRem ];
#endif
    const unsigned    qIdxBD    = std::min<unsigned>( maxLog2TrDynamicRange + 1, 8*sizeof(Intermediate_Int) + invShift - IQUANT_SHIFT - 1 );
    m_maxQIdx                   = ( 1 << (qIdxBD-1) ) - 4;
    m_thresLast                 = TCoeff( ( int64_t(3) << m_QShift ) / ( 4 * m_QScale ) );
    m_thresSSbb                 = TCoeff( ( int64_t(3) << m_QShift ) / ( 4 * m_QScale ) );

    // distortion calculation parameters
    const int64_t qScale        = g_quantScales[ qpRem ];
#if HM_QTBT_AS_IN_JEM_QUANT
#if DISTORTION_LAMBDA_BUGFIX
    const int nomDShift =
      SCALE_BITS - 2 * (nomTransformShift + DISTORTION_PRECISION_ADJUSTMENT(channelBitDepth)) + m_QShift;
#else
    const int     nomDShift     = SCALE_BITS - 2 * ( nomTransformShift + channelBitDepth - 8 ) + m_QShift;
#endif
#else
#if DISTORTION_LAMBDA_BUGFIX
    const int nomDShift = SCALE_BITS - 2 * (nomTransformShift + DISTORTION_PRECISION_ADJUSTMENT(channelBitDepth))
                          + m_QShift + (TU::needsQP3Offset(tu, compID) ? 1 : 0);
#else
    const int     nomDShift     = SCALE_BITS - 2 * ( nomTransformShift + channelBitDepth - 8 ) + m_QShift + ( TU::needsQP3Offset( tu, compID ) ? 1 : 0 );
#endif
#endif
    const double  qScale2       = double( qScale * qScale );
    const double  nomDistFactor = ( nomDShift < 0 ? 1.0/(double(int64_t(1)<<(-nomDShift))*qScale2*lambda) : double(int64_t(1)<<nomDShift)/(qScale2*lambda) );
    const int64_t pow2dfShift   = (int64_t)( nomDistFactor * qScale2 ) + 1;
    const int     dfShift       = ceil_log2( pow2dfShift );
    m_DistShift                 = 62 + m_QShift - 2*maxLog2TrDynamicRange - dfShift;
    m_DistAdd                   = (int64_t(1) << m_DistShift) >> 1;
    m_DistStepAdd               = (int64_t)( nomDistFactor * double(int64_t(1)<<(m_DistShift+m_QShift)) + .5 );
    m_DistOrgFact               = (int64_t)( nomDistFactor * double(int64_t(1)<<(m_DistShift+1       )) + .5 );
  }

  void Quantizer::dequantBlock( const TransformUnit& tu, const ComponentID compID, const QpParam& cQP, CoeffBuf& recCoeff ) const
  {
#if HEVC_USE_SCALING_LISTS
    CHECK ( tu.cs->sps->getScalingListFlag(), "Scaling lists not supported" );
#endif

    //----- set basic parameters -----
    const CompArea&     area      = tu.blocks[ compID ];
    const int           numCoeff  = area.area();
    const SizeType      hsId      = gp_sizeIdxInfo->idxFrom( area.width  );
    const SizeType      vsId      = gp_sizeIdxInfo->idxFrom( area.height );
#if HEVC_USE_MDCS
    const CoeffScanType scanType  = CoeffScanType( TU::getCoefScanIdx( tu, compID ) );
#else
    const CoeffScanType scanType  = SCAN_DIAG;
#endif
    const unsigned*     scan      = g_scanOrder[ SCAN_GROUPED_4x4 ][ scanType ][ hsId ][ vsId ];
    const TCoeff*       qCoeff    = tu.getCoeffs( compID ).buf;
          TCoeff*       tCoeff    = recCoeff.buf;

    //----- reset coefficients and get last scan index -----
    ::memset( tCoeff, 0, numCoeff * sizeof(TCoeff) );
    int lastScanIdx = -1;
    for( int scanIdx = numCoeff - 1; scanIdx >= 0; scanIdx-- )
    {
      if( qCoeff[ scan[ scanIdx ] ] )
      {
        lastScanIdx = scanIdx;
        break;
      }
    }
    if( lastScanIdx < 0 )
    {
      return;
    }

    //----- set dequant parameters -----
    const int         qpDQ                  = cQP.Qp + 1;
    const int         qpPer                 = qpDQ / 6;
    const int         qpRem                 = qpDQ - 6 * qpPer;
    const SPS&        sps                   = *tu.cs->sps;
    const ChannelType chType                = toChannelType( compID );
    const int         channelBitDepth       = sps.getBitDepth( chType );
    const int         maxLog2TrDynamicRange = sps.getMaxLog2TrDynamicRange( chType );
    const TCoeff      minTCoeff             = -( 1 << maxLog2TrDynamicRange );
    const TCoeff      maxTCoeff             =  ( 1 << maxLog2TrDynamicRange ) - 1;
    const int         nomTransformShift     = getTransformShift( channelBitDepth, area.size(), maxLog2TrDynamicRange );
    const bool        clipTransformShift    = ( tu.transformSkip[ compID ] && sps.getSpsRangeExtension().getExtendedPrecisionProcessingFlag() );
    const int         transformShift        = ( clipTransformShift ? std::max<int>( 0, nomTransformShift ) : nomTransformShift );
#if HM_QTBT_AS_IN_JEM_QUANT
    Intermediate_Int  shift                 = IQUANT_SHIFT + 1 - qpPer - transformShift + ( TU::needsBlockSizeTrafoScale( area ) ? ADJ_DEQUANT_SHIFT : 0 );
    Intermediate_Int  invQScale             = g_invQuantScales[ qpRem ] * ( TU::needsSqrt2Scale( area ) ? 181 : 1 );
#else
    Intermediate_Int  shift                 = IQUANT_SHIFT + 1 - qpPer - transformShift;
    Intermediate_Int  invQScale             = g_invQuantScales[ qpRem ];
#endif
    if( shift < 0 )
    {
      invQScale <<= -shift;
      shift       = 0;
    }
    Intermediate_Int  add       = ( 1 << shift ) >> 1;

    //----- dequant coefficients -----
    for( int state = 0, scanIdx = lastScanIdx; scanIdx >= 0; scanIdx-- )
    {
      const unsigned  rasterPos = scan  [ scanIdx   ];
      const TCoeff&   level     = qCoeff[ rasterPos ];
      if( level )
      {
        Intermediate_Int  qIdx      = ( level << 1 ) + ( level > 0 ? -(state>>1) : (state>>1) );
        Intermediate_Int  nomTCoeff = ( qIdx * invQScale + add ) >> shift;
        tCoeff[ rasterPos ]         = (TCoeff)Clip3<Intermediate_Int>( minTCoeff, maxTCoeff, nomTCoeff );
      }
      state = ( 32040 >> ((state<<2)+((level&1)<<1)) ) & 3;   // the 16-bit value "32040" represent the state transition table
    }
  }

  inline void Quantizer::preQuantCoeff(const TCoeff absCoeff, PQData *pqData) const
  {
    int64_t scaledOrg = int64_t( absCoeff ) * m_QScale;
    TCoeff  qIdx      = std::max<TCoeff>( 1, std::min<TCoeff>( m_maxQIdx, TCoeff( ( scaledOrg + m_QAdd ) >> m_QShift ) ) );
    int64_t scaledAdd = qIdx * m_DistStepAdd - scaledOrg * m_DistOrgFact;
    PQData& pq_a      = pqData[ qIdx & 3 ];
    pq_a.deltaDist    = ( scaledAdd * qIdx + m_DistAdd ) >> m_DistShift;
    pq_a.absLevel     = ( ++qIdx ) >> 1;
    scaledAdd        += m_DistStepAdd;
    PQData& pq_b      = pqData[ qIdx & 3 ];
    pq_b.deltaDist    = ( scaledAdd * qIdx + m_DistAdd ) >> m_DistShift;
    pq_b.absLevel     = ( ++qIdx ) >> 1;
    scaledAdd        += m_DistStepAdd;
    PQData& pq_c      = pqData[ qIdx & 3 ];
    pq_c.deltaDist    = ( scaledAdd * qIdx + m_DistAdd ) >> m_DistShift;
    pq_c.absLevel     = ( ++qIdx ) >> 1;
    scaledAdd        += m_DistStepAdd;
    PQData& pq_d      = pqData[ qIdx & 3 ];
    pq_d.deltaDist    = ( scaledAdd * qIdx + m_DistAdd ) >> m_DistShift;
    pq_d.absLevel     = ( ++qIdx ) >> 1;
  }







  /*================================================================================*/
  /*=====                                                                      =====*/
  /*=====   T C Q   S T A T E                                                  =====*/
  /*=====                                                                      =====*/
  /*================================================================================*/

  class State;

  struct SbbCtx
  {
    uint8_t*  sbbFlags;
    uint8_t*  levels;
  };

  class CommonCtx
  {
  public:
    CommonCtx() : m_currSbbCtx( m_allSbbCtx ), m_prevSbbCtx( m_currSbbCtx + 4 ) {}

    inline void swap() { std::swap(m_currSbbCtx, m_prevSbbCtx); }

    inline void reset(const RateEstimator &rateEst)
    {
      m_nbInfo = rateEst.nbInfoOut();
      ::memcpy( m_sbbFlagBits, rateEst.sigSbbFracBits(), 2*sizeof(BinFracBits) );
      const int numSbb    = rateEst.numSbb();
      const int chunkSize = numSbb + rateEst.numCoeff();
      uint8_t*  nextMem   = m_memory;
      for( int k = 0; k < 8; k++, nextMem += chunkSize )
      {
        m_allSbbCtx[k].sbbFlags = nextMem;
        m_allSbbCtx[k].levels   = nextMem + numSbb;
      }
    }

    inline void update(const ScanInfo &scanInfo, const State *prevState, State &currState);

  private:
    const NbInfoOut*            m_nbInfo;
    BinFracBits                 m_sbbFlagBits[2];
    SbbCtx                      m_allSbbCtx  [8];
    SbbCtx*                     m_currSbbCtx;
    SbbCtx*                     m_prevSbbCtx;
    uint8_t                     m_memory[ 8 * ( MAX_TU_SIZE * MAX_TU_SIZE + MLS_GRP_NUM ) ];
  };


  class State
  {
    friend class CommonCtx;
  public:
    State( const RateEstimator& rateEst, CommonCtx& commonCtx, const int stateId );

    template<uint8_t numIPos>
    inline void updateState(const ScanInfo &scanInfo, const State *prevStates, const Decision &decision);
    inline void updateStateEOS(const ScanInfo &scanInfo, const State *prevStates, const State *skipStates,
                               const Decision &decision);

    inline void init()
    {
      m_rdCost        = std::numeric_limits<int64_t>::max()>>1;
      m_numSigSbb     = 0;
      m_refSbbCtxId   = -1;
      m_sigFracBits   = m_sigFracBitsArray[ 0 ];
      m_coeffFracBits = m_gtxFracBitsArray[ 0 ];
      m_goRicePar     = 0;
    }

    template<ScanPosType spt> inline void checkRdCostZero(Decision &decision) const
    {
      int64_t rdCost = m_rdCost;
      if( spt == SCAN_ISCSBB )
      {
        rdCost += m_sigFracBits.intBits[0];
      }
      else if( spt == SCAN_SOCSBB )
      {
        rdCost += m_sbbFracBits.intBits[1] + m_sigFracBits.intBits[0];
      }
      else if( m_numSigSbb )
      {
        rdCost += m_sigFracBits.intBits[0];
      }
      else
      {
        return;
      }
      if( rdCost < decision.rdCost )
      {
        decision.rdCost   = rdCost;
        decision.absLevel = 0;
        decision.prevId   = m_stateId;
      }
    }

    inline int32_t getLevelBits(const unsigned level) const
    {
      if( level < 5 )
      {
        return m_coeffFracBits.bits[level];
      }
      unsigned  value   = ( level - 5 ) >> 1;
      int32_t   bits    = m_coeffFracBits.bits[ level - (value << 1) ];
      unsigned  thres   = g_auiGoRiceRange[ m_goRicePar ] << m_goRicePar;
      if( value < thres )
      {
        return bits + ( ( ( value >> m_goRicePar ) + 1 + m_goRicePar ) << SCALE_BITS );
      }
      unsigned  length  = m_goRicePar;
      unsigned  delta   = 1 << length;
      unsigned  valLeft = value - thres;
      while( valLeft >= delta )
      {
        valLeft -= delta;
        delta    = 1 << (++length);
      }
      return bits + ( ( g_auiGoRiceRange[ m_goRicePar ] + 1 + ( length << 1 ) - m_goRicePar ) << SCALE_BITS );
    }

    template<ScanPosType spt> inline void checkRdCostNonZero(const PQData &pqData, Decision &decision) const
    {
      int64_t rdCost = m_rdCost + pqData.deltaDist + getLevelBits( pqData.absLevel );
      if( spt == SCAN_ISCSBB )
      {
        rdCost += m_sigFracBits.intBits[1];
      }
      else if( spt == SCAN_SOCSBB )
      {
        rdCost += m_sbbFracBits.intBits[1] + m_sigFracBits.intBits[1];
      }
      else if( m_numSigSbb )
      {
        rdCost += m_sigFracBits.intBits[1];
      }
      if( rdCost < decision.rdCost )
      {
        decision.rdCost   = rdCost;
        decision.absLevel = pqData.absLevel;
        decision.prevId   = m_stateId;
      }
    }

    inline void checkRdCostStart(int32_t lastOffset, const PQData &pqData, Decision &decision) const
    {
      int64_t rdCost = pqData.deltaDist + lastOffset + getLevelBits( pqData.absLevel );
      if( rdCost < decision.rdCost )
      {
        decision.rdCost   = rdCost;
        decision.absLevel = pqData.absLevel;
        decision.prevId   = -1;
      }
    }

    inline void checkRdCostSkipSbb(Decision &decision) const
    {
      int64_t rdCost = m_rdCost + m_sbbFracBits.intBits[0];
      if( rdCost < decision.rdCost )
      {
        decision.rdCost   = rdCost;
        decision.absLevel = 0;
        decision.prevId   = 4+m_stateId;
      }
    }

  private:
    int64_t                   m_rdCost;
    uint16_t                  m_absLevelsAndCtxInit[24];  // 16x8bit for abs levels + 16x16bit for ctx init id
    int32_t                   m_numSigSbb;
    int32_t                   m_refSbbCtxId;
    BinFracBits               m_sbbFracBits;
    BinFracBits               m_sigFracBits;
    CoeffFracBits             m_coeffFracBits;
    int                       m_goRicePar;
    const int                 m_stateId;
    const BinFracBits*const   m_sigFracBitsArray;
    const CoeffFracBits*const m_gtxFracBitsArray;
    CommonCtx&                m_commonCtx;
  };


  State::State( const RateEstimator& rateEst, CommonCtx& commonCtx, const int stateId )
    : m_sbbFracBits     { { 0, 0 } }
    , m_stateId         ( stateId )
    , m_sigFracBitsArray( rateEst.sigFlagBits(stateId) )
    , m_gtxFracBitsArray( rateEst.gtxFracBits(stateId) )
    , m_commonCtx       ( commonCtx )
  {
  }

  template<uint8_t numIPos>
  inline void State::updateState(const ScanInfo &scanInfo, const State *prevStates, const Decision &decision)
  {
    m_rdCost = decision.rdCost;
    if( decision.prevId > -2 )
    {
      if( decision.prevId >= 0 )
      {
        const State*  prvState  = prevStates            +   decision.prevId;
        m_numSigSbb             = prvState->m_numSigSbb + !!decision.absLevel;
        m_refSbbCtxId           = prvState->m_refSbbCtxId;
        m_sbbFracBits           = prvState->m_sbbFracBits;
        ::memcpy( m_absLevelsAndCtxInit, prvState->m_absLevelsAndCtxInit, 48*sizeof(uint8_t) );
      }
      else
      {
        m_numSigSbb     =  1;
        m_refSbbCtxId   = -1;
        ::memset( m_absLevelsAndCtxInit, 0, 48*sizeof(uint8_t) );
      }

      uint8_t* levels               = reinterpret_cast<uint8_t*>(m_absLevelsAndCtxInit);
      levels[ scanInfo.insidePos ]  = (uint8_t)std::min<TCoeff>( 255, decision.absLevel );

      TCoeff  tinit   = m_absLevelsAndCtxInit[ 8 + scanInfo.nextInsidePos ];
      TCoeff  sumAbs  =   tinit >> 8;
      TCoeff  sumAbs1 = ( tinit >> 3 ) & 31;
      TCoeff  sumNum  =   tinit        & 7;
#define UPDATE(k) {TCoeff t=levels[scanInfo.nextNbInfoSbb.inPos[k]]; sumAbs+=t; sumAbs1+=std::min<TCoeff>(4-(t&1),t); sumNum+=!!t; }
      if( numIPos == 1 )
      {
        UPDATE(0);
      }
      else if( numIPos == 2 )
      {
        UPDATE(0);
        UPDATE(1);
      }
      else if( numIPos == 3 )
      {
        UPDATE(0);
        UPDATE(1);
        UPDATE(2);
      }
      else if( numIPos == 4 )
      {
        UPDATE(0);
        UPDATE(1);
        UPDATE(2);
        UPDATE(3);
      }
      else if( numIPos == 5 )
      {
        UPDATE(0);
        UPDATE(1);
        UPDATE(2);
        UPDATE(3);
        UPDATE(4);
      }
#undef UPDATE
      TCoeff sumGt1   = sumAbs1 - sumNum;
      sumAbs         -= sumNum;
      m_sigFracBits   = m_sigFracBitsArray[ scanInfo.sigCtxOffsetNext + ( sumAbs1 < 5 ? sumAbs1 : 5 ) ];
      m_coeffFracBits = m_gtxFracBitsArray[ scanInfo.gtxCtxOffsetNext + ( sumGt1  < 4 ? sumGt1  : 4 ) ];
      m_goRicePar     = g_auiGoRicePars   [ sumAbs < 31 ? sumAbs : 31 ];
    }
  }

  inline void State::updateStateEOS(const ScanInfo &scanInfo, const State *prevStates, const State *skipStates,
                                    const Decision &decision)
  {
    m_rdCost = decision.rdCost;
    if( decision.prevId > -2 )
    {
      const State* prvState = 0;
      if( decision.prevId  >= 0 )
      {
        prvState    = ( decision.prevId < 4 ? prevStates : skipStates - 4 ) +   decision.prevId;
        m_numSigSbb = prvState->m_numSigSbb                                 + !!decision.absLevel;
        ::memcpy( m_absLevelsAndCtxInit, prvState->m_absLevelsAndCtxInit, 16*sizeof(uint8_t) );
      }
      else
      {
        m_numSigSbb = 1;
        ::memset( m_absLevelsAndCtxInit, 0, 16*sizeof(uint8_t) );
      }
      reinterpret_cast<uint8_t*>(m_absLevelsAndCtxInit)[ scanInfo.insidePos ] = (uint8_t)std::min<TCoeff>( 255, decision.absLevel );

      m_commonCtx.update( scanInfo, prvState, *this );

      TCoeff  tinit   = m_absLevelsAndCtxInit[ 8 + scanInfo.nextInsidePos ];
      TCoeff  sumNum  =   tinit        & 7;
      TCoeff  sumAbs1 = ( tinit >> 3 ) & 31;
      TCoeff  sumAbs  = ( tinit >> 8 ) - sumNum;
      TCoeff  sumGt1  = sumAbs1        - sumNum;
      m_sigFracBits   = m_sigFracBitsArray[ scanInfo.sigCtxOffsetNext + ( sumAbs1 < 5 ? sumAbs1 : 5 ) ];
      m_coeffFracBits = m_gtxFracBitsArray[ scanInfo.gtxCtxOffsetNext + ( sumGt1  < 4 ? sumGt1  : 4 ) ];
      m_goRicePar     = g_auiGoRicePars   [ sumAbs < 31 ? sumAbs : 31 ];
    }
  }

  inline void CommonCtx::update(const ScanInfo &scanInfo, const State *prevState, State &currState)
  {
    uint8_t*    sbbFlags  = m_currSbbCtx[ currState.m_stateId ].sbbFlags;
    uint8_t*    levels    = m_currSbbCtx[ currState.m_stateId ].levels;
    std::size_t setCpSize = m_nbInfo[ scanInfo.scanIdx - 1 ].maxDist * sizeof(uint8_t);
    if( prevState && prevState->m_refSbbCtxId >= 0 )
    {
      ::memcpy( sbbFlags,                  m_prevSbbCtx[prevState->m_refSbbCtxId].sbbFlags,                  scanInfo.numSbb*sizeof(uint8_t) );
      ::memcpy( levels + scanInfo.scanIdx, m_prevSbbCtx[prevState->m_refSbbCtxId].levels + scanInfo.scanIdx, setCpSize );
    }
    else
    {
      ::memset( sbbFlags,                  0, scanInfo.numSbb*sizeof(uint8_t) );
      ::memset( levels + scanInfo.scanIdx, 0, setCpSize );
    }
    sbbFlags[ scanInfo.sbbPos ] = !!currState.m_numSigSbb;
    ::memcpy( levels + scanInfo.scanIdx, currState.m_absLevelsAndCtxInit, scanInfo.sbbSize*sizeof(uint8_t) );

    const int       sigNSbb   = ( ( scanInfo.nextSbbRight ? sbbFlags[ scanInfo.nextSbbRight ] : false ) || ( scanInfo.nextSbbBelow ? sbbFlags[ scanInfo.nextSbbBelow ] : false ) ? 1 : 0 );
    currState.m_numSigSbb     = 0;
    currState.m_refSbbCtxId   = currState.m_stateId;
    currState.m_sbbFracBits   = m_sbbFlagBits[ sigNSbb ];

    uint16_t          templateCtxInit[16];
    const int         scanBeg   = scanInfo.scanIdx - scanInfo.sbbSize;
    const NbInfoOut*  nbOut     = m_nbInfo + scanBeg;
    const uint8_t*    absLevels = levels   + scanBeg;
    for( int id = 0; id < scanInfo.sbbSize; id++, nbOut++ )
    {
      if( nbOut->num )
      {
        TCoeff sumAbs = 0, sumAbs1 = 0, sumNum = 0;
#define UPDATE(k) {TCoeff t=absLevels[nbOut->outPos[k]]; sumAbs+=t; sumAbs1+=std::min<TCoeff>(4-(t&1),t); sumNum+=!!t; }
        UPDATE(0);
        if( nbOut->num > 1 )
        {
          UPDATE(1);
          if( nbOut->num > 2 )
          {
            UPDATE(2);
            if( nbOut->num > 3 )
            {
              UPDATE(3);
              if( nbOut->num > 4 )
              {
                UPDATE(4);
              }
            }
          }
        }
#undef UPDATE
        templateCtxInit[id] = uint16_t(sumNum) + ( uint16_t(sumAbs1) << 3 ) + ( (uint16_t)std::min<TCoeff>( 127, sumAbs ) << 8 );
      }
      else
      {
        templateCtxInit[id] = 0;
      }
    }
    ::memset( currState.m_absLevelsAndCtxInit,     0,               16*sizeof(uint8_t) );
    ::memcpy( currState.m_absLevelsAndCtxInit + 8, templateCtxInit, 16*sizeof(uint16_t) );
  }



  /*================================================================================*/
  /*=====                                                                      =====*/
  /*=====   T C Q                                                              =====*/
  /*=====                                                                      =====*/
  /*================================================================================*/
  class DepQuant : private RateEstimator
  {
  public:
    DepQuant();

    void    quant   ( TransformUnit& tu, const CCoeffBuf& srcCoeff, const ComponentID compID, const QpParam& cQP, const double lambda, const Ctx& ctx, TCoeff& absSum );
    void    dequant ( const TransformUnit& tu,  CoeffBuf& recCoeff, const ComponentID compID, const QpParam& cQP )  const;

  private:
    void    xDecideAndUpdate  ( const TCoeff absCoeff, const ScanInfo& scanInfo );
    template<ScanPosType spt>
    void    xDecide           ( const TCoeff absCoeff, int32_t lastOffset, Decision* decisions );

  private:
    CommonCtx   m_commonCtx;
    State       m_allStates[ 12 ];
    State*      m_currStates;
    State*      m_prevStates;
    State*      m_skipStates;
    State       m_startState;
    Quantizer   m_quant;
    Decision    m_trellis[ MAX_TU_SIZE * MAX_TU_SIZE ][ 8 ];
  };


#define TINIT(x) {*this,m_commonCtx,x}
  DepQuant::DepQuant()
    : RateEstimator ()
    , m_commonCtx   ()
    , m_allStates   {TINIT(0),TINIT(1),TINIT(2),TINIT(3),TINIT(0),TINIT(1),TINIT(2),TINIT(3),TINIT(0),TINIT(1),TINIT(2),TINIT(3)}
    , m_currStates  (  m_allStates      )
    , m_prevStates  (  m_currStates + 4 )
    , m_skipStates  (  m_prevStates + 4 )
    , m_startState  TINIT(0)
  {}
#undef TINIT


  void DepQuant::dequant( const TransformUnit& tu,  CoeffBuf& recCoeff, const ComponentID compID, const QpParam& cQP ) const
  {
    m_quant.dequantBlock( tu, compID, cQP, recCoeff );
  }


#define DINIT(l,p) {std::numeric_limits<int64_t>::max()>>2,l,p}
  static const Decision startDec[8] = {DINIT(-1,-2),DINIT(-1,-2),DINIT(-1,-2),DINIT(-1,-2),DINIT(0,4),DINIT(0,5),DINIT(0,6),DINIT(0,7)};
#undef  DINIT


  template<ScanPosType spt>
  void DepQuant::xDecide( const TCoeff absCoeff, int32_t lastOffset, Decision* decisions )
  {
    ::memcpy( decisions, startDec, 8*sizeof(Decision) );

    PQData  pqData[4];
    m_quant.preQuantCoeff( absCoeff, pqData );
    m_prevStates[0].checkRdCostNonZero<spt> ( pqData[0],  decisions[0] );
    m_prevStates[0].checkRdCostNonZero<spt> ( pqData[2],  decisions[2] );
    m_prevStates[0].checkRdCostZero<spt>                ( decisions[0] );
    m_prevStates[1].checkRdCostNonZero<spt> ( pqData[2],  decisions[0] );
    m_prevStates[1].checkRdCostNonZero<spt> ( pqData[0],  decisions[2] );
    m_prevStates[1].checkRdCostZero<spt>                ( decisions[2] );
    m_prevStates[2].checkRdCostNonZero<spt> ( pqData[3],  decisions[1] );
    m_prevStates[2].checkRdCostNonZero<spt> ( pqData[1],  decisions[3] );
    m_prevStates[2].checkRdCostZero<spt>                ( decisions[1] );
    m_prevStates[3].checkRdCostNonZero<spt> ( pqData[1],  decisions[1] );
    m_prevStates[3].checkRdCostNonZero<spt> ( pqData[3],  decisions[3] );
    m_prevStates[3].checkRdCostZero<spt>                ( decisions[3] );
    if( spt==SCAN_EOCSBB )
    {
      m_skipStates[0].checkRdCostSkipSbb( decisions[0] );
      m_skipStates[1].checkRdCostSkipSbb( decisions[1] );
      m_skipStates[2].checkRdCostSkipSbb( decisions[2] );
      m_skipStates[3].checkRdCostSkipSbb( decisions[3] );
    }
    m_startState.checkRdCostStart( lastOffset, pqData[0], decisions[0] );
    m_startState.checkRdCostStart( lastOffset, pqData[2], decisions[2] );
  }

  void DepQuant::xDecideAndUpdate( const TCoeff absCoeff, const ScanInfo& scanInfo )
  {
    Decision* decisions = m_trellis[ scanInfo.scanIdx ];

    std::swap( m_prevStates, m_currStates );

    if     ( scanInfo.socsbb )  { xDecide<SCAN_SOCSBB>( absCoeff, scanInfo.lastOffset, decisions ); }
    else if( scanInfo.eocsbb )  { xDecide<SCAN_EOCSBB>( absCoeff, scanInfo.lastOffset, decisions ); }
    else                        { xDecide<SCAN_ISCSBB>( absCoeff, scanInfo.lastOffset, decisions ); }

    if( scanInfo.scanIdx )
    {
      if( scanInfo.eosbb )
      {
        m_commonCtx.swap();
        m_currStates[0].updateStateEOS( scanInfo, m_prevStates, m_skipStates, decisions[0] );
        m_currStates[1].updateStateEOS( scanInfo, m_prevStates, m_skipStates, decisions[1] );
        m_currStates[2].updateStateEOS( scanInfo, m_prevStates, m_skipStates, decisions[2] );
        m_currStates[3].updateStateEOS( scanInfo, m_prevStates, m_skipStates, decisions[3] );
        ::memcpy( decisions+4, decisions, 4*sizeof(Decision) );
      }
      else
      {
        switch( scanInfo.nextNbInfoSbb.num )
        {
        case 0:
          m_currStates[0].updateState<0>( scanInfo, m_prevStates, decisions[0] );
          m_currStates[1].updateState<0>( scanInfo, m_prevStates, decisions[1] );
          m_currStates[2].updateState<0>( scanInfo, m_prevStates, decisions[2] );
          m_currStates[3].updateState<0>( scanInfo, m_prevStates, decisions[3] );
          break;
        case 1:
          m_currStates[0].updateState<1>( scanInfo, m_prevStates, decisions[0] );
          m_currStates[1].updateState<1>( scanInfo, m_prevStates, decisions[1] );
          m_currStates[2].updateState<1>( scanInfo, m_prevStates, decisions[2] );
          m_currStates[3].updateState<1>( scanInfo, m_prevStates, decisions[3] );
          break;
        case 2:
          m_currStates[0].updateState<2>( scanInfo, m_prevStates, decisions[0] );
          m_currStates[1].updateState<2>( scanInfo, m_prevStates, decisions[1] );
          m_currStates[2].updateState<2>( scanInfo, m_prevStates, decisions[2] );
          m_currStates[3].updateState<2>( scanInfo, m_prevStates, decisions[3] );
          break;
        case 3:
          m_currStates[0].updateState<3>( scanInfo, m_prevStates, decisions[0] );
          m_currStates[1].updateState<3>( scanInfo, m_prevStates, decisions[1] );
          m_currStates[2].updateState<3>( scanInfo, m_prevStates, decisions[2] );
          m_currStates[3].updateState<3>( scanInfo, m_prevStates, decisions[3] );
          break;
        case 4:
          m_currStates[0].updateState<4>( scanInfo, m_prevStates, decisions[0] );
          m_currStates[1].updateState<4>( scanInfo, m_prevStates, decisions[1] );
          m_currStates[2].updateState<4>( scanInfo, m_prevStates, decisions[2] );
          m_currStates[3].updateState<4>( scanInfo, m_prevStates, decisions[3] );
          break;
        default:
          m_currStates[0].updateState<5>( scanInfo, m_prevStates, decisions[0] );
          m_currStates[1].updateState<5>( scanInfo, m_prevStates, decisions[1] );
          m_currStates[2].updateState<5>( scanInfo, m_prevStates, decisions[2] );
          m_currStates[3].updateState<5>( scanInfo, m_prevStates, decisions[3] );
        }
      }

      if( scanInfo.socsbb )
      {
        std::swap( m_prevStates, m_skipStates );
      }
    }
  }


  void DepQuant::quant( TransformUnit& tu, const CCoeffBuf& srcCoeff, const ComponentID compID, const QpParam& cQP, const double lambda, const Ctx& ctx, TCoeff& absSum )
  {
    //===== reset / pre-init =====
    RateEstimator::initBlock  ( tu, compID );
    m_quant.initQuantBlock    ( tu, compID, cQP, lambda );
    TCoeff*       qCoeff      = tu.getCoeffs( compID ).buf;
    const TCoeff* tCoeff      = srcCoeff.buf;
    const int     numCoeff    = tu.blocks[compID].area();
    ::memset( tu.getCoeffs( compID ).buf, 0x00, numCoeff*sizeof(TCoeff) );
    absSum          = 0;

    //===== find first test position =====
    int   firstTestPos = numCoeff - 1;
    const TCoeff thres = m_quant.getLastThreshold();
    for( ; firstTestPos >= 0; firstTestPos-- )
    {
      if( abs( tCoeff[ rasterPos(firstTestPos) ] ) > thres )
      {
        break;
      }
    }
    if( firstTestPos < 0 )
    {
      return;
    }

    //===== real init =====
    RateEstimator::initCtx( tu, ctx.getFracBitsAcess() );
    m_commonCtx.reset( *this );
    for( int k = 0; k < 12; k++ )
    {
      m_allStates[k].init();
    }
    m_startState.init();


    //===== populate trellis =====
    for( ScanData scanData(*this,firstTestPos); scanData.valid(); scanData.next() )
    {
      xDecideAndUpdate( abs( tCoeff[ scanData.rasterPos ] ), scanData );
    }

    //===== find best path =====
    Decision  decision    = { std::numeric_limits<int64_t>::max(), -1, -2 };
    int64_t   minPathCost =  0;
    for( int8_t stateId = 0; stateId < 4; stateId++ )
    {
      int64_t pathCost = m_trellis[0][stateId].rdCost;
      if( pathCost < minPathCost )
      {
        decision.prevId = stateId;
        minPathCost     = pathCost;
      }
    }

    //===== backward scanning =====
    int scanIdx = 0;
    for( ; decision.prevId >= 0; scanIdx++ )
    {
      decision          = m_trellis[ scanIdx ][ decision.prevId ];
      int32_t blkpos    = rasterPos( scanIdx );
      qCoeff[ blkpos ]  = ( tCoeff[ blkpos ] < 0 ? -decision.absLevel : decision.absLevel );
      absSum           += decision.absLevel;
    }
  }

}; // namespace DQIntern




//===== interface class =====
DepQuant::DepQuant( const Quant* other, bool enc ) : QuantRDOQ( other )
{
  const DepQuant* dq = dynamic_cast<const DepQuant*>( other );
  CHECK( other && !dq, "The DepQuant cast must be successfull!" );
  p = new DQIntern::DepQuant();
  if( enc )
  {
    DQIntern::g_Rom.init();
  }
}

DepQuant::~DepQuant()
{
  delete static_cast<DQIntern::DepQuant*>(p);
}

void DepQuant::quant( TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pSrc, TCoeff &uiAbsSum, const QpParam &cQP, const Ctx& ctx )
{
  if( tu.cs->slice->getDepQuantEnabledFlag() )
  {
    static_cast<DQIntern::DepQuant*>(p)->quant( tu, pSrc, compID, cQP, Quant::m_dLambda, ctx, uiAbsSum );
  }
  else
  {
    QuantRDOQ::quant( tu, compID, pSrc, uiAbsSum, cQP, ctx );
  }
}

void DepQuant::dequant( const TransformUnit &tu, CoeffBuf &dstCoeff, const ComponentID &compID, const QpParam &cQP )
{
  if( tu.cs->slice->getDepQuantEnabledFlag() )
  {
    static_cast<DQIntern::DepQuant*>(p)->dequant( tu, dstCoeff, compID, cQP );
  }
  else
  {
    QuantRDOQ::dequant( tu, dstCoeff, compID, cQP );
  }
}

#endif



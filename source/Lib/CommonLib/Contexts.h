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

/** \file     Contexts.h
 *  \brief    Classes providing probability descriptions and contexts (header)
 */

#ifndef __CONTEXTS__
#define __CONTEXTS__

#include "CommonDef.h"
#include "Slice.h"

#include <vector>

struct BinFracBits
{
  uint32_t intBits[2];
};


enum BPMType
{
  BPM_Undefined = 0,
  BPM_Std,
  BPM_NUM
};

class ProbModelTables
{
protected:
  static const uint8_t      m_NextState       [128][2];       // Std
  static const uint32_t     m_EstFracBits     [128];          // Std
  static const BinFracBits  m_BinFracBits_128 [128];          // Std
  static const uint32_t     m_EstFracProb     [128];          // Std
  static const uint8_t      m_LPSTable_64_4   [ 64][4];       // Std
  static const uint8_t      m_RenormTable_32  [ 32];          // Std         MP   MPI
};



class BinProbModelBase : public ProbModelTables
{
public:
  BinProbModelBase () {}
  ~BinProbModelBase() {}
  static uint32_t estFracBitsEP ()                    { return  (       1 << SCALE_BITS ); }
  static uint32_t estFracBitsEP ( unsigned numBins )  { return  ( numBins << SCALE_BITS ); }
};




class BinProbModel_Std : public BinProbModelBase
{
public:
  BinProbModel_Std  () : m_State( 0 ) {}
  ~BinProbModel_Std ()                {}
public:
  void            init              ( int qp, int initId );
  void            update            ( unsigned bin )                    { m_State = m_NextState       [m_State][bin]; }
  static uint8_t  getDefaultWinSize ()                                  { return uint8_t(0); }
  void            setLog2WindowSize ( uint8_t log2WindowSize )          {}
  void            estFracBitsUpdate ( unsigned bin, uint64_t& b )       {      b += m_EstFracBits     [m_State ^bin];
                                                                          m_State = m_NextState       [m_State][bin]; }
  uint32_t        estFracBits       ( unsigned bin )              const { return    m_EstFracBits     [m_State ^bin]; }
  static uint32_t estFracBitsTrm    ( unsigned bin )                    { return  ( bin ? 0x3bfbb : 0x0010c ); }
  BinFracBits     getFracBitsArray  ()                            const { return    m_BinFracBits_128 [m_State]; }
public:
  uint8_t         state             ()                            const { return  ( m_State >> 1 ); }
  uint8_t         mps               ()                            const { return  ( m_State  & 1 ); }
  uint8_t         getLPS            ( unsigned range )            const { return    m_LPSTable_64_4   [m_State>>1][(range>>6)&3]; }
  static uint8_t  getRenormBitsLPS  ( unsigned LPS )                    { return    m_RenormTable_32  [LPS>>3]; }
  static uint8_t  getRenormBitsRange( unsigned range )                  { return    1; }
  uint16_t        getState          ()                            const { return    uint16_t(m_State); }
  void            setState          ( uint16_t pState )                 { m_State = uint8_t ( pState); }
public:
  uint64_t        estFracExcessBits ( const BinProbModel_Std& r ) const
  {
    return ( ((uint64_t)m_EstFracProb[m_State^0]) * m_EstFracBits[r.m_State^0]
        +    ((uint64_t)m_EstFracProb[m_State^1]) * m_EstFracBits[r.m_State^1] + ( 1 << ( SCALE_BITS - 1 ) ) ) >> SCALE_BITS;
  }
private:
  uint8_t   m_State;
};






class CtxSet
{
public:
  CtxSet( uint16_t offset, uint16_t size ) : Offset( offset ), Size( size ) {}
  CtxSet( const CtxSet& ctxSet ) : Offset( ctxSet.Offset ), Size( ctxSet.Size ) {}
  CtxSet( std::initializer_list<CtxSet> ctxSets );
public:
  uint16_t  operator()  ()  const
  {
    return Offset;
  }
  uint16_t  operator()  ( uint16_t inc )  const
  {
    CHECKD( inc >= Size, "Specified context increment (" << inc << ") exceed range of context set [0;" << Size - 1 << "]." );
    return Offset + inc;
  }
  bool operator== ( const CtxSet& ctxSet ) const
  {
    return ( Offset == ctxSet.Offset && Size == ctxSet.Size );
  }
  bool operator!= ( const CtxSet& ctxSet ) const
  {
    return ( Offset != ctxSet.Offset || Size != ctxSet.Size );
  }
public:
  uint16_t  Offset;
  uint16_t  Size;
};



class ContextSetCfg
{
public:
  // context sets: specify offset and size
  static const CtxSet   SplitFlag;
  static const CtxSet   BTSplitFlag;
  static const CtxSet   SkipFlag;
  static const CtxSet   MergeFlag;
  static const CtxSet   MergeIdx;
  static const CtxSet   PartSize;
  static const CtxSet   PredMode;
  static const CtxSet   IPredMode       [2];    // [ ChannelType ]
  static const CtxSet   PdpcFlag;
  static const CtxSet   DeltaQP;
  static const CtxSet   InterDir;
  static const CtxSet   RefPic;
#if JVET_K_AFFINE
  static const CtxSet   AffineFlag;
#if JVET_K0337_AFFINE_6PARA
  static const CtxSet   AffineType;
#endif
#endif
  static const CtxSet   Mvd;
  static const CtxSet   TransSubdivFlag;
  static const CtxSet   QtRootCbf;
#if JVET_K0072
  static const CtxSet   QtCbf           [3];    // [ channel ]
#else
  static const CtxSet   QtCbf           [2];    // [ ChannelType ]
#endif
  static const CtxSet   SigCoeffGroup   [4];    // [ ChannelType ]
#if JVET_K0072
#else
  static const CtxSet   SigFlag         [4];    // [ ChannelType ]
#endif
  static const CtxSet   LastX           [2];    // [ ChannelType ]
  static const CtxSet   LastY           [2];    // [ ChannelType ]
#if JVET_K0072
  static const CtxSet   SigFlag         [6];    // [ ChannelType + State ]
  static const CtxSet   ParFlag         [2];    // [ ChannelType ]
  static const CtxSet   GtxFlag         [4];    // [ ChannelType + x ]
#else
  static const CtxSet   GreaterOneFlag  [8];    // [ ContextSet  ]
  static const CtxSet   GreaterTwoFlag;
#endif
  static const CtxSet   MVPIdx;
  static const CtxSet   SaoMergeFlag;
  static const CtxSet   SaoTypeIdx;
  static const CtxSet   TransformSkipFlag;
  static const CtxSet   TransquantBypassFlag;
  static const CtxSet   RdpcmFlag;
  static const CtxSet   RdpcmDir;
#if JVET_K1000_SIMPLIFIED_EMT
  static const CtxSet   EMTTuIndex;
  static const CtxSet   EMTCuFlag;
#endif
  static const CtxSet   CrossCompPred;
  static const CtxSet   ChromaQpAdjFlag;
  static const CtxSet   ChromaQpAdjIdc;
#if JVET_K0357_AMVR
  static const CtxSet   ImvFlag;
#endif
#if JVET_K0371_ALF
  static const CtxSet   ctbAlfFlag;
#endif
  static const unsigned NumberOfContexts;

  // combined sets for less complex copying
  // NOTE: The contained CtxSet's should directly follow each other in the initalization list;
  //       otherwise, you will copy more elements than you want !!!
  static const CtxSet   Sao;

public:
  static const std::vector<uint8_t>&  getInitTable( unsigned initId );
private:
  static std::vector<std::vector<uint8_t> > sm_InitTables;
  static CtxSet addCtxSet( std::initializer_list<std::initializer_list<uint8_t> > initSet2d );
};



class FracBitsAccess
{
public:
  virtual BinFracBits getFracBitsArray( unsigned ctxId ) const = 0;
};



template <class BinProbModel>
class CtxStore : public FracBitsAccess
{
public:
  CtxStore();
  CtxStore( bool dummy );
  CtxStore( const CtxStore<BinProbModel>& ctxStore );
public:
  void copyFrom   ( const CtxStore<BinProbModel>& src )                        { checkInit(); ::memcpy( m_Ctx,               src.m_Ctx,               sizeof( BinProbModel ) * ContextSetCfg::NumberOfContexts ); }
  void copyFrom   ( const CtxStore<BinProbModel>& src, const CtxSet& ctxSet )  { checkInit(); ::memcpy( m_Ctx+ctxSet.Offset, src.m_Ctx+ctxSet.Offset, sizeof( BinProbModel ) * ctxSet.Size ); }
  void init       ( int qp, int initId );
  void setWinSizes( const std::vector<uint8_t>&   log2WindowSizes );
  void loadPStates( const std::vector<uint16_t>&  probStates );
  void savePStates( std::vector<uint16_t>&        probStates )  const;

  const BinProbModel& operator[]      ( unsigned  ctxId  )  const { return m_Ctx[ctxId]; }
  BinProbModel&       operator[]      ( unsigned  ctxId  )        { return m_Ctx[ctxId]; }
  uint32_t            estFracBits     ( unsigned  bin,
                                        unsigned  ctxId  )  const { return m_Ctx[ctxId].estFracBits(bin); }

  BinFracBits         getFracBitsArray( unsigned  ctxId  )  const { return m_Ctx[ctxId].getFracBitsArray(); }

private:
  inline void checkInit() { if( m_Ctx ) return; m_CtxBuffer.resize( ContextSetCfg::NumberOfContexts ); m_Ctx = m_CtxBuffer.data(); }
private:
  std::vector<BinProbModel> m_CtxBuffer;
  BinProbModel*             m_Ctx;
};



class Ctx;
class SubCtx
{
  friend class Ctx;
public:
  SubCtx( const CtxSet& ctxSet, const Ctx& ctx ) : m_CtxSet( ctxSet          ), m_Ctx( ctx          ) {}
  SubCtx( const SubCtx& subCtx )                 : m_CtxSet( subCtx.m_CtxSet ), m_Ctx( subCtx.m_Ctx ) {}
  const SubCtx& operator= ( const SubCtx& ) = delete;
private:
  const CtxSet  m_CtxSet;
  const Ctx&    m_Ctx;
};



class Ctx : public ContextSetCfg
{
public:
  Ctx();
  Ctx( const BinProbModel_Std*    dummy );
  Ctx( const Ctx&                 ctx   );

public:
  const Ctx& operator= ( const Ctx& ctx )
  {
    m_BPMType = ctx.m_BPMType;
    switch( m_BPMType )
    {
    case BPM_Std:   m_CtxStore_Std  .copyFrom( ctx.m_CtxStore_Std   );  break;
    default:        break;
    }
    ::memcpy( m_GRAdaptStats, ctx.m_GRAdaptStats, sizeof( unsigned ) * RExt__GOLOMB_RICE_ADAPTATION_STATISTICS_SETS );
    return *this;
  }

  SubCtx operator= ( SubCtx&& subCtx )
  {
    m_BPMType = subCtx.m_Ctx.m_BPMType;
    switch( m_BPMType )
    {
    case BPM_Std:   m_CtxStore_Std  .copyFrom( subCtx.m_Ctx.m_CtxStore_Std,   subCtx.m_CtxSet );  break;
    default:        break;
    }
    return std::move(subCtx);
  }

  void  init ( int qp, int initId )
  {
    switch( m_BPMType )
    {
    case BPM_Std:   m_CtxStore_Std  .init( qp, initId );  break;
    default:        break;
    }
    for( std::size_t k = 0; k < RExt__GOLOMB_RICE_ADAPTATION_STATISTICS_SETS; k++ )
    {
      m_GRAdaptStats[k] = 0;
    }
  }

  void  loadPStates( const std::vector<uint16_t>& probStates )
  {
    switch( m_BPMType )
    {
    case BPM_Std:   m_CtxStore_Std  .loadPStates( probStates );  break;
    default:        break;
    }
  }

  void  savePStates( std::vector<uint16_t>& probStates ) const
  {
    switch( m_BPMType )
    {
    case BPM_Std:   m_CtxStore_Std  .savePStates( probStates );  break;
    default:        break;
    }
  }

  void  initCtxAndWinSize( unsigned ctxId, const Ctx& ctx, const uint8_t winSize )
  {
    switch( m_BPMType )
    {
    case BPM_Std:
      m_CtxStore_Std  [ctxId] = ctx.m_CtxStore_Std  [ctxId];
      m_CtxStore_Std  [ctxId] . setLog2WindowSize   (winSize);
      break;
    default:
      break;
    }
  }

  const unsigned&     getGRAdaptStats ( unsigned      id )      const { return m_GRAdaptStats[id]; }
  unsigned&           getGRAdaptStats ( unsigned      id )            { return m_GRAdaptStats[id]; }

public:
  unsigned            getBPMType      ()                        const { return m_BPMType; }
  const Ctx&          getCtx          ()                        const { return *this; }
  Ctx&                getCtx          ()                              { return *this; }

  explicit operator   const CtxStore<BinProbModel_Std>  &()     const { return m_CtxStore_Std; }
  explicit operator         CtxStore<BinProbModel_Std>  &()           { return m_CtxStore_Std; }

  const FracBitsAccess&   getFracBitsAcess()  const
  {
    switch( m_BPMType )
    {
    case BPM_Std:   return m_CtxStore_Std;
    default:        THROW("BPMType out of range");
    }
  }

private:
  BPMType                       m_BPMType;
  CtxStore<BinProbModel_Std>    m_CtxStore_Std;
protected:
  unsigned                      m_GRAdaptStats[RExt__GOLOMB_RICE_ADAPTATION_STATISTICS_SETS];
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM

public:
  int64_t cacheId;
  bool    cacheUsed;
#endif
};



typedef dynamic_cache<Ctx> CtxCache;

class TempCtx
{
  TempCtx( const TempCtx& ) = delete;
  const TempCtx& operator=( const TempCtx& ) = delete;
public:
  TempCtx ( CtxCache* cache )                     : m_ctx( *cache->get() ), m_cache( cache ) {}
  TempCtx ( CtxCache* cache, const Ctx& ctx    )  : m_ctx( *cache->get() ), m_cache( cache ) { m_ctx = ctx; }
  TempCtx ( CtxCache* cache, SubCtx&&   subCtx )  : m_ctx( *cache->get() ), m_cache( cache ) { m_ctx = std::forward<SubCtx>(subCtx); }
  ~TempCtx()                                      { m_cache->cache( &m_ctx ); }
  const Ctx& operator=( const Ctx& ctx )          { return ( m_ctx = ctx ); }
  SubCtx     operator=( SubCtx&&   subCtx )       { return m_ctx = std::forward<SubCtx>( subCtx ); }
  operator const Ctx& ()           const          { return m_ctx; }
  operator       Ctx& ()                          { return m_ctx; }
private:
  Ctx&      m_ctx;
  CtxCache* m_cache;
};



class CtxStateBuf
{
public:
  CtxStateBuf () : m_valid(false)                 {}
  ~CtxStateBuf()                                  {}
  inline void reset() { m_valid = false; }
  inline bool getIfValid(Ctx &ctx) const
  {
    if (m_valid)
    {
      ctx.loadPStates(m_states);
      return true;
    }
    return false;
  }
  inline void store(const Ctx &ctx)
  {
    ctx.savePStates(m_states);
    m_valid = true;
  }

private:
  std::vector<uint16_t> m_states;
  bool                  m_valid;
};

class CtxStateArray
{
public:
  CtxStateArray () {}
  ~CtxStateArray() {}
  inline void resetAll()
  {
    for (std::size_t k = 0; k < m_data.size(); k++)
    {
      m_data[k].reset();
    }
  }
  inline void resize(std::size_t reqSize)
  {
    if (m_data.size() < reqSize)
    {
      m_data.resize(reqSize);
    }
  }
  inline bool getIfValid(Ctx &ctx, unsigned id) const
  {
    if (id < m_data.size())
    {
      return m_data[id].getIfValid(ctx);
    }
    return false;
  }
  inline void store(const Ctx &ctx, unsigned id)
  {
    if (id >= m_data.size())
    {
      resize(id + 1);
    }
    m_data[id].store(ctx);
  }

private:
  std::vector<CtxStateBuf> m_data;
};




class CtxWSizeSet
{
public:
  CtxWSizeSet() : m_valid(false), m_changes(false), m_coded(false), m_log2WinSizes() {}
  bool                          isValid()                   const { return m_valid; }
  const std::vector<uint8_t>&   getWinSizeBuffer()          const { return m_log2WinSizes; }
  std::vector<uint8_t>&         getWinSizeBuffer()                { return m_log2WinSizes; }
  int                           getMode()                   const { return ( !m_valid || !m_changes ? 0 : ( m_coded ? 2 : 1 ) ); }
  void                          setInvalid()                      { m_valid = m_changes = m_coded = false; }
  void                          setCoded()                        { m_coded = true; }
  void                          setValidOnly()                    { m_valid = true; }
  void                          setValid( uint8_t defSize )
  {
    m_valid   = true;
    m_changes = false;
    for( std::size_t n = 0; n < m_log2WinSizes.size(); n++ )
    {
      if( m_log2WinSizes[n] && m_log2WinSizes[n] != defSize )
      {
        m_changes = true;
        return;
      }
    }
  }
private:
  bool                  m_valid;
  bool                  m_changes;
  bool                  m_coded;
  std::vector<uint8_t>  m_log2WinSizes;
};



#endif

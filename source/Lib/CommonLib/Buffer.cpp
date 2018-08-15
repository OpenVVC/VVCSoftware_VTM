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

/** \file     Buffer.cpp
 *  \brief    Low-overhead class describing 2D memory layout
 */

#define DONT_UNDEF_SIZE_AWARE_PER_EL_OP

// unit needs to come first due to a forward declaration
#include "Unit.h"
#include "Buffer.h"
#include "InterpolationFilter.h"

#if ENABLE_SIMD_OPT_BUFFER
#ifdef TARGET_SIMD_X86

#include "CommonDefX86.h"

template< typename T >
void addAvgCore( const T* src1, int src1Stride, const T* src2, int src2Stride, T* dest, int dstStride, int width, int height, int rshift, int offset, const ClpRng& clpRng )
{
#define ADD_AVG_CORE_OP( ADDR ) dest[ADDR] = ClipPel( rightShift( ( src1[ADDR] + src2[ADDR] + offset ), rshift ), clpRng )
#define ADD_AVG_CORE_INC    \
  src1 += src1Stride;       \
  src2 += src2Stride;       \
  dest +=  dstStride;       \

  SIZE_AWARE_PER_EL_OP( ADD_AVG_CORE_OP, ADD_AVG_CORE_INC );

#undef ADD_AVG_CORE_OP
#undef ADD_AVG_CORE_INC
}


template<typename T>
void reconstructCore( const T* src1, int src1Stride, const T* src2, int src2Stride, T* dest, int dstStride, int width, int height, const ClpRng& clpRng )
{
#define RECO_CORE_OP( ADDR ) dest[ADDR] = ClipPel( src1[ADDR] + src2[ADDR], clpRng )
#define RECO_CORE_INC     \
  src1 += src1Stride;     \
  src2 += src2Stride;     \
  dest +=  dstStride;     \

  SIZE_AWARE_PER_EL_OP( RECO_CORE_OP, RECO_CORE_INC );

#undef RECO_CORE_OP
#undef RECO_CORE_INC
}


template<typename T>
void linTfCore( const T* src, int srcStride, Pel *dst, int dstStride, int width, int height, int scale, int shift, int offset, const ClpRng& clpRng, bool bClip )
{
#define LINTF_CORE_OP( ADDR ) dst[ADDR] = ( Pel ) bClip ? ClipPel( rightShift( scale * src[ADDR], shift ) + offset, clpRng ) : ( rightShift( scale * src[ADDR], shift ) + offset )
#define LINTF_CORE_INC  \
  src += srcStride;     \
  dst += dstStride;     \

  SIZE_AWARE_PER_EL_OP( LINTF_CORE_OP, LINTF_CORE_INC );

#undef LINTF_CORE_OP
#undef LINTF_CORE_INC
}

PelBufferOps::PelBufferOps()
{
  addAvg4 = addAvgCore<Pel>;
  addAvg8 = addAvgCore<Pel>;

  reco4 = reconstructCore<Pel>;
  reco8 = reconstructCore<Pel>;

  linTf4 = linTfCore<Pel>;
  linTf8 = linTfCore<Pel>;
}

PelBufferOps g_pelBufOP = PelBufferOps();

#endif
#endif


template<>
void AreaBuf<Pel>::addAvg( const AreaBuf<const Pel> &other1, const AreaBuf<const Pel> &other2, const ClpRng& clpRng)
{
  const Pel* src0 = other1.buf;
  const Pel* src2 = other2.buf;
        Pel* dest =        buf;

  const unsigned src1Stride = other1.stride;
  const unsigned src2Stride = other2.stride;
  const unsigned destStride =        stride;
  const int     clipbd      = clpRng.bd;
  const int     shiftNum    = std::max<int>(2, (IF_INTERNAL_PREC - clipbd)) + 1;
  const int     offset      = (1 << (shiftNum - 1)) + 2 * IF_INTERNAL_OFFS;

#if ENABLE_SIMD_OPT_BUFFER && defined(TARGET_SIMD_X86)
  if( ( width & 7 ) == 0 )
  {
    g_pelBufOP.addAvg8( src0, src1Stride, src2, src2Stride, dest, destStride, width, height, shiftNum, offset, clpRng );
  }
  else if( ( width & 3 ) == 0 )
  {
    g_pelBufOP.addAvg4( src0, src1Stride, src2, src2Stride, dest, destStride, width, height, shiftNum, offset, clpRng );
  }
  else
#endif
  {
#define ADD_AVG_OP( ADDR ) dest[ADDR] = ClipPel( rightShift( ( src0[ADDR] + src2[ADDR] + offset ), shiftNum ), clpRng )
#define ADD_AVG_INC     \
    src0 += src1Stride; \
    src2 += src2Stride; \
    dest += destStride; \

    SIZE_AWARE_PER_EL_OP( ADD_AVG_OP, ADD_AVG_INC );

#undef ADD_AVG_OP
#undef ADD_AVG_INC
  }
}

template<>
void AreaBuf<Pel>::toLast( const ClpRng& clpRng )
{
        Pel* src       = buf;
  const uint32_t srcStride = stride;

  const int  clipbd    = clpRng.bd;
  const int  shiftNum  = std::max<int>(2, (IF_INTERNAL_PREC - clipbd));
  const int  offset    = ( 1 << ( shiftNum - 1 ) ) + IF_INTERNAL_OFFS;

  if (width == 1)
  {
    THROW( "Blocks of width = 1 not supported" );
  }
  else if (width&2)
  {
    for ( int y = 0; y < height; y++ )
    {
      for (int x=0 ; x < width; x+=2 )
      {
        src[x + 0] = ClipPel( rightShift( ( src[x + 0] + offset ), shiftNum ), clpRng );
        src[x + 1] = ClipPel( rightShift( ( src[x + 1] + offset ), shiftNum ), clpRng );
      }
      src += srcStride;
    }
  }
  else
  {
    for ( int y = 0; y < height; y++ )
    {
      for (int x=0 ; x < width; x+=4 )
      {
        src[x + 0] = ClipPel( rightShift( ( src[x + 0] + offset ), shiftNum ), clpRng );
        src[x + 1] = ClipPel( rightShift( ( src[x + 1] + offset ), shiftNum ), clpRng );
        src[x + 2] = ClipPel( rightShift( ( src[x + 2] + offset ), shiftNum ), clpRng );
        src[x + 3] = ClipPel( rightShift( ( src[x + 3] + offset ), shiftNum ), clpRng );

      }
      src += srcStride;
    }
  }
}


template<>
void AreaBuf<Pel>::copyClip( const AreaBuf<const Pel> &src, const ClpRng& clpRng )
{
  const Pel* srcp = src.buf;
        Pel* dest =     buf;

  const unsigned srcStride  = src.stride;
  const unsigned destStride = stride;

  if( width == 1 )
  {
    THROW( "Blocks of width = 1 not supported" );
  }
  else
  {
#define RECO_OP( ADDR ) dest[ADDR] = ClipPel( srcp[ADDR], clpRng )
#define RECO_INC        \
    srcp += srcStride;  \
    dest += destStride; \

    SIZE_AWARE_PER_EL_OP( RECO_OP, RECO_INC );

#undef RECO_OP
#undef RECO_INC
  }
}


template<>
void AreaBuf<Pel>::reconstruct( const AreaBuf<const Pel> &pred, const AreaBuf<const Pel> &resi, const ClpRng& clpRng )
{
  const Pel* src1 = pred.buf;
  const Pel* src2 = resi.buf;
        Pel* dest =      buf;

  const unsigned src1Stride = pred.stride;
  const unsigned src2Stride = resi.stride;
  const unsigned destStride =      stride;

#if ENABLE_SIMD_OPT_BUFFER && defined(TARGET_SIMD_X86)
  if( ( width & 7 ) == 0 )
  {
    g_pelBufOP.reco8( src1, src1Stride, src2, src2Stride, dest, destStride, width, height, clpRng );
  }
  else if( ( width & 3 ) == 0 )
  {
    g_pelBufOP.reco4( src1, src1Stride, src2, src2Stride, dest, destStride, width, height, clpRng );
  }
  else
#endif
  {
#define RECO_OP( ADDR ) dest[ADDR] = ClipPel( src1[ADDR] + src2[ADDR], clpRng )
#define RECO_INC        \
    src1 += src1Stride; \
    src2 += src2Stride; \
    dest += destStride; \

    SIZE_AWARE_PER_EL_OP( RECO_OP, RECO_INC );

#undef RECO_OP
#undef RECO_INC
  }
}

template<>
void AreaBuf<Pel>::linearTransform( const int scale, const int shift, const int offset, bool bClip, const ClpRng& clpRng )
{
  const Pel* src = buf;
        Pel* dst = buf;

  if( width == 1 )
  {
    THROW( "Blocks of width = 1 not supported" );
  }
#if ENABLE_SIMD_OPT_BUFFER && defined(TARGET_SIMD_X86)
  else if( ( width & 7 ) == 0 )
  {
    g_pelBufOP.linTf8( src, stride, dst, stride, width, height, scale, shift, offset, clpRng, bClip );
  }
  else if( ( width & 3 ) == 0 )
  {
    g_pelBufOP.linTf4( src, stride, dst, stride, width, height, scale, shift, offset, clpRng, bClip );
  }
#endif
  else
  {
#define LINTF_OP( ADDR ) dst[ADDR] = ( Pel ) bClip ? ClipPel( rightShift( scale * src[ADDR], shift ) + offset, clpRng ) : ( rightShift( scale * src[ADDR], shift ) + offset )
#define LINTF_INC        \
    src += stride;       \
    dst += stride;       \

    SIZE_AWARE_PER_EL_OP( LINTF_OP, LINTF_INC );

#undef RECO_OP
#undef RECO_INC
  }
}

#if ENABLE_SIMD_OPT_BUFFER && defined(TARGET_SIMD_X86)
template<>
void AreaBuf<Pel>::subtract( const Pel val )
{
  ClpRng clpRngDummy;
  linearTransform( 1, 0, -val, false, clpRngDummy );
}
#endif


PelStorage::PelStorage()
{
  for( uint32_t i = 0; i < MAX_NUM_COMPONENT; i++ )
  {
    m_origin[i] = nullptr;
  }
}

PelStorage::~PelStorage()
{
  destroy();
}

void PelStorage::create( const UnitArea &_UnitArea )
{
  create( _UnitArea.chromaFormat, _UnitArea.blocks[0] );
}

void PelStorage::create( const ChromaFormat &_chromaFormat, const Area& _area, const unsigned _maxCUSize, const unsigned _margin, const unsigned _alignment, const bool _scaleChromaMargin )
{
  CHECK( !bufs.empty(), "Trying to re-create an already initialized buffer" );

  chromaFormat = _chromaFormat;

  const uint32_t numCh = getNumberValidComponents( _chromaFormat );

  unsigned extHeight = _area.height;
  unsigned extWidth  = _area.width;

  if( _maxCUSize )
  {
    extHeight = ( ( _area.height + _maxCUSize - 1 ) / _maxCUSize ) * _maxCUSize;
    extWidth  = ( ( _area.width  + _maxCUSize - 1 ) / _maxCUSize ) * _maxCUSize;
  }

  for( uint32_t i = 0; i < numCh; i++ )
  {
    const ComponentID compID = ComponentID( i );
    const unsigned scaleX = ::getComponentScaleX( compID, _chromaFormat );
    const unsigned scaleY = ::getComponentScaleY( compID, _chromaFormat );

    unsigned scaledHeight = extHeight >> scaleY;
    unsigned scaledWidth  = extWidth  >> scaleX;
    unsigned ymargin      = _margin >> (_scaleChromaMargin?scaleY:0);
    unsigned xmargin      = _margin >> (_scaleChromaMargin?scaleX:0);
    unsigned totalWidth   = scaledWidth + 2*xmargin;
    unsigned totalHeight  = scaledHeight +2*ymargin;

    if( _alignment )
    {
      // make sure buffer lines are align
      CHECK( _alignment != MEMORY_ALIGN_DEF_SIZE, "Unsupported alignment" );
      totalWidth = ( ( totalWidth + _alignment - 1 ) / _alignment ) * _alignment;
    }
    uint32_t area = totalWidth * totalHeight;
    CHECK( !area, "Trying to create a buffer with zero area" );

    m_origin[i] = ( Pel* ) xMalloc( Pel, area );
    Pel* topLeft = m_origin[i] + totalWidth * ymargin + xmargin;
    bufs.push_back( PelBuf( topLeft, totalWidth, _area.width >> scaleX, _area.height >> scaleY ) );
  }
}

void PelStorage::createFromBuf( PelUnitBuf buf )
{
  chromaFormat = buf.chromaFormat;

  const uint32_t numCh = ::getNumberValidComponents( chromaFormat );

  bufs.resize(numCh);

  for( uint32_t i = 0; i < numCh; i++ )
  {
    PelBuf cPelBuf = buf.get( ComponentID( i ) );
    bufs[i] = PelBuf( cPelBuf.bufAt( 0, 0 ), cPelBuf.stride, cPelBuf.width, cPelBuf.height );
  }
}

void PelStorage::swap( PelStorage& other )
{
  const uint32_t numCh = ::getNumberValidComponents( chromaFormat );

  for( uint32_t i = 0; i < numCh; i++ )
  {
    // check this otherwise it would turn out to get very weird
    CHECK( chromaFormat                   != other.chromaFormat                  , "Incompatible formats" );
    CHECK( get( ComponentID( i ) )        != other.get( ComponentID( i ) )       , "Incompatible formats" );
    CHECK( get( ComponentID( i ) ).stride != other.get( ComponentID( i ) ).stride, "Incompatible formats" );

    std::swap( bufs[i].buf,    other.bufs[i].buf );
    std::swap( bufs[i].stride, other.bufs[i].stride );
    std::swap( m_origin[i],    other.m_origin[i] );
  }
}

void PelStorage::destroy()
{
  chromaFormat = NUM_CHROMA_FORMAT;
  for( uint32_t i = 0; i < MAX_NUM_COMPONENT; i++ )
  {
    if( m_origin[i] )
    {
      xFree( m_origin[i] );
      m_origin[i] = nullptr;
    }
  }
  bufs.clear();
}

PelBuf PelStorage::getBuf( const ComponentID CompID )
{
  return bufs[CompID];
}

const CPelBuf PelStorage::getBuf( const ComponentID CompID ) const
{
  return bufs[CompID];
}

PelBuf PelStorage::getBuf( const CompArea &blk )
{
  const PelBuf& r = bufs[blk.compID];

  CHECKD( rsAddr( blk.bottomRight(), r.stride ) >= ( ( r.height - 1 ) * r.stride + r.width ), "Trying to access a buf outside of bound!" );

  return PelBuf( r.buf + rsAddr( blk, r.stride ), r.stride, blk );
}

const CPelBuf PelStorage::getBuf( const CompArea &blk ) const
{
  const PelBuf& r = bufs[blk.compID];
  return CPelBuf( r.buf + rsAddr( blk, r.stride ), r.stride, blk );
}

PelUnitBuf PelStorage::getBuf( const UnitArea &unit )
{
  return ( chromaFormat == CHROMA_400 ) ? PelUnitBuf( chromaFormat, getBuf( unit.Y() ) ) : PelUnitBuf( chromaFormat, getBuf( unit.Y() ), getBuf( unit.Cb() ), getBuf( unit.Cr() ) );
}

const CPelUnitBuf PelStorage::getBuf( const UnitArea &unit ) const
{
  return ( chromaFormat == CHROMA_400 ) ? CPelUnitBuf( chromaFormat, getBuf( unit.Y() ) ) : CPelUnitBuf( chromaFormat, getBuf( unit.Y() ), getBuf( unit.Cb() ), getBuf( unit.Cr() ) );
}


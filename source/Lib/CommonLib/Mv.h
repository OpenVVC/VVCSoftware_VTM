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

/** \file     Mv.h
    \brief    motion vector class (header)
*/

#ifndef __MV__
#define __MV__

#include "CommonDef.h"

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// basic motion vector class
class Mv
{
public:
  int   hor;     ///< horizontal component of motion vector
  int   ver;     ///< vertical component of motion vector
#if JVET_K0346 || JVET_K_AFFINE
  bool  highPrec;///< true if the vector is high precision
#endif

  // ------------------------------------------------------------------------------------------------------------------
  // constructors
  // ------------------------------------------------------------------------------------------------------------------

#if JVET_K0346 || JVET_K_AFFINE
  Mv(                                            ) : hor( 0    ), ver( 0    ), highPrec( false     ) {}
  Mv( int iHor, int iVer, bool _highPrec = false ) : hor( iHor ), ver( iVer ), highPrec( _highPrec ) {}
#else
  Mv(                    ) : hor( 0    ), ver( 0    ) {}
  Mv( int iHor, int iVer ) : hor( iHor ), ver( iVer ) {}
#endif

  // ------------------------------------------------------------------------------------------------------------------
  // set
  // ------------------------------------------------------------------------------------------------------------------

  void  set       ( int iHor, int iVer)     { hor = iHor;  ver = iVer; }
  void  setHor    ( int i )                 { hor = i;                 }
  void  setVer    ( int i )                 { ver = i;                 }
  void  setZero   ()                        { hor = ver = 0;           }

  // ------------------------------------------------------------------------------------------------------------------
  // get
  // ------------------------------------------------------------------------------------------------------------------

  int   getHor    () const { return hor;          }
  int   getVer    () const { return ver;          }
  int   getAbsHor () const { return abs( hor );   }
  int   getAbsVer () const { return abs( ver );   }

  // ------------------------------------------------------------------------------------------------------------------
  // operations
  // ------------------------------------------------------------------------------------------------------------------

  const Mv& operator += (const Mv& _rcMv)
  {
#if JVET_K0346 || JVET_K_AFFINE
    if( highPrec == _rcMv.highPrec )
    {
      hor += _rcMv.hor;
      ver += _rcMv.ver;
    }
    else
#endif
    {
      Mv rcMv = _rcMv;

#if JVET_K0346 || JVET_K_AFFINE
      if( highPrec && !rcMv.highPrec ) rcMv.setHighPrec();
      if( !highPrec && rcMv.highPrec )      setHighPrec();
#endif
      hor += rcMv.hor;
      ver += rcMv.ver;
    }
    return  *this;
  }

  const Mv& operator-= (const Mv& _rcMv)
  {
#if JVET_K0346 || JVET_K_AFFINE
    if( highPrec == _rcMv.highPrec )
    {
      hor -= _rcMv.hor;
      ver -= _rcMv.ver;
    }
    else
#endif
    {
      Mv rcMv = _rcMv;

#if JVET_K0346 || JVET_K_AFFINE
      if( highPrec && !rcMv.highPrec ) rcMv.setHighPrec();
      if( !highPrec && rcMv.highPrec )      setHighPrec();
#endif
      hor -= rcMv.hor;
      ver -= rcMv.ver;
    }
    return  *this;
  }


  //! shift right with rounding
  void divideByPowerOf2 (const int i)
  {
#if ME_ENABLE_ROUNDING_OF_MVS
    const int offset = (i == 0) ? 0 : 1 << (i - 1);
    hor += offset;
    ver += offset;
#endif
    hor >>= i;
    ver >>= i;
  }

  const Mv& operator<<= (const int i)
  {
    hor <<= i;
    ver <<= i;
    return  *this;
  }

  const Mv& operator>>= ( const int i )
  {
    hor >>= i;
    ver >>= i;
    return  *this;
  }

  const Mv operator - ( const Mv& rcMv ) const
  {
#if JVET_K0346 || JVET_K_AFFINE
    if( rcMv.highPrec == highPrec )
    {
      return Mv( hor - rcMv.hor, ver - rcMv.ver, highPrec );
    }
    else
    {
      Mv self = *this; self.setHighPrec();
      Mv other = rcMv; other.setHighPrec();

      return self - other;
    }
#else
    return Mv( hor - rcMv.hor, ver - rcMv.ver );
#endif
  }

  const Mv operator + ( const Mv& rcMv ) const
  {
#if JVET_K0346 || JVET_K_AFFINE
    if( rcMv.highPrec == highPrec )
    {
      return Mv( hor + rcMv.hor, ver + rcMv.ver, highPrec );
    }
    else
    {
      Mv self = *this; self.setHighPrec();
      Mv other = rcMv; other.setHighPrec();

      return self + other;
    }
#else
    return Mv( hor + rcMv.hor, ver + rcMv.ver );
#endif
  }

  bool operator== ( const Mv& rcMv ) const
  {
#if JVET_K0346 || JVET_K_AFFINE
    if( rcMv.highPrec == highPrec )
    {
      return ( hor == rcMv.hor && ver == rcMv.ver );
    }
    else if( rcMv.highPrec )
    {
      return ( ( hor << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE ) == rcMv.hor && ( ver << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE ) == rcMv.ver );
    }
    else
    {
      return ( ( rcMv.hor << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE ) == hor && ( rcMv.ver << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE ) == ver );
    }
#else
    return ( hor == rcMv.hor && ver == rcMv.ver );
#endif
  }

  bool operator!= ( const Mv& rcMv ) const
  {
    return !( *this == rcMv );
  }

  const Mv scaleMv( int iScale ) const
  {
    const int mvx = Clip3( -32768, 32767, (iScale * getHor() + 127 + (iScale * getHor() < 0)) >> 8 );
    const int mvy = Clip3( -32768, 32767, (iScale * getVer() + 127 + (iScale * getVer() < 0)) >> 8 );
#if JVET_K0346 || JVET_K_AFFINE
    return Mv( mvx, mvy, highPrec );
#else
    return Mv( mvx, mvy );
#endif
  }

#if JVET_K0346 || JVET_K_AFFINE
  void roundMV2SignalPrecision()
  {
    const bool isHP = highPrec;
    setLowPrec();
    if( isHP ) setHighPrec();
  }

  void setLowPrec()
  {
    if( !highPrec ) return;
    const int nShift  = VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;
    const int nOffset = 1 << ( nShift - 1 );
    hor = hor >= 0 ? ( hor + nOffset ) >> nShift : -( ( -hor + nOffset ) >> nShift );
    ver = ver >= 0 ? ( ver + nOffset ) >> nShift : -( ( -ver + nOffset ) >> nShift );
    highPrec = false;
  }

  void setHighPrec()
  {
    if( highPrec ) return;
    hor = hor >= 0 ? ( hor ) << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE : -( ( -hor ) << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE );
    ver = ver >= 0 ? ( ver ) << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE : -( ( -ver ) << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE );
    highPrec = true;
  }
#endif
};// END CLASS DEFINITION MV

#if JVET_K0357_AMVR
void roundMV( Mv& rcMv, unsigned imvShift );
#endif
void clipMv ( Mv& rcMv, const struct Position& pos, const class SPS& sps );

#if JVET_K_AFFINE_BUG_FIXES
void roundAffineMv( int& mvx, int& mvy, int nShift );
#endif

//! \}

#endif // __MV__

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

/** \file     TrQuant_EMT.cpp
    \brief    transform and quantization class
*/

#include "TrQuant_EMT.h"

#include "Rom.h"

#include <stdlib.h>
#include <math.h>
#include <limits>
#include <memory.h>


// ********************************** DCT-II **********************************

//Fast DCT-II transforms
#if JVET_K1000_SIMPLIFIED_EMT
void fastForwardDCT2_B2(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2)
#else
void fastForwardDCT2_B2(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use)
#endif
{
  int j;
  int E, O;
  TCoeff add = (shift > 0) ? (1 << (shift - 1)) : 0;

#if JVET_K1000_SIMPLIFIED_EMT
  const TMatrixCoeff *iT = g_aiTr2[DCT2][0];
#else
  const TMatrixCoeff *iT = use ? g_aiTr2[DCT2][0] : g_aiT2[TRANSFORM_FORWARD][0];
#endif

  TCoeff *pCoef = dst;
  const int  reducedLine = line - iSkipLine;
  for (j = 0; j<reducedLine; j++)
  {
    /* E and O */
    E = src[0] + src[1];
    O = src[0] - src[1];

    dst[0] = (iT[0] * E + add) >> shift;
    dst[line] = (iT[2] * O + add) >> shift;


    src += 2;
    dst++;
  }
  if (iSkipLine)
  {
    dst = pCoef + reducedLine;
    for (j = 0; j<2; j++)
    {
      memset(dst, 0, sizeof(TCoeff)*iSkipLine);
      dst += line;
    }
  }
}

#if JVET_K1000_SIMPLIFIED_EMT
void fastInverseDCT2_B2(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum)
#else
void fastInverseDCT2_B2(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use, const TCoeff outputMinimum, const TCoeff outputMaximum)
#endif
{
  int j;
  int E, O;
  int add = 1 << (shift - 1);

#if JVET_K1000_SIMPLIFIED_EMT
  const TMatrixCoeff *iT = g_aiTr2[DCT2][0];
#else
  const TMatrixCoeff *iT = use ? g_aiTr2[DCT2][0] : g_aiT2[TRANSFORM_INVERSE][0];
#endif

  const int  reducedLine = line - iSkipLine;
  for (j = 0; j<reducedLine; j++)
  {
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
    E = iT[0] * (src[0] + src[line]);
    O = iT[2] * (src[0] - src[line]);

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
    dst[0] = Clip3(outputMinimum, outputMaximum, (E + add) >> shift);
    dst[1] = Clip3(outputMinimum, outputMaximum, (O + add) >> shift);

    src++;
    dst += 2;
  }
  if (iSkipLine)
  {
    memset(dst, 0, (iSkipLine << 1) * sizeof(TCoeff));
  }

  /*TCoeff add = (shift > 0) ? (1 << (shift - 1)) : 0;

  #define T(a,b)    ( (TCoeff)( g_aiT2[ TRANSFORM_INVERSE ][ a ][ b ] ) * src[ a * line ] )

  for (int j = 0; j < line; j++, src++, dst += 2)
  {
  dst[0] = Clip3(outputMinimum, outputMaximum, (T(0, 0) + T(1, 0) + add) >> shift);
  dst[1] = Clip3(outputMinimum, outputMaximum, (T(0, 1) + T(1, 1) + add) >> shift);
  }

  #undef  T*/
}

/** 4x4 forward transform implemented using partial butterfly structure (1D)
*  \param src   input data (residual)
*  \param dst   output data (transform coefficients)
*  \param shift specifies right shift after 1D transform
*  \param line
*/
#if JVET_K1000_SIMPLIFIED_EMT
void fastForwardDCT2_B4(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2)
#else
void fastForwardDCT2_B4(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use)
#endif
{
  int j;
  TCoeff E[2], O[2];
  TCoeff add = (shift > 0) ? (1 << (shift - 1)) : 0;

#if JVET_K1000_SIMPLIFIED_EMT
  const TMatrixCoeff *iT = g_aiTr4[DCT2][0];
#else
  const TMatrixCoeff *iT = use ? g_aiTr4[DCT2][0] : g_aiT4[TRANSFORM_FORWARD][0];
#endif

  TCoeff *pCoef = dst;
  const int  reducedLine = line - iSkipLine;
  for (j = 0; j<reducedLine; j++)
  {
    /* E and O */
    E[0] = src[0] + src[3];
    O[0] = src[0] - src[3];
    E[1] = src[1] + src[2];
    O[1] = src[1] - src[2];

    dst[0] = (iT[0] * E[0] + iT[1] * E[1] + add) >> shift;
    dst[2 * line] = (iT[8] * E[0] + iT[9] * E[1] + add) >> shift;
    dst[line] = (iT[4] * O[0] + iT[5] * O[1] + add) >> shift;
    dst[3 * line] = (iT[12] * O[0] + iT[13] * O[1] + add) >> shift;

    src += 4;
    dst++;
  }
  if (iSkipLine)
  {
    dst = pCoef + reducedLine;
    for (j = 0; j<4; j++)
    {
      memset(dst, 0, sizeof(TCoeff)*iSkipLine);
      dst += line;
    }
  }
}

/** 4x4 inverse transform implemented using partial butterfly structure (1D)
*  \param src   input data (transform coefficients)
*  \param dst   output data (residual)
*  \param shift specifies right shift after 1D transform
*  \param line
*  \param outputMinimum  minimum for clipping
*  \param outputMaximum  maximum for clipping
*/
#if JVET_K1000_SIMPLIFIED_EMT
void fastInverseDCT2_B4( const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum )
#else
void fastInverseDCT2_B4( const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use, const TCoeff outputMinimum, const TCoeff outputMaximum )
#endif
{
  int j;
  int E[2], O[2];
  int add = 1 << ( shift - 1 );

#if JVET_K1000_SIMPLIFIED_EMT
  const TMatrixCoeff *iT = g_aiTr4[DCT2][0];
#else
  const TMatrixCoeff *iT = use ? g_aiTr4[DCT2][0] : g_aiT4[TRANSFORM_INVERSE][0];
#endif

  const int  reducedLine = line - iSkipLine;
  for( j = 0; j < reducedLine; j++ )
  {
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
    O[0] = iT[1 * 4 + 0] * src[line] + iT[3 * 4 + 0] * src[3 * line];
    O[1] = iT[1 * 4 + 1] * src[line] + iT[3 * 4 + 1] * src[3 * line];
    E[0] = iT[0 * 4 + 0] * src[   0] + iT[2 * 4 + 0] * src[2 * line];
    E[1] = iT[0 * 4 + 1] * src[   0] + iT[2 * 4 + 1] * src[2 * line];

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
    dst[0] = Clip3( outputMinimum, outputMaximum, ( E[0] + O[0] + add ) >> shift );
    dst[1] = Clip3( outputMinimum, outputMaximum, ( E[1] + O[1] + add ) >> shift );
    dst[2] = Clip3( outputMinimum, outputMaximum, ( E[1] - O[1] + add ) >> shift );
    dst[3] = Clip3( outputMinimum, outputMaximum, ( E[0] - O[0] + add ) >> shift );

    src++;
    dst += 4;
  }
  if( iSkipLine )
  {
    memset( dst, 0, ( iSkipLine << 2 ) * sizeof( TCoeff ) );
  }
}


template< int uiTrSize >
#if JVET_K1000_SIMPLIFIED_EMT
inline void _fastInverseMM( const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum, const TMatrixCoeff* iT )
#else
inline void _fastInverseMM( const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use, const TCoeff outputMinimum, const TCoeff outputMaximum, const TMatrixCoeff* iT )
#endif
{
  const int  rnd_factor  = 1 << (shift - 1);
  const int  reducedLine = line - iSkipLine;
  const int  cutoff      = uiTrSize - iSkipLine2;

  for( int i = 0; i<reducedLine; i++ )
  {
    for( int j = 0; j<uiTrSize; j++ )
    {
      int iSum = 0;
      for( int k = 0; k<cutoff; k++)
      {
        iSum += src[k*line + i] * iT[k*uiTrSize + j];
      }
      dst[i*uiTrSize + j] = Clip3(outputMinimum, outputMaximum, (int)(iSum + rnd_factor) >> shift);
    }
  }

  if (iSkipLine)
  {
    memset(dst + (reducedLine*uiTrSize), 0, (iSkipLine*uiTrSize) * sizeof(TCoeff));
  }
}


template< int uiTrSize >
#if JVET_K1000_SIMPLIFIED_EMT
inline void _fastForwardMM( const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TMatrixCoeff* tc )
#else
inline void _fastForwardMM( const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use, const TMatrixCoeff* tc )
#endif
{
  const int  rnd_factor  = 1 << (shift - 1);
  const int  reducedLine = line - iSkipLine;
  const int  cutoff      = uiTrSize - iSkipLine2;
  TCoeff *pCoef;

  for( int i = 0; i<reducedLine; i++ )
  {
    pCoef = dst;
    const TMatrixCoeff* iT = tc;
    for( int j = 0; j<cutoff; j++ )
    {
      int iSum = 0;
      for( int k = 0; k<uiTrSize; k++ )
      {
        iSum += src[k] * iT[k];
      }
      pCoef[i] = (iSum + rnd_factor) >> shift;
      pCoef += line;
      iT += uiTrSize;
    }
    src += uiTrSize;
  }

  if( iSkipLine )
  {
    pCoef = dst + reducedLine;
    for( int j = 0; j<cutoff; j++ )
    {
      memset(pCoef, 0, sizeof(TCoeff) * iSkipLine);
      pCoef += line;
    }
  }

  if( iSkipLine2 )
  {
    pCoef = dst + line*cutoff;
    memset(pCoef, 0, sizeof(TCoeff) * line * iSkipLine2);
  }
}



/** 8x8 forward transform implemented using partial butterfly structure (1D)
*  \param src   input data (residual)
*  \param dst   output data (transform coefficients)
*  \param shift specifies right shift after 1D transform
*  \param line
*/
#if JVET_K1000_SIMPLIFIED_EMT
void fastForwardDCT2_B8( const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2 )
#else
void fastForwardDCT2_B8( const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use )
#endif
{
  int j, k;
  TCoeff E[4], O[4];
  TCoeff EE[2], EO[2];
  TCoeff add = ( shift > 0 ) ? ( 1 << ( shift - 1 ) ) : 0;

#if JVET_K1000_SIMPLIFIED_EMT
  const TMatrixCoeff *iT = g_aiTr8[DCT2][0];
#else
  const TMatrixCoeff *iT = use ? g_aiTr8[DCT2][0] : g_aiT8[TRANSFORM_FORWARD][0];
#endif

  TCoeff *pCoef = dst;
  const int  reducedLine = line - iSkipLine;
  for( j = 0; j < reducedLine; j++ )
  {
    /* E and O*/
    for( k = 0; k < 4; k++ )
    {
      E[k] = src[k] + src[7 - k];
      O[k] = src[k] - src[7 - k];
    }
    /* EE and EO */
    EE[0] = E[0] + E[3];
    EO[0] = E[0] - E[3];
    EE[1] = E[1] + E[2];
    EO[1] = E[1] - E[2];

    dst[0       ] = (iT[ 0] * EE[0] + iT[ 1] * EE[1] + add) >> shift;
    dst[4 * line] = (iT[32] * EE[0] + iT[33] * EE[1] + add) >> shift;
    dst[2 * line] = (iT[16] * EO[0] + iT[17] * EO[1] + add) >> shift;
    dst[6 * line] = (iT[48] * EO[0] + iT[49] * EO[1] + add) >> shift;

    dst[    line] = (iT[ 8] * O[0] + iT[ 9] * O[1] + iT[10] * O[2] + iT[11] * O[3] + add) >> shift;
    dst[3 * line] = (iT[24] * O[0] + iT[25] * O[1] + iT[26] * O[2] + iT[27] * O[3] + add) >> shift;
    dst[5 * line] = (iT[40] * O[0] + iT[41] * O[1] + iT[42] * O[2] + iT[43] * O[3] + add) >> shift;
    dst[7 * line] = (iT[56] * O[0] + iT[57] * O[1] + iT[58] * O[2] + iT[59] * O[3] + add) >> shift;

    src += 8;
    dst++;
  }
  if( iSkipLine )
  {
    dst = pCoef + reducedLine;
    for( j = 0; j < 8; j++ )
    {
      memset( dst, 0, sizeof( TCoeff )*iSkipLine );
      dst += line;
    }
  }
}

/** 8x8 inverse transform implemented using partial butterfly structure (1D)
*  \param src   input data (transform coefficients)
*  \param dst   output data (residual)
*  \param shift specifies right shift after 1D transform
*  \param line
*  \param outputMinimum  minimum for clipping
*  \param outputMaximum  maximum for clipping
*/
#if JVET_K1000_SIMPLIFIED_EMT
void fastInverseDCT2_B8(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum)
#else
void fastInverseDCT2_B8(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use, const TCoeff outputMinimum, const TCoeff outputMaximum)
#endif
{
  int j, k;
  int E[4], O[4];
  int EE[2], EO[2];
  int add = 1 << (shift - 1);

#if JVET_K1000_SIMPLIFIED_EMT
  const TMatrixCoeff *iT = g_aiTr8[DCT2][0];
#else
  const TMatrixCoeff *iT = use ? g_aiTr8[DCT2][0] : g_aiT8[TRANSFORM_INVERSE][0];
#endif

  const int  reducedLine = line - iSkipLine;
  for( j = 0; j < reducedLine; j++ )
  {
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
    for( k = 0; k < 4; k++ )
    {
      O[k] = iT[1 * 8 + k] * src[line] + iT[3 * 8 + k] * src[3 * line] + iT[5 * 8 + k] * src[5 * line] + iT[7 * 8 + k] * src[7 * line];
    }

    EO[0] = iT[2 * 8 + 0] * src[2 * line] + iT[6 * 8 + 0] * src[6 * line];
    EO[1] = iT[2 * 8 + 1] * src[2 * line] + iT[6 * 8 + 1] * src[6 * line];
    EE[0] = iT[0 * 8 + 0] * src[0       ] + iT[4 * 8 + 0] * src[4 * line];
    EE[1] = iT[0 * 8 + 1] * src[0       ] + iT[4 * 8 + 1] * src[4 * line];

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
    E[0] = EE[0] + EO[0];
    E[3] = EE[0] - EO[0];
    E[1] = EE[1] + EO[1];
    E[2] = EE[1] - EO[1];

    for( k = 0; k < 4; k++ )
    {
      dst[k    ] = Clip3( outputMinimum, outputMaximum, ( E[    k] + O[    k] + add ) >> shift );
      dst[k + 4] = Clip3( outputMinimum, outputMaximum, ( E[3 - k] - O[3 - k] + add ) >> shift );
    }
    src++;
    dst += 8;
  }
  if( iSkipLine )
  {
    memset( dst, 0, ( iSkipLine << 3 ) * sizeof( TCoeff ) );
  }
}


/** 16x16 forward transform implemented using partial butterfly structure (1D)
*  \param src   input data (residual)
*  \param dst   output data (transform coefficients)
*  \param shift specifies right shift after 1D transform
*  \param line
*/
#if JVET_K1000_SIMPLIFIED_EMT
void fastForwardDCT2_B16(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2)
#else
void fastForwardDCT2_B16(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use)
#endif
{
  int j, k;
  TCoeff E  [8], O  [8];
  TCoeff EE [4], EO [4];
  TCoeff EEE[2], EEO[2];
  TCoeff add = ( shift > 0 ) ? ( 1 << ( shift - 1 ) ) : 0;

#if JVET_K1000_SIMPLIFIED_EMT
  const TMatrixCoeff *iT = g_aiTr16[DCT2][0];
#else
  const TMatrixCoeff *iT = use ? g_aiTr16[DCT2][0] : g_aiT16[TRANSFORM_FORWARD][0];
#endif

  TCoeff *pCoef = dst;
  const int  reducedLine = line - iSkipLine;
  for( j = 0; j < reducedLine; j++ )
  {
    /* E and O*/
    for( k = 0; k < 8; k++ )
    {
      E[k] = src[k] + src[15 - k];
      O[k] = src[k] - src[15 - k];
    }
    /* EE and EO */
    for( k = 0; k < 4; k++ )
    {
      EE[k] = E[k] + E[7 - k];
      EO[k] = E[k] - E[7 - k];
    }
    /* EEE and EEO */
    EEE[0] = EE[0] + EE[3];
    EEO[0] = EE[0] - EE[3];
    EEE[1] = EE[1] + EE[2];
    EEO[1] = EE[1] - EE[2];

    dst[ 0       ] = ( iT[ 0     ] * EEE[0] + iT[          1] * EEE[1] + add ) >> shift;
    dst[ 8 * line] = ( iT[ 8 * 16] * EEE[0] + iT[ 8 * 16 + 1] * EEE[1] + add ) >> shift;
    dst[ 4 * line] = ( iT[ 4 * 16] * EEO[0] + iT[ 4 * 16 + 1] * EEO[1] + add ) >> shift;
    dst[12 * line] = ( iT[12 * 16] * EEO[0] + iT[12 * 16 + 1] * EEO[1] + add ) >> shift;

    for( k = 2; k < 16; k += 4 )
    {
      dst[k*line] = ( iT[k * 16] * EO[0] + iT[k * 16 + 1] * EO[1] + iT[k * 16 + 2] * EO[2] + iT[k * 16 + 3] * EO[3] + add ) >> shift;
    }

    for( k = 1; k < 16; k += 2 )
    {
      dst[k*line] = ( iT[k * 16    ] * O[0] + iT[k * 16 + 1] * O[1] + iT[k * 16 + 2] * O[2] + iT[k * 16 + 3] * O[3] +
                      iT[k * 16 + 4] * O[4] + iT[k * 16 + 5] * O[5] + iT[k * 16 + 6] * O[6] + iT[k * 16 + 7] * O[7] + add ) >> shift;
    }

    src += 16;
    dst++;

  }
  if( iSkipLine )
  {
    dst = pCoef + reducedLine;
    for( j = 0; j < 16; j++ )
    {
      memset( dst, 0, sizeof( TCoeff )*iSkipLine );
      dst += line;
    }
  }
}

/** 16x16 inverse transform implemented using partial butterfly structure (1D)
*  \param src            input data (transform coefficients)
*  \param dst            output data (residual)
*  \param shift          specifies right shift after 1D transform
*  \param line
*  \param outputMinimum  minimum for clipping
*  \param outputMaximum  maximum for clipping
*/
#if JVET_K1000_SIMPLIFIED_EMT
void fastInverseDCT2_B16( const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum )
#else
void fastInverseDCT2_B16( const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use, const TCoeff outputMinimum, const TCoeff outputMaximum )
#endif
{
  int j, k;
  int E  [8], O  [8];
  int EE [4], EO [4];
  int EEE[2], EEO[2];
  int add = 1 << ( shift - 1 );

#if JVET_K1000_SIMPLIFIED_EMT
  const TMatrixCoeff *iT = g_aiTr16[DCT2][0];
#else
  const TMatrixCoeff *iT = use ? g_aiTr16[DCT2][0] : g_aiT16[TRANSFORM_INVERSE][0];
#endif

  const int  reducedLine = line - iSkipLine;

  for( j = 0; j < reducedLine; j++ )
  {
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
    for( k = 0; k < 8; k++ )
    {
      O[k] = iT[1 * 16 + k] * src[    line] + iT[ 3 * 16 + k] * src[ 3 * line] + iT[ 5 * 16 + k] * src[ 5 * line] + iT[ 7 * 16 + k] * src[ 7 * line] +
        iT[9 * 16 + k] * src[9 * line] + iT[11 * 16 + k] * src[11 * line] + iT[13 * 16 + k] * src[13 * line] + iT[15 * 16 + k] * src[15 * line];
    }
    for( k = 0; k < 4; k++ )
    {
      EO[k] = iT[2 * 16 + k] * src[2 * line] + iT[6 * 16 + k] * src[6 * line] + iT[10 * 16 + k] * src[10 * line] + iT[14 * 16 + k] * src[14 * line];
    }
    EEO[0] = iT[4 * 16    ] * src[4 * line] + iT[12 * 16    ] * src[12 * line];
    EEE[0] = iT[0         ] * src[0       ] + iT[ 8 * 16    ] * src[ 8 * line];
    EEO[1] = iT[4 * 16 + 1] * src[4 * line] + iT[12 * 16 + 1] * src[12 * line];
    EEE[1] = iT[0 * 16 + 1] * src[0       ] + iT[ 8 * 16 + 1] * src[ 8 * line];

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
    for( k = 0; k < 2; k++ )
    {
      EE[k    ] = EEE[    k] + EEO[    k];
      EE[k + 2] = EEE[1 - k] - EEO[1 - k];
    }
    for( k = 0; k < 4; k++ )
    {
      E[k    ] = EE[    k] + EO[    k];
      E[k + 4] = EE[3 - k] - EO[3 - k];
    }
    for( k = 0; k < 8; k++ )
    {
      dst[k    ] = Clip3( outputMinimum, outputMaximum, ( E[    k] + O[    k] + add ) >> shift );
      dst[k + 8] = Clip3( outputMinimum, outputMaximum, ( E[7 - k] - O[7 - k] + add ) >> shift );
    }
    src++;
    dst += 16;
  }
  if( iSkipLine )
  {
    memset( dst, 0, ( iSkipLine << 4 ) * sizeof( TCoeff ) );
  }
}



/** 32x32 forward transform implemented using partial butterfly structure (1D)
*  \param src   input data (residual)
*  \param dst   output data (transform coefficients)
*  \param shift specifies right shift after 1D transform
*  \param line
*/
#if JVET_K1000_SIMPLIFIED_EMT
void fastForwardDCT2_B32( const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2 )
#else
void fastForwardDCT2_B32( const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use )
#endif
{
  int j, k;
  TCoeff E   [16], O   [16];
  TCoeff EE  [ 8], EO  [ 8];
  TCoeff EEE [ 4], EEO [ 4];
  TCoeff EEEE[ 2], EEEO[ 2];
  TCoeff add = ( shift > 0 ) ? ( 1 << ( shift - 1 ) ) : 0;

#if JVET_K1000_SIMPLIFIED_EMT
  const TMatrixCoeff *iT = g_aiTr32[DCT2][0];
#else
  const TMatrixCoeff *iT = use ? g_aiTr32[DCT2][0] : g_aiT32[TRANSFORM_FORWARD][0];
#endif

  TCoeff *pCoef = dst;
  const int  reducedLine = line - iSkipLine;
  for (j = 0; j<reducedLine; j++)
  {
    /* E and O*/
    for (k = 0;k<16;k++)
    {
      E[k] = src[k] + src[31 - k];
      O[k] = src[k] - src[31 - k];
    }
    /* EE and EO */
    for (k = 0;k<8;k++)
    {
      EE[k] = E[k] + E[15 - k];
      EO[k] = E[k] - E[15 - k];
    }
    /* EEE and EEO */
    for (k = 0;k<4;k++)
    {
      EEE[k] = EE[k] + EE[7 - k];
      EEO[k] = EE[k] - EE[7 - k];
    }
    /* EEEE and EEEO */
    EEEE[0] = EEE[0] + EEE[3];
    EEEO[0] = EEE[0] - EEE[3];
    EEEE[1] = EEE[1] + EEE[2];
    EEEO[1] = EEE[1] - EEE[2];

    dst[0] = (iT[0 * 32 + 0] * EEEE[0] + iT[0 * 32 + 1] * EEEE[1] + add) >> shift;
    dst[16 * line] = (iT[16 * 32 + 0] * EEEE[0] + iT[16 * 32 + 1] * EEEE[1] + add) >> shift;
    dst[8 * line] = (iT[8 * 32 + 0] * EEEO[0] + iT[8 * 32 + 1] * EEEO[1] + add) >> shift;
    dst[24 * line] = (iT[24 * 32 + 0] * EEEO[0] + iT[24 * 32 + 1] * EEEO[1] + add) >> shift;
    for (k = 4;k<32;k += 8)
    {
      dst[k*line] = (iT[k * 32 + 0] * EEO[0] + iT[k * 32 + 1] * EEO[1] + iT[k * 32 + 2] * EEO[2] + iT[k * 32 + 3] * EEO[3] + add) >> shift;
    }
    for (k = 2;k<32;k += 4)
    {
      dst[k*line] = (iT[k * 32 + 0] * EO[0] + iT[k * 32 + 1] * EO[1] + iT[k * 32 + 2] * EO[2] + iT[k * 32 + 3] * EO[3] +
                      iT[k * 32 + 4] * EO[4] + iT[k * 32 + 5] * EO[5] + iT[k * 32 + 6] * EO[6] + iT[k * 32 + 7] * EO[7] + add) >> shift;
    }
    for (k = 1;k<32;k += 2)
    {
      dst[k*line] = (iT[k * 32 + 0] * O[0] + iT[k * 32 + 1] * O[1] + iT[k * 32 + 2] * O[2] + iT[k * 32 + 3] * O[3] +
                      iT[k * 32 + 4] * O[4] + iT[k * 32 + 5] * O[5] + iT[k * 32 + 6] * O[6] + iT[k * 32 + 7] * O[7] +
                      iT[k * 32 + 8] * O[8] + iT[k * 32 + 9] * O[9] + iT[k * 32 + 10] * O[10] + iT[k * 32 + 11] * O[11] +
                      iT[k * 32 + 12] * O[12] + iT[k * 32 + 13] * O[13] + iT[k * 32 + 14] * O[14] + iT[k * 32 + 15] * O[15] + add) >> shift;
    }
    src += 32;
    dst++;
  }
  if (iSkipLine)
  {
    dst = pCoef + reducedLine;
    for (j = 0; j<32; j++)
    {
      memset(dst, 0, sizeof(TCoeff)*iSkipLine);
      dst += line;
    }
  }
}

/** 32x32 inverse transform implemented using partial butterfly structure (1D)
*  \param src   input data (transform coefficients)
*  \param dst   output data (residual)
*  \param shift specifies right shift after 1D transform
*  \param line
*  \param outputMinimum  minimum for clipping
*  \param outputMaximum  maximum for clipping
*/
#if JVET_K1000_SIMPLIFIED_EMT
void fastInverseDCT2_B32(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum)
#else
void fastInverseDCT2_B32(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use, const TCoeff outputMinimum, const TCoeff outputMaximum)
#endif
{

  int j, k;
  int E[16], O[16];
  int EE[8], EO[8];
  int EEE[4], EEO[4];
  int EEEE[2], EEEO[2];
  int add = 1 << (shift - 1);

#if JVET_K1000_SIMPLIFIED_EMT
  const TMatrixCoeff *iT = g_aiTr32[DCT2][0];
#else
  const TMatrixCoeff *iT = use ? g_aiTr32[DCT2][0] : g_aiT32[TRANSFORM_INVERSE][0];
#endif

  const int  reducedLine = line - iSkipLine;
  for (j = 0; j<reducedLine; j++)
  {
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
    for (k = 0;k<16;k++)
    {
      O[k] = iT[1 * 32 + k] * src[line] + iT[3 * 32 + k] * src[3 * line] + iT[5 * 32 + k] * src[5 * line] + iT[7 * 32 + k] * src[7 * line] +
        iT[9 * 32 + k] * src[9 * line] + iT[11 * 32 + k] * src[11 * line] + iT[13 * 32 + k] * src[13 * line] + iT[15 * 32 + k] * src[15 * line] +
        iT[17 * 32 + k] * src[17 * line] + iT[19 * 32 + k] * src[19 * line] + iT[21 * 32 + k] * src[21 * line] + iT[23 * 32 + k] * src[23 * line] +
        iT[25 * 32 + k] * src[25 * line] + iT[27 * 32 + k] * src[27 * line] + iT[29 * 32 + k] * src[29 * line] + iT[31 * 32 + k] * src[31 * line];
    }
    for (k = 0;k<8;k++)
    {
      EO[k] = iT[2 * 32 + k] * src[2 * line] + iT[6 * 32 + k] * src[6 * line] + iT[10 * 32 + k] * src[10 * line] + iT[14 * 32 + k] * src[14 * line] +
        iT[18 * 32 + k] * src[18 * line] + iT[22 * 32 + k] * src[22 * line] + iT[26 * 32 + k] * src[26 * line] + iT[30 * 32 + k] * src[30 * line];
    }
    for (k = 0;k<4;k++)
    {
      EEO[k] = iT[4 * 32 + k] * src[4 * line] + iT[12 * 32 + k] * src[12 * line] + iT[20 * 32 + k] * src[20 * line] + iT[28 * 32 + k] * src[28 * line];
    }
    EEEO[0] = iT[8 * 32 + 0] * src[8 * line] + iT[24 * 32 + 0] * src[24 * line];
    EEEO[1] = iT[8 * 32 + 1] * src[8 * line] + iT[24 * 32 + 1] * src[24 * line];
    EEEE[0] = iT[0 * 32 + 0] * src[0] + iT[16 * 32 + 0] * src[16 * line];
    EEEE[1] = iT[0 * 32 + 1] * src[0] + iT[16 * 32 + 1] * src[16 * line];

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
    EEE[0] = EEEE[0] + EEEO[0];
    EEE[3] = EEEE[0] - EEEO[0];
    EEE[1] = EEEE[1] + EEEO[1];
    EEE[2] = EEEE[1] - EEEO[1];
    for (k = 0;k<4;k++)
    {
      EE[k] = EEE[k] + EEO[k];
      EE[k + 4] = EEE[3 - k] - EEO[3 - k];
    }
    for (k = 0;k<8;k++)
    {
      E[k] = EE[k] + EO[k];
      E[k + 8] = EE[7 - k] - EO[7 - k];
    }
    for (k = 0;k<16;k++)
    {
      dst[k] = Clip3(outputMinimum, outputMaximum, (E[k] + O[k] + add) >> shift);
      dst[k + 16] = Clip3(outputMinimum, outputMaximum, (E[15 - k] - O[15 - k] + add) >> shift);
    }
    src++;
    dst += 32;
  }
  if (iSkipLine)
  {
    memset(dst, 0, (iSkipLine << 5) * sizeof(TCoeff));
  }
}

#if JVET_K1000_SIMPLIFIED_EMT
void fastForwardDCT2_B64(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2)
#else
void fastForwardDCT2_B64(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use)
#endif
{
  int rnd_factor = 1 << (shift - 1);

  const int uiTrSize = 64;
#if JVET_K1000_SIMPLIFIED_EMT
  const TMatrixCoeff *iT = g_aiTr64[DCT2][0];
#else
  const TMatrixCoeff *iT = use ? g_aiTr64[DCT2][0] : g_aiT64[0][0];
#endif

  int   j, k;
  TCoeff E[32], O[32];
  TCoeff EE[16], EO[16];
  TCoeff EEE[8], EEO[8];
  TCoeff EEEE[4], EEEO[4];
  TCoeff EEEEE[2], EEEEO[2];
  TCoeff *tmp = dst;

  //bool zo = iSkipLine2 >= 32;
  bool zo = iSkipLine2 != 0;
  for (j = 0; j<line - iSkipLine; j++)
  {
    /* E and O*/
    for (k = 0;k<32;k++)
    {
      E[k] = src[k] + src[63 - k];
      O[k] = src[k] - src[63 - k];
    }
    /* EE and EO */
    for (k = 0;k<16;k++)
    {
      EE[k] = E[k] + E[31 - k];
      EO[k] = E[k] - E[31 - k];
    }
    /* EEE and EEO */
    for (k = 0;k<8;k++)
    {
      EEE[k] = EE[k] + EE[15 - k];
      EEO[k] = EE[k] - EE[15 - k];
    }
    /* EEEE and EEEO */
    for (k = 0;k<4;k++)
    {
      EEEE[k] = EEE[k] + EEE[7 - k];
      EEEO[k] = EEE[k] - EEE[7 - k];
    }
    /* EEEEE and EEEEO */
    EEEEE[0] = EEEE[0] + EEEE[3];
    EEEEO[0] = EEEE[0] - EEEE[3];
    EEEEE[1] = EEEE[1] + EEEE[2];
    EEEEO[1] = EEEE[1] - EEEE[2];

    dst[0] = (iT[0 * 64 + 0] * EEEEE[0] + iT[0 * 64 + 1] * EEEEE[1] + rnd_factor) >> shift;
    dst[16 * line] = (iT[16 * 64 + 0] * EEEEO[0] + iT[16 * 64 + 1] * EEEEO[1] + rnd_factor) >> shift;

    if (!zo)
    {
      dst[32 * line] = (iT[32 * 64 + 0] * EEEEE[0] + iT[32 * 64 + 1] * EEEEE[1] + rnd_factor) >> shift;
      dst[48 * line] = (iT[48 * 64 + 0] * EEEEO[0] + iT[48 * 64 + 1] * EEEEO[1] + rnd_factor) >> shift;
    }
    for (k = 8;k<(zo ? 32 : 64);k += 16)
    {
      dst[k*line] = (iT[k * 64 + 0] * EEEO[0] + iT[k * 64 + 1] * EEEO[1] + iT[k * 64 + 2] * EEEO[2] + iT[k * 64 + 3] * EEEO[3] + rnd_factor) >> shift;
    }
    for (k = 4;k<(zo ? 32 : 64);k += 8)
    {
      dst[k*line] = (iT[k * 64 + 0] * EEO[0] + iT[k * 64 + 1] * EEO[1] + iT[k * 64 + 2] * EEO[2] + iT[k * 64 + 3] * EEO[3] +
                      iT[k * 64 + 4] * EEO[4] + iT[k * 64 + 5] * EEO[5] + iT[k * 64 + 6] * EEO[6] + iT[k * 64 + 7] * EEO[7] + rnd_factor) >> shift;
    }
    for (k = 2;k<(zo ? 32 : 64);k += 4)
    {
      dst[k*line] = (iT[k * 64 + 0] * EO[0] + iT[k * 64 + 1] * EO[1] + iT[k * 64 + 2] * EO[2] + iT[k * 64 + 3] * EO[3] +
                      iT[k * 64 + 4] * EO[4] + iT[k * 64 + 5] * EO[5] + iT[k * 64 + 6] * EO[6] + iT[k * 64 + 7] * EO[7] +
                      iT[k * 64 + 8] * EO[8] + iT[k * 64 + 9] * EO[9] + iT[k * 64 + 10] * EO[10] + iT[k * 64 + 11] * EO[11] +
                      iT[k * 64 + 12] * EO[12] + iT[k * 64 + 13] * EO[13] + iT[k * 64 + 14] * EO[14] + iT[k * 64 + 15] * EO[15] + rnd_factor) >> shift;
    }
    for (k = 1;k<(zo ? 32 : 64);k += 2)
    {
      dst[k*line] = (iT[k * 64 + 0] * O[0] + iT[k * 64 + 1] * O[1] + iT[k * 64 + 2] * O[2] + iT[k * 64 + 3] * O[3] +
                      iT[k * 64 + 4] * O[4] + iT[k * 64 + 5] * O[5] + iT[k * 64 + 6] * O[6] + iT[k * 64 + 7] * O[7] +
                      iT[k * 64 + 8] * O[8] + iT[k * 64 + 9] * O[9] + iT[k * 64 + 10] * O[10] + iT[k * 64 + 11] * O[11] +
                      iT[k * 64 + 12] * O[12] + iT[k * 64 + 13] * O[13] + iT[k * 64 + 14] * O[14] + iT[k * 64 + 15] * O[15] +
                      iT[k * 64 + 16] * O[16] + iT[k * 64 + 17] * O[17] + iT[k * 64 + 18] * O[18] + iT[k * 64 + 19] * O[19] +
                      iT[k * 64 + 20] * O[20] + iT[k * 64 + 21] * O[21] + iT[k * 64 + 22] * O[22] + iT[k * 64 + 23] * O[23] +
                      iT[k * 64 + 24] * O[24] + iT[k * 64 + 25] * O[25] + iT[k * 64 + 26] * O[26] + iT[k * 64 + 27] * O[27] +
                      iT[k * 64 + 28] * O[28] + iT[k * 64 + 29] * O[29] + iT[k * 64 + 30] * O[30] + iT[k * 64 + 31] * O[31] + rnd_factor) >> shift;
    }
    src += uiTrSize;
    dst++;
  }

  const int  reducedLine = line - iSkipLine;
  const int  cutoff = uiTrSize - iSkipLine2;
  if (iSkipLine)
  {
    dst = tmp + reducedLine;
    for (j = 0; j<cutoff; j++)
    {
      memset(dst, 0, sizeof(TCoeff)*iSkipLine);
      dst += line;
    }
  }
  if (iSkipLine2)
  {
    dst = tmp + line*cutoff;
    memset(dst, 0, sizeof(TCoeff)*line*iSkipLine2);
  }
}

#if JVET_K1000_SIMPLIFIED_EMT
void fastInverseDCT2_B64(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum)
#else
void fastInverseDCT2_B64(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use, const TCoeff outputMinimum, const TCoeff outputMaximum)
#endif
{
  int rnd_factor = 1 << (shift - 1);
  const int uiTrSize = 64;
#if JVET_K1000_SIMPLIFIED_EMT
  const TMatrixCoeff *iT = g_aiTr64[DCT2][0];
#else
  const TMatrixCoeff *iT = use ? g_aiTr64[DCT2][0] : g_aiT64[TRANSFORM_INVERSE][0];
#endif

  int    j, k;
  TCoeff E[32], O[32];
  TCoeff EE[16], EO[16];
  TCoeff EEE[8], EEO[8];
  TCoeff EEEE[4], EEEO[4];
  TCoeff EEEEE[2], EEEEO[2];
  bool zo = iSkipLine2 >= 32;
  for (j = 0; j<line - iSkipLine; j++)
  {
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
    for (k = 0;k<32;k++)
    {
      O[k] = iT[1 * 64 + k] * src[line] + iT[3 * 64 + k] * src[3 * line] + iT[5 * 64 + k] * src[5 * line] + iT[7 * 64 + k] * src[7 * line] +
        iT[9 * 64 + k] * src[9 * line] + iT[11 * 64 + k] * src[11 * line] + iT[13 * 64 + k] * src[13 * line] + iT[15 * 64 + k] * src[15 * line] +
        iT[17 * 64 + k] * src[17 * line] + iT[19 * 64 + k] * src[19 * line] + iT[21 * 64 + k] * src[21 * line] + iT[23 * 64 + k] * src[23 * line] +
        iT[25 * 64 + k] * src[25 * line] + iT[27 * 64 + k] * src[27 * line] + iT[29 * 64 + k] * src[29 * line] + iT[31 * 64 + k] * src[31 * line] +
        (zo ? 0 : (
        iT[33 * 64 + k] * src[33 * line] + iT[35 * 64 + k] * src[35 * line] + iT[37 * 64 + k] * src[37 * line] + iT[39 * 64 + k] * src[39 * line] +
        iT[41 * 64 + k] * src[41 * line] + iT[43 * 64 + k] * src[43 * line] + iT[45 * 64 + k] * src[45 * line] + iT[47 * 64 + k] * src[47 * line] +
        iT[49 * 64 + k] * src[49 * line] + iT[51 * 64 + k] * src[51 * line] + iT[53 * 64 + k] * src[53 * line] + iT[55 * 64 + k] * src[55 * line] +
        iT[57 * 64 + k] * src[57 * line] + iT[59 * 64 + k] * src[59 * line] + iT[61 * 64 + k] * src[61 * line] + iT[63 * 64 + k] * src[63 * line]));
    }
    for (k = 0;k<16;k++)
    {
      EO[k] = iT[2 * 64 + k] * src[2 * line] + iT[6 * 64 + k] * src[6 * line] + iT[10 * 64 + k] * src[10 * line] + iT[14 * 64 + k] * src[14 * line] +
        iT[18 * 64 + k] * src[18 * line] + iT[22 * 64 + k] * src[22 * line] + iT[26 * 64 + k] * src[26 * line] + iT[30 * 64 + k] * src[30 * line] +
        (zo ? 0 : (
        iT[34 * 64 + k] * src[34 * line] + iT[38 * 64 + k] * src[38 * line] + iT[42 * 64 + k] * src[42 * line] + iT[46 * 64 + k] * src[46 * line] +
        iT[50 * 64 + k] * src[50 * line] + iT[54 * 64 + k] * src[54 * line] + iT[58 * 64 + k] * src[58 * line] + iT[62 * 64 + k] * src[62 * line]));
    }
    for (k = 0;k<8;k++)
    {
      EEO[k] = iT[4 * 64 + k] * src[4 * line] + iT[12 * 64 + k] * src[12 * line] + iT[20 * 64 + k] * src[20 * line] + iT[28 * 64 + k] * src[28 * line] +
        (zo ? 0 : (
        iT[36 * 64 + k] * src[36 * line] + iT[44 * 64 + k] * src[44 * line] + iT[52 * 64 + k] * src[52 * line] + iT[60 * 64 + k] * src[60 * line]));
    }
    for (k = 0;k<4;k++)
    {
      EEEO[k] = iT[8 * 64 + k] * src[8 * line] + iT[24 * 64 + k] * src[24 * line] + (zo ? 0 : (iT[40 * 64 + k] * src[40 * line] + iT[56 * 64 + k] * src[56 * line]));
    }
    EEEEO[0] = iT[16 * 64 + 0] * src[16 * line] + (zo ? 0 : iT[48 * 64 + 0] * src[48 * line]);
    EEEEO[1] = iT[16 * 64 + 1] * src[16 * line] + (zo ? 0 : iT[48 * 64 + 1] * src[48 * line]);
    EEEEE[0] = iT[0 * 64 + 0] * src[0] + (zo ? 0 : iT[32 * 64 + 0] * src[32 * line]);
    EEEEE[1] = iT[0 * 64 + 1] * src[0] + (zo ? 0 : iT[32 * 64 + 1] * src[32 * line]);

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
    for (k = 0;k<2;k++)
    {
      EEEE[k] = EEEEE[k] + EEEEO[k];
      EEEE[k + 2] = EEEEE[1 - k] - EEEEO[1 - k];
    }
    for (k = 0;k<4;k++)
    {
      EEE[k] = EEEE[k] + EEEO[k];
      EEE[k + 4] = EEEE[3 - k] - EEEO[3 - k];
    }
    for (k = 0;k<8;k++)
    {
      EE[k] = EEE[k] + EEO[k];
      EE[k + 8] = EEE[7 - k] - EEO[7 - k];
    }
    for (k = 0;k<16;k++)
    {
      E[k] = EE[k] + EO[k];
      E[k + 16] = EE[15 - k] - EO[15 - k];
    }
    for (k = 0;k<32;k++)
    {
      dst[k] = Clip3(outputMinimum, outputMaximum, (E[k] + O[k] + rnd_factor) >> shift);
      dst[k + 32] = Clip3(outputMinimum, outputMaximum, (E[31 - k] - O[31 - k] + rnd_factor) >> shift);
    }
    src++;
    dst += uiTrSize;
  }

  memset(dst, 0, uiTrSize*iSkipLine * sizeof(TCoeff));
}

#if !JVET_K1000_SIMPLIFIED_EMT

void fastForwardDCT2_B128(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use)
{
  int    j, k;
  TCoeff E[64], O[64];
  TCoeff EE[32], EO[32];
  TCoeff EEE[16], EEO[16];
  TCoeff EEEE[8], EEEO[8];
  TCoeff EEEEE[4], EEEEO[4];
  TCoeff EEEEEE[2], EEEEEO[2];
  TCoeff add = 1 << (shift - 1);

  const TMatrixCoeff(*iT)[128] = use ? g_aiTr128[DCT2] : g_aiT128[TRANSFORM_FORWARD];

  TCoeff* tmp = dst;
  for (j = 0; j<line - iSkipLine; j++)
  {
    /* E and O*/
    for (k = 0;k< 64;k++)
    {
      E[k] = src[k] + src[127 - k];
      O[k] = src[k] - src[127 - k];
    }
    /* EE and EO */
    for (k = 0;k< 32;k++)
    {
      EE[k] = E[k] + E[63 - k];
      EO[k] = E[k] - E[63 - k];
    }

    /* EEE and EEO */
    for (k = 0;k< 16;k++)
    {
      EEE[k] = EE[k] + EE[31 - k];
      EEO[k] = EE[k] - EE[31 - k];
    }

    /* EEEE and EEEO */
    for (k = 0; k< 8; k++)
    {
      EEEE[k] = EEE[k] + EEE[15 - k];
      EEEO[k] = EEE[k] - EEE[15 - k];
    }

    for (k = 0; k< 4; k++)
    {
      EEEEE[k] = EEEE[k] + EEEE[7 - k];
      EEEEO[k] = EEEE[k] - EEEE[7 - k];
    }

    for (k = 0; k< 2; k++)
    {
      EEEEEE[k] = EEEEE[k] + EEEEE[3 - k];
      EEEEEO[k] = EEEEE[k] - EEEEE[3 - k];
    }

    //0
    dst[0] = (iT[0][0] * EEEEEE[0]
               + iT[0][1] * EEEEEE[1]
               + add) >> shift;
    dst[64 * line] = (iT[64][0] * EEEEEE[0]
                       + iT[64][1] * EEEEEE[1]
                       + add) >> shift;

    //2
    for (k = 32;k<128;k += 64)
    {
      dst[k*line] = (iT[k][0] * EEEEEO[0]
                      + iT[k][1] * EEEEEO[1]
                      + add) >> shift;
    }

    //4
    for (k = 16;k<128;k += 32)
    {
      dst[k*line] =
        (iT[k][0] * EEEEO[0]
          + iT[k][1] * EEEEO[1]
          + iT[k][2] * EEEEO[2]
          + iT[k][3] * EEEEO[3]
          + add) >> shift;
    }

    //8
    for (k = 8;k<128;k += 16)
    {
      dst[k*line] =
        (iT[k][0] * EEEO[0]
          + iT[k][1] * EEEO[1]
          + iT[k][2] * EEEO[2]
          + iT[k][3] * EEEO[3]
          + iT[k][4] * EEEO[4]
          + iT[k][5] * EEEO[5]
          + iT[k][6] * EEEO[6]
          + iT[k][7] * EEEO[7]
          + add) >> shift;
    }

    //16
    for (k = 4;k<128;k += 8)
    {
      dst[k*line] =
        (iT[k][0] * EEO[0]
          + iT[k][1] * EEO[1]
          + iT[k][2] * EEO[2]
          + iT[k][3] * EEO[3]
          + iT[k][4] * EEO[4]
          + iT[k][5] * EEO[5]
          + iT[k][6] * EEO[6]
          + iT[k][7] * EEO[7]
          + iT[k][8] * EEO[8]
          + iT[k][9] * EEO[9]
          + iT[k][10] * EEO[10]
          + iT[k][11] * EEO[11]
          + iT[k][12] * EEO[12]
          + iT[k][13] * EEO[13]
          + iT[k][14] * EEO[14]
          + iT[k][15] * EEO[15]
          + add) >> shift;
    }


    //32
    for (k = 2;k<128;k += 4)
    {
      dst[k*line] = (iT[k][0] * EO[0]
                      + iT[k][1] * EO[1]
                      + iT[k][2] * EO[2]
                      + iT[k][3] * EO[3]
                      + iT[k][4] * EO[4]
                      + iT[k][5] * EO[5]
                      + iT[k][6] * EO[6]
                      + iT[k][7] * EO[7]
                      + iT[k][8] * EO[8]
                      + iT[k][9] * EO[9]
                      + iT[k][10] * EO[10]
                      + iT[k][11] * EO[11]
                      + iT[k][12] * EO[12]
                      + iT[k][13] * EO[13]
                      + iT[k][14] * EO[14]
                      + iT[k][15] * EO[15]
                      + iT[k][16] * EO[16]
                      + iT[k][17] * EO[17]
                      + iT[k][18] * EO[18]
                      + iT[k][19] * EO[19]
                      + iT[k][20] * EO[20]
                      + iT[k][21] * EO[21]
                      + iT[k][22] * EO[22]
                      + iT[k][23] * EO[23]
                      + iT[k][24] * EO[24]
                      + iT[k][25] * EO[25]
                      + iT[k][26] * EO[26]
                      + iT[k][27] * EO[27]
                      + iT[k][28] * EO[28]
                      + iT[k][29] * EO[29]
                      + iT[k][30] * EO[30]
                      + iT[k][31] * EO[31]
                      + add) >> shift;
    }

    //64
    for (k = 1;k<128;k += 2)
    {
      dst[k*line] = (iT[k][0] * O[0]
                      + iT[k][1] * O[1]
                      + iT[k][2] * O[2]
                      + iT[k][3] * O[3]
                      + iT[k][4] * O[4]
                      + iT[k][5] * O[5]
                      + iT[k][6] * O[6]
                      + iT[k][7] * O[7]
                      + iT[k][8] * O[8]
                      + iT[k][9] * O[9]
                      + iT[k][10] * O[10]
                      + iT[k][11] * O[11]
                      + iT[k][12] * O[12]
                      + iT[k][13] * O[13]
                      + iT[k][14] * O[14]
                      + iT[k][15] * O[15]
                      + iT[k][16] * O[16]
                      + iT[k][17] * O[17]
                      + iT[k][18] * O[18]
                      + iT[k][19] * O[19]
                      + iT[k][20] * O[20]
                      + iT[k][21] * O[21]
                      + iT[k][22] * O[22]
                      + iT[k][23] * O[23]
                      + iT[k][24] * O[24]
                      + iT[k][25] * O[25]
                      + iT[k][26] * O[26]
                      + iT[k][27] * O[27]
                      + iT[k][28] * O[28]
                      + iT[k][29] * O[29]
                      + iT[k][30] * O[30]
                      + iT[k][31] * O[31]

                      + iT[k][32] * O[32]
                      + iT[k][33] * O[33]
                      + iT[k][34] * O[34]
                      + iT[k][35] * O[35]
                      + iT[k][36] * O[36]
                      + iT[k][37] * O[37]
                      + iT[k][38] * O[38]
                      + iT[k][39] * O[39]
                      + iT[k][40] * O[40]
                      + iT[k][41] * O[41]
                      + iT[k][42] * O[42]
                      + iT[k][43] * O[43]
                      + iT[k][44] * O[44]
                      + iT[k][45] * O[45]
                      + iT[k][46] * O[46]
                      + iT[k][47] * O[47]
                      + iT[k][48] * O[48]
                      + iT[k][49] * O[49]
                      + iT[k][50] * O[50]
                      + iT[k][51] * O[51]
                      + iT[k][52] * O[52]
                      + iT[k][53] * O[53]
                      + iT[k][54] * O[54]
                      + iT[k][55] * O[55]
                      + iT[k][56] * O[56]
                      + iT[k][57] * O[57]
                      + iT[k][58] * O[58]
                      + iT[k][59] * O[59]
                      + iT[k][60] * O[60]
                      + iT[k][61] * O[61]
                      + iT[k][62] * O[62]
                      + iT[k][63] * O[63]
                      + add) >> shift;
    }
    src += 128;
    dst++;
  }

  const uint32_t uiTrSize = 128;
  const int  reducedLine = line - iSkipLine;
  const int  cutoff = uiTrSize - iSkipLine2;
  if (iSkipLine)
  {
    dst = tmp + reducedLine;
    for (j = 0; j<cutoff; j++)
    {
      memset(dst, 0, sizeof(TCoeff)*iSkipLine);
      dst += line;
    }
  }
  if (iSkipLine2)
  {
    dst = tmp + line*cutoff;
    memset(dst, 0, sizeof(TCoeff)*line*iSkipLine2);
  }
}

void fastInverseDCT2_B128(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  int    j, k;
  TCoeff E[64], O[64];
  TCoeff EE[32], EO[32];
  TCoeff EEE[16], EEO[16];
  TCoeff EEEE[8], EEEO[8];
  TCoeff EEEEE[4], EEEEO[4];
  TCoeff EEEEEE[2], EEEEEO[2];
  TCoeff add = 1 << (shift - 1);

  const TMatrixCoeff(*iT)[128] = use ? g_aiTr128[DCT2] : g_aiT128[TRANSFORM_INVERSE];

  bool c1 = iSkipLine2 >= 96;
  bool c2 = iSkipLine2 >= 64;
  bool c3 = iSkipLine2 >= 32;

  for (j = 0; j<line - iSkipLine; j++)
  {
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
    if (c1)
    {
      for (k = 0;k<64;k++) //+2
      {
        O[k] = iT[1][k] * src[line]
          + iT[3][k] * src[3 * line]
          + iT[5][k] * src[5 * line]
          + iT[7][k] * src[7 * line]
          + iT[9][k] * src[9 * line]
          + iT[11][k] * src[11 * line]
          + iT[13][k] * src[13 * line]
          + iT[15][k] * src[15 * line]
          + iT[17][k] * src[17 * line]
          + iT[19][k] * src[19 * line]
          + iT[21][k] * src[21 * line]
          + iT[23][k] * src[23 * line]
          + iT[25][k] * src[25 * line]
          + iT[27][k] * src[27 * line]
          + iT[29][k] * src[29 * line]
          + iT[31][k] * src[31 * line]
          ;
      }

      for (k = 0;k<32;k++) //+4
      {
        EO[k] = iT[2][k] * src[2 * line]
          + iT[6][k] * src[6 * line]
          + iT[10][k] * src[10 * line]
          + iT[14][k] * src[14 * line]
          + iT[18][k] * src[18 * line]
          + iT[22][k] * src[22 * line]
          + iT[26][k] * src[26 * line]
          + iT[30][k] * src[30 * line]
          ;
      }
    }
    else if (c2)
    {
      for (k = 0;k<64;k++) //+2
      {
        O[k] = iT[1][k] * src[line]
          + iT[3][k] * src[3 * line]
          + iT[5][k] * src[5 * line]
          + iT[7][k] * src[7 * line]
          + iT[9][k] * src[9 * line]
          + iT[11][k] * src[11 * line]
          + iT[13][k] * src[13 * line]
          + iT[15][k] * src[15 * line]
          + iT[17][k] * src[17 * line]
          + iT[19][k] * src[19 * line]
          + iT[21][k] * src[21 * line]
          + iT[23][k] * src[23 * line]
          + iT[25][k] * src[25 * line]
          + iT[27][k] * src[27 * line]
          + iT[29][k] * src[29 * line]
          + iT[31][k] * src[31 * line]
          + iT[33][k] * src[33 * line]
          + iT[35][k] * src[35 * line]
          + iT[37][k] * src[37 * line]
          + iT[39][k] * src[39 * line]
          + iT[41][k] * src[41 * line]
          + iT[43][k] * src[43 * line]
          + iT[45][k] * src[45 * line]
          + iT[47][k] * src[47 * line]
          + iT[49][k] * src[49 * line]
          + iT[51][k] * src[51 * line]
          + iT[53][k] * src[53 * line]
          + iT[55][k] * src[55 * line]
          + iT[57][k] * src[57 * line]
          + iT[59][k] * src[59 * line]
          + iT[61][k] * src[61 * line]
          + iT[63][k] * src[63 * line]
          ;
      }

      for (k = 0;k<32;k++) //+4
      {
        EO[k] = iT[2][k] * src[2 * line]
          + iT[6][k] * src[6 * line]
          + iT[10][k] * src[10 * line]
          + iT[14][k] * src[14 * line]
          + iT[18][k] * src[18 * line]
          + iT[22][k] * src[22 * line]
          + iT[26][k] * src[26 * line]
          + iT[30][k] * src[30 * line]
          + iT[34][k] * src[34 * line]
          + iT[38][k] * src[38 * line]
          + iT[42][k] * src[42 * line]
          + iT[46][k] * src[46 * line]
          + iT[50][k] * src[50 * line]
          + iT[54][k] * src[54 * line]
          + iT[58][k] * src[58 * line]
          + iT[62][k] * src[62 * line]
          ;
      }
    }
    else if (c3)
    {
      for (k = 0;k<64;k++) //+2
      {
        O[k] = iT[1][k] * src[line]
          + iT[3][k] * src[3 * line]
          + iT[5][k] * src[5 * line]
          + iT[7][k] * src[7 * line]
          + iT[9][k] * src[9 * line]
          + iT[11][k] * src[11 * line]
          + iT[13][k] * src[13 * line]
          + iT[15][k] * src[15 * line]
          + iT[17][k] * src[17 * line]
          + iT[19][k] * src[19 * line]
          + iT[21][k] * src[21 * line]
          + iT[23][k] * src[23 * line]
          + iT[25][k] * src[25 * line]
          + iT[27][k] * src[27 * line]
          + iT[29][k] * src[29 * line]
          + iT[31][k] * src[31 * line]
          + iT[33][k] * src[33 * line]
          + iT[35][k] * src[35 * line]
          + iT[37][k] * src[37 * line]
          + iT[39][k] * src[39 * line]
          + iT[41][k] * src[41 * line]
          + iT[43][k] * src[43 * line]
          + iT[45][k] * src[45 * line]
          + iT[47][k] * src[47 * line]
          + iT[49][k] * src[49 * line]
          + iT[51][k] * src[51 * line]
          + iT[53][k] * src[53 * line]
          + iT[55][k] * src[55 * line]
          + iT[57][k] * src[57 * line]
          + iT[59][k] * src[59 * line]
          + iT[61][k] * src[61 * line]
          + iT[63][k] * src[63 * line]
          + iT[65][k] * src[65 * line]
          + iT[67][k] * src[67 * line]
          + iT[69][k] * src[69 * line]
          + iT[71][k] * src[71 * line]
          + iT[73][k] * src[73 * line]
          + iT[75][k] * src[75 * line]
          + iT[77][k] * src[77 * line]
          + iT[79][k] * src[79 * line]
          + iT[81][k] * src[81 * line]
          + iT[83][k] * src[83 * line]
          + iT[85][k] * src[85 * line]
          + iT[87][k] * src[87 * line]
          + iT[89][k] * src[89 * line]
          + iT[91][k] * src[91 * line]
          + iT[93][k] * src[93 * line]
          + iT[95][k] * src[95 * line]
          ;
      }

      for (k = 0;k<32;k++) //+4
      {
        EO[k] = iT[2][k] * src[2 * line]
          + iT[6][k] * src[6 * line]
          + iT[10][k] * src[10 * line]
          + iT[14][k] * src[14 * line]
          + iT[18][k] * src[18 * line]
          + iT[22][k] * src[22 * line]
          + iT[26][k] * src[26 * line]
          + iT[30][k] * src[30 * line]
          + iT[34][k] * src[34 * line]
          + iT[38][k] * src[38 * line]
          + iT[42][k] * src[42 * line]
          + iT[46][k] * src[46 * line]
          + iT[50][k] * src[50 * line]
          + iT[54][k] * src[54 * line]
          + iT[58][k] * src[58 * line]
          + iT[62][k] * src[62 * line]
          + iT[66][k] * src[66 * line]
          + iT[70][k] * src[70 * line]
          + iT[74][k] * src[74 * line]
          + iT[78][k] * src[78 * line]
          + iT[82][k] * src[82 * line]
          + iT[86][k] * src[86 * line]
          + iT[90][k] * src[90 * line]
          + iT[94][k] * src[94 * line]
          ;
      }
    }
    else
    {
      for (k = 0;k<64;k++) //+2
      {
        O[k] =
          iT[1][k] * src[line]
          + iT[3][k] * src[3 * line]
          + iT[5][k] * src[5 * line]
          + iT[7][k] * src[7 * line]
          + iT[9][k] * src[9 * line]
          + iT[11][k] * src[11 * line]
          + iT[13][k] * src[13 * line]
          + iT[15][k] * src[15 * line]
          + iT[17][k] * src[17 * line]
          + iT[19][k] * src[19 * line]
          + iT[21][k] * src[21 * line]
          + iT[23][k] * src[23 * line]
          + iT[25][k] * src[25 * line]
          + iT[27][k] * src[27 * line]
          + iT[29][k] * src[29 * line]
          + iT[31][k] * src[31 * line]
          + iT[33][k] * src[33 * line]
          + iT[35][k] * src[35 * line]
          + iT[37][k] * src[37 * line]
          + iT[39][k] * src[39 * line]
          + iT[41][k] * src[41 * line]
          + iT[43][k] * src[43 * line]
          + iT[45][k] * src[45 * line]
          + iT[47][k] * src[47 * line]
          + iT[49][k] * src[49 * line]
          + iT[51][k] * src[51 * line]
          + iT[53][k] * src[53 * line]
          + iT[55][k] * src[55 * line]
          + iT[57][k] * src[57 * line]
          + iT[59][k] * src[59 * line]
          + iT[61][k] * src[61 * line]
          + iT[63][k] * src[63 * line]
          + iT[65][k] * src[65 * line]
          + iT[67][k] * src[67 * line]
          + iT[69][k] * src[69 * line]
          + iT[71][k] * src[71 * line]
          + iT[73][k] * src[73 * line]
          + iT[75][k] * src[75 * line]
          + iT[77][k] * src[77 * line]
          + iT[79][k] * src[79 * line]
          + iT[81][k] * src[81 * line]
          + iT[83][k] * src[83 * line]
          + iT[85][k] * src[85 * line]
          + iT[87][k] * src[87 * line]
          + iT[89][k] * src[89 * line]
          + iT[91][k] * src[91 * line]
          + iT[93][k] * src[93 * line]
          + iT[95][k] * src[95 * line]
          + iT[97][k] * src[97 * line]
          + iT[99][k] * src[99 * line]
          + iT[101][k] * src[101 * line]
          + iT[103][k] * src[103 * line]
          + iT[105][k] * src[105 * line]
          + iT[107][k] * src[107 * line]
          + iT[109][k] * src[109 * line]
          + iT[111][k] * src[111 * line]
          + iT[113][k] * src[113 * line]
          + iT[115][k] * src[115 * line]
          + iT[117][k] * src[117 * line]
          + iT[119][k] * src[119 * line]
          + iT[121][k] * src[121 * line]
          + iT[123][k] * src[123 * line]
          + iT[125][k] * src[125 * line]
          + iT[127][k] * src[127 * line]
          ;
      }

      for (k = 0;k<32;k++) //+4
      {
        EO[k] = iT[2][k] * src[2 * line]
          + iT[6][k] * src[6 * line]
          + iT[10][k] * src[10 * line]
          + iT[14][k] * src[14 * line]
          + iT[18][k] * src[18 * line]
          + iT[22][k] * src[22 * line]
          + iT[26][k] * src[26 * line]
          + iT[30][k] * src[30 * line]
          + iT[34][k] * src[34 * line]
          + iT[38][k] * src[38 * line]
          + iT[42][k] * src[42 * line]
          + iT[46][k] * src[46 * line]
          + iT[50][k] * src[50 * line]
          + iT[54][k] * src[54 * line]
          + iT[58][k] * src[58 * line]
          + iT[62][k] * src[62 * line]
          + iT[66][k] * src[66 * line]
          + iT[70][k] * src[70 * line]
          + iT[74][k] * src[74 * line]
          + iT[78][k] * src[78 * line]
          + iT[82][k] * src[82 * line]
          + iT[86][k] * src[86 * line]
          + iT[90][k] * src[90 * line]
          + iT[94][k] * src[94 * line]
          + iT[98][k] * src[98 * line]
          + iT[102][k] * src[102 * line]
          + iT[106][k] * src[106 * line]
          + iT[110][k] * src[110 * line]
          + iT[114][k] * src[114 * line]
          + iT[118][k] * src[118 * line]
          + iT[122][k] * src[122 * line]
          + iT[126][k] * src[126 * line]
          ;
      }
    }

    for (k = 0;k<16;k++) //+8
    {
      EEO[k] = iT[4][k] * src[4 * line]
        + iT[12][k] * src[12 * line]
        + iT[20][k] * src[20 * line]
        + iT[28][k] * src[28 * line]
        + iT[36][k] * src[36 * line]
        + iT[44][k] * src[44 * line]
        + iT[52][k] * src[52 * line]
        + iT[60][k] * src[60 * line]
        + iT[68][k] * src[68 * line]
        + iT[76][k] * src[76 * line]
        + iT[84][k] * src[84 * line]
        + iT[92][k] * src[92 * line]
        + iT[100][k] * src[100 * line]
        + iT[108][k] * src[108 * line]
        + iT[116][k] * src[116 * line]
        + iT[124][k] * src[124 * line]
        ;
    }

    for (k = 0;k<8;k++) //+16
    {
      EEEO[k] = iT[8][k] * src[8 * line]
        + iT[24][k] * src[24 * line]
        + iT[40][k] * src[40 * line]
        + iT[56][k] * src[56 * line]
        + iT[72][k] * src[72 * line]
        + iT[88][k] * src[88 * line]
        + iT[104][k] * src[104 * line]
        + iT[120][k] * src[120 * line]
        ;
    }


    for (k = 0; k< 4; k++) //+32
    {
      EEEEO[k] = iT[16][k] * src[16 * line]
        + iT[48][k] * src[48 * line]
        + iT[80][k] * src[80 * line]
        + iT[112][k] * src[112 * line]
        ;
    }

    for (k = 0; k< 2; k++) //+64
    {
      EEEEEO[k] = iT[32][k] * src[32 * line]
        + iT[96][k] * src[96 * line]
        ;
    }

    EEEEEE[0] = iT[0][0] * src[0] + iT[64][0] * src[64 * line];
    EEEEEE[1] = iT[0][1] * src[0] + iT[64][1] * src[64 * line];

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
    for (k = 0;k<2;k++)
    {
      EEEEE[k] = EEEEEE[k] + EEEEEO[k];
      EEEEE[k + 2] = EEEEEE[1 - k] - EEEEEO[1 - k];
    }

    for (k = 0;k<4;k++)
    {
      EEEE[k] = EEEEE[k] + EEEEO[k];
      EEEE[k + 4] = EEEEE[3 - k] - EEEEO[3 - k];
    }

    for (k = 0;k<8;k++)
    {
      EEE[k] = EEEE[k] + EEEO[k];
      EEE[k + 8] = EEEE[7 - k] - EEEO[7 - k];
    }

    for (k = 0;k<16;k++)
    {
      EE[k] = EEE[k] + EEO[k];
      EE[k + 16] = EEE[15 - k] - EEO[15 - k];
    }

    for (k = 0;k<32;k++)
    {
      E[k] = EE[k] + EO[k];
      E[k + 32] = EE[31 - k] - EO[31 - k];
    }

    for (k = 0;k<64;k++)
    {
      dst[k] = Clip3(outputMinimum, outputMaximum, (E[k] + O[k] + add) >> shift);
      dst[k + 64] = Clip3(outputMinimum, outputMaximum, (E[63 - k] - O[63 - k] + add) >> shift);
    }
    src++;
    dst += 128;
  }

  memset(dst, 0, 128 * iSkipLine * sizeof(TCoeff));
}

#endif // !JVET_K1000_SIMPLIFIED_EMT


// ********************************** DST-VII **********************************
#if JVET_K1000_SIMPLIFIED_EMT
void fastForwardDST7_B4(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2)
#else
void fastForwardDST7_B4(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use)
#endif
{
  int i;
  TCoeff rnd_factor = (shift > 0) ? (1 << (shift - 1)) : 0;

#if HEVC_USE_4x4_DSTVII
  const TMatrixCoeff *iT = use ? g_aiTr4[DST7][0] : g_as_DST_MAT_4[TRANSFORM_FORWARD][0];
#else
  const TMatrixCoeff *iT = g_aiTr4[DST7][0];
#endif

  int c[4];
  TCoeff *pCoeff = dst;
  const int  reducedLine = line - iSkipLine;
  for (i = 0; i<reducedLine; i++)
  {
    // Intermediate Variables
    c[0] = src[0] + src[3];
    c[1] = src[1] + src[3];
    c[2] = src[0] - src[1];
    c[3] = iT[2] * src[2];

    dst[0 * line] = (iT[0] * c[0] + iT[1] * c[1] + c[3] + rnd_factor) >> shift;
    dst[1 * line] = (iT[2] * (src[0] + src[1] - src[3]) + rnd_factor) >> shift;
    dst[2 * line] = (iT[0] * c[2] + iT[1] * c[0] - c[3] + rnd_factor) >> shift;
    dst[3 * line] = (iT[1] * c[2] - iT[0] * c[1] + c[3] + rnd_factor) >> shift;

    src += 4;
    dst++;
  }
  if (iSkipLine)
  {
    dst = pCoeff + reducedLine;
    for (i = 0; i<4; i++)
    {
      memset(dst, 0, sizeof(TCoeff)*iSkipLine);
      dst += line;
    }
  }
}

#if JVET_K1000_SIMPLIFIED_EMT
void fastInverseDST7_B4(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum)
#else
void fastInverseDST7_B4(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use, const TCoeff outputMinimum, const TCoeff outputMaximum)
#endif
{
  int i;
  TCoeff c[4];
  TCoeff rnd_factor = (shift > 0) ? (1 << (shift - 1)) : 0;

#if HEVC_USE_4x4_DSTVII
  const TMatrixCoeff *iT = use ? g_aiTr4[DST7][0] : g_as_DST_MAT_4[TRANSFORM_INVERSE][0];
#else
  const TMatrixCoeff *iT = g_aiTr4[DST7][0];
#endif

  const int  reducedLine = line - iSkipLine;
  for (i = 0; i<reducedLine; i++)
  {
    // Intermediate Variables
    c[0] = src[0 * line] + src[2 * line];
    c[1] = src[2 * line] + src[3 * line];
    c[2] = src[0 * line] - src[3 * line];
    c[3] = iT[2] * src[1 * line];

    dst[0] = Clip3(outputMinimum, outputMaximum, (iT[0] * c[0] + iT[1] * c[1] + c[3] + rnd_factor) >> shift);
    dst[1] = Clip3(outputMinimum, outputMaximum, (iT[1] * c[2] - iT[0] * c[1] + c[3] + rnd_factor) >> shift);
    dst[2] = Clip3(outputMinimum, outputMaximum, (iT[2] * (src[0 * line] - src[2 * line] + src[3 * line]) + rnd_factor) >> shift);
    dst[3] = Clip3(outputMinimum, outputMaximum, (iT[1] * c[0] + iT[0] * c[2] - c[3] + rnd_factor) >> shift);

    dst += 4;
    src++;
  }
  if (iSkipLine)
  {
    memset(dst, 0, (iSkipLine << 2) * sizeof(TCoeff));
  }
}


#if JVET_K1000_SIMPLIFIED_EMT

void fastForwardDST7_B8(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2)
{
  _fastForwardMM< 8 >( src, dst, shift, line, iSkipLine, iSkipLine2, g_aiTr8[DST7][0] );
}

void fastInverseDST7_B8(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  _fastInverseMM< 8 >( src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, g_aiTr8[DST7][0] );
}


void fastForwardDST7_B16(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2)
{
  _fastForwardMM< 16 >( src, dst, shift, line, iSkipLine, iSkipLine2, g_aiTr16[DST7][0] );
}

void fastInverseDST7_B16(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  _fastInverseMM< 16 >( src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, g_aiTr16[DST7][0] );
}


void fastForwardDST7_B32(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2)
{
  _fastForwardMM< 32 >( src, dst, shift, line, iSkipLine, iSkipLine2, g_aiTr32[DST7][0] );
}

void fastInverseDST7_B32(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  _fastInverseMM< 32 >( src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, g_aiTr32[DST7][0] );
}

#endif

// ********************************** DCT-VIII **********************************
#if JVET_K1000_SIMPLIFIED_EMT
void fastForwardDCT8_B4(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2)
#else
void fastForwardDCT8_B4(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use)
#endif
{
  int i;
  int rnd_factor = 1 << (shift - 1);
  const TMatrixCoeff *iT = g_aiTr4[DCT8][0];

  int c[4];
  TCoeff *pCoeff = dst;
  const int  reducedLine = line - iSkipLine;
  for (i = 0; i<reducedLine; i++)
  {
    // Intermediate Variables
    c[0] = src[0] + src[3];
    c[1] = src[2] + src[0];
    c[2] = src[3] - src[2];
    c[3] = iT[1] * src[1];

    dst[0 * line] = (iT[3] * c[0] + iT[2] * c[1] + c[3] + rnd_factor) >> shift;
    dst[1 * line] = (iT[1] * (src[0] - src[2] - src[3]) + rnd_factor) >> shift;
    dst[2 * line] = (iT[3] * c[2] + iT[2] * c[0] - c[3] + rnd_factor) >> shift;
    dst[3 * line] = (iT[3] * c[1] - iT[2] * c[2] - c[3] + rnd_factor) >> shift;

    src += 4;
    dst++;
  }
  if (iSkipLine)
  {
    dst = pCoeff + reducedLine;
    for (i = 0; i<4; i++)
    {
      memset(dst, 0, sizeof(TCoeff)*iSkipLine);
      dst += line;
    }
  }
}

#if JVET_K1000_SIMPLIFIED_EMT
void fastInverseDCT8_B4(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum)
#else
void fastInverseDCT8_B4(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use, const TCoeff outputMinimum, const TCoeff outputMaximum)
#endif
{
  int i;
  int rnd_factor = 1 << (shift - 1);

  const TMatrixCoeff *iT = g_aiTr4[DCT8][0];

  int c[4];
  const int  reducedLine = line - iSkipLine;
  for (i = 0; i<reducedLine; i++)
  {
    // Intermediate Variables
    c[0] = src[0 * line] + src[3 * line];
    c[1] = src[2 * line] + src[0 * line];
    c[2] = src[3 * line] - src[2 * line];
    c[3] = iT[1] * src[1 * line];

    dst[0] = Clip3(outputMinimum, outputMaximum, (iT[3] * c[0] + iT[2] * c[1] + c[3] + rnd_factor) >> shift);
    dst[1] = Clip3(outputMinimum, outputMaximum, (iT[1] * (src[0 * line] - src[2 * line] - src[3 * line]) + rnd_factor) >> shift);
    dst[2] = Clip3(outputMinimum, outputMaximum, (iT[3] * c[2] + iT[2] * c[0] - c[3] + rnd_factor) >> shift);
    dst[3] = Clip3(outputMinimum, outputMaximum, (iT[3] * c[1] - iT[2] * c[2] - c[3] + rnd_factor) >> shift);

    dst += 4;
    src++;
  }
  if (iSkipLine)
  {
    memset(dst, 0, (iSkipLine << 2) * sizeof(TCoeff));
  }
}


#if JVET_K1000_SIMPLIFIED_EMT
void fastForwardDCT8_B8(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2)
{
  _fastForwardMM< 8 >( src, dst, shift, line, iSkipLine, iSkipLine2, g_aiTr8[DCT8][0] );
}

void fastInverseDCT8_B8(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  _fastInverseMM< 8 >( src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, g_aiTr8[DCT8][0] );
}


void fastForwardDCT8_B16(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2)
{
  _fastForwardMM< 16 >( src, dst, shift, line, iSkipLine, iSkipLine2, g_aiTr16[DCT8][0] );
}

void fastInverseDCT8_B16(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  _fastInverseMM< 16 >( src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, g_aiTr16[DCT8][0] );
}


void fastForwardDCT8_B32(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2)
{
  _fastForwardMM< 32 >( src, dst, shift, line, iSkipLine, iSkipLine2, g_aiTr32[DCT8][0] );
}

void fastInverseDCT8_B32(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  _fastInverseMM< 32 >( src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, g_aiTr32[DCT8][0] );
}
#else
void fastForwardDCT8_B8(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use)
{
  _fastForwardMM< 8 >( src, dst, shift, line, iSkipLine, iSkipLine2, use, g_aiTr8[DCT8][0] );
}

void fastInverseDCT8_B8(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  _fastInverseMM< 8 >( src, dst, shift, line, iSkipLine, iSkipLine2, use, outputMinimum, outputMaximum, g_aiTr8[DCT8][0] );
}


void fastForwardDCT8_B16(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use)
{
  _fastForwardMM< 16 >( src, dst, shift, line, iSkipLine, iSkipLine2, use, g_aiTr16[DCT8][0] );
}

void fastInverseDCT8_B16(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  _fastInverseMM< 16 >( src, dst, shift, line, iSkipLine, iSkipLine2, use, outputMinimum, outputMaximum, g_aiTr16[DCT8][0] );
}


void fastForwardDCT8_B32(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use)
{
  _fastForwardMM< 32 >( src, dst, shift, line, iSkipLine, iSkipLine2, use, g_aiTr32[DCT8][0] );
}

void fastInverseDCT8_B32(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  _fastInverseMM< 32 >( src, dst, shift, line, iSkipLine, iSkipLine2, use, outputMinimum, outputMaximum, g_aiTr32[DCT8][0] );
}


void fastForwardDCT8_B64(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use)
{
  _fastForwardMM< 64 >( src, dst, shift, line, iSkipLine, iSkipLine2, use, g_aiTr64[DCT8][0] );
}

void fastInverseDCT8_B64(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  _fastInverseMM< 64 >( src, dst, shift, line, iSkipLine, iSkipLine2, use, outputMinimum, outputMaximum, g_aiTr64[DCT8][0] );
}


void fastForwardDCT8_B128(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use)
{
  _fastForwardMM< 128 >( src, dst, shift, line, iSkipLine, iSkipLine2, use, g_aiTr128[DCT8][0] );
}

void fastInverseDCT8_B128(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  _fastInverseMM< 128 >( src, dst, shift, line, iSkipLine, iSkipLine2, use, outputMinimum, outputMaximum, g_aiTr128[DCT8][0] );
}
#endif // JVET_K1000_SIMPLIFIED_EMT

#if !JVET_K1000_SIMPLIFIED_EMT && !JVET_K1000_SIMPLIFIED_EMT 

// ********************************** DCT-V **********************************
void fastForwardDCT5_B4(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use)
{
  _fastForwardMM< 4 >( src, dst, shift, line, iSkipLine, iSkipLine2, use, g_aiTr4[DCT5][0] );
}

void fastInverseDCT5_B4(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  _fastInverseMM< 4 >( src, dst, shift, line, iSkipLine, iSkipLine2, use, outputMinimum, outputMaximum, g_aiTr4[DCT5][0] );
}


void fastForwardDCT5_B8(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use)
{
  _fastForwardMM< 8 >( src, dst, shift, line, iSkipLine, iSkipLine2, use, g_aiTr8[DCT5][0] );
}

void fastInverseDCT5_B8(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  _fastInverseMM< 8 >( src, dst, shift, line, iSkipLine, iSkipLine2, use, outputMinimum, outputMaximum, g_aiTr8[DCT5][0] );
}


void fastForwardDCT5_B16(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use)
{
  _fastForwardMM< 16 >( src, dst, shift, line, iSkipLine, iSkipLine2, use, g_aiTr16[DCT5][0] );
}

void fastInverseDCT5_B16(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  _fastInverseMM< 16 >( src, dst, shift, line, iSkipLine, iSkipLine2, use, outputMinimum, outputMaximum, g_aiTr16[DCT5][0] );
}


void fastForwardDCT5_B32(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use)
{
  _fastForwardMM< 32 >( src, dst, shift, line, iSkipLine, iSkipLine2, use, g_aiTr32[DCT5][0] );
}

void fastInverseDCT5_B32(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  _fastInverseMM< 32 >( src, dst, shift, line, iSkipLine, iSkipLine2, use, outputMinimum, outputMaximum, g_aiTr32[DCT5][0] );
}


void fastForwardDCT5_B64(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use)
{
  _fastForwardMM< 64 >( src, dst, shift, line, iSkipLine, iSkipLine2, use, g_aiTr64[DCT5][0] );
}

void fastInverseDCT5_B64(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  _fastInverseMM< 64 >( src, dst, shift, line, iSkipLine, iSkipLine2, use, outputMinimum, outputMaximum, g_aiTr64[DCT5][0] );
}


void fastForwardDCT5_B128(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use)
{
  _fastForwardMM< 128 >( src, dst, shift, line, iSkipLine, iSkipLine2, use, g_aiTr128[DCT5][0] );
}

void fastInverseDCT5_B128(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  _fastInverseMM< 128 >( src, dst, shift, line, iSkipLine, iSkipLine2, use, outputMinimum, outputMaximum, g_aiTr128[DCT5][0] );
}

// ********************************** DST-I **********************************
void fastForwardDST1_B4(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use)
{
  int i;
  int rnd_factor = 1 << (shift - 1);

  const TMatrixCoeff *iT = g_aiTr4[DST1][0];

  int E[2], O[2];
  TCoeff *pCoeff = dst;
  const int  reducedLine = line - iSkipLine;
  for (i = 0; i<reducedLine; i++)
  {
    /* E and O */
    E[0] = src[0] + src[3];
    O[0] = src[0] - src[3];
    E[1] = src[1] + src[2];
    O[1] = src[1] - src[2];

    dst[0] = (E[0] * iT[0] + E[1] * iT[1] + rnd_factor) >> shift;
    dst[line] = (O[0] * iT[1] + O[1] * iT[0] + rnd_factor) >> shift;
    dst[2 * line] = (E[0] * iT[1] - E[1] * iT[0] + rnd_factor) >> shift;
    dst[3 * line] = (O[0] * iT[0] - O[1] * iT[1] + rnd_factor) >> shift;

    src += 4;
    dst++;
  }
  if (iSkipLine)
  {
    dst = pCoeff + reducedLine;
    for (i = 0; i<4; i++)
    {
      memset(dst, 0, sizeof(TCoeff)*iSkipLine);
      dst += line;
    }
  }
}

void fastInverseDST1_B4(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  int i;
  int rnd_factor = 1 << (shift - 1);

  const TMatrixCoeff *iT = g_aiTr4[DST1][0];

  int E[2], O[2];
  const int  reducedLine = line - iSkipLine;
  for (i = 0; i<reducedLine; i++)
  {
    /* E and O */
    E[0] = src[0 * line] + src[3 * line];
    O[0] = src[0 * line] - src[3 * line];
    E[1] = src[1 * line] + src[2 * line];
    O[1] = src[1 * line] - src[2 * line];

    dst[0] = Clip3(outputMinimum, outputMaximum, (E[0] * iT[0] + E[1] * iT[1] + rnd_factor) >> shift);
    dst[1] = Clip3(outputMinimum, outputMaximum, (O[0] * iT[1] + O[1] * iT[0] + rnd_factor) >> shift);
    dst[2] = Clip3(outputMinimum, outputMaximum, (E[0] * iT[1] - E[1] * iT[0] + rnd_factor) >> shift);
    dst[3] = Clip3(outputMinimum, outputMaximum, (O[0] * iT[0] - O[1] * iT[1] + rnd_factor) >> shift);

    dst += 4;
    src++;
  }
  if (iSkipLine)
  {
    memset(dst, 0, (iSkipLine << 2) * sizeof(TCoeff));
  }
}


void fastForwardDST1_B8(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use)
{
  _fastForwardMM< 8 >( src, dst, shift, line, iSkipLine, iSkipLine2, use, g_aiTr8[DST1][0] );
}

void fastInverseDST1_B8(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  _fastInverseMM< 8 >( src, dst, shift, line, iSkipLine, iSkipLine2, use, outputMinimum, outputMaximum, g_aiTr8[DST1][0] );
}


void fastForwardDST1_B16(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use)
{
  _fastForwardMM< 16 >( src, dst, shift, line, iSkipLine, iSkipLine2, use, g_aiTr16[DST1][0] );
}

void fastInverseDST1_B16(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  _fastInverseMM< 16 >( src, dst, shift, line, iSkipLine, iSkipLine2, use, outputMinimum, outputMaximum, g_aiTr16[DST1][0] );
}


void fastForwardDST1_B32(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use)
{
  _fastForwardMM< 32 >( src, dst, shift, line, iSkipLine, iSkipLine2, use, g_aiTr32[DST1][0] );
}

void fastInverseDST1_B32(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  _fastInverseMM< 32 >( src, dst, shift, line, iSkipLine, iSkipLine2, use, outputMinimum, outputMaximum, g_aiTr32[DST1][0] );
}


void fastForwardDST1_B64(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use)
{
  _fastForwardMM< 64 >( src, dst, shift, line, iSkipLine, iSkipLine2, use, g_aiTr64[DST1][0] );
}

void fastInverseDST1_B64(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  _fastInverseMM< 64 >( src, dst, shift, line, iSkipLine, iSkipLine2, use, outputMinimum, outputMaximum, g_aiTr64[DST1][0] );
}


void fastForwardDST1_B128(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use)
{
  _fastForwardMM< 128 >( src, dst, shift, line, iSkipLine, iSkipLine2, use, g_aiTr128[DST1][0] );
}

void fastInverseDST1_B128(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, int use, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  _fastInverseMM< 128 >( src, dst, shift, line, iSkipLine, iSkipLine2, use, outputMinimum, outputMaximum, g_aiTr128[DST1][0] );
}


#endif

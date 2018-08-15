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

/** \file     Contexts.cpp
 *  \brief    Classes providing probability descriptions and contexts (also contains context initialization values)
 */

#include "Contexts.h"

#include <algorithm>
#include <cstring>
#include <limits>



const uint8_t ProbModelTables::m_NextState[128][2] =
{
  {   2,  1 },{   0,  3 },{   4,  0 },{   1,  5 },{   6,  2 },{   3,  7 },{   8,  4 },{   5,  9 },
  {  10,  4 },{   5, 11 },{  12,  8 },{   9, 13 },{  14,  8 },{   9, 15 },{  16, 10 },{  11, 17 },
  {  18, 12 },{  13, 19 },{  20, 14 },{  15, 21 },{  22, 16 },{  17, 23 },{  24, 18 },{  19, 25 },
  {  26, 18 },{  19, 27 },{  28, 22 },{  23, 29 },{  30, 22 },{  23, 31 },{  32, 24 },{  25, 33 },
  {  34, 26 },{  27, 35 },{  36, 26 },{  27, 37 },{  38, 30 },{  31, 39 },{  40, 30 },{  31, 41 },
  {  42, 32 },{  33, 43 },{  44, 32 },{  33, 45 },{  46, 36 },{  37, 47 },{  48, 36 },{  37, 49 },
  {  50, 38 },{  39, 51 },{  52, 38 },{  39, 53 },{  54, 42 },{  43, 55 },{  56, 42 },{  43, 57 },
  {  58, 44 },{  45, 59 },{  60, 44 },{  45, 61 },{  62, 46 },{  47, 63 },{  64, 48 },{  49, 65 },
  {  66, 48 },{  49, 67 },{  68, 50 },{  51, 69 },{  70, 52 },{  53, 71 },{  72, 52 },{  53, 73 },
  {  74, 54 },{  55, 75 },{  76, 54 },{  55, 77 },{  78, 56 },{  57, 79 },{  80, 58 },{  59, 81 },
  {  82, 58 },{  59, 83 },{  84, 60 },{  61, 85 },{  86, 60 },{  61, 87 },{  88, 60 },{  61, 89 },
  {  90, 62 },{  63, 91 },{  92, 64 },{  65, 93 },{  94, 64 },{  65, 95 },{  96, 66 },{  67, 97 },
  {  98, 66 },{  67, 99 },{ 100, 66 },{  67,101 },{ 102, 68 },{  69,103 },{ 104, 68 },{  69,105 },
  { 106, 70 },{  71,107 },{ 108, 70 },{  71,109 },{ 110, 70 },{  71,111 },{ 112, 72 },{  73,113 },
  { 114, 72 },{  73,115 },{ 116, 72 },{  73,117 },{ 118, 74 },{  75,119 },{ 120, 74 },{  75,121 },
  { 122, 74 },{  75,123 },{ 124, 76 },{  77,125 },{ 124, 76 },{  77,125 },{ 126,126 },{ 127,127 }
};

const uint32_t ProbModelTables::m_EstFracBits[128] =
{
  0x07b23, 0x085f9, 0x074a0, 0x08cbc, 0x06ee4, 0x09354, 0x067f4, 0x09c1b, 0x060b0, 0x0a62a, 0x05a9c, 0x0af5b, 0x0548d, 0x0b955, 0x04f56, 0x0c2a9,
  0x04a87, 0x0cbf7, 0x045d6, 0x0d5c3, 0x04144, 0x0e01b, 0x03d88, 0x0e937, 0x039e0, 0x0f2cd, 0x03663, 0x0fc9e, 0x03347, 0x10600, 0x03050, 0x10f95,
  0x02d4d, 0x11a02, 0x02ad3, 0x12333, 0x0286e, 0x12cad, 0x02604, 0x136df, 0x02425, 0x13f48, 0x021f4, 0x149c4, 0x0203e, 0x1527b, 0x01e4d, 0x15d00,
  0x01c99, 0x166de, 0x01b18, 0x17017, 0x019a5, 0x17988, 0x01841, 0x18327, 0x016df, 0x18d50, 0x015d9, 0x19547, 0x0147c, 0x1a083, 0x0138e, 0x1a8a3,
  0x01251, 0x1b418, 0x01166, 0x1bd27, 0x01068, 0x1c77b, 0x00f7f, 0x1d18e, 0x00eda, 0x1d91a, 0x00e19, 0x1e254, 0x00d4f, 0x1ec9a, 0x00c90, 0x1f6e0,
  0x00c01, 0x1fef8, 0x00b5f, 0x208b1, 0x00ab6, 0x21362, 0x00a15, 0x21e46, 0x00988, 0x2285d, 0x00934, 0x22ea8, 0x008a8, 0x239b2, 0x0081d, 0x24577,
  0x007c9, 0x24ce6, 0x00763, 0x25663, 0x00710, 0x25e8f, 0x006a0, 0x26a26, 0x00672, 0x26f23, 0x005e8, 0x27ef8, 0x005ba, 0x284b5, 0x0055e, 0x29057,
  0x0050c, 0x29bab, 0x004c1, 0x2a674, 0x004a7, 0x2aa5e, 0x0046f, 0x2b32f, 0x0041f, 0x2c0ad, 0x003e7, 0x2ca8d, 0x003ba, 0x2d323, 0x0010c, 0x3bfbb
};

const BinFracBits ProbModelTables::m_BinFracBits_128[128] =
{
  {{0x07b23, 0x085f9}}, {{0x085f9, 0x07b23}},   {{0x074a0, 0x08cbc}}, {{0x08cbc, 0x074a0}},   {{0x06ee4, 0x09354}}, {{0x09354, 0x06ee4}},   {{0x067f4, 0x09c1b}}, {{0x09c1b, 0x067f4}},
  {{0x060b0, 0x0a62a}}, {{0x0a62a, 0x060b0}},   {{0x05a9c, 0x0af5b}}, {{0x0af5b, 0x05a9c}},   {{0x0548d, 0x0b955}}, {{0x0b955, 0x0548d}},   {{0x04f56, 0x0c2a9}}, {{0x0c2a9, 0x04f56}},
  {{0x04a87, 0x0cbf7}}, {{0x0cbf7, 0x04a87}},   {{0x045d6, 0x0d5c3}}, {{0x0d5c3, 0x045d6}},   {{0x04144, 0x0e01b}}, {{0x0e01b, 0x04144}},   {{0x03d88, 0x0e937}}, {{0x0e937, 0x03d88}},
  {{0x039e0, 0x0f2cd}}, {{0x0f2cd, 0x039e0}},   {{0x03663, 0x0fc9e}}, {{0x0fc9e, 0x03663}},   {{0x03347, 0x10600}}, {{0x10600, 0x03347}},   {{0x03050, 0x10f95}}, {{0x10f95, 0x03050}},
  {{0x02d4d, 0x11a02}}, {{0x11a02, 0x02d4d}},   {{0x02ad3, 0x12333}}, {{0x12333, 0x02ad3}},   {{0x0286e, 0x12cad}}, {{0x12cad, 0x0286e}},   {{0x02604, 0x136df}}, {{0x136df, 0x02604}},
  {{0x02425, 0x13f48}}, {{0x13f48, 0x02425}},   {{0x021f4, 0x149c4}}, {{0x149c4, 0x021f4}},   {{0x0203e, 0x1527b}}, {{0x1527b, 0x0203e}},   {{0x01e4d, 0x15d00}}, {{0x15d00, 0x01e4d}},
  {{0x01c99, 0x166de}}, {{0x166de, 0x01c99}},   {{0x01b18, 0x17017}}, {{0x17017, 0x01b18}},   {{0x019a5, 0x17988}}, {{0x17988, 0x019a5}},   {{0x01841, 0x18327}}, {{0x18327, 0x01841}},
  {{0x016df, 0x18d50}}, {{0x18d50, 0x016df}},   {{0x015d9, 0x19547}}, {{0x19547, 0x015d9}},   {{0x0147c, 0x1a083}}, {{0x1a083, 0x0147c}},   {{0x0138e, 0x1a8a3}}, {{0x1a8a3, 0x0138e}},
  {{0x01251, 0x1b418}}, {{0x1b418, 0x01251}},   {{0x01166, 0x1bd27}}, {{0x1bd27, 0x01166}},   {{0x01068, 0x1c77b}}, {{0x1c77b, 0x01068}},   {{0x00f7f, 0x1d18e}}, {{0x1d18e, 0x00f7f}},
  {{0x00eda, 0x1d91a}}, {{0x1d91a, 0x00eda}},   {{0x00e19, 0x1e254}}, {{0x1e254, 0x00e19}},   {{0x00d4f, 0x1ec9a}}, {{0x1ec9a, 0x00d4f}},   {{0x00c90, 0x1f6e0}}, {{0x1f6e0, 0x00c90}},
  {{0x00c01, 0x1fef8}}, {{0x1fef8, 0x00c01}},   {{0x00b5f, 0x208b1}}, {{0x208b1, 0x00b5f}},   {{0x00ab6, 0x21362}}, {{0x21362, 0x00ab6}},   {{0x00a15, 0x21e46}}, {{0x21e46, 0x00a15}},
  {{0x00988, 0x2285d}}, {{0x2285d, 0x00988}},   {{0x00934, 0x22ea8}}, {{0x22ea8, 0x00934}},   {{0x008a8, 0x239b2}}, {{0x239b2, 0x008a8}},   {{0x0081d, 0x24577}}, {{0x24577, 0x0081d}},
  {{0x007c9, 0x24ce6}}, {{0x24ce6, 0x007c9}},   {{0x00763, 0x25663}}, {{0x25663, 0x00763}},   {{0x00710, 0x25e8f}}, {{0x25e8f, 0x00710}},   {{0x006a0, 0x26a26}}, {{0x26a26, 0x006a0}},
  {{0x00672, 0x26f23}}, {{0x26f23, 0x00672}},   {{0x005e8, 0x27ef8}}, {{0x27ef8, 0x005e8}},   {{0x005ba, 0x284b5}}, {{0x284b5, 0x005ba}},   {{0x0055e, 0x29057}}, {{0x29057, 0x0055e}},
  {{0x0050c, 0x29bab}}, {{0x29bab, 0x0050c}},   {{0x004c1, 0x2a674}}, {{0x2a674, 0x004c1}},   {{0x004a7, 0x2aa5e}}, {{0x2aa5e, 0x004a7}},   {{0x0046f, 0x2b32f}}, {{0x2b32f, 0x0046f}},
  {{0x0041f, 0x2c0ad}}, {{0x2c0ad, 0x0041f}},   {{0x003e7, 0x2ca8d}}, {{0x2ca8d, 0x003e7}},   {{0x003ba, 0x2d323}}, {{0x2d323, 0x003ba}},   {{0x0010c, 0x3bfbb}}, {{0x3bfbb, 0x0010c}}
};

const uint32_t ProbModelTables::m_EstFracProb[128] =
{
  0x041b5, 0x03df6, 0x04410, 0x03bbc, 0x04636, 0x039a3, 0x048e6, 0x036f6, 0x04bd3, 0x0340c, 0x04e5d, 0x03185, 0x050fa, 0x02eeb, 0x0534b, 0x02c9b,
  0x0557e, 0x02a6a, 0x057b1, 0x02839, 0x059e4, 0x02608, 0x05bba, 0x02433, 0x05d8f, 0x0225e, 0x05f58, 0x02097, 0x060f7, 0x01efa, 0x06288, 0x01d69,
  0x06427, 0x01bcb, 0x06581, 0x01a72, 0x066d4, 0x0191f, 0x0682f, 0x017c6, 0x0693e, 0x016b7, 0x06a80, 0x01576, 0x06b7e, 0x01478, 0x06ca1, 0x01356,
  0x06da2, 0x01255, 0x06e88, 0x01170, 0x06f67, 0x01092, 0x0703e, 0x00fba, 0x07116, 0x00ee3, 0x071b7, 0x00e42, 0x0728f, 0x00d6a, 0x07323, 0x00cd6,
  0x073e9, 0x00c11, 0x0747d, 0x00b7d, 0x0751e, 0x00add, 0x075b2, 0x00a49, 0x0761b, 0x009e0, 0x07697, 0x00964, 0x07719, 0x008e2, 0x07794, 0x00867,
  0x077f1, 0x0080b, 0x0785b, 0x007a1, 0x078c9, 0x00733, 0x07932, 0x006ca, 0x0798f, 0x0066d, 0x079c6, 0x00636, 0x07a23, 0x005da, 0x07a7f, 0x0057d,
  0x07ab7, 0x00546, 0x07afb, 0x00502, 0x07b32, 0x004cb, 0x07b7d, 0x00480, 0x07b9c, 0x00461, 0x07bf8, 0x00405, 0x07c17, 0x003e6, 0x07c55, 0x003a9,
  0x07c8c, 0x00371, 0x07cbf, 0x0033f, 0x07cd0, 0x0032e, 0x07cf6, 0x00308, 0x07d2c, 0x002d1, 0x07d52, 0x002ab, 0x07d71, 0x0028c, 0x07f46, 0x000b5
};

const uint8_t ProbModelTables::m_LPSTable_64_4[64][4] =
{
  { 128, 176, 208, 240 },
  { 128, 167, 197, 227 },
  { 128, 158, 187, 216 },
  { 123, 150, 178, 205 },
  { 116, 142, 169, 195 },
  { 111, 135, 160, 185 },
  { 105, 128, 152, 175 },
  { 100, 122, 144, 166 },
  {  95, 116, 137, 158 },
  {  90, 110, 130, 150 },
  {  85, 104, 123, 142 },
  {  81,  99, 117, 135 },
  {  77,  94, 111, 128 },
  {  73,  89, 105, 122 },
  {  69,  85, 100, 116 },
  {  66,  80,  95, 110 },
  {  62,  76,  90, 104 },
  {  59,  72,  86,  99 },
  {  56,  69,  81,  94 },
  {  53,  65,  77,  89 },
  {  51,  62,  73,  85 },
  {  48,  59,  69,  80 },
  {  46,  56,  66,  76 },
  {  43,  53,  63,  72 },
  {  41,  50,  59,  69 },
  {  39,  48,  56,  65 },
  {  37,  45,  54,  62 },
  {  35,  43,  51,  59 },
  {  33,  41,  48,  56 },
  {  32,  39,  46,  53 },
  {  30,  37,  43,  50 },
  {  29,  35,  41,  48 },
  {  27,  33,  39,  45 },
  {  26,  31,  37,  43 },
  {  24,  30,  35,  41 },
  {  23,  28,  33,  39 },
  {  22,  27,  32,  37 },
  {  21,  26,  30,  35 },
  {  20,  24,  29,  33 },
  {  19,  23,  27,  31 },
  {  18,  22,  26,  30 },
  {  17,  21,  25,  28 },
  {  16,  20,  23,  27 },
  {  15,  19,  22,  25 },
  {  14,  18,  21,  24 },
  {  14,  17,  20,  23 },
  {  13,  16,  19,  22 },
  {  12,  15,  18,  21 },
  {  12,  14,  17,  20 },
  {  11,  14,  16,  19 },
  {  11,  13,  15,  18 },
  {  10,  12,  15,  17 },
  {  10,  12,  14,  16 },
  {   9,  11,  13,  15 },
  {   9,  11,  12,  14 },
  {   8,  10,  12,  14 },
  {   8,   9,  11,  13 },
  {   7,   9,  11,  12 },
  {   7,   9,  10,  12 },
  {   7,   8,  10,  11 },
  {   6,   8,   9,  11 },
  {   6,   7,   9,  10 },
  {   6,   7,   8,   9 },
  {   2,   2,   2,   2 }
};

const uint8_t ProbModelTables::m_RenormTable_32[32] =
{
  6,  5,  4,  4,
  3,  3,  3,  3,
  2,  2,  2,  2,
  2,  2,  2,  2,
  1,  1,  1,  1,
  1,  1,  1,  1,
  1,  1,  1,  1,
  1,  1,  1,  1
};





void BinProbModel_Std::init( int qp, int initId )
{
  int slope     = ( ( initId >>  4 )  * 5 ) - 45;
  int offset    = ( ( initId  & 15 ) << 3 ) - 16;
  int inistate  = ( ( slope   * qp ) >> 4 ) + offset;
  if( inistate >= 64 )
  {
    m_State     = ( std::min( 62, inistate - 64 ) << 1 ) + 1;
  }
  else
  {
    m_State     = ( std::min( 62, 63 - inistate ) << 1 );
  }
}




CtxSet::CtxSet( std::initializer_list<CtxSet> ctxSets )
{
  uint16_t  minOffset = std::numeric_limits<uint16_t>::max();
  uint16_t  maxOffset = 0;
  for( auto iter = ctxSets.begin(); iter != ctxSets.end(); iter++ )
  {
    minOffset = std::min<uint16_t>( minOffset, (*iter).Offset              );
    maxOffset = std::max<uint16_t>( maxOffset, (*iter).Offset+(*iter).Size );
  }
  Offset  = minOffset;
  Size    = maxOffset - minOffset;
}





const std::vector<uint8_t>& ContextSetCfg::getInitTable( unsigned initId )
{
  CHECK( initId >= (unsigned)sm_InitTables.size(),
         "Invalid initId (" << initId << "), only " << sm_InitTables.size() << " tables defined." );
  return sm_InitTables[initId];
}


CtxSet ContextSetCfg::addCtxSet( std::initializer_list<std::initializer_list<uint8_t>> initSet2d )
{
  const std::size_t startIdx  = sm_InitTables[0].size();
  const std::size_t numValues = ( *initSet2d.begin() ).size();
        std::size_t setId     = 0;
  for( auto setIter = initSet2d.begin(); setIter != initSet2d.end() && setId < sm_InitTables.size(); setIter++, setId++ )
  {
    const std::initializer_list<uint8_t>& initSet   = *setIter;
    std::vector<uint8_t>&           initTable = sm_InitTables[setId];
    CHECK( initSet.size() != numValues,
           "Number of init values do not match for all sets (" << initSet.size() << " != " << numValues << ")." );
    initTable.resize( startIdx + numValues );
    std::size_t elemId = startIdx;
    for( auto elemIter = ( *setIter ).begin(); elemIter != ( *setIter ).end(); elemIter++, elemId++ )
    {
      initTable[elemId] = *elemIter;
    }
  }
  return CtxSet( (uint16_t)startIdx, (uint16_t)numValues );
}



#define CNU 154 // dummy initialization value for unused context models 'Context model Not Used'
std::vector<std::vector<uint8_t>> ContextSetCfg::sm_InitTables( NUMBER_OF_SLICE_TYPES );

const CtxSet ContextSetCfg::SplitFlag = ContextSetCfg::addCtxSet
({
  {  107, 139, 126, 255,   0,},
  {  107, 139, 126, 255,   0,},
  {  139, 141, 157, 255,   0,},
});

const CtxSet ContextSetCfg::BTSplitFlag = ContextSetCfg::addCtxSet
({
  {  107, 139, 126, 154, 154, 154, 154, 154, 154, 154, 154, 154,},
  {  107, 139, 126, 154, 154, 154, 154, 154, 154, 154, 154, 154,},
  {  139, 141, 157, 154, 154, 154, 154, 154, 154, 154, 154, 154,},
});

const CtxSet ContextSetCfg::SkipFlag = ContextSetCfg::addCtxSet
({
  {  197, 185, 201,},
  {  197, 185, 201,},
  {  CNU, CNU, CNU,},
});

const CtxSet ContextSetCfg::MergeFlag = ContextSetCfg::addCtxSet
({
  {  154,},
  {  110,},
  {  CNU,},
});

const CtxSet ContextSetCfg::MergeIdx = ContextSetCfg::addCtxSet
({
  {  137, CNU, CNU, CNU, CNU,},
  {  122, CNU, CNU, CNU, CNU,},
  {  CNU, CNU, CNU, CNU, CNU,},
});

const CtxSet ContextSetCfg::PartSize = ContextSetCfg::addCtxSet
({
  {  154, 139, 154, 154,},
  {  154, 139, 154, 154,},
  {  184, CNU, CNU, CNU,},
});

const CtxSet ContextSetCfg::PredMode = ContextSetCfg::addCtxSet
({
  {  134,},
  {  149,},
  {  CNU,},
});

const CtxSet ContextSetCfg::IPredMode[] =
{
  ContextSetCfg::addCtxSet
  ({
#if INTRA67_3MPM
    { 183 },
    { 154 },
    { 184 },
#else
#if JVET_B0051_NON_MPM_MODE
    {  183, CNU, CNU, CNU, 184 },
    {  154, CNU, CNU, CNU, 184 },
    {  184, CNU, CNU, CNU, 184 },
#else
    {  183, CNU, CNU, CNU,  },
    {  154, CNU, CNU, CNU,  },
    {  184, CNU, CNU, CNU,  },
#endif
#endif
  }),
  ContextSetCfg::addCtxSet
  ({
    {  139, 152, 139, 154, 154, 154, 154, 154, 154, 154, 154, 154,  },
    {  139, 152, 139, 154, 154, 154, 154, 154, 154, 154, 154, 154,  },
    {  139,  63, 139, 154, 154, 154, 154, 154, 154, 154, 154, 154,  },
  }),
};

const CtxSet ContextSetCfg::PdpcFlag = ContextSetCfg::addCtxSet
({
  {  107,},
  {  107,},
  {  139,},
});

const CtxSet ContextSetCfg::DeltaQP = ContextSetCfg::addCtxSet
({
  {  154, 154, 154,},
  {  154, 154, 154,},
  {  154, 154, 154,},
});

const CtxSet ContextSetCfg::InterDir = ContextSetCfg::addCtxSet
({
  {   95,  79,  63,  31,  31,},
  {   95,  79,  63,  31,  31,},
  {  CNU, CNU, CNU, CNU, CNU,},
});

const CtxSet ContextSetCfg::RefPic = ContextSetCfg::addCtxSet
({
  {  153, 153,},
  {  153, 153,},
  {  CNU, CNU,},
});

#if JVET_K_AFFINE
const CtxSet ContextSetCfg::AffineFlag = ContextSetCfg::addCtxSet
({
  {  197, 185, 201,},
  {  197, 185, 201,},
  {  CNU, CNU, CNU,},
});

#if JVET_K0337_AFFINE_6PARA
const CtxSet ContextSetCfg::AffineType = ContextSetCfg::addCtxSet
({
  { 92,  },
  { 77,  },
  { CNU, },
});
#endif
#endif

const CtxSet ContextSetCfg::Mvd = ContextSetCfg::addCtxSet
({
  {  169, 198,},
  {  140, 198,},
  {  CNU, CNU,},
});

const CtxSet ContextSetCfg::TransSubdivFlag = ContextSetCfg::addCtxSet
({
  {  224, 167, 122, 122, 122},
  {  124, 138,  94,  94,  94},
  {  153, 138, 138, 138, 138},
});

const CtxSet ContextSetCfg::QtRootCbf = ContextSetCfg::addCtxSet
({
  {   79,},
  {   79,},
  {  CNU,},
});

const CtxSet ContextSetCfg::QtCbf[] =
{
  ContextSetCfg::addCtxSet
  ({
    {  153, 111,  },
    {  153, 111,  },
    {  111, 141,  },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  149,  92, 167, 154, 154,  },
    {  149, 107, 167, 154, 154,  },
    {   94, 138, 182, 154, 154,  },
  }),
#if JVET_K0072
  ContextSetCfg::addCtxSet
  ({
    { 149, 149, },
    { 149, 149, },
    {  94,  94, },
  }),
#endif
};

const CtxSet ContextSetCfg::SigCoeffGroup[] =
{
  ContextSetCfg::addCtxSet
  ({
    {  121, 140,  },
    {  121, 140,  },
    {   91, 171,  },
  }),
  ContextSetCfg::addCtxSet
  ({
    {   61, 154,  },
    {   61, 154,  },
    {  134, 141,  },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  122, 143,  },
    {   78, 111,  },
    {  135, 155,  },
  }),
  ContextSetCfg::addCtxSet
  ({
    {   91, 141,  },
    {   60, 140,  },
    {  104, 139,  },
  }),
};

const CtxSet ContextSetCfg::SigFlag[] =
{
#if JVET_K0072
  ContextSetCfg::addCtxSet
  ({
    {  106, 167, 182, 124, 139, 169, 134, 167, 197, 183, 183, 184, 209, 198, 168, 168, 183, 170, CNU, CNU,  },
    {  135, 152, 167, 153, 168, 140, 149, 182, 153, 183, 154, 155, 180, 198, 197, 183, 169, 170, CNU, CNU,  },
    {  121, 138, 124, 139, 125, 111, 135, 139, 154, 140, 155, 127, 107, 185, 169, 170, 156, 143, CNU, CNU,  },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  177, 196, 153, 124, 198, 183, 166, 213, 226, 198, 198, 156,  },
    {  134, 168, 168, 154, 169, 199, 166, 214, 227, 229, 185, 142,  },
    {  149, 168, 153, 111, 140, 126, 182, 200, 111, 143, 142, 158,  },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  181, 127, 173, 201, 187, 173, 226, 173, 188, 202, 173, 188, 196, 223, 237, 223, 221, 223, CNU, CNU,  },
    {  123, 142, 202, 157, 157, 188, 138, 158, 203, 173, 158, 174, 182, 223, 223, 223, 206, 237, CNU, CNU,  },
    {  108, 157, 173, 158, 218, 189, 123, 159, 159, 174, 189, 204,  79, 223, 223, 207, 253, 191, CNU, CNU,  },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  210, 170, 143, 143, 201, 244, 182, 223, 223, 223, 159, 223,  },
    {  167, 155, 158, 186, 127, 158, 197, 223, 223, 223, 206, 237,  },
    {  137, 158, 157, 187, 204, 159, 185, 223, 238, 220, 253, 237,  },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  137, 142, 159, 158, 187, 159, 241, 174, 174, 159, 159, 203, 210, 223, 223, 223, 223, 223, CNU, CNU,  },
    {  123, 157, 174, 143, 143, 203, 138, 159, 189, 159, 173, 174, 196, 223, 223, 223, 223, 223, CNU, CNU,  },
    {  107, 143, 218, 173, 218, 189,  63, 219, 189, 175, 189, 204,  63, 223, 223, 223, 253, 191, CNU, CNU,  },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  196, 199, 143, 172, 158, 203, 196, 223, 223, 223, 223, 223,  },
    {  167, 155, 159, 157, 157, 158, 182, 223, 223, 223, 223, 223,  },
    {  181, 159, 143, 232, 143, 173, 169, 237, 223, 223, 238, 253,  },
  }),
#else
  ContextSetCfg::addCtxSet
  ({
    {  170, 154, 139, 153, 139, 123, 123,  63, 124, 166, 183, 140, 136, 153, 154, 166, 183, 140, 136, 153, 154, 166, 183, 140, 136, 153, 154, 140,  },
    {  155, 154, 139, 153, 139, 123, 123,  63, 153, 166, 183, 140, 136, 153, 154, 166, 183, 140, 136, 153, 154, 166, 183, 140, 136, 153, 154, 140,  },
    {  111, 111, 125, 110, 110,  94, 124, 108, 124, 107, 125, 141, 179, 153, 125, 107, 125, 141, 179, 153, 125, 107, 125, 141, 179, 153, 125, 141,  },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  170, 153, 138, 138, 122, 121, 122, 121, 167, 151, 183, 140, 151, 183, 140, 140,  },
    {  170, 153, 123, 123, 107, 121, 107, 121, 167, 151, 183, 140, 151, 183, 140, 140,  },
    {  140, 139, 182, 182, 152, 136, 152, 136, 153, 136, 139, 111, 136, 139, 111, 111,  },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  107, 139, 154, 140, 140, 141, 108, 154, 125, 155, 126, 127, 139, 155, 155, 141, 156, 143, 107, 139, 154, 140, 140, 141, 108, 154, 125, 155, 126, 127, 139, 155, 155, 141, 156, 143, 107, 139, 154, 140, 140, 141, 108, 154, 125, 155, 126, 127, 139, 155, 155, 141, 156, 143,  },
    {  121, 167, 153, 139, 154, 140, 137, 168, 139, 154, 169, 155, 167, 169, 169, 184, 199, 156, 121, 167, 153, 139, 154, 140, 137, 168, 139, 154, 169, 155, 167, 169, 169, 184, 199, 156, 121, 167, 153, 139, 154, 140, 137, 168, 139, 154, 169, 155, 167, 169, 169, 184, 199, 156,  },
    {  152, 139, 154, 154, 169, 155, 182, 154, 169, 184, 155, 141, 168, 214, 199, 170, 170, 171, 152, 139, 154, 154, 169, 155, 182, 154, 169, 184, 155, 141, 168, 214, 199, 170, 170, 171, 152, 139, 154, 154, 169, 155, 182, 154, 169, 184, 155, 141, 168, 214, 199, 170, 170, 171,  },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  137, 154, 154, 155, 155, 156, 124, 185, 156, 171, 142, 158,  },
    {  136, 153, 139, 154, 125, 140, 122, 154, 184, 185, 171, 157,  },
    {  167, 154, 169, 140, 155, 141, 153, 171, 185, 156, 171, 172,  },
  }),
#endif
};


#if JVET_K0072
const CtxSet ContextSetCfg::ParFlag[] =
{
  ContextSetCfg::addCtxSet
  ({
    {  162, 134, 136, 167, 153, 138, 135, 167, 182, 168, 168, 150, 182, 153, 168, 110, 180, 168, 139, 168, 154,  },
    {  133, 163, 151, 167, 138, 168, 149, 152, 153, 153, 124, 150, 153, 153, 168, 139, 166, 168, 168, 139, 139,  },
    {  134, 120, 152, 123, 153, 153, 136, 123, 153, 168, 154, 152, 153, 153, 124, 139, 123, 168, 139, 154, 139,  },
  }),
  ContextSetCfg::addCtxSet
  ({
    {   57, 192, 194, 225, 153, 139, 209, 168, 213, 123,  95,  },
    {  147, 164, 137, 153, 124, 153, 210, 183, 183, 154, 139,  },
    {  134, 121, 182, 183, 138, 183, 198, 154, 154, 124, 154,  },
  }),
};

const CtxSet ContextSetCfg::GtxFlag[] =
{
  ContextSetCfg::addCtxSet
  ({
    {   59,  57,  59, 133, 164, 165, 117, 147, 134, 150, 137, 133, 163, 121, 166, 167, 163, 135, 136, 181, 139,  },
    {   45,  57,  58,  44, 149,  91,  73,  89, 105,  91, 122,  74, 105, 121, 122, 138, 119, 106, 107, 152, 139,  },
    {  135,  43, 119,  90,  76, 107,  74,  75, 106,  77,  93, 105,  91, 122,  93,  94, 150, 136, 123, 153, 125,  },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  133,   3,  14, 120, 135,  91, 179, 103, 194,  94, 111,  },
    {  118, 102, 134, 135, 122, 123, 163, 120, 122, 153, 169,  },
    {  195,  88,  74, 105, 152, 138, 120,  90, 107, 139, 184,  },
  }),
  ContextSetCfg::addCtxSet
  ({
    {    3,   3, 103, 119,  91, 151,   3, 148, 194, 152, 138, 147, 164, 166, 167, 168, 133, 180, 196, 139, 169,  },
    {    3,   3, 118, 120, 106, 152,   3, 119, 121, 122, 153, 118, 135, 166, 138, 139, 148, 151, 182, 168, 184,  },
    {  132, 102, 104, 121,  92,  93, 104, 106, 122, 123,  94, 105, 137, 153, 139,  95, 121, 123, 139, 154, 126,  },
  }),
  ContextSetCfg::addCtxSet
  ({
    {    3,   3, 176, 193, 137, 181, 161, 137, 183, 155, 111,  },
    {    3,   3, 134, 137, 123, 124, 147, 167, 169, 199, 156,  },
    {  147,  73, 164, 151, 107, 109, 120, 152, 140, 185, 111,  },
  }),
};
#endif

const CtxSet ContextSetCfg::LastX[] =
{
  ContextSetCfg::addCtxSet
  ({
    {  125, 110, 124, 110,  95,  94, 125, 111, 111,  79, 125, 126, 111, 111,  79, 126, 111, 111,  79, CNU, CNU, CNU, CNU, CNU, CNU,  },
    {  125, 110,  94, 110,  95,  79, 125, 111, 110,  78, 110, 111, 111,  95,  94, 111, 111,  95,  94, CNU, CNU, CNU, CNU, CNU, CNU,  },
    {  110, 110, 124, 125, 140, 153, 125, 127, 140, 109, 111, 143, 127, 111,  79, 143, 127, 111,  79, CNU, CNU, CNU, CNU, CNU, CNU,  },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  108, 123,  93, 154,  },
    {  108, 123, 108, 154,  },
    {  108, 123,  63, 154,  },
  }),
};

const CtxSet ContextSetCfg::LastY[] =
{
  ContextSetCfg::addCtxSet
  ({
    {  125, 110, 124, 110,  95,  94, 125, 111, 111,  79, 125, 126, 111, 111,  79, 126, 111, 111,  79, CNU, CNU, CNU, CNU, CNU, CNU,  },
    {  125, 110,  94, 110,  95,  79, 125, 111, 110,  78, 110, 111, 111,  95,  94, 111, 111,  95,  94, CNU, CNU, CNU, CNU, CNU, CNU,  },
    {  110, 110, 124, 125, 140, 153, 125, 127, 140, 109, 111, 143, 127, 111,  79, 143, 127, 111,  79, CNU, CNU, CNU, CNU, CNU, CNU,  },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  108, 123,  93, 154,  },
    {  108, 123, 108, 154,  },
    {  108, 123,  63, 154,  },
  }),
};

#if JVET_K0072
#else
const CtxSet ContextSetCfg::GreaterOneFlag[] =
{
  ContextSetCfg::addCtxSet
  ({
    {  154, 196, 167, 167,  },
    {  154, 196, 196, 167,  },
    {  140,  92, 137, 138,  },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  154, 152, 167, 182,  },
    {  154, 152, 167, 182,  },
    {  140, 152, 138, 139,  },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  182, 134, 149, 136,  },
    {  182, 134, 149, 136,  },
    {  153,  74, 149,  92,  },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  153, 121, 136, 122,  },
    {  153, 121, 136, 137,  },
    {  139, 107, 122, 152,  },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  169, 208, 166, 167,  },
    {  169, 194, 166, 167,  },
    {  140, 179, 166, 182,  },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  154, 152, 167, 182,  },
    {  154, 167, 137, 182,  },
    {  140, 227, 122, 197,  },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  121, 135, 123, 124, 139, 125,  92, 124, 154, 125, 155, 138, 169, 155, 170, 156,  },
    {  165,  75, 152, 153, 139, 154, 121, 138, 139, 154, 140, 167, 183, 169, 170, 156,  },
    {  196, 105, 152, 153, 139, 154, 136, 138, 139, 169, 140, 196, 183, 169, 170, 171,  },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  166, 152, 140, 170, 171, 157,  },
    {  193, 181, 169, 170, 171, 172,  },
    {  195, 181, 169, 170, 156, 157,  },
  }),
};

const CtxSet ContextSetCfg::GreaterTwoFlag = ContextSetCfg::addCtxSet
({
  {  107, 167,  91, 107, 107, 167,},
  {  107, 167,  91, 122, 107, 167,},
  {  138, 153, 136, 167, 152, 152,},
});
#endif

const CtxSet ContextSetCfg::MVPIdx = ContextSetCfg::addCtxSet
({
  {  168,},
  {  168,},
  {  CNU,},
});

const CtxSet ContextSetCfg::SaoMergeFlag = ContextSetCfg::addCtxSet
({
  {  153,},
  {  153,},
  {  153,},
});

const CtxSet ContextSetCfg::SaoTypeIdx = ContextSetCfg::addCtxSet
({
  {  160,},
  {  185,},
  {  200,},
});

const CtxSet ContextSetCfg::TransformSkipFlag = ContextSetCfg::addCtxSet
({
  {  139, 139,},
  {  139, 139,},
  {  139, 139,},
});

const CtxSet ContextSetCfg::TransquantBypassFlag = ContextSetCfg::addCtxSet
({
  {  154,},
  {  154,},
  {  154,},
});

const CtxSet ContextSetCfg::RdpcmFlag = ContextSetCfg::addCtxSet
({
  {  139, 139,},
  {  139, 139,},
  {  CNU, CNU,},
});

const CtxSet ContextSetCfg::RdpcmDir = ContextSetCfg::addCtxSet
({
  {  139, 139,},
  {  139, 139,},
  {  CNU, CNU,},
});

#if JVET_K1000_SIMPLIFIED_EMT
const CtxSet ContextSetCfg::EMTTuIndex = ContextSetCfg::addCtxSet
({
  {  CNU, CNU, CNU, CNU,},
  {  CNU, CNU, CNU, CNU,},
  {  CNU, CNU, CNU, CNU,},
});

const CtxSet ContextSetCfg::EMTCuFlag = ContextSetCfg::addCtxSet
({
  {  CNU, CNU, CNU, CNU, CNU, CNU,},
  {  CNU, CNU, CNU, CNU, CNU, CNU,},
  {  CNU, CNU, CNU, CNU, CNU, CNU,},
});
#endif

const CtxSet ContextSetCfg::CrossCompPred = ContextSetCfg::addCtxSet
({
  {  154, 154, 154, 154, 154, 154, 154, 154, 154, 154,},
  {  154, 154, 154, 154, 154, 154, 154, 154, 154, 154,},
  {  154, 154, 154, 154, 154, 154, 154, 154, 154, 154,},
});

const CtxSet ContextSetCfg::ChromaQpAdjFlag = ContextSetCfg::addCtxSet
({
  {  154,},
  {  154,},
  {  154,},
});

const CtxSet ContextSetCfg::ChromaQpAdjIdc = ContextSetCfg::addCtxSet
({
  {  154,},
  {  154,},
  {  154,},
});

#if JVET_K0357_AMVR
const CtxSet ContextSetCfg::ImvFlag = ContextSetCfg::addCtxSet
({
  {  197, 185, 201, 185,},
  {  197, 185, 201, 185,},
  {  CNU, CNU, CNU, CNU,},
});
#endif

#if JVET_K0371_ALF
const CtxSet ContextSetCfg::ctbAlfFlag =
{
  ContextSetCfg::addCtxSet
  ( {
    { 100, 100, 100, 100, 100, 100, 100, 100, 100 },
    { 153, 153, 153, 153, 153, 153, 153, 153, 153 },
    { 200, 200, 200, 200, 200, 200, 200, 200, 200 },
    } )
};
#endif


const unsigned ContextSetCfg::NumberOfContexts = (unsigned)ContextSetCfg::sm_InitTables[0].size();


// combined sets
const CtxSet ContextSetCfg::Sao = { ContextSetCfg::SaoMergeFlag, ContextSetCfg::SaoTypeIdx };



template <class BinProbModel>
CtxStore<BinProbModel>::CtxStore()
  : m_CtxBuffer ()
  , m_Ctx       ( nullptr )
{}

template <class BinProbModel>
CtxStore<BinProbModel>::CtxStore( bool dummy )
  : m_CtxBuffer ( ContextSetCfg::NumberOfContexts )
  , m_Ctx       ( m_CtxBuffer.data() )
{}

template <class BinProbModel>
CtxStore<BinProbModel>::CtxStore( const CtxStore<BinProbModel>& ctxStore )
  : m_CtxBuffer ( ctxStore.m_CtxBuffer )
  , m_Ctx       ( m_CtxBuffer.data() )
{}

template <class BinProbModel>
void CtxStore<BinProbModel>::init( int qp, int initId )
{
  const std::vector<uint8_t>& initTable = ContextSetCfg::getInitTable( initId );
  CHECK( m_CtxBuffer.size() != initTable.size(),
        "Size of init table (" << initTable.size() << ") does not match size of context buffer (" << m_CtxBuffer.size() << ")." );
  int clippedQP = std::min( std::max( 0, qp ), MAX_QP );
  for( std::size_t k = 0; k < m_CtxBuffer.size(); k++ )
  {
    m_CtxBuffer[k].init( clippedQP, initTable[k] );
  }
}

template <class BinProbModel>
void CtxStore<BinProbModel>::setWinSizes( const std::vector<uint8_t>& log2WindowSizes )
{
  CHECK( m_CtxBuffer.size() != log2WindowSizes.size(),
        "Size of window size table (" << log2WindowSizes.size() << ") does not match size of context buffer (" << m_CtxBuffer.size() << ")." );
  for( std::size_t k = 0; k < m_CtxBuffer.size(); k++ )
  {
    m_CtxBuffer[k].setLog2WindowSize( log2WindowSizes[k] );
  }
}

template <class BinProbModel>
void CtxStore<BinProbModel>::loadPStates( const std::vector<uint16_t>& probStates )
{
  CHECK( m_CtxBuffer.size() != probStates.size(),
        "Size of prob states table (" << probStates.size() << ") does not match size of context buffer (" << m_CtxBuffer.size() << ")." );
  for( std::size_t k = 0; k < m_CtxBuffer.size(); k++ )
  {
    m_CtxBuffer[k].setState( probStates[k] );
  }
}

template <class BinProbModel>
void CtxStore<BinProbModel>::savePStates( std::vector<uint16_t>& probStates ) const
{
  probStates.resize( m_CtxBuffer.size(), uint16_t(0) );
  for( std::size_t k = 0; k < m_CtxBuffer.size(); k++ )
  {
    probStates[k] = m_CtxBuffer[k].getState();
  }
}





template class CtxStore<BinProbModel_Std>;





Ctx::Ctx()                                  : m_BPMType( BPM_Undefined )                        {}
Ctx::Ctx( const BinProbModel_Std*   dummy ) : m_BPMType( BPM_Std   ), m_CtxStore_Std  ( true )  {}

Ctx::Ctx( const Ctx& ctx )
  : m_BPMType         ( ctx.m_BPMType )
  , m_CtxStore_Std    ( ctx.m_CtxStore_Std    )
{
  ::memcpy( m_GRAdaptStats, ctx.m_GRAdaptStats, sizeof( unsigned ) * RExt__GOLOMB_RICE_ADAPTATION_STATISTICS_SETS );
}


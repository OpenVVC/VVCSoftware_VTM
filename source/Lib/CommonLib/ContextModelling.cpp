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

/** \file     ContextModelling.cpp
    \brief    Classes providing probability descriptions and contexts
*/

#include "ContextModelling.h"
#include "UnitTools.h"
#include "CodingStructure.h"
#include "Picture.h"

#if JVET_K0072
#else
#if !HM_QTBT_AS_IN_JEM_CONTEXT
static const uint8_t spat_bypass_luma_all           []  = { 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27 };
static const uint8_t spat_bypass_chroma_all         []  = { 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15 };
static const uint8_t spat_4x4_diag_all              []  = {  0,  2,  1,  6,  3,  4,  7,  6,  4,  5,  7,  8,  5,  8,  8,  8 };
static const uint8_t spat_4x4_hor_all               []  = {  0,  1,  4,  5,  2,  3,  4,  5,  6,  6,  8,  8,  7,  7,  8,  8 };
static const uint8_t spat_4x4_ver_all               []  = {  0,  2,  6,  7,  1,  3,  6,  7,  4,  4,  8,  8,  5,  5,  8,  8 };
static const uint8_t spat_8x8_luma_diag_first_pat0  []  = {  0, 10, 10, 10, 10, 10,  9,  9,  9,  9,  9,  9,  9,  9,  9,  9 };
static const uint8_t spat_8x8_luma_diag_first_pat1  []  = {  0, 10, 11,  9, 10, 11,  9,  9, 10, 11,  9,  9, 10,  9,  9,  9 };
static const uint8_t spat_8x8_luma_diag_first_pat2  []  = {  0, 11, 10, 11, 10,  9, 11, 10,  9,  9, 10,  9,  9,  9,  9,  9 };
static const uint8_t spat_8x8_luma_diag_first_pat3  []  = {  0, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11 };
static const uint8_t spat_8x8_luma_diag_other_pat0  []  = { 14, 13, 13, 13, 13, 13, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12 };
static const uint8_t spat_8x8_luma_diag_other_pat1  []  = { 14, 13, 14, 12, 13, 14, 12, 12, 13, 14, 12, 12, 13, 12, 12, 12 };
static const uint8_t spat_8x8_luma_diag_other_pat2  []  = { 14, 14, 13, 14, 13, 12, 14, 13, 12, 12, 13, 12, 12, 12, 12, 12 };
static const uint8_t spat_8x8_luma_diag_other_pat3  []  = { 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14 };
static const uint8_t spat_8x8_luma_hor_first_pat0   []  = {  0, 16, 16, 15, 16, 16, 15, 15, 16, 15, 15, 15, 15, 15, 15, 15 };
static const uint8_t spat_8x8_luma_hor_first_pat1   []  = {  0, 17, 17, 17, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15 };
static const uint8_t spat_8x8_luma_hor_first_pat2   []  = {  0, 16, 15, 15, 17, 16, 15, 15, 17, 16, 15, 15, 17, 16, 15, 15 };
static const uint8_t spat_8x8_luma_hor_first_pat3   []  = {  0, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17 };
static const uint8_t spat_8x8_luma_hor_other_pat0   []  = { 20, 19, 19, 18, 19, 19, 18, 18, 19, 18, 18, 18, 18, 18, 18, 18 };
static const uint8_t spat_8x8_luma_hor_other_pat1   []  = { 20, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 18, 18 };
static const uint8_t spat_8x8_luma_hor_other_pat2   []  = { 20, 19, 18, 18, 20, 19, 18, 18, 20, 19, 18, 18, 20, 19, 18, 18 };
static const uint8_t spat_8x8_luma_hor_other_pat3   []  = { 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20 };
static const uint8_t spat_8x8_luma_ver_first_pat0   []  = {  0, 16, 16, 15, 16, 16, 15, 15, 16, 15, 15, 15, 15, 15, 15, 15 };
static const uint8_t spat_8x8_luma_ver_first_pat1   []  = {  0, 16, 15, 15, 17, 16, 15, 15, 17, 16, 15, 15, 17, 16, 15, 15 };
static const uint8_t spat_8x8_luma_ver_first_pat2   []  = {  0, 17, 17, 17, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15 };
static const uint8_t spat_8x8_luma_ver_first_pat3   []  = {  0, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17 };
static const uint8_t spat_8x8_luma_ver_other_pat0   []  = { 20, 19, 19, 18, 19, 19, 18, 18, 19, 18, 18, 18, 18, 18, 18, 18 };
static const uint8_t spat_8x8_luma_ver_other_pat1   []  = { 20, 19, 18, 18, 20, 19, 18, 18, 20, 19, 18, 18, 20, 19, 18, 18 };
static const uint8_t spat_8x8_luma_ver_other_pat2   []  = { 20, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 18, 18 };
static const uint8_t spat_8x8_luma_ver_other_pat3   []  = { 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20 };
static const uint8_t spat_nxn_luma_diag_first_pat0  []  = {  0, 22, 22, 22, 22, 22, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21 };
static const uint8_t spat_nxn_luma_diag_first_pat1  []  = {  0, 22, 23, 21, 22, 23, 21, 21, 22, 23, 21, 21, 22, 21, 21, 21 };
static const uint8_t spat_nxn_luma_diag_first_pat2  []  = {  0, 23, 22, 23, 22, 21, 23, 22, 21, 21, 22, 21, 21, 21, 21, 21 };
static const uint8_t spat_nxn_luma_diag_first_pat3  []  = {  0, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23 };
static const uint8_t spat_nxn_luma_diag_other_pat0  []  = { 26, 25, 25, 25, 25, 25, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24 };
static const uint8_t spat_nxn_luma_diag_other_pat1  []  = { 26, 25, 26, 24, 25, 26, 24, 24, 25, 26, 24, 24, 25, 24, 24, 24 };
static const uint8_t spat_nxn_luma_diag_other_pat2  []  = { 26, 26, 25, 26, 25, 24, 26, 25, 24, 24, 25, 24, 24, 24, 24, 24 };
static const uint8_t spat_nxn_luma_diag_other_pat3  []  = { 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26 };
static const uint8_t spat_nxn_luma_hor_first_pat0   []  = {  0, 22, 22, 21, 22, 22, 21, 21, 22, 21, 21, 21, 21, 21, 21, 21 };
static const uint8_t spat_nxn_luma_hor_first_pat1   []  = {  0, 23, 23, 23, 22, 22, 22, 22, 21, 21, 21, 21, 21, 21, 21, 21 };
static const uint8_t spat_nxn_luma_hor_first_pat2   []  = {  0, 22, 21, 21, 23, 22, 21, 21, 23, 22, 21, 21, 23, 22, 21, 21 };
static const uint8_t spat_nxn_luma_hor_first_pat3   []  = {  0, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23 };
static const uint8_t spat_nxn_luma_hor_other_pat0   []  = { 26, 25, 25, 24, 25, 25, 24, 24, 25, 24, 24, 24, 24, 24, 24, 24 };
static const uint8_t spat_nxn_luma_hor_other_pat1   []  = { 26, 26, 26, 26, 25, 25, 25, 25, 24, 24, 24, 24, 24, 24, 24, 24 };
static const uint8_t spat_nxn_luma_hor_other_pat2   []  = { 26, 25, 24, 24, 26, 25, 24, 24, 26, 25, 24, 24, 26, 25, 24, 24 };
static const uint8_t spat_nxn_luma_hor_other_pat3   []  = { 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26 };
static const uint8_t spat_nxn_luma_ver_first_pat0   []  = {  0, 22, 22, 21, 22, 22, 21, 21, 22, 21, 21, 21, 21, 21, 21, 21 };
static const uint8_t spat_nxn_luma_ver_first_pat1   []  = {  0, 22, 21, 21, 23, 22, 21, 21, 23, 22, 21, 21, 23, 22, 21, 21 };
static const uint8_t spat_nxn_luma_ver_first_pat2   []  = {  0, 23, 23, 23, 22, 22, 22, 22, 21, 21, 21, 21, 21, 21, 21, 21 };
static const uint8_t spat_nxn_luma_ver_first_pat3   []  = {  0, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23 };
static const uint8_t spat_nxn_luma_ver_other_pat0   []  = { 26, 25, 25, 24, 25, 25, 24, 24, 25, 24, 24, 24, 24, 24, 24, 24 };
static const uint8_t spat_nxn_luma_ver_other_pat1   []  = { 26, 25, 24, 24, 26, 25, 24, 24, 26, 25, 24, 24, 26, 25, 24, 24 };
static const uint8_t spat_nxn_luma_ver_other_pat2   []  = { 26, 26, 26, 26, 25, 25, 25, 25, 24, 24, 24, 24, 24, 24, 24, 24 };
static const uint8_t spat_nxn_luma_ver_other_pat3   []  = { 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26 };
static const uint8_t spat_8x8_chroma_diag_first_pat0[]  = {  0, 10, 10, 10, 10, 10,  9,  9,  9,  9,  9,  9,  9,  9,  9,  9 };
static const uint8_t spat_8x8_chroma_diag_first_pat1[]  = {  0, 10, 11,  9, 10, 11,  9,  9, 10, 11,  9,  9, 10,  9,  9,  9 };
static const uint8_t spat_8x8_chroma_diag_first_pat2[]  = {  0, 11, 10, 11, 10,  9, 11, 10,  9,  9, 10,  9,  9,  9,  9,  9 };
static const uint8_t spat_8x8_chroma_diag_first_pat3[]  = {  0, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11 };
static const uint8_t spat_8x8_chroma_diag_other_pat0[]  = { 11, 10, 10, 10, 10, 10,  9,  9,  9,  9,  9,  9,  9,  9,  9,  9 };
static const uint8_t spat_8x8_chroma_diag_other_pat1[]  = { 11, 10, 11,  9, 10, 11,  9,  9, 10, 11,  9,  9, 10,  9,  9,  9 };
static const uint8_t spat_8x8_chroma_diag_other_pat2[]  = { 11, 11, 10, 11, 10,  9, 11, 10,  9,  9, 10,  9,  9,  9,  9,  9 };
static const uint8_t spat_8x8_chroma_diag_other_pat3[]  = { 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11 };
static const uint8_t spat_8x8_chroma_hor_first_pat0 []  = {  0, 10, 10,  9, 10, 10,  9,  9, 10,  9,  9,  9,  9,  9,  9,  9 };
static const uint8_t spat_8x8_chroma_hor_first_pat1 []  = {  0, 11, 11, 11, 10, 10, 10, 10,  9,  9,  9,  9,  9,  9,  9,  9 };
static const uint8_t spat_8x8_chroma_hor_first_pat2 []  = {  0, 10,  9,  9, 11, 10,  9,  9, 11, 10,  9,  9, 11, 10,  9,  9 };
static const uint8_t spat_8x8_chroma_hor_first_pat3 []  = {  0, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11 };
static const uint8_t spat_8x8_chroma_hor_other_pat0 []  = { 11, 10, 10,  9, 10, 10,  9,  9, 10,  9,  9,  9,  9,  9,  9,  9 };
static const uint8_t spat_8x8_chroma_hor_other_pat1 []  = { 11, 11, 11, 11, 10, 10, 10, 10,  9,  9,  9,  9,  9,  9,  9,  9 };
static const uint8_t spat_8x8_chroma_hor_other_pat2 []  = { 11, 10,  9,  9, 11, 10,  9,  9, 11, 10,  9,  9, 11, 10,  9,  9 };
static const uint8_t spat_8x8_chroma_hor_other_pat3 []  = { 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11 };
static const uint8_t spat_8x8_chroma_ver_first_pat0 []  = {  0, 10, 10,  9, 10, 10,  9,  9, 10,  9,  9,  9,  9,  9,  9,  9 };
static const uint8_t spat_8x8_chroma_ver_first_pat1 []  = {  0, 10,  9,  9, 11, 10,  9,  9, 11, 10,  9,  9, 11, 10,  9,  9 };
static const uint8_t spat_8x8_chroma_ver_first_pat2 []  = {  0, 11, 11, 11, 10, 10, 10, 10,  9,  9,  9,  9,  9,  9,  9,  9 };
static const uint8_t spat_8x8_chroma_ver_first_pat3 []  = {  0, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11 };
static const uint8_t spat_8x8_chroma_ver_other_pat0 []  = { 11, 10, 10,  9, 10, 10,  9,  9, 10,  9,  9,  9,  9,  9,  9,  9 };
static const uint8_t spat_8x8_chroma_ver_other_pat1 []  = { 11, 10,  9,  9, 11, 10,  9,  9, 11, 10,  9,  9, 11, 10,  9,  9 };
static const uint8_t spat_8x8_chroma_ver_other_pat2 []  = { 11, 11, 11, 11, 10, 10, 10, 10,  9,  9,  9,  9,  9,  9,  9,  9 };
static const uint8_t spat_8x8_chroma_ver_other_pat3 []  = { 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11 };
static const uint8_t spat_nxn_chroma_diag_first_pat0[]  = {  0, 13, 13, 13, 13, 13, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12 };
static const uint8_t spat_nxn_chroma_diag_first_pat1[]  = {  0, 13, 14, 12, 13, 14, 12, 12, 13, 14, 12, 12, 13, 12, 12, 12 };
static const uint8_t spat_nxn_chroma_diag_first_pat2[]  = {  0, 14, 13, 14, 13, 12, 14, 13, 12, 12, 13, 12, 12, 12, 12, 12 };
static const uint8_t spat_nxn_chroma_diag_first_pat3[]  = {  0, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14 };
static const uint8_t spat_nxn_chroma_diag_other_pat0[]  = { 14, 13, 13, 13, 13, 13, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12 };
static const uint8_t spat_nxn_chroma_diag_other_pat1[]  = { 14, 13, 14, 12, 13, 14, 12, 12, 13, 14, 12, 12, 13, 12, 12, 12 };
static const uint8_t spat_nxn_chroma_diag_other_pat2[]  = { 14, 14, 13, 14, 13, 12, 14, 13, 12, 12, 13, 12, 12, 12, 12, 12 };
static const uint8_t spat_nxn_chroma_diag_other_pat3[]  = { 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14 };
static const uint8_t spat_nxn_chroma_hor_first_pat0 []  = {  0, 13, 13, 12, 13, 13, 12, 12, 13, 12, 12, 12, 12, 12, 12, 12 };
static const uint8_t spat_nxn_chroma_hor_first_pat1 []  = {  0, 14, 14, 14, 13, 13, 13, 13, 12, 12, 12, 12, 12, 12, 12, 12 };
static const uint8_t spat_nxn_chroma_hor_first_pat2 []  = {  0, 13, 12, 12, 14, 13, 12, 12, 14, 13, 12, 12, 14, 13, 12, 12 };
static const uint8_t spat_nxn_chroma_hor_first_pat3 []  = {  0, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14 };
static const uint8_t spat_nxn_chroma_hor_other_pat0 []  = { 14, 13, 13, 12, 13, 13, 12, 12, 13, 12, 12, 12, 12, 12, 12, 12 };
static const uint8_t spat_nxn_chroma_hor_other_pat1 []  = { 14, 14, 14, 14, 13, 13, 13, 13, 12, 12, 12, 12, 12, 12, 12, 12 };
static const uint8_t spat_nxn_chroma_hor_other_pat2 []  = { 14, 13, 12, 12, 14, 13, 12, 12, 14, 13, 12, 12, 14, 13, 12, 12 };
static const uint8_t spat_nxn_chroma_hor_other_pat3 []  = { 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14 };
static const uint8_t spat_nxn_chroma_ver_first_pat0 []  = {  0, 13, 13, 12, 13, 13, 12, 12, 13, 12, 12, 12, 12, 12, 12, 12 };
static const uint8_t spat_nxn_chroma_ver_first_pat1 []  = {  0, 13, 12, 12, 14, 13, 12, 12, 14, 13, 12, 12, 14, 13, 12, 12 };
static const uint8_t spat_nxn_chroma_ver_first_pat2 []  = {  0, 14, 14, 14, 13, 13, 13, 13, 12, 12, 12, 12, 12, 12, 12, 12 };
static const uint8_t spat_nxn_chroma_ver_first_pat3 []  = {  0, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14 };
static const uint8_t spat_nxn_chroma_ver_other_pat0 []  = { 14, 13, 13, 12, 13, 13, 12, 12, 13, 12, 12, 12, 12, 12, 12, 12 };
static const uint8_t spat_nxn_chroma_ver_other_pat1 []  = { 14, 13, 12, 12, 14, 13, 12, 12, 14, 13, 12, 12, 14, 13, 12, 12 };
static const uint8_t spat_nxn_chroma_ver_other_pat2 []  = { 14, 14, 14, 14, 13, 13, 13, 13, 12, 12, 12, 12, 12, 12, 12, 12 };
static const uint8_t spat_nxn_chroma_ver_other_pat3 []  = { 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14 };
static const uint8_t spat_cg2_chroma_diag_first_pat0[]  = {  0, 13, 13, 12 };
static const uint8_t spat_cg2_chroma_diag_first_pat1[]  = {  0, 13, 14, 13 };
static const uint8_t spat_cg2_chroma_diag_first_pat2[]  = {  0, 14, 13, 13 };
static const uint8_t spat_cg2_chroma_diag_first_pat3[]  = {  0, 14, 14, 14 };
static const uint8_t spat_cg2_chroma_diag_other_pat0[]  = { 14, 13, 13, 12 };
static const uint8_t spat_cg2_chroma_diag_other_pat1[]  = { 14, 13, 14, 13 };
static const uint8_t spat_cg2_chroma_diag_other_pat2[]  = { 14, 14, 13, 13 };
static const uint8_t spat_cg2_chroma_diag_other_pat3[]  = { 14, 14, 14, 14 };
static const uint8_t spat_cg2_chroma_hor_first_pat0 []  = {  0, 13, 13, 12 };
static const uint8_t spat_cg2_chroma_hor_first_pat1 []  = {  0, 14, 13, 13 };
static const uint8_t spat_cg2_chroma_hor_first_pat2 []  = {  0, 13, 14, 13 };
static const uint8_t spat_cg2_chroma_hor_first_pat3 []  = {  0, 14, 14, 14 };
static const uint8_t spat_cg2_chroma_hor_other_pat0 []  = { 14, 13, 13, 12 };
static const uint8_t spat_cg2_chroma_hor_other_pat1 []  = { 14, 14, 13, 13 };
static const uint8_t spat_cg2_chroma_hor_other_pat2 []  = { 14, 13, 14, 13 };
static const uint8_t spat_cg2_chroma_hor_other_pat3 []  = { 14, 14, 14, 14 };
static const uint8_t spat_cg2_chroma_ver_first_pat0 []  = {  0, 13, 13, 12 };
static const uint8_t spat_cg2_chroma_ver_first_pat1 []  = {  0, 13, 14, 13 };
static const uint8_t spat_cg2_chroma_ver_first_pat2 []  = {  0, 14, 13, 13 };
static const uint8_t spat_cg2_chroma_ver_first_pat3 []  = {  0, 14, 14, 14 };
static const uint8_t spat_cg2_chroma_ver_other_pat0 []  = { 14, 13, 13, 12 };
static const uint8_t spat_cg2_chroma_ver_other_pat1 []  = { 14, 13, 14, 13 };
static const uint8_t spat_cg2_chroma_ver_other_pat2 []  = { 14, 14, 13, 13 };
static const uint8_t spat_cg2_chroma_ver_other_pat3 []  = { 14, 14, 14, 14 };

static const uint8_t* spat_bypass_luma    [] = { spat_bypass_luma_all,            spat_bypass_luma_all,            spat_bypass_luma_all,            spat_bypass_luma_all,            spat_bypass_luma_all,            spat_bypass_luma_all,            spat_bypass_luma_all,            spat_bypass_luma_all            };
static const uint8_t* spat_4x4_luma_diag  [] = { spat_4x4_diag_all,               spat_4x4_diag_all,               spat_4x4_diag_all,               spat_4x4_diag_all,               spat_4x4_diag_all,               spat_4x4_diag_all,               spat_4x4_diag_all,               spat_4x4_diag_all               };
static const uint8_t* spat_4x4_luma_hor   [] = { spat_4x4_hor_all,                spat_4x4_hor_all,                spat_4x4_hor_all,                spat_4x4_hor_all,                spat_4x4_hor_all,                spat_4x4_hor_all,                spat_4x4_hor_all,                spat_4x4_hor_all                };
static const uint8_t* spat_4x4_luma_ver   [] = { spat_4x4_ver_all,                spat_4x4_ver_all,                spat_4x4_ver_all,                spat_4x4_ver_all,                spat_4x4_ver_all,                spat_4x4_ver_all,                spat_4x4_ver_all,                spat_4x4_ver_all                };
static const uint8_t* spat_8x8_luma_diag  [] = { spat_8x8_luma_diag_first_pat0,   spat_8x8_luma_diag_first_pat1,   spat_8x8_luma_diag_first_pat2,   spat_8x8_luma_diag_first_pat3,   spat_8x8_luma_diag_other_pat0,   spat_8x8_luma_diag_other_pat1,   spat_8x8_luma_diag_other_pat2,   spat_8x8_luma_diag_other_pat3 } ;
static const uint8_t* spat_8x8_luma_hor   [] = { spat_8x8_luma_hor_first_pat0,    spat_8x8_luma_hor_first_pat1,    spat_8x8_luma_hor_first_pat2,    spat_8x8_luma_hor_first_pat3,    spat_8x8_luma_hor_other_pat0,    spat_8x8_luma_hor_other_pat1,    spat_8x8_luma_hor_other_pat2,    spat_8x8_luma_hor_other_pat3    };
static const uint8_t* spat_8x8_luma_ver   [] = { spat_8x8_luma_ver_first_pat0,    spat_8x8_luma_ver_first_pat1,    spat_8x8_luma_ver_first_pat2,    spat_8x8_luma_ver_first_pat3,    spat_8x8_luma_ver_other_pat0,    spat_8x8_luma_ver_other_pat1,    spat_8x8_luma_ver_other_pat2,    spat_8x8_luma_ver_other_pat3    };
static const uint8_t* spat_nxn_luma_diag  [] = { spat_nxn_luma_diag_first_pat0,   spat_nxn_luma_diag_first_pat1,   spat_nxn_luma_diag_first_pat2,   spat_nxn_luma_diag_first_pat3,   spat_nxn_luma_diag_other_pat0,   spat_nxn_luma_diag_other_pat1,   spat_nxn_luma_diag_other_pat2,   spat_nxn_luma_diag_other_pat3   };
static const uint8_t* spat_nxn_luma_hor   [] = { spat_nxn_luma_hor_first_pat0,    spat_nxn_luma_hor_first_pat1,    spat_nxn_luma_hor_first_pat2,    spat_nxn_luma_hor_first_pat3,    spat_nxn_luma_hor_other_pat0,    spat_nxn_luma_hor_other_pat1,    spat_nxn_luma_hor_other_pat2,    spat_nxn_luma_hor_other_pat3    };
static const uint8_t* spat_nxn_luma_ver   [] = { spat_nxn_luma_ver_first_pat0,    spat_nxn_luma_ver_first_pat1,    spat_nxn_luma_ver_first_pat2,    spat_nxn_luma_ver_first_pat3,    spat_nxn_luma_ver_other_pat0,    spat_nxn_luma_ver_other_pat1,    spat_nxn_luma_ver_other_pat2,    spat_nxn_luma_ver_other_pat3    };
static const uint8_t* spat_bypass_chroma  [] = { spat_bypass_chroma_all,          spat_bypass_chroma_all,          spat_bypass_chroma_all,          spat_bypass_chroma_all,          spat_bypass_chroma_all,          spat_bypass_chroma_all,          spat_bypass_chroma_all,          spat_bypass_chroma_all          };
static const uint8_t* spat_4x4_chroma_diag[] = { spat_4x4_diag_all,               spat_4x4_diag_all,               spat_4x4_diag_all,               spat_4x4_diag_all,               spat_4x4_diag_all,               spat_4x4_diag_all,               spat_4x4_diag_all,               spat_4x4_diag_all               };
static const uint8_t* spat_4x4_chroma_hor [] = { spat_4x4_hor_all,                spat_4x4_hor_all,                spat_4x4_hor_all,                spat_4x4_hor_all,                spat_4x4_hor_all,                spat_4x4_hor_all,                spat_4x4_hor_all,                spat_4x4_hor_all                };
static const uint8_t* spat_4x4_chroma_ver [] = { spat_4x4_ver_all,                spat_4x4_ver_all,                spat_4x4_ver_all,                spat_4x4_ver_all,                spat_4x4_ver_all,                spat_4x4_ver_all,                spat_4x4_ver_all,                spat_4x4_ver_all                };
static const uint8_t* spat_8x8_chroma_diag[] = { spat_8x8_chroma_diag_first_pat0, spat_8x8_chroma_diag_first_pat1, spat_8x8_chroma_diag_first_pat2, spat_8x8_chroma_diag_first_pat3, spat_8x8_chroma_diag_other_pat0, spat_8x8_chroma_diag_other_pat1, spat_8x8_chroma_diag_other_pat2, spat_8x8_chroma_diag_other_pat3 };
static const uint8_t* spat_8x8_chroma_hor [] = { spat_8x8_chroma_hor_first_pat0,  spat_8x8_chroma_hor_first_pat1,  spat_8x8_chroma_hor_first_pat2,  spat_8x8_chroma_hor_first_pat3,  spat_8x8_chroma_hor_other_pat0,  spat_8x8_chroma_hor_other_pat1,  spat_8x8_chroma_hor_other_pat2,  spat_8x8_chroma_hor_other_pat3  };
static const uint8_t* spat_8x8_chroma_ver [] = { spat_8x8_chroma_ver_first_pat0,  spat_8x8_chroma_ver_first_pat1,  spat_8x8_chroma_ver_first_pat2,  spat_8x8_chroma_ver_first_pat3,  spat_8x8_chroma_ver_other_pat0,  spat_8x8_chroma_ver_other_pat1,  spat_8x8_chroma_ver_other_pat2,  spat_8x8_chroma_ver_other_pat3  };
static const uint8_t* spat_nxn_chroma_diag[] = { spat_nxn_chroma_diag_first_pat0, spat_nxn_chroma_diag_first_pat1, spat_nxn_chroma_diag_first_pat2, spat_nxn_chroma_diag_first_pat3, spat_nxn_chroma_diag_other_pat0, spat_nxn_chroma_diag_other_pat1, spat_nxn_chroma_diag_other_pat2, spat_nxn_chroma_diag_other_pat3 };
static const uint8_t* spat_nxn_chroma_hor [] = { spat_nxn_chroma_hor_first_pat0,  spat_nxn_chroma_hor_first_pat1,  spat_nxn_chroma_hor_first_pat2,  spat_nxn_chroma_hor_first_pat3,  spat_nxn_chroma_hor_other_pat0,  spat_nxn_chroma_hor_other_pat1,  spat_nxn_chroma_hor_other_pat2,  spat_nxn_chroma_hor_other_pat3  };
static const uint8_t* spat_nxn_chroma_ver [] = { spat_nxn_chroma_ver_first_pat0,  spat_nxn_chroma_ver_first_pat1,  spat_nxn_chroma_ver_first_pat2,  spat_nxn_chroma_ver_first_pat3,  spat_nxn_chroma_ver_other_pat0,  spat_nxn_chroma_ver_other_pat1,  spat_nxn_chroma_ver_other_pat2,  spat_nxn_chroma_ver_other_pat3  };
static const uint8_t* spat_cg2_chroma_diag[] = { spat_cg2_chroma_diag_first_pat0, spat_cg2_chroma_diag_first_pat1, spat_cg2_chroma_diag_first_pat2, spat_cg2_chroma_diag_first_pat3, spat_cg2_chroma_diag_other_pat0, spat_cg2_chroma_diag_other_pat1, spat_cg2_chroma_diag_other_pat2, spat_cg2_chroma_diag_other_pat3 };
static const uint8_t* spat_cg2_chroma_hor [] = { spat_cg2_chroma_hor_first_pat0,  spat_cg2_chroma_hor_first_pat1,  spat_cg2_chroma_hor_first_pat2,  spat_cg2_chroma_hor_first_pat3,  spat_cg2_chroma_hor_other_pat0,  spat_cg2_chroma_hor_other_pat1,  spat_cg2_chroma_hor_other_pat2,  spat_cg2_chroma_hor_other_pat3  };
static const uint8_t* spat_cg2_chroma_ver [] = { spat_cg2_chroma_ver_first_pat0,  spat_cg2_chroma_ver_first_pat1,  spat_cg2_chroma_ver_first_pat2,  spat_cg2_chroma_ver_first_pat3,  spat_cg2_chroma_ver_other_pat0,  spat_cg2_chroma_ver_other_pat1,  spat_cg2_chroma_ver_other_pat2,  spat_cg2_chroma_ver_other_pat3  };

static const uint8_t** spat_sig_ctx[2][5][3] =
{
  {
    { spat_bypass_luma,     spat_bypass_luma,    spat_bypass_luma    },
    { spat_4x4_luma_diag,   spat_4x4_luma_hor,   spat_4x4_luma_ver   },
    { spat_8x8_luma_diag,   spat_8x8_luma_hor,   spat_8x8_luma_ver   },
    { spat_nxn_luma_diag,   spat_nxn_luma_hor,   spat_nxn_luma_ver   },
    { nullptr,              nullptr,             nullptr             }
  },
  {
    { spat_bypass_chroma,   spat_bypass_chroma,  spat_bypass_chroma  },
    { spat_4x4_chroma_diag, spat_4x4_chroma_hor, spat_4x4_chroma_ver },
    { spat_8x8_chroma_diag, spat_8x8_chroma_hor, spat_8x8_chroma_ver },
    { spat_nxn_chroma_diag, spat_nxn_chroma_hor, spat_nxn_chroma_ver },
    { spat_cg2_chroma_diag, spat_cg2_chroma_hor, spat_cg2_chroma_ver }
  }
};
#endif
#endif

#if HEVC_USE_SIGN_HIDING
CoeffCodingContext::CoeffCodingContext(const TransformUnit& tu, ComponentID component, bool signHide)
#else
CoeffCodingContext::CoeffCodingContext(const TransformUnit& tu, ComponentID component )
#endif
  : m_compID                    (component)
  , m_chType                    (toChannelType(m_compID))
  , m_width                     (tu.block(m_compID).width)
  , m_height                    (tu.block(m_compID).height)
  , m_log2CGWidth               ((m_width & 3) || (m_height & 3) ? 1 : 2)
  , m_log2CGHeight              ((m_width & 3) || (m_height & 3) ? 1 : 2)
  , m_log2CGSize                (m_log2CGWidth + m_log2CGHeight)
  , m_widthInGroups             (m_width  >> m_log2CGWidth)
  , m_heightInGroups            (m_height >> m_log2CGHeight)
  , m_log2BlockWidth            (g_aucLog2[m_width])
  , m_log2BlockHeight           (g_aucLog2[m_height])
  , m_log2BlockSize             ((m_log2BlockWidth + m_log2BlockHeight)>>1)
  , m_maxNumCoeff               (m_width * m_height)
#if JVET_K0072
#else
  , m_AlignFlag                 (tu.cs->sps->getSpsRangeExtension().getCabacBypassAlignmentEnabledFlag())
#endif
#if HEVC_USE_SIGN_HIDING
  , m_signHiding                (signHide)
#endif
#if JVET_K0072
#else
  , m_useGoRiceParAdapt         (tu.cs->sps->getSpsRangeExtension().getPersistentRiceAdaptationEnabledFlag())
#endif
  , m_extendedPrecision         (tu.cs->sps->getSpsRangeExtension().getExtendedPrecisionProcessingFlag())
  , m_maxLog2TrDynamicRange     (tu.cs->sps->getMaxLog2TrDynamicRange(m_chType))
#if HEVC_USE_MDCS
  , m_scanType                  (CoeffScanType(TU::getCoefScanIdx( tu, m_compID)))
#else
  , m_scanType                  (SCAN_DIAG)
#endif
  , m_scan                      (g_scanOrder     [SCAN_GROUPED_4x4][m_scanType][gp_sizeIdxInfo->idxFrom(m_width        )][gp_sizeIdxInfo->idxFrom(m_height        )])
  , m_scanPosX                  (g_scanOrderPosXY[SCAN_GROUPED_4x4][m_scanType][gp_sizeIdxInfo->idxFrom(m_width        )][gp_sizeIdxInfo->idxFrom(m_height        )][0])
  , m_scanPosY                  (g_scanOrderPosXY[SCAN_GROUPED_4x4][m_scanType][gp_sizeIdxInfo->idxFrom(m_width        )][gp_sizeIdxInfo->idxFrom(m_height        )][1])
  , m_scanCG                    (g_scanOrder[SCAN_UNGROUPED  ][m_scanType][gp_sizeIdxInfo->idxFrom(m_widthInGroups)][gp_sizeIdxInfo->idxFrom(m_heightInGroups)])
  , m_CtxSetLastX               (Ctx::LastX[m_chType])
  , m_CtxSetLastY               (Ctx::LastY[m_chType])
  , m_maxLastPosX               (g_uiGroupIdx[m_width - 1])
  , m_maxLastPosY               (g_uiGroupIdx[m_height - 1])
  , m_lastOffsetX               (0)
  , m_lastOffsetY               (0)
  , m_lastShiftX                (0)
  , m_lastShiftY                (0)
  , m_TrafoBypass               (tu.cs->sps->getSpsRangeExtension().getTransformSkipContextEnabledFlag() &&  (tu.cu->transQuantBypass || tu.transformSkip[m_compID]))
#if JVET_K0072
#else
  , m_SigBlockType              (m_TrafoBypass ? 0 : m_width == 4 && m_height == 4 ? 1 : m_width == 8 && m_height == 8 ? 2 : m_log2CGSize==2 ? 4 : 3 )
#if !HM_QTBT_AS_IN_JEM_CONTEXT
  , m_SigScanPatternBase        (spat_sig_ctx[m_chType][m_SigBlockType][m_scanType])
#endif
  , m_sigCtxSet                 (Ctx::SigFlag[m_chType])
#endif
  , m_scanPosLast               (-1)
  , m_subSetId                  (-1)
  , m_subSetPos                 (-1)
  , m_subSetPosX                (-1)
  , m_subSetPosY                (-1)
  , m_minSubPos                 (-1)
  , m_maxSubPos                 (-1)
  , m_sigGroupCtxId             (-1)
#if JVET_K0072
  , m_tmplCpSum1                (-1)
  , m_tmplCpDiag                (-1)
  , m_sigFlagCtxSet             { Ctx::SigFlag[m_chType], Ctx::SigFlag[m_chType+2], Ctx::SigFlag[m_chType+4] }
  , m_parFlagCtxSet             ( Ctx::ParFlag[m_chType] )
  , m_gtxFlagCtxSet             { Ctx::GtxFlag[m_chType], Ctx::GtxFlag[m_chType+2] }
#else
#if !HM_QTBT_AS_IN_JEM_CONTEXT
  , m_sigScanCtxId              (0)
#endif
  , m_gt1FlagCtxSet             (0, 0)
  , m_gt2FlagCtxId              (-1)
  , m_currentGolombRiceStatistic(-1)
  , m_prevGt2                   (false)
#endif
  , m_sigCoeffGroupFlag         ()
#if JVET_K0072
#else
#endif
#if JVET_K1000_SIMPLIFIED_EMT
  , m_emtNumSigCoeff            (0)
#endif
{
  // LOGTODO
  unsigned log2sizeX = m_log2BlockWidth;
  unsigned log2sizeY = m_log2BlockHeight;
#if HEVC_USE_MDCS
  if (m_scanType == SCAN_VER)
  {
    std::swap(log2sizeX, log2sizeY);
    std::swap(const_cast<unsigned&>(m_maxLastPosX), const_cast<unsigned&>(m_maxLastPosY));
  }
#endif
  if (m_chType == CHANNEL_TYPE_CHROMA)
  {
    if( tu.cs->pcv->rectCUs )
    {
#if HEVC_USE_MDCS
      const_cast<int&>(m_lastShiftX) = Clip3( 0, 2, int( ( m_scanType == SCAN_VER ? m_height : m_width  ) >> 3) );
      const_cast<int&>(m_lastShiftY) = Clip3( 0, 2, int( ( m_scanType == SCAN_VER ? m_width  : m_height ) >> 3) );
#else
      const_cast<int&>(m_lastShiftX) = Clip3( 0, 2, int( m_width  >> 3) );
      const_cast<int&>(m_lastShiftY) = Clip3( 0, 2, int( m_height >> 3) );
#endif
    }
    else
    {
      const_cast<int&>(m_lastShiftX) = log2sizeX - 2;
      const_cast<int&>(m_lastShiftY) = log2sizeY - 2;
    }
  }
  else
  {
    if( tu.cs->pcv->rectCUs )
    {
      static const int prefix_ctx[8]  = { 0, 0, 0, 3, 6, 10, 15, 21 };
      const_cast<int&>(m_lastOffsetX) = prefix_ctx[ log2sizeX ];
      const_cast<int&>(m_lastOffsetY) = prefix_ctx[ log2sizeY ];;
    }
    else
    {
      const_cast<int&>(m_lastOffsetX) = 3 * (log2sizeX - 2) + ((log2sizeX - 1) >> 2);
      const_cast<int&>(m_lastOffsetY) = 3 * (log2sizeY - 2) + ((log2sizeY - 1) >> 2);
    }
    const_cast<int&>(m_lastShiftX)  = (log2sizeX + 1) >> 2;
    const_cast<int&>(m_lastShiftY)  = (log2sizeY + 1) >> 2;
  }
#if JVET_K0072
#else
#endif
}

void CoeffCodingContext::initSubblock( int SubsetId, bool sigGroupFlag )
{
  m_subSetId                = SubsetId;
  m_subSetPos               = m_scanCG[ m_subSetId ];
  m_subSetPosY              = m_subSetPos / m_widthInGroups;
  m_subSetPosX              = m_subSetPos - ( m_subSetPosY * m_widthInGroups );
  m_minSubPos               = m_subSetId << m_log2CGSize;
  m_maxSubPos               = m_minSubPos + ( 1 << m_log2CGSize ) - 1;
  if( sigGroupFlag )
  {
    m_sigCoeffGroupFlag.set ( m_subSetPos );
  }
#if JVET_K0072
  unsigned  CGPosY    = m_subSetPosY;
  unsigned  CGPosX    = m_subSetPosX;
  unsigned  sigRight  = unsigned( ( CGPosX + 1 ) < m_widthInGroups  ? m_sigCoeffGroupFlag[ m_subSetPos + 1               ] : false );
  unsigned  sigLower  = unsigned( ( CGPosY + 1 ) < m_heightInGroups ? m_sigCoeffGroupFlag[ m_subSetPos + m_widthInGroups ] : false );
  m_sigGroupCtxId     = Ctx::SigCoeffGroup[m_chType]( sigRight | sigLower );
#else
  unsigned  CGPosY   = 0;
  unsigned  CGPosX   = 0;
  unsigned  sigRight = 0;
  unsigned  sigLower = 0;
  {
    CGPosY    = m_subSetPosY;
    CGPosX    = m_subSetPosX;
    sigRight  = unsigned( ( CGPosX + 1 ) < m_widthInGroups  ? m_sigCoeffGroupFlag[ m_subSetPos + 1               ] : false );
    sigLower  = unsigned( ( CGPosY + 1 ) < m_heightInGroups ? m_sigCoeffGroupFlag[ m_subSetPos + m_widthInGroups ] : false );
  }
  const unsigned  ctxSet    = m_prevGt2 + ( m_chType == CHANNEL_TYPE_LUMA ? ( m_subSetId > 0 ? 2 : 0 ) : 4 );
  m_sigGroupCtxId           = Ctx::SigCoeffGroup[m_chType]( sigRight | sigLower );
  m_gt1FlagCtxSet           = Ctx::GreaterOneFlag[ ctxSet ];
  m_gt2FlagCtxId            = Ctx::GreaterTwoFlag( ctxSet );
#if HM_QTBT_AS_IN_JEM_CONTEXT
  m_sigCGPattern            = sigRight + ( sigLower << 1 );
#else
  m_sigScanCtxId            = m_SigScanPatternBase[ sigRight + ( sigLower << 1 ) + ( m_subSetId ? 4 : 0 ) ] - m_minSubPos;
#endif

#endif
}


#if JVET_K0072
#else
#if HM_QTBT_AS_IN_JEM_CONTEXT // ctx modeling for subblocks != 4x4
unsigned CoeffCodingContext::sigCtxId( int scanPos ) const
{
  int offset = 0; // DC

  if( m_SigBlockType == 0 ) // bypass
  {
    offset = ( m_chType == CHANNEL_TYPE_LUMA ? 27 : 15 );
  }
  else if( scanPos )
  {
    const unsigned posY       = m_scanPosY[ scanPos ];
    const unsigned posX       = m_scanPosX[ scanPos ];

    if( m_SigBlockType == 1 ) // 4x4
    {
      //      const unsigned ctxIndMap4x4[16] = { 0, 1, 4, 5, 2, 3, 4, 5, 6, 6, 8, 8, 7, 7, 8, 8 };
      offset = ctxIndMap4x4[ ( posY << 2 ) + posX ];
    }
    else
    {
      int cnt = 0;
      switch( m_sigCGPattern )
      {
      case 0:
      {
        unsigned posIS  = ( posX & 3 ) + ( posY & 3 );
        cnt             = ( posIS >= 3 ? 0 : posIS >= 1 ? 1 : 2 );
      }
      break;
      case 1:
      {
        unsigned posIS  = ( posY & 3 );
        cnt             = ( posIS >= 2 ? 0 : posIS >= 1 ? 1 : 2 );
      }
      break;
      case 2:
      {
        unsigned posIS  = ( posX & 3 );
        cnt             = ( posIS >= 2 ? 0 : posIS >= 1 ? 1 : 2 );
      }
      break;
      case 3:
      {
        cnt             = 2;
      }
      break;
      default:
        THROW( "sig pattern must be in range [0;3]" );
      }
      offset    = ( m_chType == CHANNEL_TYPE_LUMA && ( posX > 3 || posY > 3 ) ? 3 : 0 ) + cnt;

      if( m_SigBlockType == 2 ) // 8x8
      {
        offset += ( m_scanType != SCAN_DIAG && m_chType == CHANNEL_TYPE_LUMA ? 15 : 9 );
      }
      else // NxN
      {
        offset += ( m_chType == CHANNEL_TYPE_LUMA ? 21 : 12 );
      }
    }
  }
  return m_sigCtxSet( offset );
}
#endif
#endif


unsigned DeriveCtx::CtxCUsplit( const CodingStructure& cs, Partitioner& partitioner )
{
  auto adPartitioner = dynamic_cast<AdaptiveDepthPartitioner*>( &partitioner );

  if( !adPartitioner )
  {
    return 0;
  }

  const Position pos         = partitioner.currArea().blocks[partitioner.chType];
  const unsigned curSliceIdx = cs.slice->getIndependentSliceIdx();
#if HEVC_TILES_WPP
  const unsigned curTileIdx  = cs.picture->tileMap->getTileIdxMap( partitioner.currArea().lumaPos() );
#endif
  unsigned ctxId = 0;

  // get left depth
#if HEVC_TILES_WPP
  const CodingUnit* cuLeft = cs.getCURestricted( pos.offset( -1, 0 ), curSliceIdx, curTileIdx, partitioner.chType );
#else
  const CodingUnit* cuLeft = cs.getCURestricted( pos.offset( -1, 0 ), curSliceIdx, partitioner.chType );
#endif
  ctxId = ( cuLeft && cuLeft->qtDepth > partitioner.currQtDepth ) ? 1 : 0;

  // get above depth
#if HEVC_TILES_WPP
  const CodingUnit* cuAbove = cs.getCURestricted( pos.offset( 0, -1 ), curSliceIdx, curTileIdx, partitioner.chType );
#else
  const CodingUnit* cuAbove = cs.getCURestricted( pos.offset( 0, -1 ), curSliceIdx, partitioner.chType );
#endif

  ctxId += ( cuAbove && cuAbove->qtDepth > partitioner.currQtDepth ) ? 1 : 0;

  if( cs.sps->getSpsNext().getUseLargeCTU() )
  {
    unsigned minDepth = 0;
    unsigned maxDepth = 0;
    adPartitioner->setMaxMinDepth( minDepth, maxDepth, cs );
    if( partitioner.currDepth < minDepth )
    {
      ctxId = 3;
    }
    else if( partitioner.currDepth >= maxDepth + 1 )
    {
      ctxId = 4;
    }
  }

  return ctxId;
}

#if ENABLE_BMS
#if JVET_K0072
unsigned DeriveCtx::CtxQtCbf( const ComponentID compID, const unsigned trDepth, const bool prevCbCbf )
#else
unsigned DeriveCtx::CtxQtCbf( const ComponentID compID, const unsigned trDepth )
#endif
#else
#if JVET_K0072
unsigned DeriveCtx::CtxQtCbf( const ComponentID compID, const bool prevCbCbf )
#else
unsigned DeriveCtx::CtxQtCbf( const ComponentID compID )
#endif
#endif
{
#if JVET_K0072
  if( compID == COMPONENT_Cr )
  {
    return ( prevCbCbf ? 1 : 0 );
  }
#endif
#if ENABLE_BMS
  if( isChroma( compID ) )
  {
    return trDepth;
  }
  else
  {
    return ( trDepth == 0 ? 1 : 0 );
  }
#else
  return isChroma( compID ) ? 0 : 1;
#endif
}

unsigned DeriveCtx::CtxInterDir( const PredictionUnit& pu )
{
  if( pu.cs->sps->getSpsNext().getUseLargeCTU() )
  {
    if( pu.cs->pcv->rectCUs )
    {
      return Clip3( 0, 3, 7 - ( ( g_aucLog2[pu.lumaSize().width] + g_aucLog2[pu.lumaSize().height] + 1 ) >> 1 ) );    // VG-ASYMM DONE
    }
    return Clip3( 0, 3, 6 - g_aucLog2[pu.cu->lumaSize().width] );
  }
  return pu.cu->qtDepth;
}

#if JVET_K_AFFINE
unsigned DeriveCtx::CtxAffineFlag( const CodingUnit& cu )
{
  const CodingStructure *cs = cu.cs;
  unsigned ctxId = 0;

  const CodingUnit *cuLeft = cs->getCURestricted( cu.lumaPos().offset( -1, 0 ), cu, CH_L );
  ctxId = ( cuLeft && cuLeft->affine ) ? 1 : 0;

  const CodingUnit *cuAbove = cs->getCURestricted( cu.lumaPos().offset( 0, -1 ), cu, CH_L );
  ctxId += ( cuAbove && cuAbove->affine ) ? 1 : 0;

  return ctxId;
}
#endif
unsigned DeriveCtx::CtxSkipFlag( const CodingUnit& cu )
{
  const CodingStructure *cs = cu.cs;
  unsigned ctxId = 0;

  // Get BCBP of left PU
  const CodingUnit *cuLeft = cs->getCURestricted( cu.lumaPos().offset( -1, 0 ), cu, CH_L );
  ctxId = ( cuLeft && cuLeft->skip ) ? 1 : 0;

  // Get BCBP of above PU
  const CodingUnit *cuAbove = cs->getCURestricted( cu.lumaPos().offset( 0, -1 ), cu, CH_L );
  ctxId += ( cuAbove && cuAbove->skip ) ? 1 : 0;

  return ctxId;
}


#if JVET_K0357_AMVR
unsigned DeriveCtx::CtxIMVFlag( const CodingUnit& cu )
{
  const CodingStructure *cs = cu.cs;
  unsigned ctxId = 0;

  // Get BCBP of left PU
  const CodingUnit *cuLeft = cs->getCURestricted( cu.lumaPos().offset( -1, 0 ), cu, CH_L );
  ctxId = ( cuLeft && cuLeft->imv ) ? 1 : 0;

  // Get BCBP of above PU
  const CodingUnit *cuAbove = cs->getCURestricted( cu.lumaPos().offset( 0, -1 ), cu, CH_L );
  ctxId += ( cuAbove && cuAbove->imv ) ? 1 : 0;

  return ctxId;
}
#endif

unsigned DeriveCtx::CtxBTsplit(const CodingStructure& cs, Partitioner& partitioner)
{
  const Position pos          = partitioner.currArea().blocks[partitioner.chType];
  const unsigned curSliceIdx  = cs.slice->getIndependentSliceIdx();
#if HEVC_TILES_WPP
  const unsigned curTileIdx   = cs.picture->tileMap->getTileIdxMap( pos );
#endif

  unsigned ctx                = 0;

#if HEVC_TILES_WPP
  const CodingUnit *cuLeft    = cs.getCURestricted( pos.offset( -1,  0 ), curSliceIdx, curTileIdx, partitioner.chType );
  const CodingUnit *cuAbove   = cs.getCURestricted( pos.offset(  0, -1 ), curSliceIdx, curTileIdx, partitioner.chType );
#else
  const CodingUnit *cuLeft    = cs.getCURestricted( pos.offset( -1,  0 ), curSliceIdx, partitioner.chType );
  const CodingUnit *cuAbove   = cs.getCURestricted( pos.offset(  0, -1 ), curSliceIdx, partitioner.chType );
#endif

  {
    const unsigned currDepth = partitioner.currQtDepth * 2 + partitioner.currBtDepth;

    if( cuLeft )  ctx += ( ( 2 * cuLeft->qtDepth  + cuLeft->btDepth  ) > currDepth ? 1 : 0 );
    if( cuAbove ) ctx += ( ( 2 * cuAbove->qtDepth + cuAbove->btDepth ) > currDepth ? 1 : 0 );
  }
  return ctx;
}


void MergeCtx::setMergeInfo( PredictionUnit& pu, int candIdx )
{
  CHECK( candIdx >= numValidMergeCand, "Merge candidate does not exist" );

  pu.mergeFlag               = true;
  pu.interDir                = interDirNeighbours[candIdx];
  pu.mergeIdx                = candIdx;
  pu.mergeType               = mrgTypeNeighbours[candIdx];
  pu.mv     [REF_PIC_LIST_0] = mvFieldNeighbours[(candIdx << 1) + 0].mv;
  pu.mv     [REF_PIC_LIST_1] = mvFieldNeighbours[(candIdx << 1) + 1].mv;
  pu.mvd    [REF_PIC_LIST_0] = Mv();
  pu.mvd    [REF_PIC_LIST_1] = Mv();
  pu.refIdx [REF_PIC_LIST_0] = mvFieldNeighbours[( candIdx << 1 ) + 0].refIdx;
  pu.refIdx [REF_PIC_LIST_1] = mvFieldNeighbours[( candIdx << 1 ) + 1].refIdx;
  pu.mvpIdx [REF_PIC_LIST_0] = NOT_VALID;
  pu.mvpIdx [REF_PIC_LIST_1] = NOT_VALID;
  pu.mvpNum [REF_PIC_LIST_0] = NOT_VALID;
  pu.mvpNum [REF_PIC_LIST_1] = NOT_VALID;

  
}

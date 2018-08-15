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

/** \file     CABACReader.h
 *  \brief    Reader for low level syntax
 */

#ifndef __CABACREADER__
#define __CABACREADER__

#include "BinDecoder.h"

#include "CommonLib/ContextModelling.h"
#include "CommonLib/MotionInfo.h"
#include "CommonLib/UnitPartitioner.h"


class CABACReader
{
public:
  CABACReader( BinDecoderBase& binDecoder ) : m_BinDecoder( binDecoder ), m_Bitstream( 0 ) {}
  virtual ~CABACReader() {}

public:
  void        initCtxModels             ( Slice&                        slice );
  void        initBitstream             ( InputBitstream*               bitstream )           { m_Bitstream = bitstream; m_BinDecoder.init( m_Bitstream ); }
  const Ctx&  getCtx                    ()                                            const   { return m_BinDecoder.getCtx();  }
  Ctx&        getCtx                    ()                                                    { return m_BinDecoder.getCtx();  }

public:
  // slice segment data (clause 7.3.8.1)
  bool        terminating_bit           ();
  void        remaining_bytes           ( bool                          noTrailingBytesExpected );

  // coding tree unit (clause 7.3.8.2)
  bool        coding_tree_unit          ( CodingStructure&              cs,     const UnitArea& area,     int (&qps)[2],   unsigned  ctuRsAddr );

  // sao (clause 7.3.8.3)
  void        sao                       ( CodingStructure&              cs,     unsigned        ctuRsAddr );

  // coding (quad)tree (clause 7.3.8.4)
#if JVET_K0230_DUAL_CODING_TREE_UNDER_64x64_BLOCK
  bool        coding_tree               ( CodingStructure&              cs,     Partitioner&    pm,       CUCtx& cuCtx, Partitioner* pPartitionerChroma = nullptr, CUCtx* pCuCtxChroma = nullptr);
#else
  bool        coding_tree               ( CodingStructure&              cs,     Partitioner&    pm,       CUCtx& cuCtx );
#endif
  bool        split_cu_flag             ( CodingStructure&              cs,     Partitioner&    pm );
  PartSplit   split_cu_mode_mt          ( CodingStructure&              cs,     Partitioner&    pm );

  // coding unit (clause 7.3.8.5)
  bool        coding_unit               ( CodingUnit&                   cu,     Partitioner&    pm,       CUCtx& cuCtx );
  void        cu_transquant_bypass_flag ( CodingUnit&                   cu );
  void        cu_skip_flag              ( CodingUnit&                   cu );
  void        pred_mode                 ( CodingUnit&                   cu );
  void        pcm_flag                  ( CodingUnit&                   cu );
  void        cu_pred_data              ( CodingUnit&                   cu );
  void        intra_luma_pred_modes     ( CodingUnit&                   cu );
  void        intra_chroma_pred_modes   ( CodingUnit&                   cu );
#if JVET_K0190
  bool        intra_chroma_lmc_mode     ( PredictionUnit&               pu );
#endif
  void        intra_chroma_pred_mode    ( PredictionUnit&               pu );
  void        cu_residual               ( CodingUnit&                   cu,     Partitioner&    pm,       CUCtx& cuCtx );
  void        rqt_root_cbf              ( CodingUnit&                   cu );
  bool        end_of_ctu                ( CodingUnit&                   cu,     CUCtx&          cuCtx );

  // prediction unit (clause 7.3.8.6)
  void        prediction_unit           ( PredictionUnit&               pu,     MergeCtx&       mrgCtx );
  void        merge_flag                ( PredictionUnit&               pu );
  void        merge_data                ( PredictionUnit&               pu );
#if JVET_K_AFFINE
  void        affine_flag               ( CodingUnit&                   cu );
#endif
  void        merge_idx                 ( PredictionUnit&               pu );
#if JVET_K0357_AMVR
  void        imv_mode                  ( CodingUnit&                   cu,     MergeCtx&       mrgCtx );
#endif
  void        inter_pred_idc            ( PredictionUnit&               pu );
  void        ref_idx                   ( PredictionUnit&               pu,     RefPicList      eRefList );
  void        mvp_flag                  ( PredictionUnit&               pu,     RefPicList      eRefList );

  // pcm samples (clause 7.3.8.7)
  void        pcm_samples               ( TransformUnit&                tu );

  // transform tree (clause 7.3.8.8)
  void        transform_tree            ( CodingStructure&              cs,     Partitioner&    pm,       CUCtx& cuCtx,  ChromaCbfs& chromaCbfs );
#if ENABLE_BMS
#if JVET_K0072
  bool        cbf_comp                  ( CodingStructure&              cs,     const CompArea& area,     unsigned depth, const bool prevCbCbf = false );
#else
  bool        cbf_comp                  ( CodingStructure&              cs,     const CompArea& area,     unsigned depth );
#endif
#else
#if JVET_K0072
  bool        cbf_comp                  ( CodingStructure&              cs,     const CompArea& area,     const bool prevCbCbf = false );
#else
  bool        cbf_comp                  ( CodingStructure&              cs,     const CompArea& area );
#endif
#endif

  // mvd coding (clause 7.3.8.9)
  void        mvd_coding                ( Mv &rMvd );

  // transform unit (clause 7.3.8.10)
  void        transform_unit            ( TransformUnit&                tu,     CUCtx&          cuCtx,  ChromaCbfs& chromaCbfs );
#if HM_QTBT_AS_IN_JEM_SYNTAX
  void        transform_unit_qtbt       ( TransformUnit&                tu,     CUCtx&          cuCtx,  ChromaCbfs& chromaCbfs );
#endif
  void        cu_qp_delta               ( CodingUnit&                   cu,     int             predQP, int8_t& qp );
  void        cu_chroma_qp_offset       ( CodingUnit&                   cu );
#if JVET_K1000_SIMPLIFIED_EMT && !HM_EMT_NSST_AS_IN_JEM
  void        cu_emt_pertu_idx          ( CodingUnit&                   cu );
#endif

  // residual coding (clause 7.3.8.11)
  void        residual_coding           ( TransformUnit&                tu,     ComponentID     compID );
  void        transform_skip_flag       ( TransformUnit&                tu,     ComponentID     compID );
#if JVET_K1000_SIMPLIFIED_EMT
  void        emt_tu_index              ( TransformUnit&                tu );
  void        emt_cu_flag               ( CodingUnit&                   cu );
#endif
  void        explicit_rdpcm_mode       ( TransformUnit&                tu,     ComponentID     compID );
  int         last_sig_coeff            ( CoeffCodingContext&           cctx );
#if JVET_K0072
  void        residual_coding_subblock  ( CoeffCodingContext&           cctx,   TCoeff*         coeff, const int stateTransTable, int& state );
#else
  void        residual_coding_subblock  ( CoeffCodingContext&           cctx,   TCoeff*         coeff  );
#endif

  // cross component prediction (clause 7.3.8.12)
  void        cross_comp_pred           ( TransformUnit&                tu,     ComponentID     compID );

private:
  unsigned    unary_max_symbol          ( unsigned ctxId0, unsigned ctxIdN, unsigned maxSymbol );
  unsigned    unary_max_eqprob          (                                   unsigned maxSymbol );
  unsigned    exp_golomb_eqprob         ( unsigned count );
  unsigned    decode_sparse_dt          ( DecisionTree& dt );
  unsigned    get_num_bits_read         () { return m_BinDecoder.getNumBitsRead(); }

private:
  BinDecoderBase& m_BinDecoder;
  InputBitstream* m_Bitstream;
};


class CABACDecoder
{
public:
  CABACDecoder()
    : m_CABACReaderStd  ( m_BinDecoderStd )
    , m_CABACReader     { &m_CABACReaderStd }
  {}

  CABACReader*                getCABACReader    ( int           id    )       { return m_CABACReader[id]; }

private:
  BinDecoder_Std          m_BinDecoderStd;
  CABACReader             m_CABACReaderStd;
  CABACReader*            m_CABACReader[BPM_NUM-1];
};

#endif

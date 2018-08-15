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

/** \file     VLCWriter.cpp
 *  \brief    Writer for high level syntax
 */

#include "VLCWriter.h"
#include "SEIwrite.h"

#include "CommonLib/CommonDef.h"
#include "CommonLib/Unit.h"
#include "CommonLib/Picture.h" // th remove this
#include "CommonLib/dtrace_next.h"
#if JVET_K0371_ALF
#include "EncAdaptiveLoopFilter.h"
#include "CommonLib/AdaptiveLoopFilter.h"
#endif

//! \ingroup EncoderLib
//! \{

#if ENABLE_TRACING

void  VLCWriter::xWriteCodeTr (uint32_t value, uint32_t  length, const char *pSymbolName)
{
  xWriteCode (value,length);

  if( g_HLSTraceEnable )
  {
    if( length < 10 )
    {
      DTRACE( g_trace_ctx, D_HEADER, "%-50s u(%d)  : %d\n", pSymbolName, length, value );
    }
    else
    {
      DTRACE( g_trace_ctx, D_HEADER, "%-50s u(%d) : %d\n", pSymbolName, length, value );
    }
  }
}

void  VLCWriter::xWriteUvlcTr (uint32_t value, const char *pSymbolName)
{
  xWriteUvlc (value);
  if( g_HLSTraceEnable )
  {
    DTRACE( g_trace_ctx, D_HEADER, "%-50s ue(v) : %d\n", pSymbolName, value );
  }
}

void  VLCWriter::xWriteSvlcTr (int value, const char *pSymbolName)
{
  xWriteSvlc(value);
  if( g_HLSTraceEnable )
  {
    DTRACE( g_trace_ctx, D_HEADER, "%-50s se(v) : %d\n", pSymbolName, value );
  }
}

void  VLCWriter::xWriteFlagTr(uint32_t value, const char *pSymbolName)
{
  xWriteFlag(value);
  if( g_HLSTraceEnable )
  {
    DTRACE( g_trace_ctx, D_HEADER, "%-50s u(1)  : %d\n", pSymbolName, value );
  }
}

bool g_HLSTraceEnable = true;

#endif


void VLCWriter::xWriteCode     ( uint32_t uiCode, uint32_t uiLength )
{
  CHECK( uiLength == 0, "Code of lenght '0' not supported" );
  m_pcBitIf->write( uiCode, uiLength );
}

void VLCWriter::xWriteUvlc     ( uint32_t uiCode )
{
  uint32_t uiLength = 1;
  uint32_t uiTemp = ++uiCode;

  CHECK( !uiTemp, "Integer overflow" );

  while( 1 != uiTemp )
  {
    uiTemp >>= 1;
    uiLength += 2;
  }
  // Take care of cases where uiLength > 32
  m_pcBitIf->write( 0, uiLength >> 1);
  m_pcBitIf->write( uiCode, (uiLength+1) >> 1);
}

void VLCWriter::xWriteSvlc     ( int iCode )
{
  uint32_t uiCode = uint32_t( iCode <= 0 ? (-iCode)<<1 : (iCode<<1)-1);
  xWriteUvlc( uiCode );
}

void VLCWriter::xWriteFlag( uint32_t uiCode )
{
  m_pcBitIf->write( uiCode, 1 );
}

void VLCWriter::xWriteRbspTrailingBits()
{
  WRITE_FLAG( 1, "rbsp_stop_one_bit");
  int cnt = 0;
  while (m_pcBitIf->getNumBitsUntilByteAligned())
  {
    WRITE_FLAG( 0, "rbsp_alignment_zero_bit");
    cnt++;
  }
  CHECK(cnt>=8, "More than '8' alignment bytes read");
}

void AUDWriter::codeAUD(OutputBitstream& bs, const int pictureType)
{
#if ENABLE_TRACING
  xTraceAccessUnitDelimiter();
#endif

  CHECK(pictureType >= 3, "Invalid picture type");
  setBitstream(&bs);
  WRITE_CODE(pictureType, 3, "pic_type");
  xWriteRbspTrailingBits();
}

void HLSWriter::xCodeShortTermRefPicSet( const ReferencePictureSet* rps, bool calledFromSliceHeader, int idx)
{
  //int lastBits = getNumberOfWrittenBits();

  if (idx > 0)
  {
    WRITE_FLAG( rps->getInterRPSPrediction(), "inter_ref_pic_set_prediction_flag" ); // inter_RPS_prediction_flag
  }
  if (rps->getInterRPSPrediction())
  {
    int deltaRPS = rps->getDeltaRPS();
    if(calledFromSliceHeader)
    {
      WRITE_UVLC( rps->getDeltaRIdxMinus1(), "delta_idx_minus1" ); // delta index of the Reference Picture Set used for prediction minus 1
    }

    WRITE_CODE( (deltaRPS >=0 ? 0: 1), 1, "delta_rps_sign" ); //delta_rps_sign
    WRITE_UVLC( abs(deltaRPS) - 1, "abs_delta_rps_minus1"); // absolute delta RPS minus 1

    for(int j=0; j < rps->getNumRefIdc(); j++)
    {
      int refIdc = rps->getRefIdc(j);
      WRITE_CODE( (refIdc==1? 1: 0), 1, "used_by_curr_pic_flag" ); //first bit is "1" if Idc is 1
      if (refIdc != 1)
      {
        WRITE_CODE( refIdc>>1, 1, "use_delta_flag" ); //second bit is "1" if Idc is 2, "0" otherwise.
      }
    }
  }
  else
  {
    WRITE_UVLC( rps->getNumberOfNegativePictures(), "num_negative_pics" );
    WRITE_UVLC( rps->getNumberOfPositivePictures(), "num_positive_pics" );
    int prev = 0;
    for(int j=0 ; j < rps->getNumberOfNegativePictures(); j++)
    {
      WRITE_UVLC( prev-rps->getDeltaPOC(j)-1, "delta_poc_s0_minus1" );
      prev = rps->getDeltaPOC(j);
      WRITE_FLAG( rps->getUsed(j), "used_by_curr_pic_s0_flag");
    }
    prev = 0;
    for(int j=rps->getNumberOfNegativePictures(); j < rps->getNumberOfNegativePictures()+rps->getNumberOfPositivePictures(); j++)
    {
      WRITE_UVLC( rps->getDeltaPOC(j)-prev-1, "delta_poc_s1_minus1" );
      prev = rps->getDeltaPOC(j);
      WRITE_FLAG( rps->getUsed(j), "used_by_curr_pic_s1_flag" );
    }
  }

  //DTRACE( g_trace_ctx, D_RPSINFO, "irps=%d (%2d bits) ", rps->getInterRPSPrediction(), getNumberOfWrittenBits() - lastBits );
  rps->printDeltaPOC();
}

void HLSWriter::codePPS( const PPS* pcPPS )
{
#if ENABLE_TRACING
  xTracePPSHeader ();
#endif

  WRITE_UVLC( pcPPS->getPPSId(),                             "pps_pic_parameter_set_id" );
  WRITE_UVLC( pcPPS->getSPSId(),                             "pps_seq_parameter_set_id" );
#if HEVC_DEPENDENT_SLICES
  WRITE_FLAG( pcPPS->getDependentSliceSegmentsEnabledFlag()    ? 1 : 0, "dependent_slice_segments_enabled_flag" );
#endif
  WRITE_FLAG( pcPPS->getOutputFlagPresentFlag() ? 1 : 0,     "output_flag_present_flag" );
  WRITE_CODE( pcPPS->getNumExtraSliceHeaderBits(), 3,        "num_extra_slice_header_bits");
#if JVET_K0072
#else
#if HEVC_USE_SIGN_HIDING
  WRITE_FLAG( pcPPS->getSignDataHidingEnabledFlag(),         "sign_data_hiding_enabled_flag" );
#endif
#endif
  WRITE_FLAG( pcPPS->getCabacInitPresentFlag() ? 1 : 0,   "cabac_init_present_flag" );
  WRITE_UVLC( pcPPS->getNumRefIdxL0DefaultActive()-1,     "num_ref_idx_l0_default_active_minus1");
  WRITE_UVLC( pcPPS->getNumRefIdxL1DefaultActive()-1,     "num_ref_idx_l1_default_active_minus1");

  WRITE_SVLC( pcPPS->getPicInitQPMinus26(),                  "init_qp_minus26");
  WRITE_FLAG( pcPPS->getConstrainedIntraPred() ? 1 : 0,      "constrained_intra_pred_flag" );
  WRITE_FLAG( pcPPS->getUseTransformSkip() ? 1 : 0,  "transform_skip_enabled_flag" );
  WRITE_FLAG( pcPPS->getUseDQP() ? 1 : 0, "cu_qp_delta_enabled_flag" );
  if ( pcPPS->getUseDQP() )
  {
    WRITE_UVLC( pcPPS->getMaxCuDQPDepth(), "diff_cu_qp_delta_depth" );
  }

  WRITE_SVLC( pcPPS->getQpOffset(COMPONENT_Cb), "pps_cb_qp_offset" );
  WRITE_SVLC( pcPPS->getQpOffset(COMPONENT_Cr), "pps_cr_qp_offset" );

  WRITE_FLAG( pcPPS->getSliceChromaQpFlag() ? 1 : 0,          "pps_slice_chroma_qp_offsets_present_flag" );

  WRITE_FLAG( pcPPS->getUseWP() ? 1 : 0,  "weighted_pred_flag" );   // Use of Weighting Prediction (P_SLICE)
  WRITE_FLAG( pcPPS->getWPBiPred() ? 1 : 0, "weighted_bipred_flag" );  // Use of Weighting Bi-Prediction (B_SLICE)
  WRITE_FLAG( pcPPS->getTransquantBypassEnabledFlag()  ? 1 : 0, "transquant_bypass_enabled_flag" );
#if HEVC_TILES_WPP
  WRITE_FLAG( pcPPS->getTilesEnabledFlag() ? 1 : 0, "tiles_enabled_flag" );
  WRITE_FLAG( pcPPS->getEntropyCodingSyncEnabledFlag() ? 1 : 0, "entropy_coding_sync_enabled_flag" );
  if( pcPPS->getTilesEnabledFlag() )
  {
    WRITE_UVLC( pcPPS->getNumTileColumnsMinus1(),                                    "num_tile_columns_minus1" );
    WRITE_UVLC( pcPPS->getNumTileRowsMinus1(),                                       "num_tile_rows_minus1" );
    WRITE_FLAG( pcPPS->getTileUniformSpacingFlag(),                                  "uniform_spacing_flag" );
    if( !pcPPS->getTileUniformSpacingFlag() )
    {
      for(uint32_t i=0; i<pcPPS->getNumTileColumnsMinus1(); i++)
      {
        WRITE_UVLC( pcPPS->getTileColumnWidth(i)-1,                                  "column_width_minus1" );
      }
      for(uint32_t i=0; i<pcPPS->getNumTileRowsMinus1(); i++)
      {
        WRITE_UVLC( pcPPS->getTileRowHeight(i)-1,                                    "row_height_minus1" );
      }
    }
    CHECK ((pcPPS->getNumTileColumnsMinus1() + pcPPS->getNumTileRowsMinus1()) == 0, "Invalid tile parameters read");
    WRITE_FLAG( pcPPS->getLoopFilterAcrossTilesEnabledFlag()?1 : 0,       "loop_filter_across_tiles_enabled_flag");
  }
#endif
  WRITE_FLAG( pcPPS->getLoopFilterAcrossSlicesEnabledFlag()?1 : 0,        "pps_loop_filter_across_slices_enabled_flag");
  WRITE_FLAG( pcPPS->getDeblockingFilterControlPresentFlag()?1 : 0,       "deblocking_filter_control_present_flag");
  if(pcPPS->getDeblockingFilterControlPresentFlag())
  {
    WRITE_FLAG( pcPPS->getDeblockingFilterOverrideEnabledFlag() ? 1 : 0,  "deblocking_filter_override_enabled_flag" );
    WRITE_FLAG( pcPPS->getPPSDeblockingFilterDisabledFlag() ? 1 : 0,      "pps_deblocking_filter_disabled_flag" );
    if(!pcPPS->getPPSDeblockingFilterDisabledFlag())
    {
      WRITE_SVLC( pcPPS->getDeblockingFilterBetaOffsetDiv2(),             "pps_beta_offset_div2" );
      WRITE_SVLC( pcPPS->getDeblockingFilterTcOffsetDiv2(),               "pps_tc_offset_div2" );
    }
  }
#if HEVC_USE_SCALING_LISTS
  WRITE_FLAG( pcPPS->getScalingListPresentFlag() ? 1 : 0,                          "pps_scaling_list_data_present_flag" );
  if( pcPPS->getScalingListPresentFlag() )
  {
    codeScalingList( pcPPS->getScalingList() );
  }
#endif
  WRITE_FLAG( pcPPS->getListsModificationPresentFlag(), "lists_modification_present_flag");
  WRITE_UVLC( pcPPS->getLog2ParallelMergeLevelMinus2(), "log2_parallel_merge_level_minus2");
  WRITE_FLAG( pcPPS->getSliceHeaderExtensionPresentFlag() ? 1 : 0, "slice_segment_header_extension_present_flag");

  bool pps_extension_present_flag=false;
  bool pps_extension_flags[NUM_PPS_EXTENSION_FLAGS]={false};

  pps_extension_flags[PPS_EXT__REXT] = pcPPS->getPpsRangeExtension().settingsDifferFromDefaults(pcPPS->getUseTransformSkip());

  // Other PPS extension flags checked here.

  for(int i=0; i<NUM_PPS_EXTENSION_FLAGS; i++)
  {
    pps_extension_present_flag|=pps_extension_flags[i];
  }

  WRITE_FLAG( (pps_extension_present_flag?1:0), "pps_extension_present_flag" );

  if (pps_extension_present_flag)
  {
#if ENABLE_TRACING /*|| RExt__DECODER_DEBUG_BIT_STATISTICS*/
    static const char *syntaxStrings[]={ "pps_range_extension_flag",
      "pps_multilayer_extension_flag",
      "pps_extension_6bits[0]",
      "pps_extension_6bits[1]",
      "pps_extension_6bits[2]",
      "pps_extension_6bits[3]",
      "pps_extension_6bits[4]",
      "pps_extension_6bits[5]" };
#endif

    for(int i=0; i<NUM_PPS_EXTENSION_FLAGS; i++)
    {
      WRITE_FLAG( pps_extension_flags[i]?1:0, syntaxStrings[i] );
    }

    for(int i=0; i<NUM_PPS_EXTENSION_FLAGS; i++) // loop used so that the order is determined by the enum.
    {
      if (pps_extension_flags[i])
      {
        switch (PPSExtensionFlagIndex(i))
        {
        case PPS_EXT__REXT:
        {
          const PPSRExt &ppsRangeExtension = pcPPS->getPpsRangeExtension();
          if (pcPPS->getUseTransformSkip())
          {
            WRITE_UVLC( ppsRangeExtension.getLog2MaxTransformSkipBlockSize()-2,            "log2_max_transform_skip_block_size_minus2");
          }

          WRITE_FLAG((ppsRangeExtension.getCrossComponentPredictionEnabledFlag() ? 1 : 0), "cross_component_prediction_enabled_flag" );

          WRITE_FLAG(uint32_t(ppsRangeExtension.getChromaQpOffsetListEnabledFlag()),           "chroma_qp_offset_list_enabled_flag" );
          if (ppsRangeExtension.getChromaQpOffsetListEnabledFlag())
          {
            WRITE_UVLC(ppsRangeExtension.getDiffCuChromaQpOffsetDepth(),                   "diff_cu_chroma_qp_offset_depth");
            WRITE_UVLC(ppsRangeExtension.getChromaQpOffsetListLen() - 1,                   "chroma_qp_offset_list_len_minus1");
            /* skip zero index */
            for (int cuChromaQpOffsetIdx = 0; cuChromaQpOffsetIdx < ppsRangeExtension.getChromaQpOffsetListLen(); cuChromaQpOffsetIdx++)
            {
              WRITE_SVLC(ppsRangeExtension.getChromaQpOffsetListEntry(cuChromaQpOffsetIdx+1).u.comp.CbOffset,     "cb_qp_offset_list[i]");
              WRITE_SVLC(ppsRangeExtension.getChromaQpOffsetListEntry(cuChromaQpOffsetIdx+1).u.comp.CrOffset,     "cr_qp_offset_list[i]");
            }
          }

          WRITE_UVLC( ppsRangeExtension.getLog2SaoOffsetScale(CHANNEL_TYPE_LUMA),           "log2_sao_offset_scale_luma"   );
          WRITE_UVLC( ppsRangeExtension.getLog2SaoOffsetScale(CHANNEL_TYPE_CHROMA),         "log2_sao_offset_scale_chroma" );
        }
        break;
        default:
          CHECK(pps_extension_flags[i]==false, "Unknown PPS extension signalled"); // Should never get here with an active PPS extension flag.
          break;
        } // switch
      } // if flag present
    } // loop over PPS flags
  } // pps_extension_present_flag is non-zero
  xWriteRbspTrailingBits();
}

void HLSWriter::codeVUI( const VUI *pcVUI, const SPS* pcSPS )
{
#if ENABLE_TRACING
  DTRACE( g_trace_ctx, D_HEADER, "----------- vui_parameters -----------\n");
#endif
  WRITE_FLAG(pcVUI->getAspectRatioInfoPresentFlag(),            "aspect_ratio_info_present_flag");
  if (pcVUI->getAspectRatioInfoPresentFlag())
  {
    WRITE_CODE(pcVUI->getAspectRatioIdc(), 8,                   "aspect_ratio_idc" );
    if (pcVUI->getAspectRatioIdc() == 255)
    {
      WRITE_CODE(pcVUI->getSarWidth(), 16,                      "sar_width");
      WRITE_CODE(pcVUI->getSarHeight(), 16,                     "sar_height");
    }
  }
  WRITE_FLAG(pcVUI->getOverscanInfoPresentFlag(),               "overscan_info_present_flag");
  if (pcVUI->getOverscanInfoPresentFlag())
  {
    WRITE_FLAG(pcVUI->getOverscanAppropriateFlag(),             "overscan_appropriate_flag");
  }
  WRITE_FLAG(pcVUI->getVideoSignalTypePresentFlag(),            "video_signal_type_present_flag");
  if (pcVUI->getVideoSignalTypePresentFlag())
  {
    WRITE_CODE(pcVUI->getVideoFormat(), 3,                      "video_format");
    WRITE_FLAG(pcVUI->getVideoFullRangeFlag(),                  "video_full_range_flag");
    WRITE_FLAG(pcVUI->getColourDescriptionPresentFlag(),        "colour_description_present_flag");
    if (pcVUI->getColourDescriptionPresentFlag())
    {
      WRITE_CODE(pcVUI->getColourPrimaries(), 8,                "colour_primaries");
      WRITE_CODE(pcVUI->getTransferCharacteristics(), 8,        "transfer_characteristics");
      WRITE_CODE(pcVUI->getMatrixCoefficients(), 8,             "matrix_coeffs");
    }
  }

  WRITE_FLAG(pcVUI->getChromaLocInfoPresentFlag(),              "chroma_loc_info_present_flag");
  if (pcVUI->getChromaLocInfoPresentFlag())
  {
    WRITE_UVLC(pcVUI->getChromaSampleLocTypeTopField(),         "chroma_sample_loc_type_top_field");
    WRITE_UVLC(pcVUI->getChromaSampleLocTypeBottomField(),      "chroma_sample_loc_type_bottom_field");
  }

  WRITE_FLAG(pcVUI->getNeutralChromaIndicationFlag(),           "neutral_chroma_indication_flag");
  WRITE_FLAG(pcVUI->getFieldSeqFlag(),                          "field_seq_flag");
  WRITE_FLAG(pcVUI->getFrameFieldInfoPresentFlag(),             "frame_field_info_present_flag");

  Window defaultDisplayWindow = pcVUI->getDefaultDisplayWindow();
  WRITE_FLAG(defaultDisplayWindow.getWindowEnabledFlag(),       "default_display_window_flag");
  if( defaultDisplayWindow.getWindowEnabledFlag() )
  {
    WRITE_UVLC(defaultDisplayWindow.getWindowLeftOffset()  / SPS::getWinUnitX(pcSPS->getChromaFormatIdc()), "def_disp_win_left_offset");
    WRITE_UVLC(defaultDisplayWindow.getWindowRightOffset() / SPS::getWinUnitX(pcSPS->getChromaFormatIdc()), "def_disp_win_right_offset");
    WRITE_UVLC(defaultDisplayWindow.getWindowTopOffset()   / SPS::getWinUnitY(pcSPS->getChromaFormatIdc()), "def_disp_win_top_offset");
    WRITE_UVLC(defaultDisplayWindow.getWindowBottomOffset()/ SPS::getWinUnitY(pcSPS->getChromaFormatIdc()), "def_disp_win_bottom_offset");
  }
  const TimingInfo *timingInfo = pcVUI->getTimingInfo();
  WRITE_FLAG(timingInfo->getTimingInfoPresentFlag(),          "vui_timing_info_present_flag");
  if(timingInfo->getTimingInfoPresentFlag())
  {
    WRITE_CODE(timingInfo->getNumUnitsInTick(), 32,           "vui_num_units_in_tick");
    WRITE_CODE(timingInfo->getTimeScale(),      32,           "vui_time_scale");
    WRITE_FLAG(timingInfo->getPocProportionalToTimingFlag(),  "vui_poc_proportional_to_timing_flag");
    if(timingInfo->getPocProportionalToTimingFlag())
    {
      WRITE_UVLC(timingInfo->getNumTicksPocDiffOneMinus1(),   "vui_num_ticks_poc_diff_one_minus1");
    }
    WRITE_FLAG(pcVUI->getHrdParametersPresentFlag(),              "vui_hrd_parameters_present_flag");
    if( pcVUI->getHrdParametersPresentFlag() )
    {
      codeHrdParameters(pcVUI->getHrdParameters(), 1, pcSPS->getMaxTLayers() - 1 );
    }
  }

  WRITE_FLAG(pcVUI->getBitstreamRestrictionFlag(),              "bitstream_restriction_flag");
  if (pcVUI->getBitstreamRestrictionFlag())
  {
#if HEVC_TILES_WPP
    WRITE_FLAG(pcVUI->getTilesFixedStructureFlag(),             "tiles_fixed_structure_flag");
#endif
    WRITE_FLAG(pcVUI->getMotionVectorsOverPicBoundariesFlag(),  "motion_vectors_over_pic_boundaries_flag");
    WRITE_FLAG(pcVUI->getRestrictedRefPicListsFlag(),           "restricted_ref_pic_lists_flag");
    WRITE_UVLC(pcVUI->getMinSpatialSegmentationIdc(),           "min_spatial_segmentation_idc");
    WRITE_UVLC(pcVUI->getMaxBytesPerPicDenom(),                 "max_bytes_per_pic_denom");
    WRITE_UVLC(pcVUI->getMaxBitsPerMinCuDenom(),                "max_bits_per_min_cu_denom");
    WRITE_UVLC(pcVUI->getLog2MaxMvLengthHorizontal(),           "log2_max_mv_length_horizontal");
    WRITE_UVLC(pcVUI->getLog2MaxMvLengthVertical(),             "log2_max_mv_length_vertical");
  }
}

void HLSWriter::codeHrdParameters( const HRD *hrd, bool commonInfPresentFlag, uint32_t maxNumSubLayersMinus1 )
{
  if( commonInfPresentFlag )
  {
    WRITE_FLAG( hrd->getNalHrdParametersPresentFlag() ? 1 : 0 ,  "nal_hrd_parameters_present_flag" );
    WRITE_FLAG( hrd->getVclHrdParametersPresentFlag() ? 1 : 0 ,  "vcl_hrd_parameters_present_flag" );
    if( hrd->getNalHrdParametersPresentFlag() || hrd->getVclHrdParametersPresentFlag() )
    {
      WRITE_FLAG( hrd->getSubPicCpbParamsPresentFlag() ? 1 : 0,  "sub_pic_hrd_params_present_flag" );
      if( hrd->getSubPicCpbParamsPresentFlag() )
      {
        WRITE_CODE( hrd->getTickDivisorMinus2(), 8,              "tick_divisor_minus2" );
        WRITE_CODE( hrd->getDuCpbRemovalDelayLengthMinus1(), 5,  "du_cpb_removal_delay_increment_length_minus1" );
        WRITE_FLAG( hrd->getSubPicCpbParamsInPicTimingSEIFlag() ? 1 : 0, "sub_pic_cpb_params_in_pic_timing_sei_flag" );
        WRITE_CODE( hrd->getDpbOutputDelayDuLengthMinus1(), 5,   "dpb_output_delay_du_length_minus1"  );
      }
      WRITE_CODE( hrd->getBitRateScale(), 4,                     "bit_rate_scale" );
      WRITE_CODE( hrd->getCpbSizeScale(), 4,                     "cpb_size_scale" );
      if( hrd->getSubPicCpbParamsPresentFlag() )
      {
        WRITE_CODE( hrd->getDuCpbSizeScale(), 4,                "du_cpb_size_scale" );
      }
      WRITE_CODE( hrd->getInitialCpbRemovalDelayLengthMinus1(), 5, "initial_cpb_removal_delay_length_minus1" );
      WRITE_CODE( hrd->getCpbRemovalDelayLengthMinus1(),        5, "au_cpb_removal_delay_length_minus1" );
      WRITE_CODE( hrd->getDpbOutputDelayLengthMinus1(),         5, "dpb_output_delay_length_minus1" );
    }
  }
  int i, j, nalOrVcl;
  for( i = 0; i <= maxNumSubLayersMinus1; i ++ )
  {
    WRITE_FLAG( hrd->getFixedPicRateFlag( i ) ? 1 : 0,          "fixed_pic_rate_general_flag");
    bool fixedPixRateWithinCvsFlag = true;
    if( !hrd->getFixedPicRateFlag( i ) )
    {
      fixedPixRateWithinCvsFlag = hrd->getFixedPicRateWithinCvsFlag( i );
      WRITE_FLAG( hrd->getFixedPicRateWithinCvsFlag( i ) ? 1 : 0, "fixed_pic_rate_within_cvs_flag");
    }
    if( fixedPixRateWithinCvsFlag )
    {
      WRITE_UVLC( hrd->getPicDurationInTcMinus1( i ),           "elemental_duration_in_tc_minus1");
    }
    else
    {
      WRITE_FLAG( hrd->getLowDelayHrdFlag( i ) ? 1 : 0,           "low_delay_hrd_flag");
    }
    if (!hrd->getLowDelayHrdFlag( i ))
    {
      WRITE_UVLC( hrd->getCpbCntMinus1( i ),                      "cpb_cnt_minus1");
    }

    for( nalOrVcl = 0; nalOrVcl < 2; nalOrVcl ++ )
    {
      if( ( ( nalOrVcl == 0 ) && ( hrd->getNalHrdParametersPresentFlag() ) ) ||
          ( ( nalOrVcl == 1 ) && ( hrd->getVclHrdParametersPresentFlag() ) ) )
      {
        for( j = 0; j <= ( hrd->getCpbCntMinus1( i ) ); j ++ )
        {
          WRITE_UVLC( hrd->getBitRateValueMinus1( i, j, nalOrVcl ), "bit_rate_value_minus1");
          WRITE_UVLC( hrd->getCpbSizeValueMinus1( i, j, nalOrVcl ), "cpb_size_value_minus1");
          if( hrd->getSubPicCpbParamsPresentFlag() )
          {
            WRITE_UVLC( hrd->getDuCpbSizeValueMinus1( i, j, nalOrVcl ), "cpb_size_du_value_minus1");
            WRITE_UVLC( hrd->getDuBitRateValueMinus1( i, j, nalOrVcl ), "bit_rate_du_value_minus1");
          }
          WRITE_FLAG( hrd->getCbrFlag( i, j, nalOrVcl ) ? 1 : 0, "cbr_flag");
        }
      }
    }
  }
}


void HLSWriter::codeSPSNext( const SPSNext& spsNext, const bool usePCM )
{
  // tool enabling flags
  WRITE_FLAG( spsNext.getUseQTBT() ? 1 : 0,                                                     "qtbt_flag" );
  WRITE_FLAG( spsNext.getUseLargeCTU() ? 1 : 0,                                                 "large_ctu_flag" );
#if JVET_K0346
  WRITE_FLAG(spsNext.getUseSubPuMvp() ? 1 : 0,                                                  "subpu_tmvp_flag");
#endif
#if JVET_K0357_AMVR
  WRITE_FLAG( spsNext.getUseIMV() ? 1 : 0,                                                      "imv_enable_flag" );
#endif
#if JVET_K0072
#else
#endif
#if JVET_K0346 || JVET_K_AFFINE
  WRITE_FLAG(spsNext.getUseHighPrecMv() ? 1 : 0,                                                "high_precision_motion_vectors");
#endif
  WRITE_FLAG( spsNext.getDisableMotCompress() ? 1 : 0,                                          "disable_motion_compression_flag" );
#if JVET_K0190
  WRITE_FLAG( spsNext.getUseLMChroma() ? 1 : 0,                                                 "lm_chroma_enabled_flag" );
#endif
#if JVET_K1000_SIMPLIFIED_EMT
  WRITE_FLAG( spsNext.getUseIntraEMT() ? 1 : 0,                                                 "emt_intra_enabled_flag" );
  WRITE_FLAG( spsNext.getUseInterEMT() ? 1 : 0,                                                 "emt_inter_enabled_flag" );
#endif
#if JVET_K_AFFINE
  WRITE_FLAG( spsNext.getUseAffine() ? 1 : 0,                                                   "affine_flag" );
#if JVET_K0337_AFFINE_6PARA
  if ( spsNext.getUseAffine() )
  {
    WRITE_FLAG( spsNext.getUseAffineType() ? 1 : 0,                                             "affine_type_flag" );
  }
#endif
#endif

  for( int k = 0; k < SPSNext::NumReservedFlags; k++ )
  {
    WRITE_FLAG( 0,                                                                              "reserved_flag" );
  }

  WRITE_FLAG( spsNext.getMTTEnabled() ? 1 : 0,                                                  "mtt_enabled_flag" );
#if ENABLE_WPP_PARALLELISM
  WRITE_FLAG( spsNext.getUseNextDQP(),                                                          "next_dqp_enabled_flag" );
#else
  WRITE_FLAG( 0,                                                                                "reserved_flag" );
#endif

  // additional parameters
  if( spsNext.getUseQTBT() )
  {
    WRITE_FLAG( spsNext.getUseDualITree(),                                                      "qtbt_dual_intra_tree" );
    WRITE_UVLC( g_aucLog2[spsNext.getCTUSize()]                                 - MIN_CU_LOG2,  "log2_CTU_size_minus2" );
    WRITE_UVLC( g_aucLog2[spsNext.getMinQTSize( I_SLICE ) ]                     - MIN_CU_LOG2,  "log2_minQT_ISlice_minus2" );
    WRITE_UVLC( g_aucLog2[spsNext.getMinQTSize( B_SLICE ) ]                     - MIN_CU_LOG2,  "log2_minQT_PBSlice_minus2" );
    WRITE_UVLC( spsNext.getMaxBTDepth(),                                                        "max_bt_depth" );
    WRITE_UVLC( spsNext.getMaxBTDepthI(),                                                       "max_bt_depth_i_slice" );
    if( spsNext.getUseDualITree() )
    {
      WRITE_UVLC( g_aucLog2[spsNext.getMinQTSize( I_SLICE, CHANNEL_TYPE_CHROMA )] - MIN_CU_LOG2, "log2_minQT_ISliceChroma_minus2" );
      WRITE_UVLC( spsNext.getMaxBTDepthIChroma(),                                                "max_bt_depth_i_slice_chroma" );
    }
  }

#if JVET_K0346
  if( spsNext.getUseSubPuMvp() )
  {
    WRITE_CODE( spsNext.getSubPuMvpLog2Size() - MIN_CU_LOG2, 3,                                 "log2_sub_pu_tmvp_size_minus2" );
#if ENABLE_BMS
#endif
  }
#endif

#if JVET_K0357_AMVR
  if( spsNext.getUseIMV() )
  {
    WRITE_UVLC( spsNext.getImvMode()-1,                                                         "imv_mode_minus1" );
  }
#endif

  if( spsNext.getMTTEnabled() )
  {
    WRITE_UVLC( spsNext.getMTTMode() - 1,                                                       "mtt_mode_minus1" );
  }
#if JVET_K0072
#else
#endif
  // ADD_NEW_TOOL : (sps extension writer) write tool enabling flags and associated parameters here
}

void HLSWriter::codeSPS( const SPS* pcSPS )
{

  const ChromaFormat format                = pcSPS->getChromaFormatIdc();
  const bool         chromaEnabled         = isChromaEnabled(format);

#if ENABLE_TRACING
  xTraceSPSHeader ();
#endif
#if HEVC_VPS
  WRITE_CODE( pcSPS->getVPSId (),          4,       "sps_video_parameter_set_id" );
#endif
  WRITE_CODE( pcSPS->getMaxTLayers() - 1,  3,       "sps_max_sub_layers_minus1" );
  WRITE_FLAG( pcSPS->getTemporalIdNestingFlag() ? 1 : 0, "sps_temporal_id_nesting_flag" );
  codePTL( pcSPS->getPTL(), true, pcSPS->getMaxTLayers() - 1 );
  WRITE_UVLC( pcSPS->getSPSId (),                   "sps_seq_parameter_set_id" );
  WRITE_UVLC( int(pcSPS->getChromaFormatIdc ()),    "chroma_format_idc" );
  if( format == CHROMA_444 )
  {
    WRITE_FLAG( 0,                                  "separate_colour_plane_flag");
  }

  WRITE_UVLC( pcSPS->getPicWidthInLumaSamples (),   "pic_width_in_luma_samples" );
  WRITE_UVLC( pcSPS->getPicHeightInLumaSamples(),   "pic_height_in_luma_samples" );
  Window conf = pcSPS->getConformanceWindow();

  WRITE_FLAG( conf.getWindowEnabledFlag(),          "conformance_window_flag" );
  if (conf.getWindowEnabledFlag())
  {
    WRITE_UVLC( conf.getWindowLeftOffset()   / SPS::getWinUnitX(pcSPS->getChromaFormatIdc() ), "conf_win_left_offset" );
    WRITE_UVLC( conf.getWindowRightOffset()  / SPS::getWinUnitX(pcSPS->getChromaFormatIdc() ), "conf_win_right_offset" );
    WRITE_UVLC( conf.getWindowTopOffset()    / SPS::getWinUnitY(pcSPS->getChromaFormatIdc() ), "conf_win_top_offset" );
    WRITE_UVLC( conf.getWindowBottomOffset() / SPS::getWinUnitY(pcSPS->getChromaFormatIdc() ), "conf_win_bottom_offset" );
  }

  WRITE_UVLC( pcSPS->getBitDepth(CHANNEL_TYPE_LUMA) - 8,                      "bit_depth_luma_minus8" );

  WRITE_UVLC( chromaEnabled ? (pcSPS->getBitDepth(CHANNEL_TYPE_CHROMA) - 8):0,  "bit_depth_chroma_minus8" );

  WRITE_UVLC( pcSPS->getBitsForPOC()-4,                 "log2_max_pic_order_cnt_lsb_minus4" );

  const bool subLayerOrderingInfoPresentFlag = 1;
  WRITE_FLAG(subLayerOrderingInfoPresentFlag,       "sps_sub_layer_ordering_info_present_flag");
  for(uint32_t i=0; i <= pcSPS->getMaxTLayers()-1; i++)
  {
    WRITE_UVLC( pcSPS->getMaxDecPicBuffering(i) - 1,       "sps_max_dec_pic_buffering_minus1[i]" );
    WRITE_UVLC( pcSPS->getNumReorderPics(i),               "sps_max_num_reorder_pics[i]" );
    WRITE_UVLC( pcSPS->getMaxLatencyIncreasePlus1(i),      "sps_max_latency_increase_plus1[i]" );
    if (!subLayerOrderingInfoPresentFlag)
    {
      break;
    }
  }
  CHECK( pcSPS->getMaxCUWidth() != pcSPS->getMaxCUHeight(),                          "Rectangular CTUs not supported" );
  WRITE_UVLC( pcSPS->getLog2MinCodingBlockSize() - 3,                                "log2_min_luma_coding_block_size_minus3" );
  WRITE_UVLC( pcSPS->getLog2DiffMaxMinCodingBlockSize(),                             "log2_diff_max_min_luma_coding_block_size" );
  WRITE_UVLC( pcSPS->getQuadtreeTULog2MinSize() - 2,                                 "log2_min_luma_transform_block_size_minus2" );
  WRITE_UVLC( pcSPS->getQuadtreeTULog2MaxSize() - pcSPS->getQuadtreeTULog2MinSize(), "log2_diff_max_min_luma_transform_block_size" );
  WRITE_UVLC( pcSPS->getQuadtreeTUMaxDepthInter() - 1,                               "max_transform_hierarchy_depth_inter" );
  WRITE_UVLC( pcSPS->getQuadtreeTUMaxDepthIntra() - 1,                               "max_transform_hierarchy_depth_intra" );
#if JVET_K0371_ALF
  WRITE_FLAG( pcSPS->getUseALF(), "sps_alf_enable_flag" );
#endif
#if HEVC_USE_SCALING_LISTS
  WRITE_FLAG( pcSPS->getScalingListFlag() ? 1 : 0,                                   "scaling_list_enabled_flag" );
  if(pcSPS->getScalingListFlag())
  {
    WRITE_FLAG( pcSPS->getScalingListPresentFlag() ? 1 : 0,                          "sps_scaling_list_data_present_flag" );
    if(pcSPS->getScalingListPresentFlag())
    {
      codeScalingList( pcSPS->getScalingList() );
    }
  }
#endif
  WRITE_FLAG( pcSPS->getUseAMP() ? 1 : 0,                                            "amp_enabled_flag" );
  WRITE_FLAG( pcSPS->getUseSAO() ? 1 : 0,                                            "sample_adaptive_offset_enabled_flag");

  WRITE_FLAG( pcSPS->getUsePCM() ? 1 : 0,                                            "pcm_enabled_flag");
  if( pcSPS->getUsePCM() )
  {
    WRITE_CODE( pcSPS->getPCMBitDepth(CHANNEL_TYPE_LUMA) - 1, 4,                            "pcm_sample_bit_depth_luma_minus1" );
    WRITE_CODE( chromaEnabled ? (pcSPS->getPCMBitDepth(CHANNEL_TYPE_CHROMA) - 1) : 0, 4,    "pcm_sample_bit_depth_chroma_minus1" );
    WRITE_UVLC( pcSPS->getPCMLog2MinSize() - 3,                                      "log2_min_pcm_luma_coding_block_size_minus3" );
    WRITE_UVLC( pcSPS->getPCMLog2MaxSize() - pcSPS->getPCMLog2MinSize(),             "log2_diff_max_min_pcm_luma_coding_block_size" );
    WRITE_FLAG( pcSPS->getPCMFilterDisableFlag()?1 : 0,                              "pcm_loop_filter_disable_flag");
  }

  CHECK( pcSPS->getMaxTLayers() == 0, "Maximum number of T-layers is '0'" );

  const RPSList* rpsList = pcSPS->getRPSList();

  WRITE_UVLC(rpsList->getNumberOfReferencePictureSets(), "num_short_term_ref_pic_sets" );
  for(int i=0; i < rpsList->getNumberOfReferencePictureSets(); i++)
  {
    const ReferencePictureSet*rps = rpsList->getReferencePictureSet(i);
    xCodeShortTermRefPicSet( rps,false, i);
  }
  WRITE_FLAG( pcSPS->getLongTermRefsPresent() ? 1 : 0,         "long_term_ref_pics_present_flag" );
  if (pcSPS->getLongTermRefsPresent())
  {
    WRITE_UVLC(pcSPS->getNumLongTermRefPicSPS(), "num_long_term_ref_pics_sps" );
    for (uint32_t k = 0; k < pcSPS->getNumLongTermRefPicSPS(); k++)
    {
      WRITE_CODE( pcSPS->getLtRefPicPocLsbSps(k), pcSPS->getBitsForPOC(), "lt_ref_pic_poc_lsb_sps");
      WRITE_FLAG( pcSPS->getUsedByCurrPicLtSPSFlag(k), "used_by_curr_pic_lt_sps_flag[i]");
    }
  }
  WRITE_FLAG( pcSPS->getSPSTemporalMVPEnabledFlag()  ? 1 : 0,  "sps_temporal_mvp_enabled_flag" );

#if HEVC_USE_INTRA_SMOOTHING_T32 || HEVC_USE_INTRA_SMOOTHING_T64
  WRITE_FLAG( pcSPS->getUseStrongIntraSmoothing(),             "strong_intra_smoothing_enable_flag" );

#endif
  WRITE_FLAG( pcSPS->getVuiParametersPresentFlag(),            "vui_parameters_present_flag" );
  if (pcSPS->getVuiParametersPresentFlag())
  {
    codeVUI(pcSPS->getVuiParameters(), pcSPS);
  }

  // KTA tools

  bool sps_extension_present_flag=false;
  bool sps_extension_flags[NUM_SPS_EXTENSION_FLAGS]={false};

  sps_extension_flags[SPS_EXT__REXT] = pcSPS->getSpsRangeExtension().settingsDifferFromDefaults();
  sps_extension_flags[SPS_EXT__NEXT] = pcSPS->getSpsNext().nextToolsEnabled();

  // Other SPS extension flags checked here.

  for(int i=0; i<NUM_SPS_EXTENSION_FLAGS; i++)
  {
    sps_extension_present_flag|=sps_extension_flags[i];
  }

  WRITE_FLAG( (sps_extension_present_flag?1:0), "sps_extension_present_flag" );

  if (sps_extension_present_flag)
  {
#if ENABLE_TRACING /*|| RExt__DECODER_DEBUG_BIT_STATISTICS*/
    static const char *syntaxStrings[]={ "sps_range_extension_flag",
      "sps_multilayer_extension_flag",
      "sps_extension_6bits[0]",
      "sps_extension_6bits[1]",
      "sps_extension_6bits[2]",
      "sps_extension_6bits[3]",
      "sps_extension_6bits[4]",
      "sps_extension_6bits[5]" };
#endif

    for(int i=0; i<NUM_SPS_EXTENSION_FLAGS; i++)
    {
      WRITE_FLAG( sps_extension_flags[i]?1:0, syntaxStrings[i] );
    }

    for(int i=0; i<NUM_SPS_EXTENSION_FLAGS; i++) // loop used so that the order is determined by the enum.
    {
      if (sps_extension_flags[i])
      {
        switch (SPSExtensionFlagIndex(i))
        {
        case SPS_EXT__REXT:
        {
          const SPSRExt &spsRangeExtension=pcSPS->getSpsRangeExtension();

          WRITE_FLAG( (spsRangeExtension.getTransformSkipRotationEnabledFlag() ? 1 : 0),      "transform_skip_rotation_enabled_flag");
          WRITE_FLAG( (spsRangeExtension.getTransformSkipContextEnabledFlag() ? 1 : 0),       "transform_skip_context_enabled_flag");
          WRITE_FLAG( (spsRangeExtension.getRdpcmEnabledFlag(RDPCM_SIGNAL_IMPLICIT) ? 1 : 0), "implicit_rdpcm_enabled_flag" );
          WRITE_FLAG( (spsRangeExtension.getRdpcmEnabledFlag(RDPCM_SIGNAL_EXPLICIT) ? 1 : 0), "explicit_rdpcm_enabled_flag" );
          WRITE_FLAG( (spsRangeExtension.getExtendedPrecisionProcessingFlag() ? 1 : 0),       "extended_precision_processing_flag" );
          WRITE_FLAG( (spsRangeExtension.getIntraSmoothingDisabledFlag() ? 1 : 0),            "intra_smoothing_disabled_flag" );
          WRITE_FLAG( (spsRangeExtension.getHighPrecisionOffsetsEnabledFlag() ? 1 : 0),       "high_precision_offsets_enabled_flag" );
          WRITE_FLAG( (spsRangeExtension.getPersistentRiceAdaptationEnabledFlag() ? 1 : 0),   "persistent_rice_adaptation_enabled_flag" );
          WRITE_FLAG( (spsRangeExtension.getCabacBypassAlignmentEnabledFlag() ? 1 : 0),       "cabac_bypass_alignment_enabled_flag" );
          break;
        }
        case SPS_EXT__NEXT:
        {
          codeSPSNext( pcSPS->getSpsNext(), pcSPS->getUsePCM() );
          break;
        }
        default:
          CHECK(sps_extension_flags[i]!=false, "Unknown PPS extension signalled"); // Should never get here with an active SPS extension flag.
          break;
        }
      }
    }
  }
  xWriteRbspTrailingBits();
}

#if HEVC_VPS
void HLSWriter::codeVPS( const VPS* pcVPS )
{
#if ENABLE_TRACING
  xTraceVPSHeader();
#endif
  WRITE_CODE( pcVPS->getVPSId(),                    4,        "vps_video_parameter_set_id" );
  WRITE_FLAG(                                       1,        "vps_base_layer_internal_flag" );
  WRITE_FLAG(                                       1,        "vps_base_layer_available_flag" );
  WRITE_CODE( 0,                                    6,        "vps_max_layers_minus1" );
  WRITE_CODE( pcVPS->getMaxTLayers() - 1,           3,        "vps_max_sub_layers_minus1" );
  WRITE_FLAG( pcVPS->getTemporalNestingFlag(),                "vps_temporal_id_nesting_flag" );
  CHECK(pcVPS->getMaxTLayers()<=1&&!pcVPS->getTemporalNestingFlag(), "Invalud parameters");
  WRITE_CODE( 0xffff,                              16,        "vps_reserved_0xffff_16bits" );
  codePTL( pcVPS->getPTL(), true, pcVPS->getMaxTLayers() - 1 );
  const bool subLayerOrderingInfoPresentFlag = 1;
  WRITE_FLAG(subLayerOrderingInfoPresentFlag,              "vps_sub_layer_ordering_info_present_flag");
  for(uint32_t i=0; i <= pcVPS->getMaxTLayers()-1; i++)
  {
    WRITE_UVLC( pcVPS->getMaxDecPicBuffering(i) - 1,       "vps_max_dec_pic_buffering_minus1[i]" );
    WRITE_UVLC( pcVPS->getNumReorderPics(i),               "vps_max_num_reorder_pics[i]" );
    WRITE_UVLC( pcVPS->getMaxLatencyIncrease(i),           "vps_max_latency_increase_plus1[i]" );
    if (!subLayerOrderingInfoPresentFlag)
    {
      break;
    }
  }

  CHECK( pcVPS->getNumHrdParameters() > MAX_VPS_NUM_HRD_PARAMETERS, "Too many HRD parameters" );
  CHECK( pcVPS->getMaxNuhReservedZeroLayerId() >= MAX_VPS_NUH_RESERVED_ZERO_LAYER_ID_PLUS1, "Invalid parameters read" );
  WRITE_CODE( pcVPS->getMaxNuhReservedZeroLayerId(), 6,     "vps_max_layer_id" );
  WRITE_UVLC( pcVPS->getMaxOpSets() - 1,                    "vps_num_layer_sets_minus1" );
  for( uint32_t opsIdx = 1; opsIdx <= ( pcVPS->getMaxOpSets() - 1 ); opsIdx ++ )
  {
    // Operation point set
    for( uint32_t i = 0; i <= pcVPS->getMaxNuhReservedZeroLayerId(); i ++ )
    {
      // Only applicable for version 1
      // pcVPS->setLayerIdIncludedFlag( true, opsIdx, i );
      WRITE_FLAG( pcVPS->getLayerIdIncludedFlag( opsIdx, i ) ? 1 : 0, "layer_id_included_flag[opsIdx][i]" );
    }
  }
  const TimingInfo *timingInfo = pcVPS->getTimingInfo();
  WRITE_FLAG(timingInfo->getTimingInfoPresentFlag(),          "vps_timing_info_present_flag");
  if(timingInfo->getTimingInfoPresentFlag())
  {
    WRITE_CODE(timingInfo->getNumUnitsInTick(), 32,           "vps_num_units_in_tick");
    WRITE_CODE(timingInfo->getTimeScale(),      32,           "vps_time_scale");
    WRITE_FLAG(timingInfo->getPocProportionalToTimingFlag(),  "vps_poc_proportional_to_timing_flag");
    if(timingInfo->getPocProportionalToTimingFlag())
    {
      WRITE_UVLC(timingInfo->getNumTicksPocDiffOneMinus1(),   "vps_num_ticks_poc_diff_one_minus1");
    }
    WRITE_UVLC( pcVPS->getNumHrdParameters(),                 "vps_num_hrd_parameters" );

    if( pcVPS->getNumHrdParameters() > 0 )
    {
      for( uint32_t i = 0; i < pcVPS->getNumHrdParameters(); i ++ )
      {
        // Only applicable for version 1
        WRITE_UVLC( pcVPS->getHrdOpSetIdx( i ),                "hrd_layer_set_idx" );
        if( i > 0 )
        {
          WRITE_FLAG( pcVPS->getCprmsPresentFlag( i ) ? 1 : 0, "cprms_present_flag[i]" );
        }
        codeHrdParameters(pcVPS->getHrdParameters(i), pcVPS->getCprmsPresentFlag( i ), pcVPS->getMaxTLayers() - 1);
      }
    }
  }
  WRITE_FLAG( 0,                     "vps_extension_flag" );

  //future extensions here..
  xWriteRbspTrailingBits();
}
#endif

void HLSWriter::codeSliceHeader         ( Slice* pcSlice )
{
#if ENABLE_TRACING
  xTraceSliceHeader ();
#endif

  CodingStructure& cs = *pcSlice->getPic()->cs;
  const ChromaFormat format                = pcSlice->getSPS()->getChromaFormatIdc();
  const uint32_t         numberValidComponents = getNumberValidComponents(format);
  const bool         chromaEnabled         = isChromaEnabled(format);

  //calculate number of bits required for slice address
  int maxSliceSegmentAddress = cs.pcv->sizeInCtus;
  int bitsSliceSegmentAddress = 0;
  while(maxSliceSegmentAddress>(1<<bitsSliceSegmentAddress))
  {
    bitsSliceSegmentAddress++;
  }
#if HEVC_DEPENDENT_SLICES
  const int ctuTsAddress = pcSlice->getSliceSegmentCurStartCtuTsAddr();
#else
  const int ctuTsAddress = pcSlice->getSliceCurStartCtuTsAddr();
#endif

  //write slice address
#if HEVC_TILES_WPP
  const int sliceSegmentRsAddress = pcSlice->getPic()->tileMap->getCtuTsToRsAddrMap(ctuTsAddress);
#else
  const int sliceSegmentRsAddress = ctuTsAddress;
#endif

  WRITE_FLAG( sliceSegmentRsAddress==0, "first_slice_segment_in_pic_flag" );
  if ( pcSlice->getRapPicFlag() )
  {
    WRITE_FLAG( pcSlice->getNoOutputPriorPicsFlag() ? 1 : 0, "no_output_of_prior_pics_flag" );
  }
  WRITE_UVLC( pcSlice->getPPS()->getPPSId(), "slice_pic_parameter_set_id" );
#if HEVC_DEPENDENT_SLICES
  if ( pcSlice->getPPS()->getDependentSliceSegmentsEnabledFlag() && (sliceSegmentRsAddress!=0) )
  {
    WRITE_FLAG( pcSlice->getDependentSliceSegmentFlag() ? 1 : 0, "dependent_slice_segment_flag" );
  }
#endif
  if(sliceSegmentRsAddress>0)
  {
    WRITE_CODE( sliceSegmentRsAddress, bitsSliceSegmentAddress, "slice_segment_address" );
  }
#if HEVC_DEPENDENT_SLICES
  if( !pcSlice->getDependentSliceSegmentFlag() )
  {
#endif
    for( int i = 0; i < pcSlice->getPPS()->getNumExtraSliceHeaderBits(); i++ )
    {
      WRITE_FLAG( 0, "slice_reserved_flag[]" );
    }

    WRITE_UVLC( pcSlice->getSliceType(), "slice_type" );

    if( pcSlice->getPPS()->getOutputFlagPresentFlag() )
    {
      WRITE_FLAG( pcSlice->getPicOutputFlag() ? 1 : 0, "pic_output_flag" );
    }

    if( !pcSlice->getIdrPicFlag() )
    {
      int picOrderCntLSB = ( pcSlice->getPOC() - pcSlice->getLastIDR() + ( 1 << pcSlice->getSPS()->getBitsForPOC() ) ) & ( ( 1 << pcSlice->getSPS()->getBitsForPOC() ) - 1 );
      WRITE_CODE( picOrderCntLSB, pcSlice->getSPS()->getBitsForPOC(), "slice_pic_order_cnt_lsb" );
      const ReferencePictureSet* rps = pcSlice->getRPS();

      // check for bitstream restriction stating that:
      // If the current picture is a BLA or CRA picture, the value of NumPocTotalCurr shall be equal to 0.
      // Ideally this process should not be repeated for each slice in a picture
      if( pcSlice->isIRAP() )
      {
        for( int picIdx = 0; picIdx < rps->getNumberOfPictures(); picIdx++ )
        {
          CHECK( rps->getUsed( picIdx ), "Picture should not be used" );
        }
      }

      if( pcSlice->getRPSidx() < 0 )
      {
        WRITE_FLAG( 0, "short_term_ref_pic_set_sps_flag" );
        xCodeShortTermRefPicSet( rps, true, pcSlice->getSPS()->getRPSList()->getNumberOfReferencePictureSets() );
      }
      else
      {
        WRITE_FLAG( 1, "short_term_ref_pic_set_sps_flag" );
        int numBits = 0;
        while( ( 1 << numBits ) < pcSlice->getSPS()->getRPSList()->getNumberOfReferencePictureSets() )
        {
          numBits++;
        }
        if( numBits > 0 )
        {
          WRITE_CODE( pcSlice->getRPSidx(), numBits, "short_term_ref_pic_set_idx" );
        }
      }
      if( pcSlice->getSPS()->getLongTermRefsPresent() )
      {
        int numLtrpInSH = rps->getNumberOfLongtermPictures();
        int ltrpInSPS[MAX_NUM_REF_PICS];
        int numLtrpInSPS = 0;
        uint32_t ltrpIndex;
        int counter = 0;
        // WARNING: The following code only works only if a matching long-term RPS is
        //          found in the SPS for ALL long-term pictures
        //          The problem is that the SPS coded long-term pictures are moved to the
        //          beginning of the list which causes a mismatch when no reference picture
        //          list reordering is used
        //          NB: Long-term coding is currently not supported in general by the HM encoder
        for( int k = rps->getNumberOfPictures() - 1; k > rps->getNumberOfPictures() - rps->getNumberOfLongtermPictures() - 1; k-- )
        {
          if( xFindMatchingLTRP( pcSlice, &ltrpIndex, rps->getPOC( k ), rps->getUsed( k ) ) )
          {
            ltrpInSPS[numLtrpInSPS] = ltrpIndex;
            numLtrpInSPS++;
          }
          else
          {
            counter++;
          }
        }
        numLtrpInSH -= numLtrpInSPS;
        // check that either all long-term pictures are coded in SPS or in slice header (no mixing)
        CHECK( numLtrpInSH != 0 && numLtrpInSPS != 0, "Long term picture not coded" );

        int bitsForLtrpInSPS = 0;
        while( pcSlice->getSPS()->getNumLongTermRefPicSPS() > ( 1 << bitsForLtrpInSPS ) )
        {
          bitsForLtrpInSPS++;
        }
        if( pcSlice->getSPS()->getNumLongTermRefPicSPS() > 0 )
        {
          WRITE_UVLC( numLtrpInSPS, "num_long_term_sps" );
        }
        WRITE_UVLC( numLtrpInSH, "num_long_term_pics" );
        // Note that the LSBs of the LT ref. pic. POCs must be sorted before.
        // Not sorted here because LT ref indices will be used in setRefPicList()
        int prevDeltaMSB = 0, prevLSB = 0;
        int offset = rps->getNumberOfNegativePictures() + rps->getNumberOfPositivePictures();
        counter = 0;
        // Warning: If some pictures are moved to ltrpInSPS, i is referring to a wrong index
        //          (mapping would be required)
        for( int i = rps->getNumberOfPictures() - 1; i > offset - 1; i--, counter++ )
        {
          if( counter < numLtrpInSPS )
          {
            if( bitsForLtrpInSPS > 0 )
            {
              WRITE_CODE( ltrpInSPS[counter], bitsForLtrpInSPS, "lt_idx_sps[i]" );
            }
          }
          else
          {
            WRITE_CODE( rps->getPocLSBLT( i ), pcSlice->getSPS()->getBitsForPOC(), "poc_lsb_lt" );
            WRITE_FLAG( rps->getUsed( i ), "used_by_curr_pic_lt_flag" );
          }
          WRITE_FLAG( rps->getDeltaPocMSBPresentFlag( i ), "delta_poc_msb_present_flag" );

          if( rps->getDeltaPocMSBPresentFlag( i ) )
          {
            bool deltaFlag = false;
            //  First LTRP from SPS                 ||  First LTRP from SH                              || curr LSB            != prev LSB
            if( ( i == rps->getNumberOfPictures() - 1 ) || ( i == rps->getNumberOfPictures() - 1 - numLtrpInSPS ) || ( rps->getPocLSBLT( i ) != prevLSB ) )
            {
              deltaFlag = true;
            }
            if( deltaFlag )
            {
              WRITE_UVLC( rps->getDeltaPocMSBCycleLT( i ), "delta_poc_msb_cycle_lt[i]" );
            }
            else
            {
              int differenceInDeltaMSB = rps->getDeltaPocMSBCycleLT( i ) - prevDeltaMSB;
              CHECK( differenceInDeltaMSB < 0, "Negative diff. delta MSB" );
              WRITE_UVLC( differenceInDeltaMSB, "delta_poc_msb_cycle_lt[i]" );
            }
            prevLSB = rps->getPocLSBLT( i );
            prevDeltaMSB = rps->getDeltaPocMSBCycleLT( i );
          }
        }
      }
      if( pcSlice->getSPS()->getSPSTemporalMVPEnabledFlag() )
      {
        WRITE_FLAG( pcSlice->getEnableTMVPFlag() ? 1 : 0, "slice_temporal_mvp_enabled_flag" );
      }
    }
    if( pcSlice->getSPS()->getUseSAO() )
    {
      WRITE_FLAG( pcSlice->getSaoEnabledFlag( CHANNEL_TYPE_LUMA ), "slice_sao_luma_flag" );
      if( chromaEnabled )
      {
        WRITE_FLAG( pcSlice->getSaoEnabledFlag( CHANNEL_TYPE_CHROMA ), "slice_sao_chroma_flag" );
      }
    }

#if JVET_K0371_ALF
    if( pcSlice->getSPS()->getUseALF() )
    {
      alf( pcSlice->getAlfSliceParam() );
    }
#endif

    //check if numrefidxes match the defaults. If not, override

    if( !pcSlice->isIntra() )
    {
      bool overrideFlag = ( pcSlice->getNumRefIdx( REF_PIC_LIST_0 ) != pcSlice->getPPS()->getNumRefIdxL0DefaultActive() || ( pcSlice->isInterB() && pcSlice->getNumRefIdx( REF_PIC_LIST_1 ) != pcSlice->getPPS()->getNumRefIdxL1DefaultActive() ) );
      WRITE_FLAG( overrideFlag ? 1 : 0, "num_ref_idx_active_override_flag" );
      if( overrideFlag )
      {
        WRITE_UVLC( pcSlice->getNumRefIdx( REF_PIC_LIST_0 ) - 1, "num_ref_idx_l0_active_minus1" );
        if( pcSlice->isInterB() )
        {
          WRITE_UVLC( pcSlice->getNumRefIdx( REF_PIC_LIST_1 ) - 1, "num_ref_idx_l1_active_minus1" );
        }
        else
        {
          pcSlice->setNumRefIdx( REF_PIC_LIST_1, 0 );
        }
      }
    }
    else
    {
      pcSlice->setNumRefIdx( REF_PIC_LIST_0, 0 );
      pcSlice->setNumRefIdx( REF_PIC_LIST_1, 0 );
    }

    if( pcSlice->getPPS()->getListsModificationPresentFlag() && pcSlice->getNumRpsCurrTempList() > 1 )
    {
      RefPicListModification* refPicListModification = pcSlice->getRefPicListModification();
      if( !pcSlice->isIntra() )
      {
        WRITE_FLAG( pcSlice->getRefPicListModification()->getRefPicListModificationFlagL0() ? 1 : 0, "ref_pic_list_modification_flag_l0" );
        if( pcSlice->getRefPicListModification()->getRefPicListModificationFlagL0() )
        {
          int numRpsCurrTempList0 = pcSlice->getNumRpsCurrTempList();
          if( numRpsCurrTempList0 > 1 )
          {
            int length = 1;
            numRpsCurrTempList0--;
            while( numRpsCurrTempList0 >>= 1 )
            {
              length++;
            }
            for( int i = 0; i < pcSlice->getNumRefIdx( REF_PIC_LIST_0 ); i++ )
            {
              WRITE_CODE( refPicListModification->getRefPicSetIdxL0( i ), length, "list_entry_l0" );
            }
          }
        }
      }
      if( pcSlice->isInterB() )
      {
        WRITE_FLAG( pcSlice->getRefPicListModification()->getRefPicListModificationFlagL1() ? 1 : 0, "ref_pic_list_modification_flag_l1" );
        if( pcSlice->getRefPicListModification()->getRefPicListModificationFlagL1() )
        {
          int numRpsCurrTempList1 = pcSlice->getNumRpsCurrTempList();
          if( numRpsCurrTempList1 > 1 )
          {
            int length = 1;
            numRpsCurrTempList1--;
            while( numRpsCurrTempList1 >>= 1 )
            {
              length++;
            }
            for( int i = 0; i < pcSlice->getNumRefIdx( REF_PIC_LIST_1 ); i++ )
            {
              WRITE_CODE( refPicListModification->getRefPicSetIdxL1( i ), length, "list_entry_l1" );
            }
          }
        }
      }
    }

    if( pcSlice->isInterB() )
    {
      WRITE_FLAG( pcSlice->getMvdL1ZeroFlag() ? 1 : 0, "mvd_l1_zero_flag" );
    }

    if( !pcSlice->isIntra() )
    {
      if( !pcSlice->isIntra() && pcSlice->getPPS()->getCabacInitPresentFlag() )
      {
        SliceType sliceType = pcSlice->getSliceType();
        SliceType  encCABACTableIdx = pcSlice->getEncCABACTableIdx();
        bool encCabacInitFlag = ( sliceType != encCABACTableIdx && encCABACTableIdx != I_SLICE ) ? true : false;
        pcSlice->setCabacInitFlag( encCabacInitFlag );
        WRITE_FLAG( encCabacInitFlag ? 1 : 0, "cabac_init_flag" );
      }
    }

    if( pcSlice->getEnableTMVPFlag() )
    {
      if( pcSlice->getSliceType() == B_SLICE )
      {
        WRITE_FLAG( pcSlice->getColFromL0Flag(), "collocated_from_l0_flag" );
      }

      if( pcSlice->getSliceType() != I_SLICE &&
        ( ( pcSlice->getColFromL0Flag() == 1 && pcSlice->getNumRefIdx( REF_PIC_LIST_0 ) > 1 ) ||
          ( pcSlice->getColFromL0Flag() == 0 && pcSlice->getNumRefIdx( REF_PIC_LIST_1 ) > 1 ) ) )
      {
        WRITE_UVLC( pcSlice->getColRefIdx(), "collocated_ref_idx" );
      }
    }
    if( ( pcSlice->getPPS()->getUseWP() && pcSlice->getSliceType() == P_SLICE ) || ( pcSlice->getPPS()->getWPBiPred() && pcSlice->getSliceType() == B_SLICE ) )
    {
      xCodePredWeightTable( pcSlice );
    }
#if JVET_K0072
    WRITE_FLAG( pcSlice->getDepQuantEnabledFlag() ? 1 : 0, "dep_quant_enable_flag" );
#if HEVC_USE_SIGN_HIDING
    if( !pcSlice->getDepQuantEnabledFlag() )
    {
      WRITE_FLAG( pcSlice->getSignDataHidingEnabledFlag() ? 1 : 0, "sign_data_hiding_enable_flag" );
    }
    else
    {
      CHECK( pcSlice->getSignDataHidingEnabledFlag(), "sign data hiding not supported when dependent quantization is enabled" );
    }
#endif
#endif
    if( pcSlice->getSPS()->getSpsNext().getUseQTBT() )
    {
      if( !pcSlice->isIntra() )
      {
        if( pcSlice->getSPS()->getSpsNext().getCTUSize() > pcSlice->getMaxBTSize() )
        {
          WRITE_UVLC( g_aucLog2[pcSlice->getSPS()->getSpsNext().getCTUSize()] - g_aucLog2[pcSlice->getMaxBTSize()], "max_binary_tree_unit_size" );
        }
        else
        {
          WRITE_UVLC( 0, "max_binary_tree_unit_size" );
        }
      }
    }
    if( !pcSlice->isIntra() )
    {
#if JVET_K0346
      CHECK( pcSlice->getMaxNumMergeCand() > ( MRG_MAX_NUM_CANDS - ( pcSlice->getSPS()->getSpsNext().getUseSubPuMvp() ? 0 : 2 ) ), "More merge candidates signalled than supported" );
      WRITE_UVLC( MRG_MAX_NUM_CANDS - pcSlice->getMaxNumMergeCand() - ( pcSlice->getSPS()->getSpsNext().getUseSubPuMvp() ? 0 : 2 ), pcSlice->getSPS()->getSpsNext().getUseSubPuMvp() ? "seven_minus_max_num_merge_cand" : "five_minus_max_num_merge_cand" );
#else
      CHECK( pcSlice->getMaxNumMergeCand() > ( MRG_MAX_NUM_CANDS - 2 ), "More merge candidates signalled than supported" );
      WRITE_UVLC( MRG_MAX_NUM_CANDS - pcSlice->getMaxNumMergeCand() - 2, "five_minus_max_num_merge_cand" );
#endif
    }
    int iCode = pcSlice->getSliceQp() - ( pcSlice->getPPS()->getPicInitQPMinus26() + 26 );
    WRITE_SVLC( iCode, "slice_qp_delta" );
    if (pcSlice->getPPS()->getSliceChromaQpFlag())
    {
      if (numberValidComponents > COMPONENT_Cb)
      {
        WRITE_SVLC( pcSlice->getSliceChromaQpDelta(COMPONENT_Cb), "slice_cb_qp_offset" );
      }
      if (numberValidComponents > COMPONENT_Cr)
      {
        WRITE_SVLC( pcSlice->getSliceChromaQpDelta(COMPONENT_Cr), "slice_cr_qp_offset" );
      }
      CHECK(numberValidComponents < COMPONENT_Cr+1, "Too many valid components");
    }

    if (pcSlice->getPPS()->getPpsRangeExtension().getChromaQpOffsetListEnabledFlag())
    {
      WRITE_FLAG(pcSlice->getUseChromaQpAdj(), "cu_chroma_qp_offset_enabled_flag");
    }

    if (pcSlice->getPPS()->getDeblockingFilterControlPresentFlag())
    {
      if (pcSlice->getPPS()->getDeblockingFilterOverrideEnabledFlag() )
      {
        WRITE_FLAG(pcSlice->getDeblockingFilterOverrideFlag(), "deblocking_filter_override_flag");
      }
      if (pcSlice->getDeblockingFilterOverrideFlag())
      {
        WRITE_FLAG(pcSlice->getDeblockingFilterDisable(), "slice_deblocking_filter_disabled_flag");
        if(!pcSlice->getDeblockingFilterDisable())
        {
          WRITE_SVLC (pcSlice->getDeblockingFilterBetaOffsetDiv2(), "slice_beta_offset_div2");
          WRITE_SVLC (pcSlice->getDeblockingFilterTcOffsetDiv2(),   "slice_tc_offset_div2");
        }
      }
    }

    bool isSAOEnabled = pcSlice->getSPS()->getUseSAO() && (pcSlice->getSaoEnabledFlag(CHANNEL_TYPE_LUMA) || (chromaEnabled && pcSlice->getSaoEnabledFlag(CHANNEL_TYPE_CHROMA)));
    bool isDBFEnabled = (!pcSlice->getDeblockingFilterDisable());

    if(pcSlice->getPPS()->getLoopFilterAcrossSlicesEnabledFlag() && ( isSAOEnabled || isDBFEnabled ))
    {
      WRITE_FLAG(pcSlice->getLFCrossSliceBoundaryFlag()?1:0, "slice_loop_filter_across_slices_enabled_flag");
    }
#if HEVC_DEPENDENT_SLICES
  }
#endif

#if JVET_K0346
  if (pcSlice->getSPS()->getSpsNext().getUseSubPuMvp() && !pcSlice->isIntra())
  {
    WRITE_FLAG(pcSlice->getSubPuMvpSliceSubblkSizeEnable(), "slice_atmvp_subblk_size_enable_flag");
    if (pcSlice->getSubPuMvpSliceSubblkSizeEnable())
    {
      WRITE_CODE(pcSlice->getSubPuMvpSubblkLog2Size() - MIN_CU_LOG2, 3, "log2_slice_sub_pu_tmvp_size_minus2");
    }
  }
#endif
  if(pcSlice->getPPS()->getSliceHeaderExtensionPresentFlag())
  {
    WRITE_UVLC(0,"slice_segment_header_extension_length");
  }

}


void HLSWriter::codePTL( const PTL* pcPTL, bool profilePresentFlag, int maxNumSubLayersMinus1)
{
  if(profilePresentFlag)
  {
    codeProfileTier(pcPTL->getGeneralPTL(), false);    // general_...
  }
  WRITE_CODE( int(pcPTL->getGeneralPTL()->getLevelIdc()), 8, "general_level_idc" );

  for (int i = 0; i < maxNumSubLayersMinus1; i++)
  {
    WRITE_FLAG( pcPTL->getSubLayerProfilePresentFlag(i), "sub_layer_profile_present_flag[i]" );
    WRITE_FLAG( pcPTL->getSubLayerLevelPresentFlag(i),   "sub_layer_level_present_flag[i]" );
  }

  if (maxNumSubLayersMinus1 > 0)
  {
    for (int i = maxNumSubLayersMinus1; i < 8; i++)
    {
      WRITE_CODE(0, 2, "reserved_zero_2bits");
    }
  }

  for(int i = 0; i < maxNumSubLayersMinus1; i++)
  {
    if( pcPTL->getSubLayerProfilePresentFlag(i) )
    {
      codeProfileTier(pcPTL->getSubLayerPTL(i), true);  // sub_layer_...
    }
    if( pcPTL->getSubLayerLevelPresentFlag(i) )
    {
      WRITE_CODE( int(pcPTL->getSubLayerPTL(i)->getLevelIdc()), 8, "sub_layer_level_idc[i]" );
    }
  }
}

#if ENABLE_TRACING || RExt__DECODER_DEBUG_BIT_STATISTICS
void HLSWriter::codeProfileTier( const ProfileTierLevel* ptl, const bool bIsSubLayer )
#define PTL_TRACE_TEXT(txt) bIsSubLayer?("sub_layer_" txt) : ("general_" txt)
#else
void HLSWriter::codeProfileTier( const ProfileTierLevel* ptl, const bool /*bIsSubLayer*/ )
#define PTL_TRACE_TEXT(txt) txt
#endif
{
  WRITE_CODE( ptl->getProfileSpace(), 2 ,      PTL_TRACE_TEXT("profile_space"                   ));
  WRITE_FLAG( ptl->getTierFlag()==Level::HIGH, PTL_TRACE_TEXT("tier_flag"                       ));
  WRITE_CODE( int(ptl->getProfileIdc()), 5 ,   PTL_TRACE_TEXT("profile_idc"                     ));
  for(int j = 0; j < 32; j++)
  {
    WRITE_FLAG( ptl->getProfileCompatibilityFlag(j), PTL_TRACE_TEXT("profile_compatibility_flag[][j]" ));
  }

  WRITE_FLAG(ptl->getProgressiveSourceFlag(),   PTL_TRACE_TEXT("progressive_source_flag"         ));
  WRITE_FLAG(ptl->getInterlacedSourceFlag(),    PTL_TRACE_TEXT("interlaced_source_flag"          ));
  WRITE_FLAG(ptl->getNonPackedConstraintFlag(), PTL_TRACE_TEXT("non_packed_constraint_flag"      ));
  WRITE_FLAG(ptl->getFrameOnlyConstraintFlag(), PTL_TRACE_TEXT("frame_only_constraint_flag"      ));

  if (ptl->getProfileIdc() == Profile::MAINREXT || ptl->getProfileIdc() == Profile::HIGHTHROUGHPUTREXT )
  {
    const uint32_t         bitDepthConstraint=ptl->getBitDepthConstraint();
    WRITE_FLAG(bitDepthConstraint<=12,          PTL_TRACE_TEXT("max_12bit_constraint_flag"       ));
    WRITE_FLAG(bitDepthConstraint<=10,          PTL_TRACE_TEXT("max_10bit_constraint_flag"       ));
    WRITE_FLAG(bitDepthConstraint<= 8,          PTL_TRACE_TEXT("max_8bit_constraint_flag"        ));
    const ChromaFormat chromaFmtConstraint=ptl->getChromaFormatConstraint();
    WRITE_FLAG(chromaFmtConstraint==CHROMA_422||chromaFmtConstraint==CHROMA_420||chromaFmtConstraint==CHROMA_400, PTL_TRACE_TEXT("max_422chroma_constraint_flag" ));
    WRITE_FLAG(chromaFmtConstraint==CHROMA_420||chromaFmtConstraint==CHROMA_400,                                  PTL_TRACE_TEXT("max_420chroma_constraint_flag" ));
    WRITE_FLAG(chromaFmtConstraint==CHROMA_400,                                                                   PTL_TRACE_TEXT("max_monochrome_constraint_flag"));
    WRITE_FLAG(ptl->getIntraConstraintFlag(),          PTL_TRACE_TEXT("intra_constraint_flag"           ));
    WRITE_FLAG(ptl->getOnePictureOnlyConstraintFlag(), PTL_TRACE_TEXT("one_picture_only_constraint_flag"));
    WRITE_FLAG(ptl->getLowerBitRateConstraintFlag(),   PTL_TRACE_TEXT("lower_bit_rate_constraint_flag"  ));
    WRITE_CODE(0 , 16, PTL_TRACE_TEXT("reserved_zero_34bits[0..15]"     ));
    WRITE_CODE(0 , 16, PTL_TRACE_TEXT("reserved_zero_34bits[16..31]"    ));
    WRITE_CODE(0 ,  2, PTL_TRACE_TEXT("reserved_zero_34bits[32..33]"    ));
  }
  else
  {
    WRITE_CODE(0x0000 , 16, PTL_TRACE_TEXT("reserved_zero_43bits[0..15]"     ));
    WRITE_CODE(0x0000 , 16, PTL_TRACE_TEXT("reserved_zero_43bits[16..31]"    ));
    WRITE_CODE(0x000  , 11, PTL_TRACE_TEXT("reserved_zero_43bits[32..42]"    ));
  }
  WRITE_FLAG(false,   PTL_TRACE_TEXT("reserved_zero_bit" ));
#undef PTL_TRACE_TEXT
}

#if HEVC_TILES_WPP
/**
* Write tiles and wavefront substreams sizes for the slice header (entry points).
*
* \param pSlice Slice structure that contains the substream size information.
*/
void  HLSWriter::codeTilesWPPEntryPoint( Slice* pSlice )
{
  if (!pSlice->getPPS()->getTilesEnabledFlag() && !pSlice->getPPS()->getEntropyCodingSyncEnabledFlag())
  {
    return;
  }
  uint32_t maxOffset = 0;
  for(int idx=0; idx<pSlice->getNumberOfSubstreamSizes(); idx++)
  {
    uint32_t offset=pSlice->getSubstreamSize(idx);
    if ( offset > maxOffset )
    {
      maxOffset = offset;
    }
  }

  // Determine number of bits "offsetLenMinus1+1" required for entry point information
  uint32_t offsetLenMinus1 = 0;
  while (maxOffset >= (1u << (offsetLenMinus1 + 1)))
  {
    offsetLenMinus1++;
    CHECK(offsetLenMinus1 + 1 >= 32, "Invalid offset lenght minus 1");
  }

  WRITE_UVLC(pSlice->getNumberOfSubstreamSizes(), "num_entry_point_offsets");
  if (pSlice->getNumberOfSubstreamSizes()>0)
  {
    WRITE_UVLC(offsetLenMinus1, "offset_len_minus1");

    for (uint32_t idx=0; idx<pSlice->getNumberOfSubstreamSizes(); idx++)
    {
      WRITE_CODE(pSlice->getSubstreamSize(idx)-1, offsetLenMinus1+1, "entry_point_offset_minus1");
    }
  }
}
#endif


// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

//! Code weighted prediction tables
void HLSWriter::xCodePredWeightTable( Slice* pcSlice )
{
  WPScalingParam  *wp;
  const ChromaFormat    format                = pcSlice->getSPS()->getChromaFormatIdc();
  const uint32_t            numberValidComponents = getNumberValidComponents(format);
  const bool            bChroma               = isChromaEnabled(format);
  const int             iNbRef                = (pcSlice->getSliceType() == B_SLICE ) ? (2) : (1);
  bool            bDenomCoded           = false;
  uint32_t            uiTotalSignalledWeightFlags = 0;

  if ( (pcSlice->getSliceType()==P_SLICE && pcSlice->getPPS()->getUseWP()) || (pcSlice->getSliceType()==B_SLICE && pcSlice->getPPS()->getWPBiPred()) )
  {
    for ( int iNumRef=0 ; iNumRef<iNbRef ; iNumRef++ ) // loop over l0 and l1 syntax elements
    {
      RefPicList  eRefPicList = ( iNumRef ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );

      // NOTE: wp[].uiLog2WeightDenom and wp[].bPresentFlag are actually per-channel-type settings.

      for ( int iRefIdx=0 ; iRefIdx<pcSlice->getNumRefIdx(eRefPicList) ; iRefIdx++ )
      {
        pcSlice->getWpScaling(eRefPicList, iRefIdx, wp);
        if ( !bDenomCoded )
        {
          int iDeltaDenom;
          WRITE_UVLC( wp[COMPONENT_Y].uiLog2WeightDenom, "luma_log2_weight_denom" );

          if( bChroma )
          {
            CHECK( wp[COMPONENT_Cb].uiLog2WeightDenom != wp[COMPONENT_Cr].uiLog2WeightDenom, "Chroma blocks of different size not supported" );
            iDeltaDenom = (wp[COMPONENT_Cb].uiLog2WeightDenom - wp[COMPONENT_Y].uiLog2WeightDenom);
            WRITE_SVLC( iDeltaDenom, "delta_chroma_log2_weight_denom" );
          }
          bDenomCoded = true;
        }
        WRITE_FLAG( wp[COMPONENT_Y].bPresentFlag, iNumRef==0?"luma_weight_l0_flag[i]":"luma_weight_l1_flag[i]" );
        uiTotalSignalledWeightFlags += wp[COMPONENT_Y].bPresentFlag;
      }
      if (bChroma)
      {
        for ( int iRefIdx=0 ; iRefIdx<pcSlice->getNumRefIdx(eRefPicList) ; iRefIdx++ )
        {
          pcSlice->getWpScaling( eRefPicList, iRefIdx, wp );
          CHECK( wp[COMPONENT_Cb].bPresentFlag != wp[COMPONENT_Cr].bPresentFlag, "Inconsistent settings for chroma channels" );
          WRITE_FLAG( wp[COMPONENT_Cb].bPresentFlag, iNumRef==0?"chroma_weight_l0_flag[i]":"chroma_weight_l1_flag[i]" );
          uiTotalSignalledWeightFlags += 2*wp[COMPONENT_Cb].bPresentFlag;
        }
      }

      for ( int iRefIdx=0 ; iRefIdx<pcSlice->getNumRefIdx(eRefPicList) ; iRefIdx++ )
      {
        pcSlice->getWpScaling(eRefPicList, iRefIdx, wp);
        if ( wp[COMPONENT_Y].bPresentFlag )
        {
          int iDeltaWeight = (wp[COMPONENT_Y].iWeight - (1<<wp[COMPONENT_Y].uiLog2WeightDenom));
          WRITE_SVLC( iDeltaWeight, iNumRef==0?"delta_luma_weight_l0[i]":"delta_luma_weight_l1[i]" );
          WRITE_SVLC( wp[COMPONENT_Y].iOffset, iNumRef==0?"luma_offset_l0[i]":"luma_offset_l1[i]" );
        }

        if ( bChroma )
        {
          if ( wp[COMPONENT_Cb].bPresentFlag )
          {
            for ( int j = COMPONENT_Cb ; j < numberValidComponents ; j++ )
            {
              CHECK(wp[COMPONENT_Cb].uiLog2WeightDenom != wp[COMPONENT_Cr].uiLog2WeightDenom, "Chroma blocks of different size not supported");
              int iDeltaWeight = (wp[j].iWeight - (1<<wp[COMPONENT_Cb].uiLog2WeightDenom));
              WRITE_SVLC( iDeltaWeight, iNumRef==0?"delta_chroma_weight_l0[i]":"delta_chroma_weight_l1[i]" );

              int range=pcSlice->getSPS()->getSpsRangeExtension().getHighPrecisionOffsetsEnabledFlag() ? (1<<pcSlice->getSPS()->getBitDepth(CHANNEL_TYPE_CHROMA))/2 : 128;
              int pred = ( range - ( ( range*wp[j].iWeight)>>(wp[j].uiLog2WeightDenom) ) );
              int iDeltaChroma = (wp[j].iOffset - pred);
              WRITE_SVLC( iDeltaChroma, iNumRef==0?"delta_chroma_offset_l0[i]":"delta_chroma_offset_l1[i]" );
            }
          }
        }
      }
    }
    CHECK(uiTotalSignalledWeightFlags>24, "Too many signalled weight flags");
  }
}

#if HEVC_USE_SCALING_LISTS
/** code quantization matrix
*  \param scalingList quantization matrix information
*/
void HLSWriter::codeScalingList( const ScalingList &scalingList )
{
  //for each size
  for(uint32_t sizeId = SCALING_LIST_FIRST_CODED; sizeId <= SCALING_LIST_LAST_CODED; sizeId++)
  {
    const int predListStep = (sizeId == SCALING_LIST_32x32? (SCALING_LIST_NUM/NUMBER_OF_PREDICTION_MODES) : 1); // if 32x32, skip over chroma entries.

    for(uint32_t listId = 0; listId < SCALING_LIST_NUM; listId+=predListStep)
    {
      bool scalingListPredModeFlag = scalingList.getScalingListPredModeFlag(sizeId, listId);
      WRITE_FLAG( scalingListPredModeFlag, "scaling_list_pred_mode_flag" );
      if(!scalingListPredModeFlag)// Copy Mode
      {
        if (sizeId == SCALING_LIST_32x32)
        {
          // adjust the code, to cope with the missing chroma entries
          WRITE_UVLC( ((int)listId - (int)scalingList.getRefMatrixId (sizeId,listId)) / (SCALING_LIST_NUM/NUMBER_OF_PREDICTION_MODES), "scaling_list_pred_matrix_id_delta");
        }
        else
        {
          WRITE_UVLC( (int)listId - (int)scalingList.getRefMatrixId (sizeId,listId), "scaling_list_pred_matrix_id_delta");
        }
      }
      else// DPCM Mode
      {
        xCodeScalingList(&scalingList, sizeId, listId);
      }
    }
  }
  return;
}
/** code DPCM
* \param scalingList quantization matrix information
* \param sizeId      size index
* \param listId      list index
*/
void HLSWriter::xCodeScalingList(const ScalingList* scalingList, uint32_t sizeId, uint32_t listId)
{
  int coefNum = std::min( MAX_MATRIX_COEF_NUM, ( int ) g_scalingListSize[sizeId] );
  uint32_t* scan = g_scanOrder[SCAN_UNGROUPED][SCAN_DIAG][gp_sizeIdxInfo->idxFrom( 1 << ( sizeId == SCALING_LIST_FIRST_CODED ? 2 : 3 ) )][gp_sizeIdxInfo->idxFrom( 1 << ( sizeId == SCALING_LIST_FIRST_CODED ? 2 : 3 ) )];
  int nextCoef = SCALING_LIST_START_VALUE;
  int data;
  const int *src = scalingList->getScalingListAddress(sizeId, listId);
  if( sizeId > SCALING_LIST_8x8 )
  {
    WRITE_SVLC( scalingList->getScalingListDC(sizeId,listId) - 8, "scaling_list_dc_coef_minus8");
    nextCoef = scalingList->getScalingListDC(sizeId,listId);
  }
  for(int i=0;i<coefNum;i++)
  {
    data = src[scan[i]] - nextCoef;
    nextCoef = src[scan[i]];
    if(data > 127)
    {
      data = data - 256;
    }
    if(data < -128)
    {
      data = data + 256;
    }

    WRITE_SVLC( data,  "scaling_list_delta_coef");
  }
}
#endif

bool HLSWriter::xFindMatchingLTRP(Slice* pcSlice, uint32_t *ltrpsIndex, int ltrpPOC, bool usedFlag)
{
  // bool state = true, state2 = false;
  int lsb = ltrpPOC & ((1<<pcSlice->getSPS()->getBitsForPOC())-1);
  for (int k = 0; k < pcSlice->getSPS()->getNumLongTermRefPicSPS(); k++)
  {
    if ( (lsb == pcSlice->getSPS()->getLtRefPicPocLsbSps(k)) && (usedFlag == pcSlice->getSPS()->getUsedByCurrPicLtSPSFlag(k)) )
    {
      *ltrpsIndex = k;
      return true;
    }
  }
  return false;
}

#if JVET_K0371_ALF
void HLSWriter::alf( const AlfSliceParam& alfSliceParam )
{
  WRITE_FLAG( alfSliceParam.enabledFlag[COMPONENT_Y], "alf_slice_enable_flag" );
  if( !alfSliceParam.enabledFlag[COMPONENT_Y] )
  {
    return;
  }

  const int alfChromaIdc = alfSliceParam.enabledFlag[COMPONENT_Cb] * 2 + alfSliceParam.enabledFlag[COMPONENT_Cr];
  truncatedUnaryEqProb( alfChromaIdc, 3 );   // alf_chroma_idc

  xWriteTruncBinCode( alfSliceParam.numLumaFilters - 1, MAX_NUM_ALF_CLASSES );  //number_of_filters_minus1
  WRITE_FLAG( alfSliceParam.lumaFilterType == ALF_FILTER_5 ? 1 : 0, "filter_type_flag" );

  if( alfSliceParam.numLumaFilters > 1 )
  {
    for( int i = 0; i < MAX_NUM_ALF_CLASSES; i++ )
    {
      xWriteTruncBinCode( (uint32_t)alfSliceParam.filterCoeffDeltaIdx[i], alfSliceParam.numLumaFilters );  //filter_coeff_delta[i]
    }
  }

  alfFilter( alfSliceParam, false );

  if( alfChromaIdc )
  {
    WRITE_FLAG( alfSliceParam.chromaCtbPresentFlag, "alf_chroma_ctb_present_flag" );
    alfFilter( alfSliceParam, true );
  }
}

void HLSWriter::alfGolombEncode( int coeff, int k )
{
  int symbol = abs( coeff );

  int m = (int)pow( 2.0, k );
  int q = symbol / m;

  for( int i = 0; i < q; i++ )
  {
    xWriteFlag( 1 );
  }
  xWriteFlag( 0 );
  // write one zero

  for( int i = 0; i < k; i++ )
  {
    xWriteFlag( symbol & 0x01 );
    symbol >>= 1;
  }

  if( coeff != 0 )
  {
    int sign = ( coeff > 0 ) ? 1 : 0;
    xWriteFlag( sign );
  }
}

void HLSWriter::alfFilter( const AlfSliceParam& alfSliceParam, const bool isChroma )
{
  if( !isChroma )
  {
    WRITE_FLAG( alfSliceParam.coeffDeltaFlag, "alf_coefficients_delta_flag" );
    if( !alfSliceParam.coeffDeltaFlag )
    {
      if( alfSliceParam.numLumaFilters > 1 )
      {
        WRITE_FLAG( alfSliceParam.coeffDeltaPredModeFlag, "coeff_delta_pred_mode_flag" );
      }
    }
  }

  static int bitsCoeffScan[EncAdaptiveLoopFilter::m_MAX_SCAN_VAL][EncAdaptiveLoopFilter::m_MAX_EXP_GOLOMB];
  memset( bitsCoeffScan, 0, sizeof( bitsCoeffScan ) );
  AlfFilterShape alfShape( isChroma ? 5 : ( alfSliceParam.lumaFilterType == ALF_FILTER_5 ? 5 : 7 ) );
  const int maxGolombIdx = AdaptiveLoopFilter::getMaxGolombIdx( alfShape.filterType );
  const short* coeff = isChroma ? alfSliceParam.chromaCoeff : alfSliceParam.lumaCoeff;
  const int numFilters = isChroma ? 1 : alfSliceParam.numLumaFilters;

  // vlc for all
  for( int ind = 0; ind < numFilters; ++ind )
  {
    if( isChroma || !alfSliceParam.coeffDeltaFlag || alfSliceParam.filterCoeffFlag[ind] )
    {
      for( int i = 0; i < alfShape.numCoeff - 1; i++ )
      {
        int coeffVal = abs( coeff[ind * MAX_NUM_ALF_LUMA_COEFF + i] );

        for( int k = 1; k < 15; k++ )
        {
          bitsCoeffScan[alfShape.golombIdx[i]][k] += EncAdaptiveLoopFilter::lengthGolomb( coeffVal, k );
        }
      }
    }
  }

  static int kMinTab[MAX_NUM_ALF_COEFF];
  int kMin = EncAdaptiveLoopFilter::getGolombKMin( alfShape, numFilters, kMinTab, bitsCoeffScan );

  // Golomb parameters
  WRITE_UVLC( kMin - 1, "min_golomb_order" );

  for( int idx = 0; idx < maxGolombIdx; idx++ )
  {
    bool golombOrderIncreaseFlag = ( kMinTab[idx] != kMin ) ? true : false;
    CHECK( !( kMinTab[idx] <= kMin + 1 ), "ALF Golomb parameter not consistent" );
    WRITE_FLAG( golombOrderIncreaseFlag, "golomb_order_increase_flag" );
    kMin = kMinTab[idx];
  }

  if( !isChroma )
  {
    if( alfSliceParam.coeffDeltaFlag )
    {
      for( int ind = 0; ind < numFilters; ++ind )
      {
        WRITE_FLAG( alfSliceParam.filterCoeffFlag[ind], "filter_coefficient_flag[i]" );
      }
    }
  }

  // Filter coefficients
  for( int ind = 0; ind < numFilters; ++ind )
  {
    if( !isChroma && !alfSliceParam.filterCoeffFlag[ind] && alfSliceParam.coeffDeltaFlag )
    {
      continue;
    }

    for( int i = 0; i < alfShape.numCoeff - 1; i++ )
    {
      alfGolombEncode( coeff[ind* MAX_NUM_ALF_LUMA_COEFF + i], kMinTab[alfShape.golombIdx[i]] );  // alf_coeff_chroma[i], alf_coeff_luma_delta[i][j]
    }
  }
}

void HLSWriter::xWriteTruncBinCode( uint32_t uiSymbol, const int uiMaxSymbol )
{
  int uiThresh;
  if( uiMaxSymbol > 256 )
  {
    int uiThreshVal = 1 << 8;
    uiThresh = 8;
    while( uiThreshVal <= uiMaxSymbol )
    {
      uiThresh++;
      uiThreshVal <<= 1;
    }
    uiThresh--;
  }
  else
  {
    uiThresh = g_tbMax[uiMaxSymbol];
  }

  int uiVal = 1 << uiThresh;
  assert( uiVal <= uiMaxSymbol );
  assert( ( uiVal << 1 ) > uiMaxSymbol );
  assert( uiSymbol < uiMaxSymbol );
  int b = uiMaxSymbol - uiVal;
  assert( b < uiVal );
  if( uiSymbol < uiVal - b )
  {
    xWriteCode( uiSymbol, uiThresh );
  }
  else
  {
    uiSymbol += uiVal - b;
    assert( uiSymbol < ( uiVal << 1 ) );
    assert( ( uiSymbol >> 1 ) >= uiVal - b );
    xWriteCode( uiSymbol, uiThresh + 1 );
  }
}

void HLSWriter::truncatedUnaryEqProb( int symbol, const int maxSymbol )
{
  if( maxSymbol == 0 )
  {
    return;
  }

  bool codeLast = ( maxSymbol > symbol );
  int bins = 0;
  int numBins = 0;

  while( symbol-- )
  {
    bins <<= 1;
    bins++;
    numBins++;
  }
  if( codeLast )
  {
    bins <<= 1;
    numBins++;
  }
  CHECK( !( numBins <= 32 ), "Unspecified error" );
  xWriteCode( bins, numBins );
}
#endif

//! \}

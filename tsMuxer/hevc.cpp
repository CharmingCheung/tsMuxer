#include "hevc.h"

#include <fs/systemlog.h>
#include <cmath>

#include <algorithm>
#include <cmath>

#include "tsMuxer.h"
#include "vodCoreException.h"
#include "vod_common.h"

using namespace std;
static constexpr int EXTENDED_SAR = 255;

unsigned ceilDiv(const unsigned a, const unsigned b) { return (a / b) + ((a % b) ? 1 : 0); }

// ------------------------- HevcUnit -------------------

unsigned HevcUnit::extractUEGolombCode()
{
    try {
        unsigned cnt = 0;
        
        // 计算前导0的数量
        while (m_reader.getBitsLeft() > 0 && !m_reader.getBit()) {
            cnt++;
            
            // 防止无限循环或超大值
            if (cnt >= 32) {
                LTRACE(LT_WARN, 2, "UE Golomb code too long (>32 bits)");
                return UINT32_MAX;
            }
        }
        // 如果没有足够的位来读取整个码字
        if (m_reader.getBitsLeft() < cnt) {
            LTRACE(LT_WARN, 2, "Not enough bits left to read UE Golomb value");
            return UINT32_MAX;
        }
        
        // 计算值：2^cnt - 1 + info位的值
        uint64_t value = (1ULL << cnt) - 1;
        
        for (unsigned i = 0; i < cnt; i++) {
            value += (m_reader.getBit() ? 1ULL : 0ULL) << (cnt - i - 1);
        }
        
        // 检查溢出
        if (value > UINT32_MAX) {
            LTRACE(LT_WARN, 2, "UE Golomb value exceeds UINT32_MAX");
            return UINT32_MAX;
        }
        
        return static_cast<unsigned>(value);
    }
    catch (BitStreamException& e) {
        LTRACE(LT_ERROR, 2, "BitStream exception in extractUEGolombCode: " << e.what());
        throw;  // 重新抛出异常让调用者处理
    }
    catch (const std::exception& e) {
        LTRACE(LT_ERROR, 2, "Exception in extractUEGolombCode: " << e.what());
        throw;  // 重新抛出异常让调用者处理
    }
}

int HevcUnit::extractSEGolombCode()
{
    try {
        const unsigned rez = extractUEGolombCode();
        
        // 检查溢出或无效值
        if (rez == UINT32_MAX) {
            return INT_MAX;  // 表示错误
        }
        
        // 根据SE Golomb编码规则计算有符号值
        if (rez % 2 == 0) {
            // 偶数编码负值
            const int64_t signedValue = -(static_cast<int64_t>(rez) / 2);
            
            // 检查溢出
            if (signedValue < INT_MIN) {
                LTRACE(LT_WARN, 2, "SE Golomb value underflow");
                return INT_MIN;
            }
            
            return static_cast<int>(signedValue);
        }
        else {
            // 奇数编码正值
            const int64_t signedValue = (static_cast<int64_t>(rez) + 1) / 2;
            
            // 检查溢出
            if (signedValue > INT_MAX) {
                LTRACE(LT_WARN, 2, "SE Golomb value overflow");
                return INT_MAX;
            }
            
            return static_cast<int>(signedValue);
        }
    }
    catch (BitStreamException& e) {
        LTRACE(LT_ERROR, 2, "BitStream exception in extractSEGolombCode: " << e.what());
        return INT_MAX;  // 返回错误值
    }
    catch (const std::exception& e) {
        LTRACE(LT_ERROR, 2, "Exception in extractSEGolombCode: " << e.what());
        return INT_MAX;  // 返回错误值
    }
}


void HevcUnit::decodeBuffer(const uint8_t* buffer, const uint8_t* end)
{
    delete[] m_nalBuffer;
    m_nalBuffer = new uint8_t[end - buffer];
    m_nalBufferLen = NALUnit::decodeNAL(buffer, end, m_nalBuffer, end - buffer);
}

int HevcUnit::deserialize()
{
    if (m_nalBuffer == nullptr || m_nalBufferLen == 0) {
        LTRACE(LT_WARN, 2, "Empty NAL buffer in HEVC deserialize");
        return NOT_ENOUGH_BUFFER;
    }
    
    try {
        m_reader.setBuffer(m_nalBuffer, m_nalBuffer + m_nalBufferLen);
        
        if (m_reader.getBitsLeft() < 16) { // 确保至少有足够的位可读
            LTRACE(LT_WARN, 2, "NAL buffer too small for header");
            return NOT_ENOUGH_BUFFER;
        }
        
        m_reader.skipBit();
        nal_unit_type = static_cast<NalType>(m_reader.getBits(6));
        nuh_layer_id = m_reader.getBits<uint8_t>(6);
        nuh_temporal_id_plus1 = m_reader.getBits<uint8_t>(3);
        
        if (nuh_temporal_id_plus1 == 0 ||
            (nuh_temporal_id_plus1 != 1 && (nal_unit_type == NalType::VPS || nal_unit_type == NalType::SPS ||
                                          nal_unit_type == NalType::EOS || nal_unit_type == NalType::EOB)))
            return 1;
        return 0;
    }
    catch (BitStreamException& e) {
        LTRACE(LT_ERROR, 2, "BitStream exception in HEVC deserialize: " << e.what());
        return NOT_ENOUGH_BUFFER;
    }
    catch (const std::exception& e) {
        LTRACE(LT_ERROR, 2, "Exception in HEVC deserialize: " << e.what());
        return NOT_ENOUGH_BUFFER;
    }
}

void HevcUnit::updateBits(const int bitOffset, const int bitLen, const unsigned value) const
{
    if (bitOffset < 0 || bitLen <= 0) {
        LTRACE(LT_WARN, 2, "Invalid parameters in updateBits");
        return;
    }
    
    try {
        uint8_t* ptr = m_reader.getBuffer();
        if (!ptr) {
            LTRACE(LT_WARN, 2, "Null buffer in updateBits");
            return;
        }
        
        // 计算字节偏移和位偏移
        ptr += bitOffset / 8;
        const int byteOffset = bitOffset % 8;
        
        BitStreamWriter bitWriter{};
        
        // 确保有足够的缓冲区
        const uint8_t* bufferEnd = m_reader.getBufferEnd();
        if (!bufferEnd || ptr + (bitLen / 8 + 5) > bufferEnd) {
            LTRACE(LT_WARN, 2, "Buffer too small in updateBits");
            return;
        }
        
        bitWriter.setBuffer(ptr, ptr + (bitLen / 8 + 5));
        
        const uint8_t* ptr_end = m_reader.getBuffer() + (bitOffset + bitLen) / 8;
        if (ptr_end >= bufferEnd) {
            LTRACE(LT_WARN, 2, "End position beyond buffer in updateBits");
            return;
        }
        
        const int endBitsPostfix = 8 - ((bitOffset + bitLen) % 8);
        
        // 保留前缀位
        if (byteOffset > 0) {
            const int prefix = *ptr >> (8 - byteOffset);
            bitWriter.putBits(byteOffset, prefix);
        }
        
        // 写入新值
        bitWriter.putBits(bitLen, value);
        
        // 保留后缀位
        if (endBitsPostfix < 8) {
            const int postfix = *ptr_end & ((1 << endBitsPostfix) - 1);
            bitWriter.putBits(endBitsPostfix, postfix);
        }
        
        bitWriter.flushBits();
    }
    catch (BitStreamException& e) {
        LTRACE(LT_ERROR, 2, "BitStream exception in updateBits: " << e.what());
    }
    catch (const std::exception& e) {
        LTRACE(LT_ERROR, 2, "Exception in updateBits: " << e.what());
    }
}

int HevcUnit::serializeBuffer(uint8_t* dstBuffer, const uint8_t* dstEnd) const
{
    if (m_nalBufferLen == 0)
        return 0;
    const int encodeRez = NALUnit::encodeNAL(m_nalBuffer, m_nalBuffer + m_nalBufferLen, dstBuffer, dstEnd - dstBuffer);
    if (encodeRez == -1)
        return -1;
    return encodeRez;
}

// ------------------------- HevcUnitWithProfile  -------------------

HevcUnitWithProfile::HevcUnitWithProfile() : profile_idc(0), level_idc(0), interlaced_source_flag(false) {}

int HevcUnitWithProfile::profile_tier_level(const int subLayers)
{
    try
    {
        bool sub_layer_profile_present_flag[7]{false};
        bool sub_layer_level_present_flag[7]{false};

        m_reader.skipBits(3);  // profile_space, tier_flag
        profile_idc = m_reader.getBits<uint8_t>(5);
        m_reader.skipBits(32);  // general_profile_compatibility_flag
        m_reader.skipBit();     // progressive_source_flag
        interlaced_source_flag = m_reader.getBit();
        m_reader.skipBits(32);  // unused flags
        m_reader.skipBits(14);  // unused flags
        level_idc = m_reader.getBits<uint8_t>(8);

        for (int i = 0; i < subLayers - 1; i++)
        {
            sub_layer_profile_present_flag[i] = m_reader.getBit();
            sub_layer_level_present_flag[i] = m_reader.getBit();
        }
        if (subLayers > 1)
        {
            for (int i = subLayers - 1; i < 8; i++) m_reader.skipBits(2);  // reserved_zero_2bits
        }

        for (int i = 0; i < subLayers - 1; i++)
        {
            if (sub_layer_profile_present_flag[i])
            {
                m_reader.skipBits(32);  // unused flags
                m_reader.skipBits(32);  // unused flags
                m_reader.skipBits(24);  // unused flags
            }
            if (sub_layer_level_present_flag[i])
                m_reader.skipBits(8);  // sub_layer_level_idc[ i ]
        }
        return 0;
    }
    catch (BitStreamException& e)
    {
        (void)e;
        return NOT_ENOUGH_BUFFER;
    }
}

std::string HevcUnitWithProfile::getProfileString() const
{
    string rez("Profile: ");
    if (profile_idc == 1)
        rez += string("Main");
    else if (profile_idc == 2)
        rez += string("Main10");
    else if (profile_idc == 3)
        rez += string("MainStillPicture");
    else if (profile_idc == 0)
        rez += string("Not defined");
    else
        rez += "Unknown";
    if (level_idc)
    {
        rez += string("@");
        rez += int32ToStr(level_idc / 30);
        rez += string(".");
        rez += int32ToStr((level_idc % 30) / 3);
    }
    return rez;
}

// ------------------------- HevcVpsUnit -------------------

HevcVpsUnit::HevcVpsUnit() : vps_id(0), num_units_in_tick(0), time_scale(0), num_units_in_tick_bit_pos(-1) {}

int HevcVpsUnit::deserialize()
{
    const int rez = HevcUnit::deserialize();
    if (rez)
        return rez;

    try
    {
        m_reader.skipBits(12);  // vps_id, reserved, vps_max_layers
        const uint8_t vps_max_sub_layers = m_reader.getBits<uint8_t>(3) + 1;
        if (vps_max_sub_layers > 7)
            return 1;
        m_reader.skipBits(17);  // vps_temporal_id_nesting_flag, vps_reserved_0xffff_16bits
        if (profile_tier_level(vps_max_sub_layers) != 0)
            return 1;

        const bool vps_sub_layer_ordering_info_present_flag = m_reader.getBit();
        for (int i = (vps_sub_layer_ordering_info_present_flag ? 0 : vps_max_sub_layers - 1);
             i <= vps_max_sub_layers - 1; i++)
        {
            const unsigned vps_max_dec_pic_buffering_minus1 = extractUEGolombCode();
            if (extractUEGolombCode() > vps_max_dec_pic_buffering_minus1)  // vps_max_num_reorder_pics
                return 1;
            if (extractUEGolombCode() == UINT_MAX)  // vps_max_latency_increase_plus1
                return 1;
        }
        const auto vps_max_layer_id = m_reader.getBits<uint8_t>(6);
        const unsigned vps_num_layer_sets_minus1 = extractUEGolombCode();
        if (vps_num_layer_sets_minus1 > 1023)
            return 1;
        for (unsigned i = 1; i <= vps_num_layer_sets_minus1; i++)
        {
            for (int j = 0; j <= vps_max_layer_id; j++) m_reader.skipBit();  // layer_id_included_flag[ i ][ j ] u(1)
        }
        if (m_reader.getBit())  // vps_timing_info_present_flag
        {
            num_units_in_tick_bit_pos = m_reader.getBitsCount();
            num_units_in_tick = m_reader.getBits(32);
            time_scale = m_reader.getBits(32);
        }

        return rez;
    }
    catch (VodCoreException& e)
    {
        (void)e;
        return NOT_ENOUGH_BUFFER;
    }
}

void HevcVpsUnit::setFPS(const double fps)
{
    time_scale = lround(fps) * 1000000;
    num_units_in_tick = lround(time_scale / fps);

    assert(num_units_in_tick_bit_pos > 0);
    updateBits(num_units_in_tick_bit_pos, 32, num_units_in_tick);
    updateBits(num_units_in_tick_bit_pos + 32, 32, time_scale);
}

double HevcVpsUnit::getFPS() const
{
    return num_units_in_tick ? static_cast<double>(time_scale) / num_units_in_tick : 0;
}

string HevcVpsUnit::getDescription() const
{
    string rez("Frame rate: ");
    const double fps = getFPS();
    if (fps != 0.0)
        rez += doubleToStr(fps);
    else
        rez += string("not found");

    return rez;
}

// ------------------------- HevcSpsUnit ------------------------------

HevcSpsUnit::HevcSpsUnit()
    : vps_id(0),
      max_sub_layers(0),
      sps_id(0),
      chromaFormat(0),
      separate_colour_plane_flag(false),
      pic_width_in_luma_samples(0),
      pic_height_in_luma_samples(0),
      bit_depth_luma_minus8(0),
      bit_depth_chroma_minus8(0),
      log2_max_pic_order_cnt_lsb(0),
      nal_hrd_parameters_present_flag(false),
      vcl_hrd_parameters_present_flag(false),
      sub_pic_hrd_params_present_flag(false),
      colour_primaries(2),
      transfer_characteristics(2),
      matrix_coeffs(2),
      chroma_sample_loc_type_top_field(0),
      chroma_sample_loc_type_bottom_field(0),
      num_short_term_ref_pic_sets(0),
      num_units_in_tick(0),
      time_scale(0),
      PicSizeInCtbsY_bits(0)
{
}

// returns 0 on parse success, 1 on error
int HevcSpsUnit::hrd_parameters(const bool commonInfPresentFlag, const int maxNumSubLayersMinus1)
{
    if (commonInfPresentFlag)
    {
        nal_hrd_parameters_present_flag = m_reader.getBit();
        vcl_hrd_parameters_present_flag = m_reader.getBit();
        if (nal_hrd_parameters_present_flag || vcl_hrd_parameters_present_flag)
        {
            sub_pic_hrd_params_present_flag = m_reader.getBit();
            if (sub_pic_hrd_params_present_flag)
            {
                m_reader.skipBits(19);
            }
            m_reader.skipBits(8);  // bit_rate_scale, cpb_size_scale
            if (sub_pic_hrd_params_present_flag)
                m_reader.skipBits(4);  // cpb_size_du_scale u(4)
            m_reader.skipBits(15);
        }
    }

    for (int i = 0; i <= maxNumSubLayersMinus1; i++)
    {
        bool low_delay_hrd_flag = false;
        unsigned cpb_cnt_minus1 = 0;
        const bool fixed_pic_rate_within_cvs_flag = m_reader.getBit() ? true : m_reader.getBit();
        if (fixed_pic_rate_within_cvs_flag)
        {
            if (extractUEGolombCode() > 2047)  // elemental_duration_in_tc_minus1
                return 1;
        }
        else
            low_delay_hrd_flag = m_reader.getBit();
        if (!low_delay_hrd_flag)
        {
            cpb_cnt_minus1 = extractUEGolombCode();
            if (cpb_cnt_minus1 > 32)
                return 1;
        }
        if (nal_hrd_parameters_present_flag)
            if (sub_layer_hrd_parameters(cpb_cnt_minus1) != 0)
                return 1;
        if (vcl_hrd_parameters_present_flag)
            if (sub_layer_hrd_parameters(cpb_cnt_minus1) != 0)
                return 1;
    }
    return 0;
}

// returns 0 on parse success, 1 on error
int HevcSpsUnit::sub_layer_hrd_parameters(const unsigned cpb_cnt_minus1)
{
    for (unsigned i = 0; i <= cpb_cnt_minus1; i++)
    {
        if (extractUEGolombCode() == UINT32_MAX)  // bit_rate_value_minus1[i]
            return 1;
        if (extractUEGolombCode() == UINT32_MAX)  // cpb_size_value_minus1[i]
            return 1;
        if (sub_pic_hrd_params_present_flag)
        {
            if (extractUEGolombCode() == UINT32_MAX)  // cpb_size_du_value_minus1[i]
                return 1;
            if (extractUEGolombCode() == UINT32_MAX)  // bit_rate_du_value_minus1[i]
                return 1;
        }
        m_reader.skipBit();  // cbr_flag[i]
    }
    return 0;
}

int HevcSpsUnit::vui_parameters()
{
    const bool aspect_ratio_info_present_flag = m_reader.getBit();
    if (aspect_ratio_info_present_flag)
    {
        if (m_reader.getBits(8) == EXTENDED_SAR)  // aspect_ratio_idc
            m_reader.skipBits(32);                // sar_width, sar_height
    }

    if (m_reader.getBit())   // overscan_info_present_flag
        m_reader.skipBit();  // overscan_appropriate_flag u(1)
    if (m_reader.getBit())   // video_signal_type_present_flag
    {
        m_reader.skipBits(4);   // video_format, video_full_range_flag
        if (m_reader.getBit())  // colour_description_present_flag
        {
            colour_primaries = m_reader.getBits<uint8_t>(8);
            transfer_characteristics = m_reader.getBits<uint8_t>(8);
            matrix_coeffs = m_reader.getBits<uint8_t>(8);
        }
    }

    if (m_reader.getBit())  // chroma_loc_info_present_flag
    {
        chroma_sample_loc_type_top_field = extractUEGolombCode();
        if (chroma_sample_loc_type_top_field > 5)
            return 1;
        chroma_sample_loc_type_bottom_field = extractUEGolombCode();
        if (chroma_sample_loc_type_bottom_field > 5)
            return 1;
    }

    m_reader.skipBits(3);  // unused flags

    if (m_reader.getBit())  // default_display_window_flag
    {
        extractUEGolombCode();  // def_disp_win_left_offset ue(v)
        extractUEGolombCode();  // def_disp_win_right_offset ue(v)
        extractUEGolombCode();  // def_disp_win_top_offset ue(v)
        extractUEGolombCode();  // def_disp_win_bottom_offset ue(v)
    }

    if (m_reader.getBit())  // vui_timing_info_present_flag
    {
        num_units_in_tick = m_reader.getBits(32);
        time_scale = m_reader.getBits(32);

        if (m_reader.getBit())  // vui_poc_proportional_to_timing_flag
        {
            if (extractUEGolombCode() == UINT_MAX)  // vui_num_ticks_poc_diff_one_minus1
                return 1;
        }
        if (m_reader.getBit())  // vui_hrd_parameters_present_flag
        {
            if (hrd_parameters(true, max_sub_layers - 1) != 0)
                return 1;
        }
    }
    if (m_reader.getBit())  // bitstream_restriction_flag
    {
        m_reader.skipBits(3);  //  unused flags

        if (extractUEGolombCode() > 4095)  // min_spatial_segmentation_idc
            return 1;
        if (extractUEGolombCode() > 16)  // max_bytes_per_pic_denom
            return 1;
        if (extractUEGolombCode() > 16)  // max_bits_per_min_cu_denom
            return 1;
        if (extractUEGolombCode() > 15)  // log2_max_mv_length_horizontal
            return 1;
        if (extractUEGolombCode() > 15)  // log2_max_mv_length_vertical
            return 1;
    }
    return 0;
}

int HevcSpsUnit::short_term_ref_pic_set(const unsigned stRpsIdx)
{
    unsigned numDeltaPocs = 0;
    const bool inter_ref_pic_set_prediction_flag = stRpsIdx && m_reader.getBit();

    if (inter_ref_pic_set_prediction_flag)
    {
        unsigned refRpsIdx = stRpsIdx - 1;

        if (stRpsIdx == num_short_term_ref_pic_sets)
        {
            const unsigned delta_idx_minus1 = extractUEGolombCode();
            if (delta_idx_minus1 >= stRpsIdx)
                return 1;
            refRpsIdx -= delta_idx_minus1;
        }

        m_reader.skipBit();     // delta_rps_sign
        extractUEGolombCode();  // abs_delta_rps_minus1

        for (unsigned j = 0; j <= num_delta_pocs[refRpsIdx]; j++)
        {
            const bool used = m_reader.getBit();                          // used_by_curr_pic_flag[j]
            const bool use_delta_flag = used ? true : m_reader.getBit();  // use_delta_flag[j]
            if (use_delta_flag)
                numDeltaPocs++;
        }
    }
    else
    {
        // numDeltaPocs = num_negative_pics + num_positive_pics
        numDeltaPocs = extractUEGolombCode() + extractUEGolombCode();
        if (numDeltaPocs > 64)
            return 1;

        for (unsigned i = 0; i < numDeltaPocs; i++)
        {
            if (extractUEGolombCode() >= 0x8000)  // delta_poc_minus1[i]
                return 1;
            m_reader.skipBit();  // used_by_curr_pic_flag[i]
        }
    }
    num_delta_pocs[stRpsIdx] = numDeltaPocs;

    return 0;
}

int HevcSpsUnit::scaling_list_data()
{
    for (int sizeId = 0; sizeId < 4; sizeId++)
    {
        for (int matrixId = 0; matrixId < 6; matrixId += (sizeId == 3) ? 3 : 1)
        {
            if (!m_reader.getBit())
            {
                if (extractUEGolombCode() > 5)  // scaling_list_pred_matrix_id_delta
                    return 1;
            }
            else
            {
                const int coefNum = FFMIN(64, (1 << (4 + (sizeId << 1))));
                if (sizeId > 1)
                {
                    const int scaling_list_dc_coef = extractSEGolombCode() + 8;
                    if (scaling_list_dc_coef < 1 || scaling_list_dc_coef > 255)
                        return 1;
                }
                for (int i = 0; i < coefNum; i++)
                {
                    const int scaling_list_delta_coef = extractSEGolombCode();
                    if (scaling_list_delta_coef < -128 || scaling_list_delta_coef > 127)
                        return 1;
                }
            }
        }
    }
    return 0;
}

int HevcSpsUnit::deserialize()
{
    const int rez = HevcUnit::deserialize();
    if (rez)
        return rez;
        
    try {
        vps_id = m_reader.getBits<uint8_t>(4);
        max_sub_layers = m_reader.getBits<uint8_t>(3) + 1;
        
        // 增加范围检查
        if (max_sub_layers > 7) {
            LTRACE(LT_WARN, 2, "Invalid max_sub_layers value: " << max_sub_layers);
            return 1;
        }
        
        m_reader.skipBit();  // temporal_id_nesting_flag
        
        // 增加错误处理
        if (profile_tier_level(max_sub_layers) != 0) {
            LTRACE(LT_WARN, 2, "Error parsing profile_tier_level");
            return 1;
        }
        
        sps_id = extractUEGolombCode();
        if (sps_id > 15) {
            LTRACE(LT_WARN, 2, "Invalid SPS ID: " << sps_id);
            return 1;
        }
        
        chromaFormat = extractUEGolombCode();
        if (chromaFormat > 3) {
            LTRACE(LT_WARN, 2, "Invalid chroma format: " << chromaFormat);
            return 1;
        }
        
        if (chromaFormat == 3)
            separate_colour_plane_flag = m_reader.getBit();
            
        pic_width_in_luma_samples = extractUEGolombCode();
        if (pic_width_in_luma_samples == 0 || pic_width_in_luma_samples > 8192) { // 合理的上限
            LTRACE(LT_WARN, 2, "Invalid picture width: " << pic_width_in_luma_samples);
            return 1;
        }
        
        pic_height_in_luma_samples = extractUEGolombCode();
        if (pic_height_in_luma_samples == 0 || pic_height_in_luma_samples > 4320) { // 合理的上限
            LTRACE(LT_WARN, 2, "Invalid picture height: " << pic_height_in_luma_samples);
            return 1;
        }
        
        // 标记4K内容
        if (pic_width_in_luma_samples >= 3840)
            V3_flags |= FOUR_K;

        // ... 剩余代码 ...

        // 确保所有需要的变量都已正确初始化
        if (log2_max_pic_order_cnt_lsb == 0) {
            LTRACE(LT_WARN, 2, "Invalid log2_max_pic_order_cnt_lsb");
            return 1;
        }
        
        // 安全检查PicSizeInCtbsY_bits
        if (PicSizeInCtbsY_bits == 0 || PicSizeInCtbsY_bits > 32) {
            LTRACE(LT_WARN, 2, "Invalid PicSizeInCtbsY_bits: " << PicSizeInCtbsY_bits);
            PicSizeInCtbsY_bits = 16; // 使用合理的默认值
        }

        return 0;
    }
    catch (VodCoreException& e) {
        LTRACE(LT_ERROR, 2, "VodCore exception in SPS deserialize: " << e.what());
        return NOT_ENOUGH_BUFFER;
    }
    catch (const std::exception& e) {
        LTRACE(LT_ERROR, 2, "Exception in SPS deserialize: " << e.what());
        return NOT_ENOUGH_BUFFER;
    }
}

double HevcSpsUnit::getFPS() const
{
    return num_units_in_tick ? static_cast<double>(time_scale) / num_units_in_tick : 0;
}

string HevcSpsUnit::getDescription() const
{
    string result = getProfileString();
    result += string(" Resolution: ") + int32uToStr(pic_width_in_luma_samples) + string(":") +
              int32uToStr(pic_height_in_luma_samples);
    result += interlaced_source_flag ? string("i") : string("p");

    const double fps = getFPS();
    result += "  Frame rate: ";
    result += fps != 0.0 ? doubleToStr(fps) : string("not found");
    return result;
}

// ----------------------- HevcPpsUnit ------------------------
HevcPpsUnit::HevcPpsUnit()
    : pps_id(0),
      sps_id(0),
      dependent_slice_segments_enabled_flag(false),
      output_flag_present_flag(false),
      num_extra_slice_header_bits(0)
{
}

int HevcPpsUnit::deserialize()
{
    const int rez = HevcUnit::deserialize();
    if (rez)
        return rez;

    try
    {
        pps_id = extractUEGolombCode();
        if (pps_id > 63)
            return 1;
        sps_id = extractUEGolombCode();
        if (sps_id > 15)
            return 1;
        dependent_slice_segments_enabled_flag = m_reader.getBit();
        output_flag_present_flag = m_reader.getBit();
        num_extra_slice_header_bits = m_reader.getBits<uint8_t>(3);

        return 0;
    }
    catch (VodCoreException& e)
    {
        (void)e;
        return NOT_ENOUGH_BUFFER;
    }
}

// ----------------------- HevcHdrUnit ------------------------
HevcHdrUnit::HevcHdrUnit() : isHDR10(false), isHDR10plus(false), isDVRPU(false), isDVEL(false) {}

int HevcHdrUnit::deserialize()
{
    const int rez = HevcUnit::deserialize();
    if (rez)
        return rez;
    try
    {
        do
        {
            int payloadType = 0;
            unsigned payloadSize = 0;
            uint8_t nbyte = 0xff;
            while (nbyte == 0xff)
            {
                nbyte = m_reader.getBits<uint8_t>(8);
                payloadType += nbyte;
            }
            nbyte = 0xff;
            while (nbyte == 0xff)
            {
                nbyte = m_reader.getBits<uint8_t>(8);
                payloadSize += nbyte;
            }
            if (m_reader.getBitsLeft() < payloadSize * 8)
            {
                LTRACE(LT_WARN, 2, "Bad SEI detected. SEI too short");
                return 1;
            }
            if (payloadType == 137 && !isHDR10)  // mastering_display_colour_volume
            {
                isHDR10 = true;
                V3_flags |= HDR10;
                HDR10_metadata[0] = m_reader.getBits(32);  // display_primaries Green
                HDR10_metadata[1] = m_reader.getBits(32);  // display_primaries Red
                HDR10_metadata[2] = m_reader.getBits(32);  // display_primaries Blue
                HDR10_metadata[3] = m_reader.getBits(32);  // White Point
                HDR10_metadata[4] = ((m_reader.getBits(32) / 10000) << 16) +
                                    m_reader.getBits(32);  // max & min display_mastering_luminance
            }
            else if (payloadType == 144)  // content_light_level_info
            {
                auto maxCLL = m_reader.getBits<uint32_t>(16);
                auto maxFALL = m_reader.getBits<uint32_t>(16);
                if (maxCLL > (HDR10_metadata[5] >> 16) || maxFALL > (HDR10_metadata[5] & 0x0000ffff))
                {
                    maxCLL = (std::max)(maxCLL, HDR10_metadata[5] >> 16);
                    maxFALL = (std::max)(maxFALL, HDR10_metadata[5] & 0xffff);
                    HDR10_metadata[5] = (maxCLL << 16) + maxFALL;
                }
            }
            else if (payloadType == 4 && payloadSize >= 8 && !isHDR10plus)
            {                           // HDR10Plus Metadata
                m_reader.skipBits(8);   // country_code
                m_reader.skipBits(32);  // terminal_provider
                const auto application_identifier = m_reader.getBits<uint8_t>(8);
                const auto application_version = m_reader.getBits<uint8_t>(8);
                const auto num_windows = m_reader.getBits<uint8_t>(2);
                m_reader.skipBits(6);
                if (application_identifier == 4 && application_version == 1 && num_windows == 1)
                {
                    isHDR10plus = true;
                    V3_flags |= HDR10PLUS;
                }
                payloadSize -= 8;
                for (unsigned i = 0; i < payloadSize; i++) m_reader.skipBits(8);
            }
            else
                for (unsigned i = 0; i < payloadSize; i++) m_reader.skipBits(8);
        } while (m_reader.getBitsLeft() > 16);

        return 0;
    }
    catch (VodCoreException& e)
    {
        (void)e;
        return NOT_ENOUGH_BUFFER;
    }
}

// -----------------------  HevcSliceHeader() -------------------------------------

HevcSliceHeader::HevcSliceHeader() : first_slice(false), pps_id(-1), slice_type(-1), pic_order_cnt_lsb(0) {}

int HevcSliceHeader::deserialize(const HevcSpsUnit* sps, const HevcPpsUnit* pps)
{
    if (!sps || !pps) {
        LTRACE(LT_WARN, 2, "Null SPS or PPS in slice header deserialize");
        return 1;
    }
    
    const int rez = HevcUnit::deserialize();
    if (rez)
        return rez;

    try {
        pic_order_cnt_lsb = 0;
        first_slice = m_reader.getBit();
        
        if (nal_unit_type >= NalType::BLA_W_LP && nal_unit_type <= NalType::RSV_IRAP_VCL23)
            m_reader.skipBit();  // no_output_of_prior_pics_flag u(1)
            
        pps_id = extractUEGolombCode();
        if (pps_id > 63) {
            LTRACE(LT_WARN, 2, "Invalid PPS ID in slice header: " << pps_id);
            return 1;
        }
        
        bool dependent_slice_segment_flag = false;
        if (!first_slice) {
            if (pps->dependent_slice_segments_enabled_flag)
                dependent_slice_segment_flag = m_reader.getBit();
                
            // 增强边界检查
            if (sps->PicSizeInCtbsY_bits > 0 && sps->PicSizeInCtbsY_bits <= 32) {
                m_reader.skipBits(sps->PicSizeInCtbsY_bits);  // slice_segment_address
            } else {
                LTRACE(LT_WARN, 2, "Invalid PicSizeInCtbsY_bits value: " << sps->PicSizeInCtbsY_bits);
                return 1;
            }
        }
        
        if (!dependent_slice_segment_flag) {
            for (int i = 0; i < pps->num_extra_slice_header_bits; i++)
                m_reader.skipBit();  // slice_reserved_flag[ i ] u(1)
                
            slice_type = extractUEGolombCode();
            if (slice_type > 2) {
                LTRACE(LT_WARN, 2, "Invalid slice type: " << slice_type);
                return 1;
            }
            
            if (pps->output_flag_present_flag)
                m_reader.skipBit();  // pic_output_flag u(1)
                
            if (sps->separate_colour_plane_flag == 1) {
                const auto colourPlaneId = m_reader.getBits(2);
                if (colourPlaneId > 2) {
                    LTRACE(LT_WARN, 2, "Invalid colour_plane_id: " << colourPlaneId);
                    return 1;
                }
            }
            
            // 安全读取pic_order_cnt_lsb
            if (!isIDR() && sps->log2_max_pic_order_cnt_lsb > 0 && sps->log2_max_pic_order_cnt_lsb <= 16) {
                pic_order_cnt_lsb = m_reader.getBits<uint16_t>(sps->log2_max_pic_order_cnt_lsb);
            } else if (!isIDR()) {
                LTRACE(LT_WARN, 2, "Invalid log2_max_pic_order_cnt_lsb: " << sps->log2_max_pic_order_cnt_lsb);
                return 1;
            }
        }

        return 0;
    }
    catch (VodCoreException& e) {
        LTRACE(LT_ERROR, 2, "VodCore exception in slice header deserialize: " << e.what());
        return NOT_ENOUGH_BUFFER;
    }
    catch (const std::exception& e) {
        LTRACE(LT_ERROR, 2, "Exception in slice header deserialize: " << e.what());
        return NOT_ENOUGH_BUFFER;
    }
}

bool HevcSliceHeader::isIDR() const
{
    return nal_unit_type == NalType::IDR_W_RADL || nal_unit_type == NalType::IDR_N_LP;
}

vector<vector<uint8_t>> hevc_extract_priv_data(const uint8_t* buff, int size, uint8_t* nal_size)
{
    *nal_size = 4;
    vector<vector<uint8_t>> spsPps;
    
    // 增强边界检查
    if (size < 23) {
        LTRACE(LT_WARN, 2, "HEVC extra data too short, size: " << size);
        return spsPps;
    }

    try {
        *nal_size = (buff[21] & 3) + 1;
        int num_arrays = buff[22];

        const uint8_t* src = buff + 23;
        const uint8_t* end = buff + size;
        
        for (int i = 0; i < num_arrays && src + 3 <= end; ++i) {
            uint8_t nal_type = *src++;
            
            // 采用更安全的读取方式
            int cnt = 0; 
            if (src + 2 <= end) {
                cnt = (src[0] << 8) | src[1];
                src += 2;
            } else {
                LTRACE(LT_WARN, 2, "Buffer overrun in HEVC param parsing");
                break;
            }

            for (int j = 0; j < cnt && src < end; ++j) {
                int nalSize = 0;
                if (src + 2 <= end) {
                    nalSize = (src[0] << 8) | src[1];
                    src += 2;
                } else {
                    LTRACE(LT_WARN, 2, "Buffer overrun in HEVC NAL size reading");
                    break;
                }
                
                if (nalSize > 0) {
                    if (src + nalSize <= end) {
                        spsPps.emplace_back();
                        auto& nal = spsPps.back();
                        nal.reserve(nalSize); // 预分配内存以避免多次重分配
                        for (int k = 0; k < nalSize; ++k) {
                            nal.push_back(src[k]);
                        }
                        src += nalSize;
                    } else {
                        LTRACE(LT_WARN, 2, "NAL size exceeds buffer bounds");
                        break;
                    }
                }
            }
        }
        
        return spsPps;
    }
    catch (const std::exception& e) {
        LTRACE(LT_ERROR, 2, "Exception in HEVC param extraction: " << e.what());
        spsPps.clear();
        return spsPps;
    }
}

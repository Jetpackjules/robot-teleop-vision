#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

#if defined(__SSE2__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP >= 2)
#define RGBD_PACK_SSE2 1
#include <emmintrin.h>
#include <tmmintrin.h>
#if defined(_MSC_VER)
#include <intrin.h>
#endif
#endif

namespace rgbd_pixels {

#ifdef RGBD_PACK_SSE2
inline bool has_ssse3() {
#if defined(__GNUC__) || defined(__clang__)
    static const bool supported = __builtin_cpu_supports("ssse3");
#else
    static const bool supported = [] { int info[4]; __cpuid(info,1); return (info[2] & (1<<9)) != 0; }();
#endif
    return supported;
}
#if defined(__GNUC__) || defined(__clang__)
__attribute__((target("ssse3")))
#endif
inline int bgr_ssse3(const uint8_t *source, uint8_t *destination, int count, bool mask_black) {
    const __m128i shuffle = _mm_setr_epi8(2,1,0,-128,5,4,3,-128,8,7,6,-128,11,10,9,-128);
    const __m128i alpha = _mm_set1_epi32(int32_t(0xff000000u));
    int i=0;
    // A 16-byte load produces four RGBA pixels from twelve BGR bytes.
    // Keep the entire load inside this row, including its last iteration.
    for (;i+6<=count;i+=4) {
        const __m128i rgb = _mm_shuffle_epi8(_mm_loadu_si128(reinterpret_cast<const __m128i *>(source+i*3)),shuffle);
        const __m128i a = mask_black ? _mm_andnot_si128(_mm_cmpeq_epi32(rgb,_mm_setzero_si128()),alpha) : alpha;
        _mm_storeu_si128(reinterpret_cast<__m128i *>(destination+i*4),_mm_or_si128(rgb,a));
    }
    return i;
}
#endif

inline void bgr_to_rgba(const uint8_t *source, uint8_t *destination, int count, bool mask_black, bool vectorize=true) {
    int i=0;
#ifdef RGBD_PACK_SSE2
    if (vectorize && has_ssse3()) i=bgr_ssse3(source,destination,count,mask_black);
#endif
    for (;i<count;i++) {
        const uint8_t *bgr=source+i*3;
        destination[i*4]=bgr[2];destination[i*4+1]=bgr[1];destination[i*4+2]=bgr[0];
        destination[i*4+3]=!mask_black || (bgr[0]|bgr[1]|bgr[2])!=0 ? 255 : 0;
    }
}

inline int z16_to_rf(const uint16_t *source, float *destination, int count, float units,
        float minimum, float maximum, bool vectorize=true) {
    int i=0,valid=0;
#ifdef RGBD_PACK_SSE2
    if (vectorize) {
        const __m128 scale=_mm_set1_ps(units),lo=_mm_set1_ps(minimum),hi=_mm_set1_ps(maximum);
        __m128i counts=_mm_setzero_si128();
        for (;i+8<=count;i+=8) {
            const __m128i z=_mm_loadu_si128(reinterpret_cast<const __m128i *>(source+i));
            const __m128 a=_mm_mul_ps(_mm_cvtepi32_ps(_mm_unpacklo_epi16(z,_mm_setzero_si128())),scale);
            const __m128 b=_mm_mul_ps(_mm_cvtepi32_ps(_mm_unpackhi_epi16(z,_mm_setzero_si128())),scale);
            _mm_storeu_ps(destination+i,a);_mm_storeu_ps(destination+i+4,b);
            counts=_mm_sub_epi32(counts,_mm_castps_si128(_mm_and_ps(_mm_cmpge_ps(a,lo),_mm_cmple_ps(a,hi))));
            counts=_mm_sub_epi32(counts,_mm_castps_si128(_mm_and_ps(_mm_cmpge_ps(b,lo),_mm_cmple_ps(b,hi))));
        }
        int lanes[4];_mm_storeu_si128(reinterpret_cast<__m128i *>(lanes),counts);
        valid=lanes[0]+lanes[1]+lanes[2]+lanes[3];
    }
#endif
    for (;i<count;i++) {
        const float depth=float(source[i])*units;destination[i]=depth;
        if (depth>=minimum && depth<=maximum) valid++;
    }
    return valid;
}

inline void rf_to_mm(const float *source, uint8_t *destination, int count, float maximum, bool vectorize=true) {
    int i=0;
#ifdef RGBD_PACK_SSE2
    if (vectorize) {
        const __m128 zero=_mm_setzero_ps(),hi=_mm_set1_ps(std::min(maximum,std::numeric_limits<float>::max()));
        const __m128d scale=_mm_set1_pd(1000.0),half=_mm_set1_pd(.5),limit=_mm_set1_pd(65535.0);
        for (;i+4<=count;i+=4) {
            __m128 depth=_mm_loadu_ps(source+i);
            depth=_mm_and_ps(depth,_mm_and_ps(_mm_cmpgt_ps(depth,zero),_mm_cmple_ps(depth,hi)));
            const __m128d a=_mm_min_pd(_mm_mul_pd(_mm_cvtps_pd(depth),scale),limit);
            const __m128d b=_mm_min_pd(_mm_mul_pd(_mm_cvtps_pd(_mm_movehl_ps(depth,depth)),scale),limit);
            const __m128i packed=_mm_unpacklo_epi64(_mm_cvttpd_epi32(_mm_add_pd(a,half)),_mm_cvttpd_epi32(_mm_add_pd(b,half)));
            int values[4];_mm_storeu_si128(reinterpret_cast<__m128i *>(values),packed);
            for (int lane=0;lane<4;lane++) {destination[(i+lane)*2]=uint8_t(values[lane]);destination[(i+lane)*2+1]=uint8_t(values[lane]>>8);}
        }
    }
#endif
    for (;i<count;i++) {
        const float depth=source[i];
        const uint16_t mm=std::isfinite(depth) && depth>0.0f && depth<=maximum
            ? uint16_t(std::min(65535.0,std::round(double(depth)*1000.0))) : 0;
        destination[i*2]=uint8_t(mm);destination[i*2+1]=uint8_t(mm>>8);
    }
}

inline void predict_vertical(const uint8_t *source,uint8_t *destination,int count,int width,bool vectorize=true) {
    if (width<=0) return;
    uint8_t *low=destination,*high=destination+count;
    for (int row=0;row<count;row+=width) {
        const int end=row+std::min(width,count-row);
        int i=row;
#ifdef RGBD_PACK_SSE2
        if (vectorize) {
            const __m128i mask=_mm_set1_epi16(255),zero=_mm_setzero_si128();
            for (;i+8<=end;i+=8) {
                const __m128i current=_mm_loadu_si128(reinterpret_cast<const __m128i *>(source+i*2));
                const __m128i previous=row==0?zero:_mm_loadu_si128(reinterpret_cast<const __m128i *>(source+(i-width)*2));
                const __m128i delta=_mm_sub_epi16(current,previous);
                const __m128i zigzag=_mm_xor_si128(_mm_slli_epi16(delta,1),_mm_srai_epi16(delta,15));
                _mm_storel_epi64(reinterpret_cast<__m128i *>(low+i),_mm_packus_epi16(_mm_and_si128(zigzag,mask),zero));
                _mm_storel_epi64(reinterpret_cast<__m128i *>(high+i),_mm_packus_epi16(_mm_srli_epi16(zigzag,8),zero));
            }
        }
#endif
        for (;i<end;i++) {
            const uint16_t current=uint16_t(source[i*2])|(uint16_t(source[i*2+1])<<8);
            const uint16_t previous=row==0?0:uint16_t(source[(i-width)*2])|(uint16_t(source[(i-width)*2+1])<<8);
            const int16_t delta=static_cast<int16_t>(uint16_t(current-previous));
            const uint16_t zigzag=delta>=0?uint16_t(uint32_t(delta)*2U):uint16_t(uint32_t(-int32_t(delta))*2U-1U);
            low[i]=uint8_t(zigzag);high[i]=uint8_t(zigzag>>8);
        }
    }
}

} // namespace rgbd_pixels

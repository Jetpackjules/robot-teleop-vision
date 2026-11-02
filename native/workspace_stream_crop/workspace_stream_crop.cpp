// Exact integer kernels for the host's per-viewer RGB-D crop.
// SPDX-License-Identifier: EUPL-1.2
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <vector>
#include <algorithm>
#include <cstring>
#if (defined(__x86_64__) || defined(__i386__)) && defined(__GNUC__)
#include <immintrin.h>

__attribute__((target("avx2")))
static void sparse_outcodes(const uint16_t *depth,const uint16_t *const *threshold,
                           const uint8_t *const *kind,const uint8_t *mode,uint8_t *code,size_t count) {
    const auto sign=_mm256_set1_epi16(short(0x8000));
    const auto zero=_mm256_setzero_si256();
    size_t i=0;
    for(;i+16<=count;i+=16) {
        const auto d=_mm256_loadu_si256(reinterpret_cast<const __m256i *>(depth+i));
        const auto u=_mm256_xor_si256(d,sign);
        auto bits=zero;
        for(int plane=0;plane<6;++plane) {
            const auto t=_mm256_xor_si256(_mm256_loadu_si256(reinterpret_cast<const __m256i *>(threshold[plane]+i)),sign);
            auto outside=_mm256_cmpgt_epi16(u,t);
            if(mode[plane]!=1) {
                const auto k=_mm256_cvtepu8_epi16(_mm_loadu_si128(reinterpret_cast<const __m128i *>(kind[plane]+i)));
                const auto lower=_mm256_cmpgt_epi16(t,u),never=_mm256_cmpeq_epi16(k,_mm256_set1_epi16(3));
                outside=mode[plane]==2?_mm256_or_si256(lower,never):_mm256_or_si256(never,
                    _mm256_or_si256(_mm256_and_si256(outside,_mm256_cmpeq_epi16(k,_mm256_set1_epi16(1))),
                                     _mm256_and_si256(lower,_mm256_cmpeq_epi16(k,_mm256_set1_epi16(2)))));
            }
            bits=_mm256_or_si256(bits,_mm256_and_si256(outside,_mm256_set1_epi16(1<<plane)));
        }
        bits=_mm256_or_si256(bits,_mm256_and_si256(_mm256_cmpeq_epi16(d,zero),_mm256_set1_epi16(255)));
        _mm_storeu_si128(reinterpret_cast<__m128i *>(code+i),_mm_packus_epi16(
            _mm256_castsi256_si128(bits),_mm256_extracti128_si256(bits,1)));
    }
    for(;i<count;++i) {
        uint8_t bits=0;
        for(int p=0;p<6;++p) {
            const auto d=depth[i],t=threshold[p][i];const auto k=kind[p][i];
            const bool outside=mode[p]==1?d>t:mode[p]==2?(d<t||k==3):((k==1&&d>t)||(k==2&&d<t)||k==3);
            bits|=outside?1<<p:0;
        }
        code[i]=depth[i]?bits:255;
    }
}
#endif

extern "C" {
int workspace_crop_abi() { return 1; }

// Same quad, tile and two-pixel halo as crop_cells + retain_cells, without
// materializing their intermediate full-image cell masks. Inputs stay owned
// by the caller and never refer to an SDK/UVC frame buffer.
int workspace_sparse_retain(const uint16_t *depth,const uint16_t *const *threshold,
                            const uint8_t *const *kind,const uint8_t *mode,
                            uint16_t *out,size_t width,size_t height) {
    if(!depth||!threshold||!kind||!mode||!out||!width||!height||width>8192||height>8192||width*height>16*1024*1024)return 0;
    for(int p=0;p<6;++p)if(!threshold[p]||!kind[p]||mode[p]>2)return 0;
    thread_local std::vector<uint8_t> code,tiles;
    code.resize(width*height);
    const size_t columns=(width+15)/16,rows=(height+7)/8;
    tiles.assign(columns*rows,0);
    bool vectorized=false;
#if (defined(__x86_64__) || defined(__i386__)) && defined(__GNUC__)
    if(__builtin_cpu_supports("avx2")) {
        sparse_outcodes(depth,threshold,kind,mode,code.data(),code.size());vectorized=true;
    }
#endif
    if(!vectorized)for(size_t i=0;i<code.size();++i) {
        uint8_t bits=0;
        for(int p=0;p<6;++p) {
            const auto d=depth[i],t=threshold[p][i];const auto k=kind[p][i];
            const bool outside=mode[p]==1?d>t:mode[p]==2?(d<t||k==3):((k==1&&d>t)||(k==2&&d<t)||k==3);
            bits|=outside?1<<p:0;
        }
        code[i]=depth[i]?bits:255;
    }
    for(size_t y=0;y<height;++y)for(size_t tx=0;tx<columns;++tx) {
        auto &tile=tiles[(y/8)*columns+tx];
        if(tile)continue;
        const size_t begin=tx*16,end=std::min(width,begin+16);
        for(size_t x=begin;x<end;++x) {
            const size_t i=y*width+x;
            uint8_t bits=code[i];
            if(x+1<width)bits&=code[i+1];
            if(y+1<height) {bits&=code[i+width];if(x+1<width)bits&=code[i+width+1];}
            if(!bits){tile=1;break;}
        }
    }
    for(size_t y=0;y<height;++y) {
        const auto *top=tiles.data()+((y>2?y-2:0)/8)*columns;
        const auto *bottom=tiles.data()+(std::min(height-1,y+2)/8)*columns;
        for(size_t tx=0;tx<columns;++tx) {
            const size_t x=tx*16,n=std::min(size_t(16),width-x),offset=y*width+x;
            if(top[tx]||bottom[tx])std::memcpy(out+offset,depth+offset,n*2);
            else {
                std::memset(out+offset,0,n*2);
                if(tx&&(top[tx-1]||bottom[tx-1]))std::memcpy(out+offset,depth+offset,std::min(n,size_t(2))*2);
                if(tx+1<columns&&(top[tx+1]||bottom[tx+1]))std::memcpy(out+offset+n-2,depth+offset+n-2,4);
            }
        }
    }
    return 1;
}

void workspace_clip_plane(const float *coefficient, double limit, uint16_t *threshold,
                          uint8_t *kind, size_t count) {
    for (size_t i = 0; i < count; ++i) {
        const float c = coefficient[i];
        const float ratio = c != 0 ? float(limit*1000.0 / double(c)) : 0.0f;
        uint8_t k = c > 0 ? 1 : c < 0 ? 2 : 0;
        if ((c > 0 && ratio < 0) || (c < 0 && ratio > 65535) || (c == 0 && limit < 0)) k = 3;
        if ((c > 0 && ratio >= 65535) || (c < 0 && ratio <= 0)) k = 0;
        const float bound = c > 0 ? std::floor(ratio) : std::ceil(ratio);
        threshold[i] = bound <= 0 ? 0 : bound >= 65535 ? 65535 : uint16_t(bound);
        kind[i] = k;
    }
}

void workspace_decode_predictive(const uint8_t *shuffled, uint16_t *out,
                                size_t count, size_t width) {
    for (size_t row = 0; row < count; row += width) {
        for (size_t x = 0; x < width; ++x) {
            const size_t i = row + x;
            const uint16_t zigzag = uint16_t(shuffled[i]) | (uint16_t(shuffled[count+i]) << 8);
            const uint16_t delta = uint16_t((zigzag >> 1) ^ uint16_t(0u - (zigzag & 1u)));
            out[i] = uint16_t(delta + (row ? out[i-width] : 0));
        }
    }
}

void workspace_outcode_plane(const uint16_t *__restrict depth, const uint16_t *__restrict threshold,
                            const uint8_t *__restrict kind, uint8_t *__restrict out, size_t count, uint8_t bit) {
    for (size_t i = 0; i < count; ++i) {
        const bool outside = ((kind[i] == 1) & (depth[i] > threshold[i]))
                           | ((kind[i] == 2) & (depth[i] < threshold[i])) | (kind[i] == 3);
        out[i] |= outside ? bit : 0;
    }
}

void workspace_outcode_upper(const uint16_t *__restrict depth, const uint16_t *__restrict threshold,
                            uint8_t *__restrict out, size_t count, uint8_t bit) {
    for (size_t i = 0; i < count; ++i) out[i] |= depth[i] > threshold[i] ? bit : 0;
}

void workspace_outcode_lower(const uint16_t *__restrict depth, const uint16_t *__restrict threshold,
                            const uint8_t *__restrict kind, uint8_t *__restrict out, size_t count, uint8_t bit) {
    for (size_t i = 0; i < count; ++i) out[i] |= ((depth[i] < threshold[i]) | (kind[i] == 3)) ? bit : 0;
}

void workspace_quad_cells(const uint16_t *__restrict depth, uint8_t *__restrict code,
                          uint8_t *__restrict cells, size_t width, size_t height) {
    for (size_t i = 0; i < width*height; ++i) if (!depth[i]) code[i] = 255;
    for (size_t y = 0; y+1 < height; ++y) {
      for (size_t x = 0; x+1 < width; ++x) {
        const size_t i = y*width+x;
        cells[i] &= (code[i]&code[i+1]&code[i+width]&code[i+width+1]) == 0;
      }
      const size_t i = (y+1)*width-1;
      cells[i] &= (code[i]&code[i+width]) == 0;
    }
    for (size_t x = 0; x+1 < width; ++x) {
      const size_t i = (height-1)*width+x;
      cells[i] &= (code[i]&code[i+1]) == 0;
    }
    cells[width*height-1] &= code[width*height-1] == 0;
}

void workspace_retain_cells(const uint16_t *depth, const uint8_t *cells,
                            uint16_t *out, size_t width, size_t height) {
    const size_t columns = (width+15)/16, rows = (height+7)/8;
    std::vector<uint8_t> tiles(columns*rows, 0);
    for (size_t y = 0; y < height; ++y) for (size_t x = 0; x < width; ++x)
        tiles[(y/8)*columns+x/16] |= cells[y*width+x];
    for (size_t y = 0; y < height; ++y) {
        const size_t top = (y > 2 ? y-2 : 0)/8, bottom = (y+2 < height ? y+2 : height-1)/8;
        for (size_t x = 0; x < width; ++x) {
            const size_t left = (x > 2 ? x-2 : 0)/16, right = (x+2 < width ? x+2 : width-1)/16;
            const bool keep = tiles[top*columns+left] | tiles[top*columns+right]
                            | tiles[bottom*columns+left] | tiles[bottom*columns+right];
            out[y*width+x] = keep ? depth[y*width+x] : 0;
        }
    }
}
}

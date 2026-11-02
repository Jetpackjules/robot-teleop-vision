// Native camera-buffer kernels; no sensor ownership or calibration writes.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstddef>
#if (defined(__x86_64__) || defined(__i386__)) && defined(__GNUC__)
#include <immintrin.h>
// Retain double-precision conversion and positive truncation exactly as the
// scalar floor(z*1000+.5), without paying for floor once per depth pixel.
__attribute__((target("avx2")))
static void depth_mm_avx2(const float *source,uint16_t *out,size_t count,double maximum) {
    const auto zero=_mm256_setzero_pd(),limit=_mm256_set1_pd(maximum);
    const auto scale=_mm256_set1_pd(1000),half=_mm256_set1_pd(.5),cap=_mm256_set1_pd(65535);
    size_t i=0;
    for(;i+8<=count;i+=8) {
        const auto convert=[&](const float *p) __attribute__((target("avx2"))) {
            const auto z=_mm256_cvtps_pd(_mm_loadu_ps(p));
            const auto valid=_mm256_and_pd(_mm256_cmp_pd(z,zero,_CMP_GT_OQ),_mm256_cmp_pd(z,limit,_CMP_LE_OQ));
            const auto rounded=_mm256_min_pd(cap,_mm256_add_pd(_mm256_mul_pd(z,scale),half));
            return _mm256_cvttpd_epi32(_mm256_and_pd(rounded,valid));
        };
        _mm_storeu_si128(reinterpret_cast<__m128i *>(out+i),_mm_packus_epi32(convert(source+i),convert(source+i+4)));
    }
    for(;i<count;++i) {
        const double z=source[i];
        out[i]=std::isfinite(z)&&z>0&&z<=maximum?uint16_t(std::min(65535.0,std::floor(z*1000.0+.5))):0;
    }
}
#endif

extern "C" {
void sparse_depth_mm(const float *source,uint16_t *out,size_t count,double maximum) {
#if (defined(__x86_64__) || defined(__i386__)) && defined(__GNUC__)
    if(__builtin_cpu_supports("avx2"))return depth_mm_avx2(source,out,count,maximum);
#endif
    for(size_t i=0;i<count;++i) {
        const double z=source[i];
        out[i]=std::isfinite(z)&&z>0&&z<=maximum
            ?uint16_t(std::min(65535.0,std::floor(z*1000.0+.5))):0;
    }
}

uint32_t sparse_pack(const uint16_t *depth,int w,int h,const uint8_t *bgr,
        int rw,int rh,int pitch,int bpp,const double *p,uint32_t budget,
        uint8_t *out,size_t capacity,uint32_t *source_count) {
    if(!depth||!bgr||!p||!out||!source_count||w<1||h<1||rw<2||rh<2||
            (bpp!=3&&bpp!=4)||pitch<rw*bpp||!budget||budget>16*1024*1024)return 0;
    const size_t pixels=size_t(w)*h;
    uint32_t valid=0;
    for(size_t i=0;i<pixels;++i)valid+=depth[i]!=0;
    *source_count=valid;
    const uint32_t count=std::min(valid,budget);
    if(capacity<size_t(count)*9||!count)return 0;
    uint32_t rank=0,selected=0,target=uint32_t(std::floor(double(valid)/count*.5));
    for(size_t i=0;i<pixels&&selected<count;++i) {
        const uint16_t mm=depth[i];
        if(!mm)continue;
        if(rank++!=target)continue;
        uint8_t *index=out+selected*4;
        for(int k=0;k<4;++k)index[k]=uint8_t(i>>(8*k));
        out[count*4+selected*2]=uint8_t(mm);
        out[count*4+selected*2+1]=uint8_t(mm>>8);
        const double z=double(mm)*.001;
        const double x=(double(i%w)-p[2])*z/p[0],y=(double(i/w)-p[3])*z/p[1];
        const double X=p[8]*x+p[11]*y+p[14]*z+p[17];
        const double Y=p[9]*x+p[12]*y+p[15]*z+p[18];
        const double Z=p[10]*x+p[13]*y+p[16]*z+p[19];
        const double u=X/(Z>0?Z:1)*p[4]+p[6],v=Y/(Z>0?Z:1)*p[5]+p[7];
        const bool inside=Z>0&&u>=0&&v>=0&&u<rw-1&&v<rh-1;
        const int a=inside?int(std::floor(u)):0,b=inside?int(std::floor(v)):0;
        const double du=u-a,dv=v-b;
        for(int channel=0;channel<3;++channel) {
            double value=140;
            if(inside) {
                const auto pixel=[&](int xx,int yy){return double(bgr[size_t(yy)*pitch+size_t(xx)*bpp+2-channel]);};
                value=std::nearbyint(pixel(a,b)*(1-du)*(1-dv)+pixel(a+1,b)*du*(1-dv)
                                  +pixel(a,b+1)*(1-du)*dv+pixel(a+1,b+1)*du*dv);
                value=std::clamp(value,0.0,255.0);
            }
            if(p[26]!=0)value=std::floor(std::clamp(value/255*p[20+channel]+p[23+channel],0.0,1.0)*255+.5);
            out[count*6+selected*3+channel]=uint8_t(value);
        }
        ++selected;
        target=uint32_t(std::floor((selected+.5)*valid/count));
    }
    return count;
}
}

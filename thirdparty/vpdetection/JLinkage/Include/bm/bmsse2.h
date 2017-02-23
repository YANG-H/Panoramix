/*
Copyright(c) 2002-2005 Anatoliy Kuznetsov(anatoliy_kuznetsov at yahoo.com)

Permission is hereby granted, free of charge, to any person 
obtaining a copy of this software and associated documentation 
files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, 
publish, distribute, sublicense, and/or sell copies of the Software, 
and to permit persons to whom the Software is furnished to do so, 
subject to the following conditions:

The above copyright notice and this permission notice shall be included 
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES 
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR 
OTHER DEALINGS IN THE SOFTWARE.

For more information please visit:  http://bmagic.sourceforge.net

*/


#ifndef BMSSE2__H__INCLUDED__
#define BMSSE2__H__INCLUDED__


//    Header implements processor specific intrinsics declarations for SSE2
//    instruction set
#include<emmintrin.h>



namespace bm
{

/** @defgroup SSE2 Processor specific optimizations for SSE2 instructions
 *  @ingroup bmagic
 */


/*! 
  @brief SSE2 reinitialization guard class

  SSE2 requires to call _mm_empty() if we are intermixing
  MMX integer commands with floating point arithmetics.
  This class guards critical code fragments where SSE2 integer
  is used.

  @ingroup SSE2

*/
class sse2_empty_guard
{
public:
    BMFORCEINLINE sse2_empty_guard() 
    {
        _mm_empty();
    }

    BMFORCEINLINE ~sse2_empty_guard() 
    {
        _mm_empty();
    }
};

/*
# ifndef BM_SET_MMX_GUARD
#  define BM_SET_MMX_GUARD  sse2_empty_guard  bm_mmx_guard_;
# endif
*/

/*! 
    @brief XOR array elements to specified mask
    *dst = *src ^ mask

    @ingroup SSE2
*/
BMFORCEINLINE 
void sse2_xor_arr_2_mask(__m128i* BMRESTRICT dst, 
                         const __m128i* BMRESTRICT src, 
                         const __m128i* BMRESTRICT src_end,
                         bm::word_t mask)
{
     __m128i xmm2 = _mm_set_epi32(mask, mask, mask, mask);
     do
     {
        __m128i xmm1 = _mm_load_si128(src);

        xmm1 = _mm_xor_si128(xmm1, xmm2);
        _mm_store_si128(dst, xmm1);
        ++dst;
        ++src;

     } while (src < src_end);
}

/*! 
    @brief Inverts array elements and NOT them to specified mask
    *dst = ~*src & mask

    @ingroup SSE2
*/
BMFORCEINLINE 
void sse2_andnot_arr_2_mask(__m128i* BMRESTRICT dst, 
                            const __m128i* BMRESTRICT src, 
                            const __m128i* BMRESTRICT src_end,
                            bm::word_t mask)
{
     __m128i xmm2 = _mm_set_epi32(mask, mask, mask, mask);
     do
     {
        //_mm_prefetch((const char*)(src)+1024, _MM_HINT_NTA);
        //_mm_prefetch((const char*)(src)+1088, _MM_HINT_NTA);

        __m128i xmm1 = _mm_load_si128(src);

        xmm1 = _mm_andnot_si128(xmm1, xmm2); // xmm1 = (~xmm1) & xmm2 
        _mm_store_si128(dst, xmm1);
        ++dst;
        ++src;

     } while (src < src_end);
}

/*! 
    @brief AND array elements against another array
    *dst &= *src

    @ingroup SSE2
*/
BMFORCEINLINE 
void sse2_and_arr(__m128i* BMRESTRICT dst, 
                  const __m128i* BMRESTRICT src, 
                  const __m128i* BMRESTRICT src_end)
{
    __m128i xmm1, xmm2;
    do
    {
        _mm_prefetch((const char*)(src)+512,  _MM_HINT_NTA);
    
        xmm1 = _mm_load_si128(src++);
        xmm2 = _mm_load_si128(dst);
        xmm1 = _mm_and_si128(xmm1, xmm2);
        _mm_store_si128(dst++, xmm1);
        
        xmm1 = _mm_load_si128(src++);
        xmm2 = _mm_load_si128(dst);
        xmm1 = _mm_and_si128(xmm1, xmm2);
        _mm_store_si128(dst++, xmm1);

        xmm1 = _mm_load_si128(src++);
        xmm2 = _mm_load_si128(dst);
        xmm1 = _mm_and_si128(xmm1, xmm2);
        _mm_store_si128(dst++, xmm1);

        xmm1 = _mm_load_si128(src++);
        xmm2 = _mm_load_si128(dst);
        xmm1 = _mm_and_si128(xmm1, xmm2);
        _mm_store_si128(dst++, xmm1);

    } while (src < src_end);

}



/*! 
    @brief OR array elements against another array
    *dst |= *src

    @ingroup SSE2
*/
BMFORCEINLINE 
void sse2_or_arr(__m128i* BMRESTRICT dst, 
                 const __m128i* BMRESTRICT src, 
                 const __m128i* BMRESTRICT src_end)
{
    __m128i xmm1, xmm2;
    do
    {
        _mm_prefetch((const char*)(src)+512,  _MM_HINT_NTA);
    
        xmm1 = _mm_load_si128(src++);
        xmm2 = _mm_load_si128(dst);
        xmm1 = _mm_or_si128(xmm1, xmm2);
        _mm_store_si128(dst++, xmm1);
        
        xmm1 = _mm_load_si128(src++);
        xmm2 = _mm_load_si128(dst);
        xmm1 = _mm_or_si128(xmm1, xmm2);
        _mm_store_si128(dst++, xmm1);

        xmm1 = _mm_load_si128(src++);
        xmm2 = _mm_load_si128(dst);
        xmm1 = _mm_or_si128(xmm1, xmm2);
        _mm_store_si128(dst++, xmm1);

        xmm1 = _mm_load_si128(src++);
        xmm2 = _mm_load_si128(dst);
        xmm1 = _mm_or_si128(xmm1, xmm2);
        _mm_store_si128(dst++, xmm1);

    } while (src < src_end);
}

/*! 
    @brief OR array elements against another array
    *dst |= *src

    @ingroup SSE2
*/
BMFORCEINLINE 
void sse2_xor_arr(__m128i* BMRESTRICT dst, 
                  const __m128i* BMRESTRICT src, 
                  const __m128i* BMRESTRICT src_end)
{
    __m128i xmm1, xmm2;
    do
    {
        _mm_prefetch((const char*)(src)+512,  _MM_HINT_NTA);
    
        xmm1 = _mm_load_si128(src++);
        xmm2 = _mm_load_si128(dst);
        xmm1 = _mm_xor_si128(xmm1, xmm2);
        _mm_store_si128(dst++, xmm1);
        
        xmm1 = _mm_load_si128(src++);
        xmm2 = _mm_load_si128(dst);
        xmm1 = _mm_xor_si128(xmm1, xmm2);
        _mm_store_si128(dst++, xmm1);

        xmm1 = _mm_load_si128(src++);
        xmm2 = _mm_load_si128(dst);
        xmm1 = _mm_xor_si128(xmm1, xmm2);
        _mm_store_si128(dst++, xmm1);

        xmm1 = _mm_load_si128(src++);
        xmm2 = _mm_load_si128(dst);
        xmm1 = _mm_xor_si128(xmm1, xmm2);
        _mm_store_si128(dst++, xmm1);

    } while (src < src_end);
}


/*! 
    @brief AND-NOT (SUB) array elements against another array
    *dst &= ~*src

    @ingroup SSE2
*/
BMFORCEINLINE 
void sse2_sub_arr(__m128i* BMRESTRICT dst, 
                 const __m128i* BMRESTRICT src, 
                 const __m128i* BMRESTRICT src_end)
{
    __m128i xmm1, xmm2;
    do
    {
        _mm_prefetch((const char*)(src)+512,  _MM_HINT_NTA);
    
        xmm1 = _mm_load_si128(src++);
        xmm2 = _mm_load_si128(dst);
        xmm1 = _mm_andnot_si128(xmm1, xmm2);
        _mm_store_si128(dst++, xmm1);
        
        xmm1 = _mm_load_si128(src++);
        xmm2 = _mm_load_si128(dst);
        xmm1 = _mm_andnot_si128(xmm1, xmm2);
        _mm_store_si128(dst++, xmm1);

        xmm1 = _mm_load_si128(src++);
        xmm2 = _mm_load_si128(dst);
        xmm1 = _mm_andnot_si128(xmm1, xmm2);
        _mm_store_si128(dst++, xmm1);

        xmm1 = _mm_load_si128(src++);
        xmm2 = _mm_load_si128(dst);
        xmm1 = _mm_andnot_si128(xmm1, xmm2);
        _mm_store_si128(dst++, xmm1);

    } while (src < src_end);    
}

/*! 
    @brief SSE2 block memset
    *dst = value

    @ingroup SSE2
*/

BMFORCEINLINE 
void sse2_set_block(__m128i* BMRESTRICT dst, 
                    __m128i* BMRESTRICT dst_end, 
                    bm::word_t value)
{
    __m128i xmm0 = _mm_set_epi32 (value, value, value, value);
    do
    {            
        _mm_store_si128(dst, xmm0);
/*        
        _mm_store_si128(dst+1, xmm0);
        _mm_store_si128(dst+2, xmm0);
        _mm_store_si128(dst+3, xmm0);

        _mm_store_si128(dst+4, xmm0);
        _mm_store_si128(dst+5, xmm0);
        _mm_store_si128(dst+6, xmm0);
        _mm_store_si128(dst+7, xmm0);

        dst += 8;
*/        
    } while (++dst < dst_end);
    
    _mm_sfence();
}

/*! 
    @brief SSE2 block copy
    *dst = *src

    @ingroup SSE2
*/
BMFORCEINLINE 
void sse2_copy_block(__m128i* BMRESTRICT dst, 
                     const __m128i* BMRESTRICT src, 
                     const __m128i* BMRESTRICT src_end)
{
    __m128i xmm0, xmm1, xmm2, xmm3;
    do
    {
        _mm_prefetch((const char*)(src)+512,  _MM_HINT_NTA);
    
        xmm0 = _mm_load_si128(src+0);
        xmm1 = _mm_load_si128(src+1);
        xmm2 = _mm_load_si128(src+2);
        xmm3 = _mm_load_si128(src+3);
        
        _mm_store_si128(dst+0, xmm0);
        _mm_store_si128(dst+1, xmm1);
        _mm_store_si128(dst+2, xmm2);
        _mm_store_si128(dst+3, xmm3);
        
        xmm0 = _mm_load_si128(src+4);
        xmm1 = _mm_load_si128(src+5);
        xmm2 = _mm_load_si128(src+6);
        xmm3 = _mm_load_si128(src+7);
        
        _mm_store_si128(dst+4, xmm0);
        _mm_store_si128(dst+5, xmm1);
        _mm_store_si128(dst+6, xmm2);
        _mm_store_si128(dst+7, xmm3);
        
        src += 8;
        dst += 8;
        
    } while (src < src_end);    
}


/*! 
    @brief Invert array elements
    *dst = ~*dst
    or
    *dst ^= *dst 

    @ingroup SSE2
*/
BMFORCEINLINE 
void sse2_invert_arr(bm::word_t* first, bm::word_t* last)
{
    __m128i xmm1 = _mm_set_epi32(0xFFFFFFFF, 0xFFFFFFFF, 
                                 0xFFFFFFFF, 0xFFFFFFFF);
    __m128i* wrd_ptr = (__m128i*)first;

    do 
    {
        _mm_prefetch((const char*)(wrd_ptr)+512,  _MM_HINT_NTA);
        
        __m128i xmm0 = _mm_load_si128(wrd_ptr);
        xmm0 = _mm_xor_si128(xmm0, xmm1);
        _mm_store_si128(wrd_ptr, xmm0);
        ++wrd_ptr;
    } while (wrd_ptr < (__m128i*)last);
}



/*!
    SSE2 optimized bitcounting function implements parallel bitcounting
    algorithm for SSE2 instruction set.

<pre>
unsigned CalcBitCount32(unsigned b)
{
    b = (b & 0x55555555) + (b >> 1 & 0x55555555);
    b = (b & 0x33333333) + (b >> 2 & 0x33333333);
    b = (b + (b >> 4)) & 0x0F0F0F0F;
    b = b + (b >> 8);
    b = (b + (b >> 16)) & 0x0000003F;
    return b;
}
</pre>

    @ingroup SSE2

*/
inline 
bm::id_t sse2_bit_count(const __m128i* block, const __m128i* block_end)
{
    const unsigned mu1 = 0x55555555;
    const unsigned mu2 = 0x33333333;
    const unsigned mu3 = 0x0F0F0F0F;
    const unsigned mu4 = 0x0000003F;

    // Loading masks
    __m128i m1 = _mm_set_epi32 (mu1, mu1, mu1, mu1);
    __m128i m2 = _mm_set_epi32 (mu2, mu2, mu2, mu2);
    __m128i m3 = _mm_set_epi32 (mu3, mu3, mu3, mu3);
    __m128i m4 = _mm_set_epi32 (mu4, mu4, mu4, mu4);
    __m128i mcnt;
    mcnt = _mm_xor_si128(m1, m1); // cnt = 0

    __m128i tmp1, tmp2;
    do
    {        
        __m128i b = _mm_load_si128(block);
        ++block;

        // b = (b & 0x55555555) + (b >> 1 & 0x55555555);
        tmp1 = _mm_srli_epi32(b, 1);                    // tmp1 = (b >> 1 & 0x55555555)
        tmp1 = _mm_and_si128(tmp1, m1); 
        tmp2 = _mm_and_si128(b, m1);                    // tmp2 = (b & 0x55555555)
        b    = _mm_add_epi32(tmp1, tmp2);               //  b = tmp1 + tmp2

        // b = (b & 0x33333333) + (b >> 2 & 0x33333333);
        tmp1 = _mm_srli_epi32(b, 2);                    // (b >> 2 & 0x33333333)
        tmp1 = _mm_and_si128(tmp1, m2); 
        tmp2 = _mm_and_si128(b, m2);                    // (b & 0x33333333)
        b    = _mm_add_epi32(tmp1, tmp2);               // b = tmp1 + tmp2

        // b = (b + (b >> 4)) & 0x0F0F0F0F;
        tmp1 = _mm_srli_epi32(b, 4);                    // tmp1 = b >> 4
        b = _mm_add_epi32(b, tmp1);                     // b = b + (b >> 4)
        b = _mm_and_si128(b, m3);                       //           & 0x0F0F0F0F

        // b = b + (b >> 8);
        tmp1 = _mm_srli_epi32 (b, 8);                   // tmp1 = b >> 8
        b = _mm_add_epi32(b, tmp1);                     // b = b + (b >> 8)

        // b = (b + (b >> 16)) & 0x0000003F;
        tmp1 = _mm_srli_epi32 (b, 16);                  // b >> 16
        b = _mm_add_epi32(b, tmp1);                     // b + (b >> 16)
        b = _mm_and_si128(b, m4);                       // (b >> 16) & 0x0000003F;

        mcnt = _mm_add_epi32(mcnt, b);                  // mcnt += b

    } while (block < block_end);

    __declspec(align(16)) bm::id_t tcnt[4];
    _mm_store_si128((__m128i*)tcnt, mcnt);

    return tcnt[0] + tcnt[1] + tcnt[2] + tcnt[3];
}

BMFORCEINLINE 
__m128i sse2_and(__m128i a, __m128i b)
{
    return _mm_and_si128(a, b);
}

BMFORCEINLINE 
__m128i sse2_or(__m128i a, __m128i b)
{
    return _mm_or_si128(a, b);
}


BMFORCEINLINE 
__m128i sse2_xor(__m128i a, __m128i b)
{
    return _mm_xor_si128(a, b);
}

BMFORCEINLINE 
__m128i sse2_sub(__m128i a, __m128i b)
{
    return _mm_andnot_si128(b, a);
}


template<class Func>
bm::id_t sse2_bit_count_op(const __m128i* BMRESTRICT block, 
                           const __m128i* BMRESTRICT block_end,
                           const __m128i* BMRESTRICT mask_block,
                           Func sse2_func)
{
    const unsigned mu1 = 0x55555555;
    const unsigned mu2 = 0x33333333;
    const unsigned mu3 = 0x0F0F0F0F;
    const unsigned mu4 = 0x0000003F;

    // Loading masks
    __m128i m1 = _mm_set_epi32 (mu1, mu1, mu1, mu1);
    __m128i m2 = _mm_set_epi32 (mu2, mu2, mu2, mu2);
    __m128i m3 = _mm_set_epi32 (mu3, mu3, mu3, mu3);
    __m128i m4 = _mm_set_epi32 (mu4, mu4, mu4, mu4);
    __m128i mcnt;
    mcnt = _mm_xor_si128(m1, m1); // cnt = 0
    do
    {
        __m128i tmp1, tmp2;
        __m128i b = _mm_load_si128(block++);

        tmp1 = _mm_load_si128(mask_block++);
        
        b = sse2_func(b, tmp1);
                        
        // b = (b & 0x55555555) + (b >> 1 & 0x55555555);
        tmp1 = _mm_srli_epi32(b, 1);                    // tmp1 = (b >> 1 & 0x55555555)
        tmp1 = _mm_and_si128(tmp1, m1); 
        tmp2 = _mm_and_si128(b, m1);                    // tmp2 = (b & 0x55555555)
        b    = _mm_add_epi32(tmp1, tmp2);               //  b = tmp1 + tmp2

        // b = (b & 0x33333333) + (b >> 2 & 0x33333333);
        tmp1 = _mm_srli_epi32(b, 2);                    // (b >> 2 & 0x33333333)
        tmp1 = _mm_and_si128(tmp1, m2); 
        tmp2 = _mm_and_si128(b, m2);                    // (b & 0x33333333)
        b    = _mm_add_epi32(tmp1, tmp2);               // b = tmp1 + tmp2

        // b = (b + (b >> 4)) & 0x0F0F0F0F;
        tmp1 = _mm_srli_epi32(b, 4);                    // tmp1 = b >> 4
        b = _mm_add_epi32(b, tmp1);                     // b = b + (b >> 4)
        b = _mm_and_si128(b, m3);                       //           & 0x0F0F0F0F

        // b = b + (b >> 8);
        tmp1 = _mm_srli_epi32 (b, 8);                   // tmp1 = b >> 8
        b = _mm_add_epi32(b, tmp1);                     // b = b + (b >> 8)
        
        // b = (b + (b >> 16)) & 0x0000003F;
        tmp1 = _mm_srli_epi32 (b, 16);                  // b >> 16
        b = _mm_add_epi32(b, tmp1);                     // b + (b >> 16)
        b = _mm_and_si128(b, m4);                       // (b >> 16) & 0x0000003F;

        mcnt = _mm_add_epi32(mcnt, b);                  // mcnt += b

    } while (block < block_end);

    __declspec(align(16)) bm::id_t tcnt[4];
    _mm_store_si128((__m128i*)tcnt, mcnt);

    return tcnt[0] + tcnt[1] + tcnt[2] + tcnt[3];
}




#define VECT_XOR_ARR_2_MASK(dst, src, src_end, mask)\
    sse2_xor_arr_2_mask((__m128i*)(dst), (__m128i*)(src), (__m128i*)(src_end), mask)

#define VECT_ANDNOT_ARR_2_MASK(dst, src, src_end, mask)\
    sse2_andnot_arr_2_mask((__m128i*)(dst), (__m128i*)(src), (__m128i*)(src_end), mask)

#define VECT_BITCOUNT(first, last) \
    sse2_bit_count((__m128i*) (first), (__m128i*) (last)) 

#define VECT_BITCOUNT_AND(first, last, mask) \
    sse2_bit_count_op((__m128i*) (first), (__m128i*) (last), (__m128i*) (mask), sse2_and) 

#define VECT_BITCOUNT_OR(first, last, mask) \
    sse2_bit_count_op((__m128i*) (first), (__m128i*) (last), (__m128i*) (mask), sse2_or) 

#define VECT_BITCOUNT_XOR(first, last, mask) \
    sse2_bit_count_op((__m128i*) (first), (__m128i*) (last), (__m128i*) (mask), sse2_xor) 

#define VECT_BITCOUNT_SUB(first, last, mask) \
    sse2_bit_count_op((__m128i*) (first), (__m128i*) (last), (__m128i*) (mask), sse2_sub) 

#define VECT_INVERT_ARR(first, last) \
    sse2_invert_arr(first, last);

#define VECT_AND_ARR(dst, src, src_end) \
    sse2_and_arr((__m128i*) dst, (__m128i*) (src), (__m128i*) (src_end))

#define VECT_OR_ARR(dst, src, src_end) \
    sse2_or_arr((__m128i*) dst, (__m128i*) (src), (__m128i*) (src_end))

#define VECT_SUB_ARR(dst, src, src_end) \
    sse2_sub_arr((__m128i*) dst, (__m128i*) (src), (__m128i*) (src_end))

#define VECT_XOR_ARR(dst, src, src_end) \
    sse2_xor_arr((__m128i*) dst, (__m128i*) (src), (__m128i*) (src_end))

#define VECT_COPY_BLOCK(dst, src, src_end) \
    sse2_copy_block((__m128i*) dst, (__m128i*) (src), (__m128i*) (src_end))

#define VECT_SET_BLOCK(dst, dst_end, value) \
    sse2_set_block((__m128i*) dst, (__m128i*) (dst_end), (value))

} // namespace

#endif

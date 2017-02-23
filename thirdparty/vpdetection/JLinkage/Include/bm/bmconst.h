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

#ifndef BMCONST__H__INCLUDED__
#define BMCONST__H__INCLUDED__

namespace bm
{

#if defined(_WIN32) || defined (_WIN64)

typedef unsigned __int64 id64_t;

#else

typedef unsigned long long id64_t;

#endif

typedef unsigned int   id_t;
typedef unsigned int   word_t;
typedef unsigned short short_t;



const unsigned id_max = 0xFFFFFFFF;

// Data Block parameters

const unsigned set_block_size  = 2048u;
const unsigned set_block_shift = 16u;
const unsigned set_block_mask  = 0xFFFFu;
const unsigned set_blkblk_mask = 0xFFFFFFu;

// Word parameters

const unsigned set_word_shift = 5u;
const unsigned set_word_mask  = 0x1Fu;


// GAP related parameters.

typedef unsigned short gap_word_t;

const unsigned gap_max_buff_len = 1280;
const unsigned gap_max_bits = 65536;
const unsigned gap_equiv_len = 
   (sizeof(bm::word_t) * bm::set_block_size) / sizeof(gap_word_t);
const unsigned gap_levels = 4;
const unsigned gap_max_level = bm::gap_levels - 1;


// Block Array parameters

const unsigned set_array_size = 256u;
const unsigned set_array_shift = 8u;
const unsigned set_array_mask  = 0xFFu;
const unsigned set_total_blocks = (bm::set_array_size * bm::set_array_size);

const unsigned bits_in_block = bm::set_block_size * sizeof(bm::word_t) * 8;
const unsigned bits_in_array = bm::bits_in_block * bm::set_array_size;


#ifdef BM64OPT

typedef id64_t  wordop_t;
const id64_t    all_bits_mask = 0xffffffffffffffff;

# define DECLARE_TEMP_BLOCK(x)  bm::id64_t x[bm::set_block_size / 2]; 
const unsigned set_block_size_op  = bm::set_block_size / 2;


#else

typedef word_t wordop_t;
const word_t all_bits_mask = 0xffffffff;

# define DECLARE_TEMP_BLOCK(x)  unsigned x[bm::set_block_size]; 
const unsigned set_block_size_op  = bm::set_block_size;

#endif



/*!
   @brief Block allocation strategies.
   @ingroup bvector
*/
enum strategy
{
    BM_BIT = 0, //!< No GAP compression strategy. All new blocks are bit blocks.
    BM_GAP = 1  //!< GAP compression is ON.
};


} // namespace

#endif


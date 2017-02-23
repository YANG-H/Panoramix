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

#ifndef BMSERIAL__H__INCLUDED__
#define BMSERIAL__H__INCLUDED__

/*! \defgroup bvserial bvector serialization  
 *  bvector serialization
 *  \ingroup bmagic 
 *
 */

#ifndef BM__H__INCLUDED__
#define BM__H__INCLUDED__

#include "bm.h"

#endif

#ifdef _MSC_VER
#pragma warning( disable : 4311 4312)
#endif



#include "encoding.h"
#include "bmdef.h"
#include "bmfunc.h"
#include "bmalgo_impl.h"


// Serialization related constants


const unsigned char set_block_end   = 0;   //!< End of serialization
const unsigned char set_block_1zero = 1;   //!< One all-zero block
const unsigned char set_block_1one  = 2;   //!< One block all-set (1111...)
const unsigned char set_block_8zero = 3;   //!< Up to 256 zero blocks
const unsigned char set_block_8one  = 4;   //!< Up to 256 all-set blocks
const unsigned char set_block_16zero= 5;   //!< Up to 65536 zero blocks
const unsigned char set_block_16one = 6;   //!< UP to 65536 all-set blocks
const unsigned char set_block_32zero= 7;   //!< Up to 4G zero blocks
const unsigned char set_block_32one = 8;   //!< UP to 4G all-set blocks
const unsigned char set_block_azero = 9;   //!< All other blocks zero
const unsigned char set_block_aone  = 10;  //!< All other blocks one

const unsigned char set_block_bit     = 11;  //!< Plain bit block
const unsigned char set_block_sgapbit = 12;  //!< SGAP compressed bitblock
const unsigned char set_block_sgapgap = 13;  //!< SGAP compressed GAP block
const unsigned char set_block_gap     = 14;  //!< Plain GAP block
const unsigned char set_block_gapbit  = 15;  //!< GAP compressed bitblock 
const unsigned char set_block_arrbit  = 16;  //!< List of bits ON
const unsigned char set_block_bit_interval = 17; //!< Interval block
const unsigned char set_block_arrgap  = 18;  //!< List of bits ON (GAP block)






#define SER_NEXT_GRP(enc, nb, B_1ZERO, B_8ZERO, B_16ZERO, B_32ZERO) \
   if (nb == 1) \
      enc.put_8(B_1ZERO); \
   else if (nb < 256) \
   { \
      enc.put_8(B_8ZERO); \
      enc.put_8((unsigned char)nb); \
   } \
   else if (nb < 65536) \
   { \
      enc.put_8(B_16ZERO); \
      enc.put_16((unsigned short)nb); \
   } \
   else \
   {\
      enc.put_8(B_32ZERO); \
      enc.put_32(nb); \
   }


#define BM_SET_ONE_BLOCKS(x) \
    {\
         unsigned end_block = i + x; \
         for (;i < end_block; ++i) \
            bman.set_block_all_set(i); \
    } \
    --i



namespace bm
{

/// \internal
/// \ingroup bvserial 
enum serialization_header_mask {
    BM_HM_DEFAULT = 1,
    BM_HM_RESIZE  = (1 << 1), // resized vector
    BM_HM_ID_LIST = (1 << 2), // id list stored
    BM_HM_NO_BO   = (1 << 3), // no byte-order
    BM_HM_NO_GAPL = (1 << 4)  // no GAP levels
};


/// Bit mask flags for serialization algorithm
/// \ingroup bvserial 
enum serialization_flags {
    BM_NO_BYTE_ORDER = 1,       ///< save no byte-order info (save some space)
    BM_NO_GAP_LENGTH = (1 << 1) ///< save no GAP info (save some space)
};

/*!
   \brief Saves bitvector into memory.

   Function serializes content of the bitvector into memory.
   Serialization adaptively uses compression(variation of GAP encoding) 
   when it is benefitial. 

   \param buf - pointer on target memory area. No range checking in the
   function. It is responsibility of programmer to allocate sufficient 
   amount of memory using information from calc_stat function.

   \param temp_block - pointer on temporary memory block. Cannot be 0; 
   If you want to save memory across multiple bvectors
   allocate temporary block using allocate_tempblock and pass it to 
   serialize.
   (Of course serialize does not deallocate temp_block.)

   \param serialization_flags
   Flags controlling serilization (bit-mask) 
   (use OR-ed serialization flags)

   \ingroup bvserial 

   \return Size of serialization block.
   \sa calc_stat, serialization_flags

*/
/*
 Serialization format:
 <pre>

 | HEADER | BLOCKS |

 Header structure:
   BYTE : Serialization header (bit mask of BM_HM_*)
   BYTE : Byte order ( 0 - Big Endian, 1 - Little Endian)
   INT16: Reserved (0)
   INT16: Reserved Flags (0)

 </pre>
*/
template<class BV>
unsigned serialize(const BV& bv, 
                   unsigned char* buf, 
                   bm::word_t*    temp_block,
                   unsigned       serialization_flags = 0)
{
    BM_ASSERT(temp_block);
    
    typedef typename BV::blocks_manager_type blocks_manager_type;
    const blocks_manager_type& bman = bv.get_blocks_manager();

    gap_word_t*  gap_temp_block = (gap_word_t*) temp_block;
    
    
    bm::encoder enc(buf, 0);

    // Header

    unsigned char header_flag = 0;
    if (bv.size() == bm::id_max) // no dynamic resize
        header_flag |= BM_HM_DEFAULT;
    else 
        header_flag |= BM_HM_RESIZE;

    if (serialization_flags & BM_NO_BYTE_ORDER) 
        header_flag |= BM_HM_NO_BO;

    if (serialization_flags & BM_NO_GAP_LENGTH) 
        header_flag |= BM_HM_NO_GAPL;


    enc.put_8(header_flag);

    if (!(serialization_flags & BM_NO_BYTE_ORDER))
    {
        ByteOrder bo = globals<true>::byte_order();
        enc.put_8((unsigned char)bo);
    }

    unsigned i,j;

    // keep GAP levels information
    if (!(serialization_flags & BM_NO_GAP_LENGTH))
    {
        enc.put_16(bman.glen(), bm::gap_levels);
    }

    // save size (only if bvector has been down-sized)
    if (header_flag & BM_HM_RESIZE) 
    {
        enc.put_32(bv.size());
    }


    // save blocks.
    for (i = 0; i < bm::set_total_blocks; ++i)
    {
        bm::word_t* blk = bman.get_block(i);

        // -----------------------------------------
        // Empty or ONE block serialization

        bool flag;
        flag = bman.is_block_zero(i, blk);
        if (flag)
        {
        zero_block:
            if (bman.is_no_more_blocks(i+1)) 
            {
                enc.put_8(set_block_azero);
                break; 
            }

            // Look ahead for similar blocks
            for(j = i+1; j < bm::set_total_blocks; ++j)
            {
               bm::word_t* blk_next = bman.get_block(j);
               if (flag != bman.is_block_zero(j, blk_next))
                   break;
            }
            if (j == bm::set_total_blocks)
            {
                enc.put_8(set_block_azero);
                break; 
            }
            else
            {
               unsigned nb = j - i;
               SER_NEXT_GRP(enc, nb, set_block_1zero, 
                                     set_block_8zero, 
                                     set_block_16zero, 
                                     set_block_32zero) 
            }
            i = j - 1;
            continue;
        }
        else
        {
            flag = bman.is_block_one(i, blk);
            if (flag)
            {
                // Look ahead for similar blocks
                for(j = i+1; j < bm::set_total_blocks; ++j)
                {
                   bm::word_t* blk_next = bman.get_block(j);
                   if (flag != bman.is_block_one(j, blk_next))
                       break;
                }
                if (j == bm::set_total_blocks)
                {
                    enc.put_8(set_block_aone);
                    break;
                }
                else
                {
                   unsigned nb = j - i;
                   SER_NEXT_GRP(enc, nb, set_block_1one, 
                                         set_block_8one, 
                                         set_block_16one, 
                                         set_block_32one) 
                }
                i = j - 1;
                continue;
            }
        }

        // ------------------------------
        // GAP serialization

        if (BM_IS_GAP(bman, blk, i))
        {
            gap_word_t* gblk = BMGAP_PTR(blk);
            unsigned len = gap_length(gblk);
            unsigned bc = gap_bit_count(gblk);

            if (bc+1 < len-1) 
            {
                gap_word_t len;
                len = gap_convert_to_arr(gap_temp_block,
                                         gblk,
                                         bm::gap_equiv_len-10);
                if (len == 0) 
                {
                    goto save_as_gap;
                }
                enc.put_8(set_block_arrgap);
                enc.put_16(len);
                enc.put_16(gap_temp_block, len);
            }
            else 
            {
            save_as_gap:
                enc.put_8(set_block_gap);
                enc.put_16(gblk, len-1);
            }
            continue;
        }
        
        // -------------------------------
        // BIT BLOCK serialization

        // Try to reduce the size up to the reasonable limit.
        /*
        unsigned len = bm::bit_convert_to_gap(gap_temp_block, 
                                              blk, 
                                              bm::GAP_MAX_BITS, 
                                              bm::GAP_EQUIV_LEN-64);
        */
        gap_word_t len;

        len = bit_convert_to_arr(gap_temp_block, 
                                 blk, 
                                 bm::gap_max_bits, 
                                 bm::gap_equiv_len-64);

        if (len)  // reduced
        {
            enc.put_8(set_block_arrbit);
            if (sizeof(gap_word_t) == 2)
            {
                enc.put_16(len);
            }
            else
            {
                enc.put_32(len);
            }
            enc.put_16(gap_temp_block, len);
        }
        else
        {
            unsigned head_idx = 0;
            unsigned tail_idx = 0;
            bit_find_head_tail(blk, &head_idx, &tail_idx);

            if (head_idx == (unsigned)-1) // zero block
            {
                goto zero_block;
            }
            else  // full block
            if (head_idx == 0 && tail_idx == bm::set_block_size-1)
            {
                enc.put_8(set_block_bit);
                enc.put_32(blk, bm::set_block_size);
            } 
            else  // interval block
            {
                enc.put_8(set_block_bit_interval);
                enc.put_16((short)head_idx);
                enc.put_16((short)tail_idx);
                enc.put_32(blk + head_idx, tail_idx - head_idx + 1);
            }
        }

    }

    enc.put_8(set_block_end);


    unsigned encoded_size = enc.size();
    
    // check if bit-vector encoding is inefficient 
    // (can happen for very sparse vectors)

    bm::id_t cnt = bv.count();
    unsigned id_capacity = cnt * sizeof(bm::id_t);
    id_capacity += 16;
    if (id_capacity < encoded_size) 
    {
        // plain list of ints is better than serialization
        bm::encoder enc(buf, 0);
        header_flag = BM_HM_ID_LIST;
        if (bv.size() != bm::id_max) // no dynamic resize
            header_flag |= BM_HM_RESIZE;
        if (serialization_flags & BM_NO_BYTE_ORDER) 
            header_flag |= BM_HM_NO_BO;

        enc.put_8(header_flag);
        if (!(serialization_flags & BM_NO_BYTE_ORDER))
        {
            ByteOrder bo = globals<true>::byte_order();
            enc.put_8((unsigned char)bo);
        }

        if (bv.size() != bm::id_max) // no dynamic resize
        {
            enc.put_32(bv.size());
        }

        enc.put_32(cnt);
        typename BV::enumerator en(bv.first());
        for (;en.valid(); ++en,--cnt) 
        {
            bm::id_t id = *en;
            enc.put_32(id);
        }
        return enc.size();
    } // if capacity

    return encoded_size;

}

/*!
   @brief Saves bitvector into memory.
   Allocates temporary memory block for bvector.
   \ingroup bvserial 
*/

template<class BV>
unsigned serialize(BV& bv, 
                   unsigned char* buf, 
                   unsigned  serialization_flags=0)
{
    typename BV::blocks_manager_type& bman = bv.get_blocks_manager();

    return serialize(bv, buf, bman.check_allocate_tempblock());
}

/*!
    @brief Serialization stream iterator

    Iterates blocks and control tokens of serialized bit-stream
    \ingroup bvserial 
*/
template<class DEC>
class serial_stream_iterator
{
public:
    typedef DEC decoder_type;
public:
    serial_stream_iterator(const unsigned char* buf);

    /// serialized bitvector size
    unsigned bv_size() const { return bv_size_; }

    /// Returns true if end of bit-stream reached 
    bool is_eof() const { return end_of_stream_; }

    /// get next block
    void next();

    /// read bit block, using logical operation
    unsigned get_bit_block(bm::word_t*       dst_block, 
                           bm::word_t*       tmp_block,
                           set_operation     op);


    /// Read gap block data (with head)
    void get_gap_block(bm::gap_word_t* dst_block);

    /// Return current decoder size
    unsigned dec_size() const { return decoder_.size(); }

    /// Get low level access to the decoder (use carefully)
    decoder_type& decoder() { return decoder_; }

    /// iterator is a state machine, this enum encodes 
    /// its key value
    ///
    enum iterator_state 
    {
        e_unknown = 0,
        e_list_ids,
        e_blocks,       ///< stream of blocks
        e_zero_blocks,  ///< one or more zero bit blocks
        e_one_blocks,   ///< one or more all-1 bit blocks
        e_bit_block,    ///< one bit block
        e_gap_block     ///< one gap block

    };

    /// Returns iterator internal state
    iterator_state state() const { return this->state_; }

    iterator_state get_state() const { return this->state_; }
    /// Number of ids in the inverted list (valid for e_list_ids)
    unsigned get_id_count() const { return this->id_cnt_; }

    /// Get last id from the id list
    bm::id_t get_id() const { return this->last_id_; }

    /// Get current block index 
    unsigned block_idx() const { return this->block_idx_; }


public:
    /// member function pointer for bitset-bitset get operations
    /// 
    typedef 
        unsigned (serial_stream_iterator<DEC>::*get_bit_func_type)
                                                (bm::word_t*,bm::word_t*);

    unsigned 
    get_bit_block_ASSIGN(bm::word_t* dst_block, bm::word_t* tmp_block);
    unsigned 
    get_bit_block_OR    (bm::word_t* dst_block, bm::word_t* tmp_block);
    unsigned 
    get_bit_block_AND   (bm::word_t* dst_block, bm::word_t* tmp_block);
    unsigned 
    get_bit_block_SUB   (bm::word_t* dst_block, bm::word_t* tmp_block);
    unsigned 
    get_bit_block_XOR   (bm::word_t* dst_block, bm::word_t* tmp_block);
    unsigned 
    get_bit_block_COUNT (bm::word_t* dst_block, bm::word_t* tmp_block);
    unsigned 
    get_bit_block_COUNT_AND(bm::word_t* dst_block, bm::word_t* tmp_block);
    unsigned 
    get_bit_block_COUNT_OR(bm::word_t* dst_block, bm::word_t* tmp_block);
    unsigned 
    get_bit_block_COUNT_XOR(bm::word_t* dst_block, bm::word_t* tmp_block);
    unsigned 
    get_bit_block_COUNT_SUB_AB(bm::word_t* dst_block, bm::word_t* tmp_block);
    unsigned 
    get_bit_block_COUNT_SUB_BA(bm::word_t* dst_block, bm::word_t* tmp_block);
    unsigned 
    get_bit_block_COUNT_A(bm::word_t* dst_block, bm::word_t* tmp_block);
    unsigned 
    get_bit_block_COUNT_B(bm::word_t* dst_block, bm::word_t* tmp_block)
    {
        return get_bit_block_COUNT(dst_block, tmp_block);
    }

    /// Get array of bits out of the decoder into bit block
    /// (Converts inverted list into bits)
    /// Returns number of words (bits) being read
    unsigned get_arr_bit(bm::word_t* dst_block, bool clear_target=true);

protected:
    get_bit_func_type  bit_func_table_[bm::set_END];

    decoder_type       decoder_;
    bool               end_of_stream_;
    unsigned           bv_size_;
    iterator_state     state_;
    unsigned           id_cnt_;  ///< Id counter for id list
    bm::id_t           last_id_; ///< Last id from the id list
    gap_word_t         glevels_[bm::gap_levels]; ///< GAP levels

    unsigned           block_type_; ///< current block type
    unsigned           block_idx_;  ///< current block index
    unsigned           mono_block_cnt_; ///< number of 0 or 1 blocks

    gap_word_t         gap_head_;
};

/**
    Class deserializer, can perform logical operation on bit-vector and
    serialized bit-vector.

    \ingroup bvserial 
*/
template<class BV>
class operation_deserializer
{
public:
    typedef BV bvector_type;
public:
    static
    unsigned deserialize(bvector_type&        bv, 
                         const unsigned char* buf, 
                         bm::word_t*          temp_block,
                         set_operation        op = bm::set_OR);
private:
    typedef 
        typename BV::blocks_manager_type               blocks_manager_type;
    typedef 
        serial_stream_iterator<bm::decoder>            serial_stream_current;
    typedef 
        serial_stream_iterator<bm::decoder_big_endian> serial_stream_be;
    typedef 
        serial_stream_iterator<bm::decoder_little_endian> serial_stream_le;

};

/**
    Iterator to walk forward the serialized stream.

    \internal
    \ingroup bvserial 
*/
template<class BV, class SerialIterator>
class iterator_deserializer
{
public:
    typedef BV              bvector_type;
    typedef SerialIterator  serial_iterator_type;
public:
    static
    unsigned deserialize(bvector_type&         bv, 
                         serial_iterator_type& sit, 
                         bm::word_t*           temp_block,
                         set_operation         op = bm::set_OR);
private:
    typedef typename BV::blocks_manager_type blocks_manager_type;

    /// load data from the iterator of type "id list"
    static
    void load_id_list(bvector_type&         bv, 
                      serial_iterator_type& sit,
                      unsigned              id_count,
                      bool                  set_clear);
};

/**
    Class deserializer

    \ingroup bvserial 
*/
template<class BV, class DEC>
class deserializer
{
public:
    typedef BV bvector_type;
    typedef DEC decoder_type;
public:
    static
    unsigned deserialize(bvector_type&        bv, 
                         const unsigned char* buf, 
                         bm::word_t*          temp_block);
protected:
   typedef typename BV::blocks_manager_type blocks_manager_type;
   typedef typename BV::allocator_type allocator_type;

};


/*!
    @brief Bitvector deserialization from memory.

    @param buf - pointer on memory which keeps serialized bvector
    @param temp_block - pointer on temporary block, 
            if NULL bvector allocates own.
    @return Number of bytes consumed by deserializer.

    Function desrializes bitvector from memory block containig results
    of previous serialization. Function does not remove bits 
    which are currently set. Effectively it means OR logical operation 
    between current bitset and previously serialized one.

    @ingroup bvserial
*/
template<class BV>
unsigned deserialize(BV& bv, 
                     const unsigned char* buf, 
                     bm::word_t* temp_block=0)
{
    ByteOrder bo_current = globals<true>::byte_order();

    bm::decoder dec(buf);
    unsigned char header_flag = dec.get_8();
    ByteOrder bo = bo_current;
    if (!(header_flag & BM_HM_NO_BO))
    {
        bo = (bm::ByteOrder) dec.get_8();
    }

    if (bo_current == bo)
    {
        return 
            deserializer<BV, bm::decoder>::deserialize(bv, buf, temp_block);
    }
    switch (bo_current) 
    {
    case BigEndian:
        return 
        deserializer<BV, bm::decoder_big_endian>::deserialize(bv, 
                                                              buf, 
                                                              temp_block);
    case LittleEndian:
        return 
        deserializer<BV, bm::decoder_little_endian>::deserialize(bv, 
                                                                 buf, 
                                                                 temp_block);
    default:
        BM_ASSERT(0);
    };
    return 0;
}



template<class BV, class DEC>
unsigned deserializer<BV, DEC>::deserialize(bvector_type&        bv, 
                                            const unsigned char* buf,
                                            bm::word_t*          temp_block)
{
    blocks_manager_type& bman = bv.get_blocks_manager();

    bm::wordop_t* tmp_buf = 
        temp_block ? (bm::wordop_t*) temp_block 
                   : (bm::wordop_t*)bman.check_allocate_tempblock();

    temp_block = (word_t*)tmp_buf;

    gap_word_t   gap_temp_block[set_block_size*2+10];

    decoder_type dec(buf);

    bv.forget_count();

    BM_SET_MMX_GUARD

    // Reading header

    unsigned char header_flag =  dec.get_8();
    if (!(header_flag & BM_HM_NO_BO))
    {
        /*ByteOrder bo = (bm::ByteOrder)*/dec.get_8();
    }

    if (header_flag & BM_HM_ID_LIST)
    {
        // special case: the next comes plain list of integers
        if (header_flag & BM_HM_RESIZE)
        {
            unsigned bv_size = dec.get_32();
            if (bv_size > bv.size())
            {
                bv.resize(bv_size);
            }
        }

        for (unsigned cnt = dec.get_32(); cnt; --cnt) {
            bm::id_t id = dec.get_32();
            bv.set(id);
        } // for
        return dec.size();
    }

    unsigned i;

    if (!(header_flag & BM_HM_NO_GAPL)) 
    {
        gap_word_t glevels[bm::gap_levels];
        // read GAP levels information
        for (i = 0; i < bm::gap_levels; ++i)
        {
            glevels[i] = dec.get_16();
        }
    }

    if (header_flag & (1 << 1))
    {
        unsigned bv_size = dec.get_32();
        if (bv_size > bv.size())
        {
            bv.resize(bv_size);
        }
    }


    // Reading blocks

    unsigned char btype;
    unsigned nb;

    for (i = 0; i < bm::set_total_blocks; ++i)
    {
        // get the block type

        btype = dec.get_8();


        // In a few next blocks of code we simply ignoring all coming zero blocks.

        if (btype == set_block_azero || btype == set_block_end)
        {
            break;
        }

        if (btype == set_block_1zero)
        {
            continue;
        }

        if (btype == set_block_8zero)
        {
            nb = dec.get_8();
            i += nb-1;
            continue;
        }

        if (btype == set_block_16zero)
        {
            nb = dec.get_16();
            i += nb-1;
            continue;
        }

        if (btype == set_block_32zero)
        {
            nb = dec.get_32();
            i += nb-1;
            continue;
        }

        // Now it is turn of all-set blocks (111)

        if (btype == set_block_aone)
        {
            for (unsigned j = i; j < bm::set_total_blocks; ++j)
            {
                bman.set_block_all_set(j);
            }
            break;
        }

        if (btype == set_block_1one)
        {
            bman.set_block_all_set(i);
            continue;
        }

        if (btype == set_block_8one)
        {
            BM_SET_ONE_BLOCKS(dec.get_8());
            continue;
        }

        if (btype == set_block_16one)
        {
            BM_SET_ONE_BLOCKS(dec.get_16());
            continue;
        }

        if (btype == set_block_32one)
        {
            BM_SET_ONE_BLOCKS(dec.get_32());
            continue;
        }

        bm::word_t* blk = bman.get_block(i);


        if (btype == set_block_bit) 
        {
            if (blk == 0)
            {
                blk = bman.get_allocator().alloc_bit_block();
                bman.set_block(i, blk);
                dec.get_32(blk, bm::set_block_size);
                continue;                
            }
            dec.get_32(temp_block, bm::set_block_size);
            bv.combine_operation_with_block(i, 
                                            temp_block, 
                                            0, BM_OR);
            continue;
        }

        if (btype == set_block_bit_interval) 
        {

            unsigned head_idx, tail_idx;
            head_idx = dec.get_16();
            tail_idx = dec.get_16();

            if (blk == 0)
            {
                blk = bman.get_allocator().alloc_bit_block();
                bman.set_block(i, blk);
                for (unsigned i = 0; i < head_idx; ++i)
                {
                    blk[i] = 0;
                }
                dec.get_32(blk + head_idx, tail_idx - head_idx + 1);
                for (unsigned j = tail_idx + 1; j < bm::set_block_size; ++j)
                {
                    blk[j] = 0;
                }
                continue;
            }
            bit_block_set(temp_block, 0);
            dec.get_32(temp_block + head_idx, tail_idx - head_idx + 1);

            bv.combine_operation_with_block(i, 
                                            temp_block,
                                            0, BM_OR);
            continue;
        }

        if (btype == set_block_gap || btype == set_block_gapbit)
        {
            gap_word_t gap_head = 
                sizeof(gap_word_t) == 2 ? dec.get_16() : dec.get_32();

            unsigned len = gap_length(&gap_head);
            int level = gap_calc_level(len, bman.glen());
            --len;
            if (level == -1)  // Too big to be GAP: convert to BIT block
            {
                *gap_temp_block = gap_head;
                dec.get_16(gap_temp_block+1, len - 1);
                //dec.memcpy(gap_temp_block+1, (len-1) * sizeof(gap_word_t));
                gap_temp_block[len] = gap_max_bits - 1;

                if (blk == 0)  // block does not exist yet
                {
                    blk = bman.get_allocator().alloc_bit_block();
                    bman.set_block(i, blk);
                    gap_convert_to_bitset(blk, gap_temp_block);                
                }
                else  // We have some data already here. Apply OR operation.
                {
                    gap_convert_to_bitset(temp_block, 
                                          gap_temp_block);

                    bv.combine_operation_with_block(i, 
                                                    temp_block, 
                                                    0, 
                                                    BM_OR);
                }

                continue;
            } // level == -1

            set_gap_level(&gap_head, level);

            if (blk == 0)
            {
                gap_word_t* gap_blk = 
                  bman.get_allocator().alloc_gap_block(level, bman.glen());
                gap_word_t* gap_blk_ptr = BMGAP_PTR(gap_blk);
                *gap_blk_ptr = gap_head;
                set_gap_level(gap_blk_ptr, level);
                bman.set_block(i, (bm::word_t*)gap_blk);
                bman.set_block_gap(i);
                for (unsigned k = 1; k < len; ++k) 
                {
                     gap_blk[k] = dec.get_16();
                }
                gap_blk[len] = bm::gap_max_bits - 1;
            }
            else
            {
                *gap_temp_block = gap_head;
                for (unsigned k = 1; k < len; ++k) 
                {
                     gap_temp_block[k] = dec.get_16();
                }
                gap_temp_block[len] = bm::gap_max_bits - 1;

                bv.combine_operation_with_block(i, 
                                               (bm::word_t*)gap_temp_block, 
                                                1, 
                                                BM_OR);
            }

            continue;
        }

        if (btype == set_block_arrgap) 
        {
            gap_word_t len = dec.get_16();
            int block_type;
            unsigned k = 0;

        get_block_ptr:
            bm::word_t* blk =
                bman.check_allocate_block(i,
                                          true,
                                          bv.get_new_blocks_strat(),
                                          &block_type,
                                          false /* no null return*/);

            // block is all 1, no need to do anything
            if (!blk)
            {
                for (; k < len; ++k)
                {
                    dec.get_16();
                }
            }

            if (block_type == 1) // gap
            {            
                bm::gap_word_t* gap_blk = BMGAP_PTR(blk);
                unsigned threshold = bm::gap_limit(gap_blk, bman.glen());
                
                for (; k < len; ++k)
                {
                    gap_word_t bit_idx = dec.get_16();
                    unsigned is_set;
                    unsigned new_block_len =
                        gap_set_value(true, gap_blk, bit_idx, &is_set);
                    if (new_block_len > threshold) 
                    {
                        bman.extend_gap_block(i, gap_blk);
                        ++k;
                        goto get_block_ptr;  // block pointer changed - reset
                    }

                } // for
            }
            else // bit block
            {
                // Get the array one by one and set the bits.
                for(;k < len; ++k)
                {
                    gap_word_t bit_idx = dec.get_16();
                    or_bit_block(blk, bit_idx, 1);
                }
            }
            continue;
        }

        if (btype == set_block_arrbit)
        {
            gap_word_t len = 
                sizeof(gap_word_t) == 2 ? dec.get_16() : dec.get_32();

            // check the block type.
            if (bman.is_block_gap(i))
            {
                // Here we most probably does not want to keep
                // the block GAP since generic bitblock offers better
                // performance.
                blk = bman.convert_gap2bitset(i);
            }
            else
            {
                if (blk == 0)  // block does not exists yet
                {
                    blk = bman.get_allocator().alloc_bit_block();
                    bit_block_set(blk, 0);
                    bman.set_block(i, blk);
                }
            }

            // Get the array one by one and set the bits.
            for (unsigned k = 0; k < len; ++k)
            {
                gap_word_t bit_idx = dec.get_16();
                or_bit_block(blk, bit_idx, 1);
            }

            continue;
        }
/*
        if (btype == set_block_gapbit)
        {
            gap_word_t gap_head = 
                sizeof(gap_word_t) == 2 ? dec.get_16() : dec.get_32();

            unsigned len = gap_length(&gap_head)-1;

            *gap_temp_block = gap_head;
            dec.memcpy(gap_temp_block+1, (len-1) * sizeof(gap_word_t));
            gap_temp_block[len] = GAP_MAX_BITS - 1;

            if (blk == 0)  // block does not exists yet
            {
                blk = A::alloc_bit_block();
                blockman_.set_block(i, blk);
                gap_convert_to_bitset(blk, 
                                      gap_temp_block, 
                                      bm::SET_BLOCK_SIZE);                
            }
            else  // We have some data already here. Apply OR operation.
            {
                gap_convert_to_bitset(temp_block, 
                                      gap_temp_block, 
                                      bm::SET_BLOCK_SIZE);

                combine_operation_with_block(i, 
                                             temp_block, 
                                             0, 
                                             BM_OR);
            }

            continue;
        }
*/
        BM_ASSERT(0); // unknown block type


    } // for i

    return dec.size();

}



template<class DEC>
serial_stream_iterator<DEC>::serial_stream_iterator(const unsigned char* buf)
  : decoder_(buf),
    end_of_stream_(false),
    bv_size_(0),
    state_(e_unknown),
    id_cnt_(0),
    block_idx_(0),
    mono_block_cnt_(0)
{
    ::memset(bit_func_table_, 0, sizeof(bit_func_table_));

    bit_func_table_[bm::set_AND] = 
        &serial_stream_iterator<DEC>::get_bit_block_AND;
    bit_func_table_[bm::set_ASSIGN] = 
        &serial_stream_iterator<DEC>::get_bit_block_ASSIGN;
    bit_func_table_[bm::set_OR]     = 
        &serial_stream_iterator<DEC>::get_bit_block_OR;
    bit_func_table_[bm::set_SUB] = 
        &serial_stream_iterator<DEC>::get_bit_block_SUB;
    bit_func_table_[bm::set_XOR] = 
        &serial_stream_iterator<DEC>::get_bit_block_XOR;
    bit_func_table_[bm::set_COUNT] = 
        &serial_stream_iterator<DEC>::get_bit_block_COUNT;
    bit_func_table_[bm::set_COUNT_AND] = 
        &serial_stream_iterator<DEC>::get_bit_block_COUNT_AND;
    bit_func_table_[bm::set_COUNT_XOR] = 
        &serial_stream_iterator<DEC>::get_bit_block_COUNT_XOR;
    bit_func_table_[bm::set_COUNT_OR] = 
        &serial_stream_iterator<DEC>::get_bit_block_COUNT_OR;
    bit_func_table_[bm::set_COUNT_SUB_AB] = 
        &serial_stream_iterator<DEC>::get_bit_block_COUNT_SUB_AB;
    bit_func_table_[bm::set_COUNT_SUB_BA] = 
        &serial_stream_iterator<DEC>::get_bit_block_COUNT_SUB_BA;
    bit_func_table_[bm::set_COUNT_A] = 
        &serial_stream_iterator<DEC>::get_bit_block_COUNT_A;
    bit_func_table_[bm::set_COUNT_B] = 
        &serial_stream_iterator<DEC>::get_bit_block_COUNT;


    // reading stream header
    unsigned char header_flag =  decoder_.get_8();
    if (!(header_flag & BM_HM_NO_BO))
    {
        /*ByteOrder bo = (bm::ByteOrder)*/decoder_.get_8();
    }

    // check if bitvector comes as an inverted, sorted list of ints
    //
    if (header_flag & BM_HM_ID_LIST)
    {
        // special case: the next comes plain list of unsigned integers
        if (header_flag & BM_HM_RESIZE)
        {
            bv_size_ = decoder_.get_32();
        }

        state_ = e_list_ids;
        id_cnt_ = decoder_.get_32();
        next(); // read first id
    }
    else
    {
        if (!(header_flag & BM_HM_NO_GAPL)) 
        {
            unsigned i;
            // keep GAP levels info
            for (i = 0; i < bm::gap_levels; ++i)
            {
                glevels_[i] = decoder_.get_16();
            }
        }

        if (header_flag & (1 << 1))
        {
            bv_size_ = decoder_.get_32();
        }
        state_ = e_blocks;
    }
}

template<class DEC>
void serial_stream_iterator<DEC>::next()
{
    if (is_eof()) return;

    switch (state_) 
    {
    case e_list_ids:
        // read inverted ids one by one
        if (id_cnt_ == 0)
        {
            end_of_stream_ = true;
            state_ = e_unknown;
            break;
        }
        last_id_ = decoder_.get_32();
        --id_cnt_;
        break;

    case e_blocks:
        if (block_idx_ == bm::set_total_blocks)
        {
            end_of_stream_ = true;
            state_ = e_unknown;
            break;
        }
        block_type_ = decoder_.get_8();
        switch (block_type_)
        {
        case set_block_azero:
        case set_block_end:
            end_of_stream_ = true; state_ = e_unknown;
            break;
        case set_block_1zero:
            state_ = e_zero_blocks;
            mono_block_cnt_ = 0;
            break;
        case set_block_8zero:
            state_ = e_zero_blocks;
            mono_block_cnt_ = decoder_.get_8()-1;
            break;
        case set_block_16zero:
            state_ = e_zero_blocks;
            mono_block_cnt_ = decoder_.get_16()-1;
            break;
        case set_block_32zero:
            state_ = e_zero_blocks;
            mono_block_cnt_ = decoder_.get_32()-1;
            break;
        case set_block_aone:
            state_ = e_one_blocks;
            mono_block_cnt_ = bm::set_total_blocks - block_idx_;
            break;
        case set_block_1one:
            state_ = e_one_blocks;
            mono_block_cnt_ = 0;
            break;
        case set_block_8one:
            state_ = e_one_blocks;
            mono_block_cnt_ = decoder_.get_8()-1;
            break;
        case set_block_16one:
            state_ = e_one_blocks;
            mono_block_cnt_ = decoder_.get_16()-1;
            break;
        case set_block_32one:
            state_ = e_one_blocks;
            mono_block_cnt_ = decoder_.get_32()-1;
            break;

        case set_block_bit:
        case set_block_bit_interval:
        case set_block_arrbit:
            state_ = e_bit_block;
            break;

        case set_block_gap:
            gap_head_ = 
                sizeof(gap_word_t) == 2 ? 
                    decoder_.get_16() : decoder_.get_32();
        case set_block_arrgap:
            state_ = e_gap_block;
            break;
        /*
        case set_block_gapbit:
            break;
        */
        default:
            BM_ASSERT(0);
        }// switch

        break;

    case e_zero_blocks:
    case e_one_blocks:
        ++block_idx_;
        if (!mono_block_cnt_)
        {
            state_ = e_blocks; // get new block token
            break;
        }
        --mono_block_cnt_;
        break;

    case e_unknown:
    default:
        BM_ASSERT(0);
    } // switch
}



template<class DEC>
unsigned 
serial_stream_iterator<DEC>::get_bit_block_ASSIGN(
                                            bm::word_t*  dst_block,
                                            bm::word_t*  /*tmp_block*/)
{
    BM_ASSERT(this->state_ == e_bit_block);
    unsigned count = 0;

    switch (this->block_type_)
    {
    case set_block_bit:
        decoder_.get_32(dst_block, bm::set_block_size);
        break;
    case set_block_bit_interval:
        {
            unsigned head_idx = decoder_.get_16();
            unsigned tail_idx = decoder_.get_16();
            if (dst_block) 
            {
                for (unsigned i = 0; i < head_idx; ++i)
                    dst_block[i] = 0;
                decoder_.get_32(dst_block + head_idx, 
                                tail_idx - head_idx + 1);
                for (unsigned j = tail_idx + 1; j < bm::set_block_size; ++j)
                    dst_block[j] = 0;
            }
            else
            {
                decoder_.seek((tail_idx - head_idx + 1) * 4);
            }
        }
        break;
    case set_block_arrbit:
        get_arr_bit(dst_block, true /*clear target*/);
        break;
    default:
        BM_ASSERT(0);
    } // switch
    return count;
}

template<class DEC>
unsigned 
serial_stream_iterator<DEC>::get_bit_block_OR(bm::word_t*  dst_block,
                                              bm::word_t*  /*tmp_block*/)
{
    BM_ASSERT(this->state_ == e_bit_block);
    unsigned count = 0;

    switch (block_type_)
    {
    case set_block_bit:
        {
        bitblock_get_adapter ga(dst_block);
        bit_OR<bm::word_t> func;
        bitblock_store_adapter sa(dst_block);

        bit_recomb(ga,
                   decoder_,
                   func,
                   sa
                  );
        }
        break;
    case set_block_bit_interval:
        {
        unsigned head_idx = decoder_.get_16();
        unsigned tail_idx = decoder_.get_16();
        for (unsigned i = head_idx; i <= tail_idx; ++i)
            dst_block[i] |= decoder_.get_32();
        }
        break;
    case set_block_arrbit:
        get_arr_bit(dst_block, false /* don't clear target*/);
        break;
    default:
        BM_ASSERT(0);
    } // switch
    return count;
}

template<class DEC>
unsigned 
serial_stream_iterator<DEC>::get_bit_block_AND(bm::word_t*  dst_block,
                                               bm::word_t*  tmp_block)
{
    BM_ASSERT(this->state_ == e_bit_block);
    BM_ASSERT(dst_block != tmp_block);

    unsigned count = 0;
    switch (block_type_)
    {
    case set_block_bit:
        for (unsigned i = 0; i < bm::set_block_size; ++i)
            dst_block[i] &= decoder_.get_32();
        break;
    case set_block_bit_interval:
        {
            unsigned head_idx = decoder_.get_16();
            unsigned tail_idx = decoder_.get_16();
            unsigned i;
            for ( i = 0; i < head_idx; ++i)
                dst_block[i] = 0;
            for ( i = head_idx; i <= tail_idx; ++i)
                dst_block[i] &= decoder_.get_32();
            for ( i = tail_idx + 1; i < bm::set_block_size; ++i)
                dst_block[i] = 0;
        }
        break;
    case set_block_arrbit:
        get_arr_bit(tmp_block, true /*clear target*/);
        if (dst_block)
            bit_block_and(dst_block, tmp_block);
        break;
    default:
        BM_ASSERT(0);
    } // switch
    return count;
}

template<class DEC>
unsigned 
serial_stream_iterator<DEC>::get_bit_block_XOR(bm::word_t*  dst_block,
                                               bm::word_t*  tmp_block)
{
    BM_ASSERT(this->state_ == e_bit_block);
    BM_ASSERT(dst_block != tmp_block);

    unsigned count = 0;
    switch (block_type_)
    {
    case set_block_bit:
        for (unsigned i = 0; i < bm::set_block_size; ++i)
            dst_block[i] ^= decoder_.get_32();
        break;
    case set_block_bit_interval:
        {
            unsigned head_idx = decoder_.get_16();
            unsigned tail_idx = decoder_.get_16();
            for (unsigned i = head_idx; i <= tail_idx; ++i)
                dst_block[i] ^= decoder_.get_32();
        }
        break;
    case set_block_arrbit:
        get_arr_bit(tmp_block, true /*clear target*/);
        if (dst_block)
            bit_block_xor(dst_block, tmp_block);
        break;
    default:
        BM_ASSERT(0);
    } // switch
    return count;
}

template<class DEC>
unsigned 
serial_stream_iterator<DEC>::get_bit_block_SUB(bm::word_t*  dst_block,
                                               bm::word_t*  tmp_block)
{
    BM_ASSERT(this->state_ == e_bit_block);
    BM_ASSERT(dst_block != tmp_block);

    unsigned count = 0;
    switch (block_type_)
    {
    case set_block_bit:
        for (unsigned i = 0; i < bm::set_block_size; ++i)
            dst_block[i] &= ~decoder_.get_32();
        break;
    case set_block_bit_interval:
        {
            unsigned head_idx = decoder_.get_16();
            unsigned tail_idx = decoder_.get_16();
            for (unsigned i = head_idx; i <= tail_idx; ++i)
                dst_block[i] &= ~decoder_.get_32();
        }
        break;
    case set_block_arrbit:
        get_arr_bit(tmp_block, true /*clear target*/);
        if (dst_block)
            bit_block_sub(dst_block, tmp_block);
        break;
    default:
        BM_ASSERT(0);
    } // switch
    return count;
}


template<class DEC>
unsigned 
serial_stream_iterator<DEC>::get_bit_block_COUNT(bm::word_t*  /*dst_block*/,
                                                 bm::word_t*  /*tmp_block*/)
{
    BM_ASSERT(this->state_ == e_bit_block);

    unsigned count = 0;
    switch (block_type_)
    {
    case set_block_bit:
        for (unsigned i = 0; i < bm::set_block_size; ++i)
            count += word_bitcount(decoder_.get_32());
        break;
    case set_block_bit_interval:
        {
            unsigned head_idx = decoder_.get_16();
            unsigned tail_idx = decoder_.get_16();
            for (unsigned i = head_idx; i <= tail_idx; ++i)
                count += word_bitcount(decoder_.get_32());
        }
        break;
    case set_block_arrbit:
        count += get_arr_bit(0);
        break;
    default:
        BM_ASSERT(0);
    } // switch
    return count;
}

template<class DEC>
unsigned 
serial_stream_iterator<DEC>::get_bit_block_COUNT_A(bm::word_t*  dst_block,
                                                   bm::word_t*  /*tmp_block*/)
{
    BM_ASSERT(this->state_ == e_bit_block);
    unsigned count = 0;
    if (dst_block)
    {
        // count the block bitcount
        count = 
            bit_block_calc_count(dst_block, 
                                 dst_block + bm::set_block_size);
    }

    switch (block_type_)
    {
    case set_block_bit:
        decoder_.get_32(0, bm::set_block_size);
        break;
    case set_block_bit_interval:
        {
            unsigned head_idx = decoder_.get_16();
            unsigned tail_idx = decoder_.get_16();
            for (unsigned i = head_idx; i <= tail_idx; ++i)
                decoder_.get_32();
        }
        break;
    case set_block_arrbit:
        get_arr_bit(0);
        break;
    default:
        BM_ASSERT(0);
    } // switch
    return count;
}


template<class DEC>
unsigned 
serial_stream_iterator<DEC>::get_bit_block_COUNT_AND(bm::word_t*  dst_block,
                                                     bm::word_t*  tmp_block)
{
    BM_ASSERT(this->state_ == e_bit_block);
    BM_ASSERT(dst_block);

    unsigned count = 0;
    switch (block_type_)
    {
    case set_block_bit:
        for (unsigned i = 0; i < bm::set_block_size; ++i)
            count += word_bitcount(dst_block[i] & decoder_.get_32());
        break;
    case set_block_bit_interval:
        {
        unsigned head_idx = decoder_.get_16();
        unsigned tail_idx = decoder_.get_16();
        for (unsigned i = head_idx; i <= tail_idx; ++i)
            count += word_bitcount(dst_block[i] & decoder_.get_32());
        }
        break;
    case set_block_arrbit:
        get_arr_bit(tmp_block, true /*clear target*/);
        count += 
            bit_operation_and_count(dst_block, dst_block + bm::set_block_size, 
                                    tmp_block);
        break;
    default:
        BM_ASSERT(0);
    } // switch
    return count;
}

template<class DEC>
unsigned 
serial_stream_iterator<DEC>::get_bit_block_COUNT_OR(bm::word_t*  dst_block,
                                                    bm::word_t*  tmp_block)
{
    BM_ASSERT(this->state_ == e_bit_block);
    BM_ASSERT(dst_block);

    bitblock_sum_adapter count_adapter;

    switch (block_type_)
    {
    case set_block_bit:
        {
        bitblock_get_adapter ga(dst_block);
        bit_COUNT_OR<bm::word_t> func;
        
        bit_recomb(ga,
                   decoder_,
                   func,
                   count_adapter
                  );
        }
        break;
    case set_block_bit_interval:
        {
        unsigned head_idx = decoder_.get_16();
        unsigned tail_idx = decoder_.get_16();
        unsigned count = 0;
        unsigned i;
        for (i = 0; i < head_idx; ++i)
            count += word_bitcount(dst_block[i]);
        for (i = head_idx; i <= tail_idx; ++i)
            count += word_bitcount(dst_block[i] | decoder_.get_32());
        for (i = tail_idx + 1; i < bm::set_block_size; ++i)
            count += word_bitcount(dst_block[i]);
        return count;
        }
    case set_block_arrbit:
        get_arr_bit(tmp_block, true /* clear target*/);
        return 
            bit_operation_or_count(dst_block, 
                                   dst_block + bm::set_block_size,
                                   tmp_block);
    default:
        BM_ASSERT(0);
    } // switch
    return count_adapter.sum();
}

template<class DEC>
unsigned 
serial_stream_iterator<DEC>::get_bit_block_COUNT_XOR(bm::word_t*  dst_block,
                                                     bm::word_t*  tmp_block)
{
    BM_ASSERT(this->state_ == e_bit_block);
    BM_ASSERT(dst_block);

    bitblock_sum_adapter count_adapter;
    switch (block_type_)
    {
    case set_block_bit:
        {
        bitblock_get_adapter ga(dst_block);
        bit_COUNT_XOR<bm::word_t> func;
        
        bit_recomb(ga,
                   decoder_,
                   func,
                   count_adapter
                  );
        }
        break;
    case set_block_bit_interval:
        {
        unsigned head_idx = decoder_.get_16();
        unsigned tail_idx = decoder_.get_16();
        unsigned count = 0;
        unsigned i;
        for (i = 0; i < head_idx; ++i)
            count += word_bitcount(dst_block[i]);
        for (i = head_idx; i <= tail_idx; ++i)
            count += word_bitcount(dst_block[i] ^ decoder_.get_32());
        for (i = tail_idx + 1; i < bm::set_block_size; ++i)
            count += word_bitcount(dst_block[i]);
        return count;
        }
    case set_block_arrbit:
        get_arr_bit(tmp_block, true /* clear target*/);
        return 
            bit_operation_xor_count(dst_block, 
                                    dst_block + bm::set_block_size,
                                    tmp_block);
    default:
        BM_ASSERT(0);
    } // switch
    return count_adapter.sum();
}

template<class DEC>
unsigned 
serial_stream_iterator<DEC>::get_bit_block_COUNT_SUB_AB(bm::word_t*  dst_block,
                                                        bm::word_t*  tmp_block)
{
    BM_ASSERT(this->state_ == e_bit_block);
    BM_ASSERT(dst_block);

    bitblock_sum_adapter count_adapter;
    switch (block_type_)
    {
    case set_block_bit:
        {
        bitblock_get_adapter ga(dst_block);
        bit_COUNT_SUB_AB<bm::word_t> func;
        
        bit_recomb(ga, 
                   decoder_,
                   func,
                   count_adapter
                  );
        }
        break;
    case set_block_bit_interval:
        {
        unsigned head_idx = decoder_.get_16();
        unsigned tail_idx = decoder_.get_16();
        unsigned count = 0;
        unsigned i;
        for (i = 0; i < head_idx; ++i)
            count += word_bitcount(dst_block[i]);
        for (i = head_idx; i <= tail_idx; ++i)
            count += word_bitcount(dst_block[i] & (~decoder_.get_32()));
        for (i = tail_idx + 1; i < bm::set_block_size; ++i)
            count += word_bitcount(dst_block[i]);
        return count;
        }
        break;
    case set_block_arrbit:
        get_arr_bit(tmp_block, true /* clear target*/);
        return 
            bit_operation_sub_count(dst_block, 
                                    dst_block + bm::set_block_size,
                                    tmp_block);
    default:
        BM_ASSERT(0);
    } // switch
    return count_adapter.sum();
}

template<class DEC>
unsigned 
serial_stream_iterator<DEC>::get_bit_block_COUNT_SUB_BA(bm::word_t*  dst_block,
                                                        bm::word_t*  tmp_block)
{
    BM_ASSERT(this->state_ == e_bit_block);
    BM_ASSERT(dst_block);

    bitblock_sum_adapter count_adapter;
    switch (block_type_)
    {
    case set_block_bit:
        {
        bitblock_get_adapter ga(dst_block);
        bit_COUNT_SUB_BA<bm::word_t> func;

        bit_recomb(ga,
                   decoder_,
                   func,
                   count_adapter
                  );
        }
        break;
    case set_block_bit_interval:
        {
        unsigned head_idx = decoder_.get_16();
        unsigned tail_idx = decoder_.get_16();
        unsigned count = 0;
        unsigned i;
        for (i = head_idx; i <= tail_idx; ++i)
            count += word_bitcount(decoder_.get_32() & (~dst_block[i]));
        return count;
        }
        break;
    case set_block_arrbit:
        get_arr_bit(tmp_block, true /* clear target*/);
        return 
            bit_operation_sub_count(tmp_block,
                                    tmp_block + bm::set_block_size,
                                    dst_block);
    default:
        BM_ASSERT(0);
    } // switch
    return count_adapter.sum();
}



template<class DEC>
unsigned serial_stream_iterator<DEC>::get_arr_bit(bm::word_t* dst_block, 
                                                  bool        clear_target)
{
    BM_ASSERT(this->block_type_ == set_block_arrbit);

    gap_word_t len = 
        sizeof(gap_word_t) == 2 ? decoder_.get_16() : decoder_.get_32();

    if (dst_block)
    {
        if (clear_target)
            bit_block_set(dst_block, 0);
        for (unsigned k = 0; k < len; ++k)
        {
            gap_word_t bit_idx = decoder_.get_16();
            or_bit_block(dst_block, bit_idx, 1);
        }
    }
    else
    {
        decoder_.seek(len * 2);
    }
    return len;
}

template<class DEC>
void 
serial_stream_iterator<DEC>::get_gap_block(bm::gap_word_t* dst_block)
{
    BM_ASSERT(this->state_ == e_gap_block);
    BM_ASSERT(dst_block);

    if (this->block_type_ == set_block_gap)
    {
        unsigned len = gap_length(&this->gap_head_);
        --len;
        *dst_block = this->gap_head_;
        decoder_.get_16(dst_block+1, len - 1);
        dst_block[len] = gap_max_bits - 1;
    }
    else
    if (this->block_type_ == set_block_arrgap)
    {
        gap_set_all(dst_block, bm::gap_max_bits, 0);
        gap_word_t len = decoder_.get_16();

        for (gap_word_t k = 0; k < len; ++k)
        {
            gap_word_t bit_idx = this->decoder_.get_16();
            unsigned is_set;
            /* unsigned new_block_len = */
                gap_set_value(true, dst_block, bit_idx, &is_set);
        } // for
    }
    else
    {
        BM_ASSERT(0);
    }
    ++(this->block_idx_);
    this->state_ = e_blocks;
}


template<class DEC>
unsigned 
serial_stream_iterator<DEC>::get_bit_block(bm::word_t*    dst_block,
                                           bm::word_t*    tmp_block,
                                           set_operation  op)
{
    BM_ASSERT(this->state_ == e_bit_block);
    get_bit_func_type bit_func = bit_func_table_[op];
    BM_ASSERT(bit_func);
    unsigned cnt = ((*this).*(bit_func))(dst_block, tmp_block);
    this->state_ = e_blocks;
    ++(this->block_idx_);
    return cnt;
}



template<class BV>
unsigned operation_deserializer<BV>::deserialize(
                                        bvector_type&        bv, 
                                        const unsigned char* buf, 
                                        bm::word_t*          temp_block,
                                        set_operation        op)
{
    ByteOrder bo_current = globals<true>::byte_order();

    bm::decoder dec(buf);
    unsigned char header_flag = dec.get_8();
    ByteOrder bo = bo_current;
    if (!(header_flag & BM_HM_NO_BO))
    {
        bo = (bm::ByteOrder) dec.get_8();
    }

    blocks_manager_type& bman = bv.get_blocks_manager();
    bit_block_guard<blocks_manager_type> bg(bman);
    if (temp_block == 0)
    {
        temp_block = bg.allocate();
    }

    if (bo_current == bo)
    {
        serial_stream_current ss(buf);
        return 
            iterator_deserializer<BV, serial_stream_current>::
                deserialize(bv, ss, temp_block, op);
    }
    switch (bo_current) 
    {
    case BigEndian:
        {
        serial_stream_be ss(buf);
        return 
            iterator_deserializer<BV, serial_stream_be>::
                deserialize(bv, ss, temp_block, op);
        }
    case LittleEndian:
        {
        serial_stream_le ss(buf);
        return 
            iterator_deserializer<BV, serial_stream_le>::
                deserialize(bv, ss, temp_block, op);
        }
    default:
        BM_ASSERT(0);
    };
    return 0;
}


template<class BV, class SerialIterator>
void iterator_deserializer<BV, SerialIterator>::load_id_list(
                                            bvector_type&         bv, 
                                            serial_iterator_type& sit,
                                            unsigned              id_count,
                                            bool                  set_clear)
{
    const unsigned win_size = 64;
    bm::id_t id_buffer[win_size+1];

    if (set_clear)  // set bits
    {
        for (unsigned i = 0; i < id_count;)
        {
            unsigned j;
            for (j = 0; j < win_size && i <= id_count; ++j, ++i) 
            {
                id_buffer[j] = sit.get_id();
                sit.next();
            } // for j
            bm::combine_or(bv, id_buffer, id_buffer + j);
        } // for i
    } 
    else // clear bits
    {
        for (unsigned i = 0; i < id_count;)
        {
            unsigned j;
            for (j = 0; j < win_size && i <= id_count; ++j, ++i) 
            {
                id_buffer[j] = sit.get_id();
                sit.next();
            } // for j
            bm::combine_sub(bv, id_buffer, id_buffer + j);
        } // for i
    }
}



template<class BV, class SerialIterator>
unsigned 
iterator_deserializer<BV, SerialIterator>::deserialize(
                                       bvector_type&         bv, 
                                       serial_iterator_type& sit, 
                                       bm::word_t*           temp_block,
                                       set_operation         op)
{
    BM_ASSERT(temp_block);

    unsigned count = 0;
    gap_word_t   gap_temp_block[set_block_size*2+10] = {0,};

    blocks_manager_type& bman = bv.get_blocks_manager();

    bv.forget_count();

    if (sit.bv_size() && (sit.bv_size() > bv.size())) 
    {
        bv.resize(sit.bv_size());
    }

    BM_SET_MMX_GUARD

    typename serial_iterator_type::iterator_state state;
    state = sit.get_state();
    if (state == serial_iterator_type::e_list_ids)
    {
        unsigned id_count = sit.get_id_count();
        bool set_clear = true;
        switch (op)
        {
        case set_AND:
            {
            // TODO: use some more optimal algorithm without temp vector
            BV bv_tmp(BM_GAP);
            load_id_list(bv_tmp, sit, id_count, true);
            bv &= bv_tmp;
            }
            break;
        case set_ASSIGN:
            bv.clear(true);
            // intentional case fall through here (not a bug)
        case set_OR:
            set_clear = true;
            // fall to SUB
        case set_SUB:
            load_id_list(bv, sit, id_count, set_clear);
            break;
        case set_XOR:
            for (unsigned i = 0; i < id_count; ++i)
            {
                bm::id_t id = sit.get_id();
                bv[id] ^= true;
                sit.next();
            } // for
            break;
        case set_COUNT: case set_COUNT_B:
            for (unsigned i = 0; i < id_count; ++i)
            {
                /* bm::id_t id = */ sit.get_id();
                ++count;
                sit.next();
            } // for
            break;
        case set_COUNT_A:
            return bv.count();
        case set_COUNT_AND:
            for (unsigned i = 0; i < id_count; ++i)
            {
                bm::id_t id = sit.get_id();
                count += bv.get_bit(id);
                sit.next();
            } // for
            break;
        case set_COUNT_XOR:
            {
            // TODO: get rid of the temp vector
            BV bv_tmp(BM_GAP);
            load_id_list(bv_tmp, sit, id_count, true);
            count += count_xor(bv, bv_tmp);
            }
            break;
        case set_COUNT_OR:
            {
            // TODO: get rid of the temp. vector
            BV bv_tmp(BM_GAP);
            load_id_list(bv_tmp, sit, id_count, true);
            count += count_or(bv, bv_tmp);
            }
            break;
        case set_COUNT_SUB_AB:
            {
            // TODO: get rid of the temp. vector
            BV bv_tmp(bv);
            load_id_list(bv_tmp, sit, id_count, false);
            count += bv_tmp.count();
            }
            break;
        default:
            BM_ASSERT(0);
        } // switch
        return count;
    } // state

    unsigned bv_block_idx = 0;

    for (;1;)
    {
        bm::set_operation sop = op;
        if (sit.is_eof()) // argument stream ended
        {
            // deserialization finalization
            switch (op)
            {
            case set_OR:    case set_SUB:     case set_XOR:
            case set_COUNT: case set_COUNT_B: case set_COUNT_AND:
            case set_COUNT_SUB_BA:
                // nothing to do
                break;
            case set_AND: case set_ASSIGN:
                // clear the rest of the target vector
                {
                unsigned i, j;
                bman.get_block_coord(bv_block_idx, &i, &j);
                bm::word_t*** blk_root = bman.get_rootblock();
                unsigned effective_top_size = 
                    bman.effective_top_block_size();
                for (;i < effective_top_size; ++i) 
                {
                    bm::word_t** blk_blk = blk_root[i];
                    if (blk_blk == 0) 
                    {
                        bv_block_idx+=bm::set_array_size-j;
                        j = 0;
                        continue;
                    }
                    for (;j < bm::set_array_size; ++j, ++bv_block_idx)
                    {
                        if (blk_blk[j])
                            bman.zero_block(bv_block_idx);
                    } // for j
                    j = 0;
                } // for i

                }
                break;
            case set_COUNT_A: case set_COUNT_OR: case set_COUNT_XOR:
            case set_COUNT_SUB_AB:
                // count bits in the target vector
                {
                unsigned i, j;
                bman.get_block_coord(bv_block_idx, &i, &j);
                bm::word_t*** blk_root = bman.get_rootblock();
                unsigned effective_top_size = 
                    bman.effective_top_block_size();
                for (;i < effective_top_size; ++i) 
                {
                    bm::word_t** blk_blk = blk_root[i];
                    if (blk_blk == 0) 
                    {
                        bv_block_idx+=bm::set_array_size-j;
                        j = 0;
                        continue;
                    }
                    for (;j < bm::set_array_size; ++j, ++bv_block_idx)
                    {
                        if (blk_blk[j])
                            count += bman.block_bitcount(blk_blk[j], bv_block_idx);
                    } // for j
                    j = 0;
                } // for i

                }
                break;
            default:
                BM_ASSERT(0);
            }
            return count;
        }

        state = sit.state();
        switch (state)
        {
        case serial_iterator_type::e_blocks:
            sit.next();
            continue;
        case serial_iterator_type::e_bit_block:
            {
            BM_ASSERT(sit.block_idx() == bv_block_idx);
            bm::word_t* blk = bman.get_block(bv_block_idx);

            if (!blk) 
            {
                switch (op)
                {
                case set_AND:          case set_SUB: case set_COUNT_AND:
                case set_COUNT_SUB_AB: case set_COUNT_A:
                    // one arg is 0, so the operation gives us zero
                    // all we need to do is to seek the input stream
                    sop = set_ASSIGN;
                    break;

                case set_OR: case set_XOR: case set_ASSIGN:
                    blk = bman.make_bit_block(bv_block_idx);
                    break;

                case set_COUNT:        case set_COUNT_XOR: case set_COUNT_OR:
                case set_COUNT_SUB_BA: case set_COUNT_B:
                    // first arg is not required (should work as is)
                    sop = set_COUNT;
                    break;

                case set_END:
                default:
                    BM_ASSERT(0);
                }
            }
            else // block exists
            {
                int is_gap = BM_IS_GAP(bman, blk, bv_block_idx);
                if (is_gap || IS_FULL_BLOCK(blk))
                {
                    if (IS_FULL_BLOCK(blk) && is_const_set_operation(op))
                    {
                    }
                    else
                    {
                        // TODO: make sure const operations do not 
                        // deoptimize GAP blocks
                        blk = bman.deoptimize_block(bv_block_idx);
                    }
                }
            }

            // 2 bit blocks recombination
            count += sit.get_bit_block(blk, temp_block, sop);
            }
            break;

        case serial_iterator_type::e_zero_blocks:
            {
            BM_ASSERT(bv_block_idx == sit.block_idx());
            bm::word_t* blk = bman.get_block(bv_block_idx);
            sit.next();

            if (blk)
            {
                switch (op)
                {
                case set_AND: case set_ASSIGN:
                    // the result is 0
                    blk = bman.zero_block(bv_block_idx);
                    break;

                case set_SUB: case set_COUNT_AND:    case set_OR:
                case set_XOR: case set_COUNT_SUB_BA: case set_COUNT_B:
                    // nothing to do
                    break;
                
                case set_COUNT_SUB_AB: case set_COUNT_A: case set_COUNT_OR:
                case set_COUNT:        case set_COUNT_XOR:
                    // valid bit block recombined with 0 block
                    // results with same block data
                    // all we need is to bitcount bv block
                    count += bman.block_bitcount(blk, bv_block_idx);
                    break;
                case set_END:
                default:
                    BM_ASSERT(0);
                } // switch op
            } // if blk
            }
            break;

        case serial_iterator_type::e_one_blocks:
            {
            BM_ASSERT(bv_block_idx == sit.block_idx());
            bm::word_t* blk = bman.get_block(bv_block_idx);
            sit.next();

            switch (op)
            {
            case set_OR: case set_ASSIGN:
                bman.set_block_all_set(bv_block_idx);
                break;
            case set_COUNT_OR: case set_COUNT_B: case set_COUNT:
                // block becomes all set
                count += bm::bits_in_block;
                break;
            case set_SUB:
                blk = bman.zero_block(bv_block_idx);
                break;
            case set_COUNT_SUB_AB: case set_AND:
                // nothing to do
                break;
            case set_COUNT_AND: case set_COUNT_A:
                count += bman.block_bitcount(blk, bv_block_idx);
                break;
            default:
                if (blk)
                {
                    switch (op)
                    {
                    case set_XOR:
                        blk = bman.deoptimize_block(bv_block_idx);
                        bit_block_xor(blk, FULL_BLOCK_ADDR);
                        break;
                    case set_COUNT_XOR:
                        {
                        int is_gap = BM_IS_GAP(bman, blk, bv_block_idx);
                        count += 
                            combine_count_operation_with_block(
                                                blk,
                                                is_gap,
                                                FULL_BLOCK_ADDR,
                                                0,
                                                temp_block,
                                                bm::COUNT_XOR);
                        }
                        break;
                    case set_COUNT_SUB_BA:
                        {
                        int is_gap = BM_IS_GAP(bman, blk, bv_block_idx);
                        count += 
                            combine_count_operation_with_block(
                                                blk,
                                                is_gap,
                                                FULL_BLOCK_ADDR,
                                                0,
                                                temp_block,
                                                bm::COUNT_SUB_BA);
                        }
                        break;
                    default:
                        BM_ASSERT(0);
                    } // switch
                }
                else // blk == 0 
                {
                    switch (op)
                    {
                    case set_XOR:
                        // 0 XOR 1 = 1
                        bman.set_block_all_set(bv_block_idx);
                        break;
                    case set_COUNT_XOR:
                        count += bm::bits_in_block;
                        break;
                    case set_COUNT_SUB_BA:
                        // 1 - 0 = 1
                        count += bm::bits_in_block;
                        break;
                    default:
                        break;
                    } // switch
                } // else
            } // switch

            }
            break;

        case serial_iterator_type::e_gap_block:
            {
            BM_ASSERT(bv_block_idx == sit.block_idx());
            bm::word_t* blk = bman.get_block(bv_block_idx);

            sit.get_gap_block(gap_temp_block);
            // gap_word_t   gap_head = gap_temp_block[0];

            unsigned len = gap_length(gap_temp_block);
            int level = gap_calc_level(len, bman.glen());
            --len;

            bool const_op = is_const_set_operation(op);
            if (const_op)
            {
                distance_metric metric = operation2metric(op);
                int is_gap = blk ? BM_IS_GAP(bman, blk, bv_block_idx) : 0;
                count += 
                    combine_count_operation_with_block(
                                        blk,
                                        is_gap,
                                        (bm::word_t*)gap_temp_block,
                                        1, // gap
                                        temp_block,
                                        metric);

            }
            else // non-const set operation
            {
                if ((sop == set_ASSIGN) && blk) // target block override
                {
                    blk = bman.zero_block(bv_block_idx);
                    sop = set_OR;
                }
                if (blk == 0) // target block not found
                {
                    switch (sop)
                    {
                    case set_AND: case set_SUB:
                        break;
                    case set_OR: case set_XOR: case set_ASSIGN:
                        if (level == -1) // too big to be GAP: convert to BIT
                        {
                            blk = bman.get_allocator().alloc_bit_block();
                            bman.set_block(bv_block_idx, blk);
                            gap_convert_to_bitset(blk, gap_temp_block);
                        }
                        else // GAP block fits
                        {
                            gap_word_t* gap_blk = 
                                bman.get_allocator().alloc_gap_block(
                                                        level, bman.glen());
                            gap_word_t* gap_blk_ptr = BMGAP_PTR(gap_blk);
                            ::memcpy(gap_blk_ptr, 
                                     gap_temp_block, 
                                     gap_length(gap_temp_block) 
                                                 * sizeof(gap_word_t));
                            set_gap_level(gap_blk_ptr, level);
                            bman.set_block(bv_block_idx, 
                                          (bm::word_t*)gap_blk, true /*GAP*/);
                        }
                        break;
                    default:
                        BM_ASSERT(0);
                    } // switch
                }
                else  // target block exists
                {
                    bm::operation bop = bm::setop2op(op);
                    if (level == -1) // too big to GAP
                    {
                        gap_convert_to_bitset(temp_block, gap_temp_block);
                        bv.combine_operation_with_block(bv_block_idx, 
                                                        temp_block, 
                                                        0, // BIT
                                                        bop);
                    }
                    else // GAP fits
                    {
                        set_gap_level(gap_temp_block, level);
                        bv.combine_operation_with_block(
                                                bv_block_idx, 
                                                (bm::word_t*)gap_temp_block, 
                                                1,  // GAP
                                                bop);
                    }
                }
            }
            }
            break;

        default:
            BM_ASSERT(0);
        } // switch

        ++bv_block_idx;

    } // for (deserialization)

    return count;
}




} // namespace bm

#include "bmundef.h"

#ifdef _MSC_VER
#pragma warning( default : 4311 4312)
#endif


#endif

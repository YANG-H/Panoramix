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


For more information please visit:   http://bmagic.sourceforge.net

*/

#ifndef ENCODING_H__INCLUDED__
#define ENCODING_H__INCLUDED__

#include <memory.h>

namespace bm
{

// ----------------------------------------------------------------
/*!
   \brief Memory encoding.
   
   Class for encoding data into memory. 
   Properly handles aligment issues with integer data types.
*/
class encoder
{
public:
    encoder(unsigned char* buf, unsigned size);
    void put_8(unsigned char c);
    void put_16(bm::short_t  s);
    void put_16(const bm::short_t* s, unsigned count);
    void put_32(bm::word_t  w);
    void put_32(const bm::word_t* w, unsigned count);
    unsigned size() const;
private:
    unsigned char*  buf_;
    unsigned char*  start_;
    unsigned int    size_;
};

// ----------------------------------------------------------------
/**
    Base class for all decoding functionality
*/
class decoder_base
{
public:
    decoder_base(const unsigned char* buf) { buf_ = start_ = buf; }
    /// Reads character from the decoding buffer. 
    BMFORCEINLINE unsigned char get_8() { return *buf_++; }
    /// Returns size of the current decoding stream.
    BMFORCEINLINE 
    unsigned size() const { return (unsigned)(buf_ - start_); }
    /// change current position
    BMFORCEINLINE
    void seek(int delta) { buf_ += delta; }
protected:
   const unsigned char*   buf_;
   const unsigned char*   start_;
};


// ----------------------------------------------------------------
/**
   Class for decoding data from memory buffer.
   Properly handles aligment issues with integer data types.
*/
class decoder : public decoder_base
{
public:
    decoder(const unsigned char* buf);
    bm::short_t get_16();
    bm::word_t get_32();
    void get_32(bm::word_t* w, unsigned count);
    void get_16(bm::short_t* s, unsigned count);
};

// ----------------------------------------------------------------
/**
   Class for decoding data from memory buffer.
   Properly handles aligment issues with integer data types.
   Converts data to big endian architecture 
   (presumed it was encoded as little endian)
*/
typedef decoder decoder_big_endian;


// ----------------------------------------------------------------
/**
   Class for decoding data from memory buffer.
   Properly handles aligment issues with integer data types.
   Converts data to little endian architecture 
   (presumed it was encoded as big endian)
*/
class decoder_little_endian : public decoder_base
{
public:
    decoder_little_endian(const unsigned char* buf);
    bm::short_t get_16();
    bm::word_t get_32();
    void get_32(bm::word_t* w, unsigned count);
    void get_16(bm::short_t* s, unsigned count);
};



// ----------------------------------------------------------------
// Implementation details. 
// ----------------------------------------------------------------

/*! 
    \fn encoder::encoder(unsigned char* buf, unsigned size) 
    \brief Construction.
    \param buf - memory buffer pointer.
    \param size - size of the buffer
*/
inline encoder::encoder(unsigned char* buf, unsigned size)
: buf_(buf), start_(buf), size_(size)
{
}

/*!
   \fn void encoder::put_8(unsigned char c) 
   \brief Puts one character into the encoding buffer.
   \param c - character to encode
*/
BMFORCEINLINE void encoder::put_8(unsigned char c)
{
    *buf_++ = c;
}

/*!
   \fn encoder::put_16(bm::short_t s)
   \brief Puts short word (16 bits) into the encoding buffer.
   \param s - short word to encode
*/
BMFORCEINLINE void encoder::put_16(bm::short_t s)
{
    *buf_++ = (unsigned char) s;
    s >>= 8;
    *buf_++ = (unsigned char) s;
}

/*!
   \brief Method puts array of short words (16 bits) into the encoding buffer.
*/
inline void encoder::put_16(const bm::short_t* s, unsigned count)
{
    unsigned char* buf = buf_;
    const bm::short_t* s_end = s + count;
    do 
    {
        bm::short_t w16 = *s++;
        unsigned char a = (unsigned char)  w16;
        unsigned char b = (unsigned char) (w16 >> 8);
        
        *buf++ = a;
        *buf++ = b;
                
    } while (s < s_end);
    
    buf_ = (unsigned char*)buf;
}


/*!
   \fn unsigned encoder::size() const
   \brief Returns size of the current encoding stream.
*/
inline unsigned encoder::size() const
{
    return (unsigned)(buf_ - start_);
}

/*!
   \fn void encoder::put_32(bm::word_t w)
   \brief Puts 32 bits word into encoding buffer.
   \param w - word to encode.
*/
BMFORCEINLINE void encoder::put_32(bm::word_t w)
{
    *buf_++ = (unsigned char) w;
    *buf_++ = (unsigned char) (w >> 8);
    *buf_++ = (unsigned char) (w >> 16);
    *buf_++ = (unsigned char) (w >> 24);
}

/*!
    \brief Encodes array of 32-bit words
*/
inline 
void encoder::put_32(const bm::word_t* w, unsigned count)
{
    unsigned char* buf = buf_;
    const bm::word_t* w_end = w + count;
    do 
    {
        bm::word_t w32 = *w++;
        unsigned char a = (unsigned char) w32;
        unsigned char b = (unsigned char) (w32 >> 8);
        unsigned char c = (unsigned char) (w32 >> 16);
        unsigned char d = (unsigned char) (w32 >> 24);

        *buf++ = a;
        *buf++ = b;
        *buf++ = c;
        *buf++ = d;
    } while (w < w_end);
    
    buf_ = (unsigned char*)buf;
}


// ---------------------------------------------------------------------

/*!
   \fn decoder::decoder(const unsigned char* buf) 
   \brief Construction
   \param buf - pointer to the decoding memory. 
*/
inline decoder::decoder(const unsigned char* buf) 
: decoder_base(buf)
{
}

/*!
   \fn bm::short_t decoder::get_16()
   \brief Reads 16bit word from the decoding buffer.
*/
BMFORCEINLINE bm::short_t decoder::get_16() 
{
    bm::short_t a = (bm::short_t)(buf_[0] + ((bm::short_t)buf_[1] << 8));
    buf_ += sizeof(a);
    return a;
}

/*!
   \fn bm::word_t decoder::get_32()
   \brief Reads 32 bit word from the decoding buffer.
*/
BMFORCEINLINE bm::word_t decoder::get_32() 
{
    bm::word_t a = buf_[0]+ ((unsigned)buf_[1] << 8) +
                   ((unsigned)buf_[2] << 16) + ((unsigned)buf_[3] << 24);
    buf_+=sizeof(a);
    return a;
}


/*!
   \fn void decoder::get_32(bm::word_t* w, unsigned count)
   \brief Reads block of 32-bit words from the decoding buffer.
   \param w - pointer on memory block to read into.
   \param count - size of memory block in words.
*/
inline void decoder::get_32(bm::word_t* w, unsigned count)
{
    if (!w) 
    {
        seek(count * 4);
        return;
    }
    const unsigned char* buf = buf_;
    const bm::word_t* w_end = w + count;
    do 
    {
        bm::word_t a = buf[0]+ ((unsigned)buf[1] << 8) +
                   ((unsigned)buf[2] << 16) + ((unsigned)buf[3] << 24);
        *w++ = a;
        buf += sizeof(a);
    } while (w < w_end);
    buf_ = (unsigned char*)buf;
}

/*!
   \fn void decoder::get_16(bm::short_t* s, unsigned count)
   \brief Reads block of 32-bit words from the decoding buffer.
   \param s - pointer on memory block to read into.
   \param count - size of memory block in words.
*/
inline void decoder::get_16(bm::short_t* s, unsigned count)
{
    if (!s) 
    {
        seek(count * 2);
        return;
    }

    const unsigned char* buf = buf_;
    const bm::short_t* s_end = s + count;
    do 
    {
        bm::short_t a = (bm::short_t)(buf[0] + ((bm::short_t)buf[1] << 8));
        *s++ = a;
        buf += sizeof(a);
    } while (s < s_end);
    buf_ = (unsigned char*)buf;
}



// ---------------------------------------------------------------------

inline decoder_little_endian::decoder_little_endian(const unsigned char* buf)
: decoder_base(buf)
{
}

BMFORCEINLINE bm::short_t decoder_little_endian::get_16()
{
    bm::short_t a = ((bm::short_t)buf_[0] << 8) + ((bm::short_t)buf_[1]);
    buf_ += sizeof(a);
    return a;
}

BMFORCEINLINE bm::word_t decoder_little_endian::get_32() 
{
    bm::word_t a = ((unsigned)buf_[0] << 24)+ ((unsigned)buf_[1] << 16) +
                   ((unsigned)buf_[2] << 8) + ((unsigned)buf_[3]);
    buf_+=sizeof(a);
    return a;
}

inline void decoder_little_endian::get_32(bm::word_t* w, unsigned count)
{
    if (!w) 
    {
        seek(count * 4);
        return;
    }

    const unsigned char* buf = buf_;
    const bm::word_t* w_end = w + count;
    do 
    {
        bm::word_t a = ((unsigned)buf[0] << 24)+ ((unsigned)buf[1] << 16) +
                       ((unsigned)buf[2] << 8) + ((unsigned)buf[3]);
        *w++ = a;
        buf += sizeof(a);
    } while (w < w_end);
    buf_ = (unsigned char*)buf;
}

inline void decoder_little_endian::get_16(bm::short_t* s, unsigned count)
{
    if (!s) 
    {
        seek(count * 2);
        return;
    }

    const unsigned char* buf = buf_;
    const bm::short_t* s_end = s + count;
    do 
    {
        bm::short_t a = ((bm::short_t)buf[0] << 8) + ((bm::short_t)buf[1]);
        *s++ = a;
        buf += sizeof(a);
    } while (s < s_end);
    buf_ = (unsigned char*)buf;
}


} // namespace bm

#endif



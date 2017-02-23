// Copyright(c) 2002-2005 Anatoliy Kuznetsov(anatoliy_kuznetsov at yahoo.com)


// BM library internal header
//
// Set all required preprocessor defines



#ifndef BM_ASSERT

# ifndef BM_NOASSERT
#  include <assert.h>
#  define BM_ASSERT assert
# else
#  define BM_ASSERT(x)
# endif

#endif


#define FULL_BLOCK_ADDR all_set<true>::_block._p
#define IS_VALID_ADDR(addr) (addr && (addr != FULL_BLOCK_ADDR))
#define IS_FULL_BLOCK(addr) (addr == FULL_BLOCK_ADDR)
#define IS_EMPTY_BLOCK(addr) (addr == 0)

// Macro definitions to manipulate bits in pointers
// This trick is based on the fact that pointers allocated by malloc are
// aligned and bit 0 is never set. It means we are safe to use it.
// BM library keeps GAP flag in pointer.

// Note: this hack is not universally portable so if it does not work
// in some particular case disable it by defining BM_DISBALE_BIT_IN_PTR

#ifdef BM_DISBALE_BIT_IN_PTR

# define BMGAP_PTR(ptr)    ((bm::gap_word_t*)ptr)
# define BMSET_PTRGAP(ptr) (void(0))
# define BM_IS_GAP(obj, ptr, idx) ( (obj).is_block_gap(idx) ) 

#else

# if ULONG_MAX != 0xffffffff || defined(_WIN64)  // 64-bit

#  define BMPTR_SETBIT0(ptr)   ( ((bm::id64_t)ptr) | 1 )
#  define BMPTR_CLEARBIT0(ptr) ( ((bm::id64_t)ptr) & ~(bm::id64_t)1 )
#  define BMPTR_TESTBIT0(ptr)  ( ((bm::id64_t)ptr) & 1 )

# else // 32-bit

#  define BMPTR_SETBIT0(ptr)   ( ((bm::id_t)ptr) | 1 )
#  define BMPTR_CLEARBIT0(ptr) ( ((bm::id_t)ptr) & ~(bm::id_t)1 )
#  define BMPTR_TESTBIT0(ptr)  ( ((bm::id_t)ptr) & 1 )

# endif

# define BMGAP_PTR(ptr) ((bm::gap_word_t*)BMPTR_CLEARBIT0(ptr))
# define BMSET_PTRGAP(ptr) ptr = (bm::word_t*)BMPTR_SETBIT0(ptr)
# define BM_IS_GAP(obj, ptr, idx) ( BMPTR_TESTBIT0(ptr)!=0 )

#endif



#ifdef BM_HASRESTRICT
# ifndef BMRESTRICT
#  define BMRESTRICT restrict
# endif
#else
# define BMRESTRICT 
#endif


#ifdef BM_HASFORCEINLINE
# ifndef BMFORCEINLINE
#  define BMFORCEINLINE __forceinline
# endif
#else
# define BMFORCEINLINE inline
#endif



#ifndef BMSSE2OPT

# ifndef BM_SET_MMX_GUARD
#  define BM_SET_MMX_GUARD
# endif

#else

# ifndef BM_SET_MMX_GUARD
#  define BM_SET_MMX_GUARD  sse2_empty_guard  bm_mmx_guard_;
# endif

#endif


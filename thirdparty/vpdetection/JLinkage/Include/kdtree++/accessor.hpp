/** \file
 * Defines the interface for the Accessor class.
 *
 * \author Martin F. Krafft <libkdtree@pobox.madduck.net>
 */

#ifndef INCLUDE_KDTREE_ACCESSOR_HPP
#define INCLUDE_KDTREE_ACCESSOR_HPP

#include <cstddef>

namespace KDTree
{
   template <typename _Val>
      struct _Bracket_accessor
      {
         typedef typename _Val::value_type result_type;

         result_type
            operator()(_Val const& V, size_t const N) const throw ()
            {
               return V[N];
            }
      };

} // namespace KDTree

#endif // include guard

/* COPYRIGHT --
 *
 * This file is part of libkdtree++, a C++ template KD-Tree sorting container.
 * libkdtree++ is (c) 2004-2007 Martin F. Krafft <libkdtree@pobox.madduck.net>
 * and Sylvain Bougerel <sylvain.bougerel.devel@gmail.com> distributed under the
 * terms of the Artistic License 2.0. See the ./COPYING file in the source tree
 * root for more information.
 *
 * THIS PACKAGE IS PROVIDED "AS IS" AND WITHOUT ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED WARRANTIES
 * OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

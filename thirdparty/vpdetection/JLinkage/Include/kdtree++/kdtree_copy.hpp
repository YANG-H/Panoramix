/** \file
 * Defines the interface for the KDTree class.
 *
 * \author Martin F. Krafft <libkdtree@pobox.madduck.net>
 *
 * Paul Harris figured this stuff out (below)
 * Notes:
 * This is similar to a binary tree, but its not the same.
 * There are a few important differences:
 *
 *  * Each level is sorted by a different criteria (this is fundamental to the design).
 *
 *  * It is possible to have children IDENTICAL to its parent in BOTH branches
 *    This is different to a binary tree, where identical children are always to the right
 *    So, KDTree has the relationships:
 *    * The left branch is <= its parent (in binary tree, this relationship is a plain < )
 *    * The right branch is <= its parent (same as binary tree)
 *
 *    This is done for mostly for performance.
 *    Its a LOT easier to maintain a consistent tree if we use the <= relationship.
 *    Note that this relationship only makes a difference when searching for an exact
 *    item with find() or find_exact, other search, erase and insert functions don't notice
 *    the difference.
 *
 *    In the case of binary trees, you can safely assume that the next identical item
 *    will be the child leaf,
 *    but in the case of KDTree, the next identical item might
 *    be a long way down a subtree, because of the various different sort criteria.
 *
 *    So erase()ing a node from a KDTree could require serious and complicated
 *    tree rebalancing to maintain consistency... IF we required binary-tree-like relationships.
 *
 *    This has no effect on insert()s, a < test is good enough to keep consistency.
 *
 *    It has an effect on find() searches:
 *      * Instead of using compare(child,node) for a < relationship and following 1 branch,
 *        we must use !compare(node,child) for a <= relationship, and test BOTH branches, as
 *        we could potentially go down both branches.
 *
 *    It has no real effect on bounds-based searches (like find_nearest, find_within_range)
 *    as it compares vs a boundary and would follow both branches if required.
 *
 *    This has no real effect on erase()s, a < test is good enough to keep consistency.
 */

#ifndef INCLUDE_KDTREE_KDTREE_HPP
#define INCLUDE_KDTREE_KDTREE_HPP

#include <vector>

#ifdef KDTREE_DEFINE_OSTREAM_OPERATORS
#  include <ostream>
#  include <stack>
#endif

#include <cmath>
#include <cstddef>
#include <cassert>

#include <kdtree++/accessor.hpp>
#include <kdtree++/allocator.hpp>
#include <kdtree++/iterator.hpp>
#include <kdtree++/node.hpp>
#include <kdtree++/region.hpp>

namespace KDTree
{

#ifdef KDTREE_CHECK_PERFORMANCE
   size_t num_dist_calcs = 0;
#endif


  template <size_t const __K, typename _Val,
            typename _Acc = _Bracket_accessor<_Val>,
            typename _Cmp = std::less<typename _Acc::result_type>,
            typename _Alloc = std::allocator<_Node<_Val> > >
    class KDTree : protected _Alloc_base<_Val, _Alloc>
    {
    protected:
      // typedef _Alloc allocator_type;
      typedef _Alloc_base<_Val, _Alloc> _Base;
      typedef typename _Base::allocator_type allocator_type;

      typedef _Node_base* _Base_ptr;
      typedef _Node_base const* _Base_const_ptr;
      typedef ::KDTree::_Node<_Val>* _Link_type;
      typedef ::KDTree::_Node<_Val> const* _Link_const_type;

      typedef _Node_compare<_Val, _Acc, _Cmp> _Node_compare;

      typedef _Region<__K, _Val, typename _Acc::result_type, _Acc, _Cmp>
        _Region;

    public:
      typedef _Region Region;
      typedef _Val value_type;
      typedef value_type* pointer;
      typedef value_type const* const_pointer;
      typedef value_type& reference;
      typedef value_type const& const_reference;
      typedef typename _Acc::result_type subvalue_type;
      typedef subvalue_type distance_type;   // NEW TYPE - for returning distances
      // I wanted to distinguish it from subvaluetype, for the future when/if we have
      // types that need a fancy distance calculation.
      // This is not complete yet, eg Region still uses subvalue_type for distance_type.
      typedef size_t size_type;
      typedef ptrdiff_t difference_type;

      KDTree( _Acc const& acc = _Acc(), const allocator_type& __a = allocator_type()) throw ()
        : _Base(__a), _M_header(_Base::_M_allocate_node()), _M_count(0), _M_acc(acc)
      {
         _M_empty_initialise();
      }

      KDTree(const KDTree& __x) throw ()
         : _Base(__x.get_allocator()), _M_header(_Base::_M_allocate_node()), _M_count(0), _M_acc(__x._M_acc)
      {
         _M_empty_initialise();
         this->insert(begin(), __x.begin(), __x.end());
         this->optimise();
      }

      template<typename _InputIterator>
        KDTree(_InputIterator __first, _InputIterator __last,
             _Acc const& acc = _Acc(), const allocator_type& __a = allocator_type()) throw ()
        : _Base(__a), _M_header(_Base::_M_allocate_node()), _M_count(0), _M_acc(acc)
      {
         _M_empty_initialise();
         this->insert(begin(), __first, __last);
         this->optimise();
      }

      KDTree&
      operator=(const KDTree& __x)
      {
         if (this != &__x)
         {
            this->clear();
            this->insert(begin(),__x.begin(),__x.end());
            this->optimize();
         }
         return *this;
      }

      ~KDTree() throw ()
      {
        this->clear();
        _M_deallocate_node(_M_header);
      }

      allocator_type
      get_allocator() const
      {
        return _Base::get_allocator();
      }

      size_type
      size() const
      {
        return _M_count;
      }

      size_type
      max_size() const
      {
        return size_type(-1);
      }

      bool
      empty() const
      {
        return this->size() == 0;
      }

      void
      clear()
      {
        _M_erase_subtree(_M_get_root());
        _M_set_leftmost(_M_header);
        _M_set_rightmost(_M_header);
        _M_set_root(NULL);
        _M_count = 0;
      }

      // typedef _Iterator<_Val, reference, pointer> iterator;
      typedef _Iterator<_Val, const_reference, const_pointer> const_iterator;
      // No mutable iterator at this stage
      typedef const_iterator iterator;

      typedef std::reverse_iterator<const_iterator> const_reverse_iterator;
      typedef std::reverse_iterator<iterator> reverse_iterator;

      const_iterator begin() const { return const_iterator(_M_get_leftmost()); }
      const_iterator end() const { return const_iterator(_M_header); }
      const_reverse_iterator rbegin() const { return const_reverse_iterator(_M_get_rightmost()); }
      const_reverse_iterator rend() const { return const_reverse_iterator(_M_header); }

      // iterator begin() { return iterator(_M_get_leftmost()); }
      // iterator end() { return iterator(_M_header); }
      // reverse_iterator rbegin() { return reverse_iterator(end()); }
      // reverse_iterator rend() { return reverse_iterator(begin()); }

      iterator
      insert(iterator /* ignored */, const_reference __V) throw (std::bad_alloc)
      {
         return this->insert(__V);
      }

      iterator
      insert(const_reference __V) throw (std::bad_alloc)
      {
        if (!_M_get_root())
          {
            _Link_type __n = _M_new_node(__V, _M_header);
            ++_M_count;
            _M_set_root(__n);
            _M_set_leftmost(__n);
            _M_set_rightmost(__n);
            return iterator(__n);
          }
        return _M_insert(_M_get_root(), __V, 0);
      }

      template <class _InputIterator>
      void insert(_InputIterator __first, _InputIterator __last) {
         for (; __first != __last; ++__first)
            this->insert(*__first);
      }

      void
      insert(iterator __pos, size_type __n, const value_type& __x)
      {
        for (; __n > 0; --__n)
          this->insert(__pos, __x);
      }

      template<typename _InputIterator>
      void
      insert(iterator __pos, _InputIterator __first, _InputIterator __last) {
         for (; __first != __last; ++__first)
            this->insert(__pos, *__first);
      }

      // Note: this uses the find() to location the item you want to erase.
      // find() compares by equivalence of location ONLY.  See the comments
      // above find_exact() for why you may not want this.
      //
      // If you want to erase ANY item that has the same location as __V,
      // then use this function.
      //
      // If you want to erase a PARTICULAR item, and not any other item
      // that might happen to have the same location, then you should use
      // erase_exact().
      void
      erase(const_reference __V) throw () {
        this->erase(this->find(__V));
      }

      void
      erase_exact(const_reference __V) throw () {
        this->erase(this->find_exact(__V));
      }

      // note: kept as const because its easier to const-cast it away
      void
      erase(const_iterator const& __IT) throw ()
      {
         assert(__IT != this->end());
        _Link_const_type target = static_cast<_Link_const_type>(__IT._M_node);
        _Link_const_type n = target;
        size_t level = 0;
        while ((n = _S_parent(n)) != _M_header)
           ++level;
        _M_erase( const_cast<_Link_type>(target), level );
        _M_delete_node( const_cast<_Link_type>(target) );
        --_M_count;
      }

/* this does not work since erasure changes sort order
      void
      erase(const_iterator __A, const_iterator const& __B) throw ()
      {
        if (0 && __A == this->begin() && __B == this->end())
          {
            this->clear();
          }
        else
          {
            while (__A != __B)
              this->erase(__A++);
          }
      }
*/

      // compares via equivalence
      // so if you are looking for any item with the same location,
      // according to the standard accessor comparisions,
      // then this is the function for you.
      const_iterator
      find(const_reference __V) const throw ()
      {
        if (!_M_get_root()) return this->end();
        return _M_find(_M_get_root(), __V, 0);
      }

      // compares via equality
      // if you are looking for a particular item in the tree,
      // and (for example) it has an ID that is checked via an == comparison
      // eg
      // struct Item
      // {
      //    unsigned int unique_id;
      //    bool operator==(Item const& a, Item const& b) { return a.unique_id == b.unique_id; }
      //    Location location;
      // };
      // Two items may be equivalent in location.  find() would return
      // either one of them.  But no two items have the same ID, so
      // find_exact() would always return the item with the same location AND id.
      //
      const_iterator
      find_exact(const_reference __V) const throw ()
      {
        if (!_M_get_root()) return this->end();
        return _M_find_exact(_M_get_root(), __V, 0);
      }

      size_t
        count_within_range(const_reference __V, subvalue_type const __R) const throw ()
        {
          if (!_M_get_root()) return 0;
          _Region __region(_M_acc,__V,__R);
          return this->count_within_range(__region);
        }

      size_t
        count_within_range(_Region const& __REGION) const throw ()
        {
          if (!_M_get_root()) return 0;

          _Region __bounds(__REGION);
          return _M_count_within_range(_M_get_root(),
                               __REGION, __bounds, 0);
        }

      template <typename SearchVal, class Visitor>
        Visitor
        visit_within_range(SearchVal V, subvalue_type const R, Visitor visitor) const throw ()
        {
          if (!_M_get_root()) return visitor;
          _Region region(_M_acc,V,R);
          return this->visit_within_range(region, visitor);
        }

      template <class Visitor>
        Visitor
        visit_within_range(_Region const& REGION, Visitor visitor) const throw ()
        {
          if (_M_get_root())
            {
              _Region bounds(REGION);
              return _M_visit_within_range(visitor, _M_get_root(), REGION, bounds, 0);
            }
          return visitor;
        }



      template <typename SearchVal, typename _OutputIterator>
        _OutputIterator
        find_within_range(SearchVal __V, subvalue_type const __R,
                          _OutputIterator __out) const throw ()
        {
          if (!_M_get_root()) return __out;
          _Region __region(_M_acc,__V,__R);
          return this->find_within_range(__region, __out);
        }

      template <typename _OutputIterator>
        _OutputIterator
        find_within_range(_Region const& __REGION,
                          _OutputIterator __out) const throw ()
        {
          if (_M_get_root())
            {
              _Region __bounds(__REGION);
              __out = _M_find_within_range(__out, _M_get_root(),
                                   __REGION, __bounds, 0);
            }
          return __out;
        }


      template <typename SearchVal>
         std::pair<const_iterator,distance_type>
        find_nearest(SearchVal __V, subvalue_type const __Max_R ) const throw ()
        {
          if (!_M_get_root())
             return std::pair<const_iterator,distance_type>(this->end(),__Max_R);
          _Region __region(_M_acc,__V);  // note: zero-area!
          typename _Region::_CenterPt __pt(__region,__Max_R);
          return this->find_nearest(__pt);
        }



      template <typename SearchVal, class Predicate>
         std::pair<const_iterator,distance_type>
        find_nearest_if(SearchVal __V, subvalue_type const __Max_R, Predicate predicate ) const throw ()
        {
          if (!_M_get_root())
             return std::pair<const_iterator,distance_type>(this->end(),__Max_R);
          _Region __region(_M_acc,__V);  // note: zero-area!
          typename _Region::_CenterPt __pt(__region,__Max_R);
          return this->find_nearest_if(__pt,predicate);
        }


      std::pair<const_iterator,distance_type>
        find_nearest( typename _Region::_CenterPt const& __CENTER ) const throw ()
        {
          if (_M_get_root())
          {
             // note: we set the initial 'bounds' to the exact point.
             // they expand from there outwards.
             _Region __bounds(__CENTER.first);
             std::pair<const_iterator,distance_type> best = _M_find_nearest(_M_get_root(), __CENTER, __bounds, 0, always_true());
             // ensure we return end() if we didn't find it
             // however, also return the best distance we did find, it might be useful to someone.
             if (best.second > __CENTER.second)
                best.first = this->end();
             return best;
          }
         return std::pair<const_iterator,distance_type>(this->end(),__CENTER.second);
        }


      template <class Predicate>
      std::pair<const_iterator,distance_type>
        find_nearest_if( typename _Region::_CenterPt const& __CENTER, Predicate predicate ) const throw ()
        {
          if (_M_get_root())
          {
             // note: we set the initial 'bounds' to the exact point.
             // they expand from there outwards.
             _Region __bounds(__CENTER.first);
             std::pair<const_iterator,distance_type> best = _M_find_nearest(_M_get_root(), __CENTER, __bounds, 0, predicate);
             // ensure we return end() if we didn't find it
             // however, also return the best distance we did find, it might be useful to someone.
             if (best.second > __CENTER.second)
                best.first = this->end();
             return best;
          }
         return std::pair<const_iterator,distance_type>(this->end(),__CENTER.second);
        }


      void
      optimise()
      {
        std::vector<value_type> __v(this->begin(),this->end());
        this->clear();
        _M_optimise(__v.begin(), __v.end(), 0);
      }

      void
      optimize()
      { // cater for people who cannot spell :)
        this->optimise();
      }

      void check_tree()
      {
         _M_check_node(_M_get_root(),0);
      }

    protected:

      void _M_check_children( _Link_const_type child, _Link_const_type parent, size_t const level, bool to_the_left )
      {
         assert(parent);
         if (child)
         {
            _Node_compare compare(level % __K,_M_acc);
            // REMEMBER! its a <= relationship for BOTH branches
            // for left-case (true), child<=node --> !(node<child)
            // for right-case (false), node<=child --> !(child<node)
            assert(!to_the_left or !compare(parent,child));  // check the left
            assert(to_the_left or !compare(child,parent));   // check the right
            // and recurse down the tree, checking everything
            _M_check_children(_S_left(child),parent,level,to_the_left);
            _M_check_children(_S_right(child),parent,level,to_the_left);
         }
      }

      void _M_check_node( _Link_const_type node, size_t const level )
      {
         if (node)
         {
            // (comparing on this level)
            // everything to the left of this node must be smaller than this
            _M_check_children( _S_left(node), node, level, true );
            // everything to the right of this node must be larger than this
            _M_check_children( _S_right(node), node, level, false );

            _M_check_node( _S_left(node), level+1 );
            _M_check_node( _S_right(node), level+1 );
         }
      }

      void _M_empty_initialise()
      {
        _M_set_leftmost(_M_header);
        _M_set_rightmost(_M_header);
        _M_set_root(NULL);
      }

      iterator
      _M_insert_left(_Link_type __N, const_reference __V)
      {
        _S_set_left(__N, _M_new_node(__V)); ++_M_count;
        _S_set_parent( _S_left(__N), __N );
        if (__N == _M_get_leftmost())
           _M_set_leftmost( _S_left(__N) );
        return iterator(_S_left(__N));
      }

      iterator
      _M_insert_right(_Link_type __N, const_reference __V)
      {
        _S_set_right(__N, _M_new_node(__V)); ++_M_count;
        _S_set_parent( _S_right(__N), __N );
        if (__N == _M_get_rightmost())
           _M_set_rightmost( _S_right(__N) );
        return iterator(_S_right(__N));
      }

      iterator
      _M_insert(_Link_type __N, const_reference __V,
             size_t const __L) throw (std::bad_alloc)
      {
        if (_Node_compare(__L % __K,_M_acc)(__V, __N))
          {
            if (!_S_left(__N))
              return _M_insert_left(__N, __V);
            return _M_insert(_S_left(__N), __V, __L+1);
          }
        else
          {
            if (!_S_right(__N) || __N == _M_get_rightmost())
              return _M_insert_right(__N, __V);
            return _M_insert(_S_right(__N), __V, __L+1);
          }
      }

      _Link_type
      _M_erase(_Link_type dead_dad, size_t const level) throw ()
      {
         // find a new step_dad, he will become a drop-in replacement.
        _Link_type step_dad = _M_get_erase_replacement(dead_dad, level);

         // tell dead_dad's parent that his new child is step_dad
        if (dead_dad == _M_get_root())
           _S_set_parent(_M_header, step_dad);
        else if (_S_left(_S_parent(dead_dad)) == dead_dad)
            _S_set_left(_S_parent(dead_dad), step_dad);
        else
            _S_set_right(_S_parent(dead_dad), step_dad);

        // deal with the left and right edges of the tree...
        // if the dead_dad was at the edge, then substitude...
        // but if there IS no new dead, then left_most is the dead_dad's parent
         if (dead_dad == _M_get_leftmost())
           _M_set_leftmost( (step_dad ? step_dad : _S_parent(dead_dad)) );
         if (dead_dad == _M_get_rightmost())
           _M_set_rightmost( (step_dad ? step_dad : _S_parent(dead_dad)) );

        if (step_dad)
          {
             // step_dad gets dead_dad's parent
            _S_set_parent(step_dad, _S_parent(dead_dad));

            // first tell the children that step_dad is their new dad
            if (_S_left(dead_dad))
               _S_set_parent(_S_left(dead_dad), step_dad);
            if (_S_right(dead_dad))
               _S_set_parent(_S_right(dead_dad), step_dad);

            // step_dad gets dead_dad's children
            _S_set_left(step_dad, _S_left(dead_dad));
            _S_set_right(step_dad, _S_right(dead_dad));
          }

        return step_dad;
      }



      _Link_type
      _M_get_erase_replacement(_Link_type node, size_t const level) throw ()
      {
         // if 'node' is null, then we can't do any better
        if (_S_is_leaf(node))
           return NULL;

        std::pair<_Link_type,size_t> candidate;
        // if there is nothing to the left, find a candidate on the right tree
        if (!_S_left(node))
          candidate = _M_get_j_min( std::pair<_Link_type,size_t>(_S_right(node),level), level+1);
        // ditto for the right
        else if ((!_S_right(node)))
          candidate = _M_get_j_max( std::pair<_Link_type,size_t>(_S_left(node),level), level+1);
        // we have both children ...
        else
         {
            // we need to do a little more work in order to find a good candidate
            // this is actually a technique used to choose a node from either the
            // left or right branch RANDOMLY, so that the tree has a greater change of
            // staying balanced.
            // If this were a true binary tree, we would always hunt down the right branch.
            // See top for notes.
            _Node_compare compare(level % __K,_M_acc);
            // compare the children based on this level's criteria...
            // (this gives virtually random results)
            if (compare(_S_right(node), _S_left(node)))
               // the right is smaller, get our replacement from the SMALLEST on the right
               candidate = _M_get_j_min(std::pair<_Link_type,size_t>(_S_right(node),level), level+1);
            else
               candidate = _M_get_j_max( std::pair<_Link_type,size_t>(_S_left(node),level), level+1);
         }

        // we have a candidate replacement by now.
        // remove it from the tree, but don't delete it.
        // it must be disconnected before it can be reconnected.
        _Link_type parent = _S_parent(candidate.first);
        if (_S_left(parent) == candidate.first)
           _S_set_left(parent, _M_erase(candidate.first, candidate.second));
        else
           _S_set_right(parent, _M_erase(candidate.first, candidate.second));

        return candidate.first;
      }



      std::pair<_Link_type,size_t>
      _M_get_j_min( std::pair<_Link_type,size_t> const node, size_t const level) throw ()
      {
         typedef std::pair<_Link_type,size_t> Result;
        if (_S_is_leaf(node.first))
            return Result(node.first,level);

        _Node_compare compare(node.second % __K,_M_acc);
        Result candidate = node;
        if (_S_left(node.first))
          {
            Result left = _M_get_j_min(Result(_S_left(node.first), node.second), level+1);
            if (compare(left.first, candidate.first))
                candidate = left;
          }
        if (_S_right(node.first))
          {
            Result right = _M_get_j_min( Result(_S_right(node.first),node.second), level+1);
            if (compare(right.first, candidate.first))
                candidate = right;
          }
        if (candidate.first == node.first)
           return Result(candidate.first,level);

        return candidate;
      }



      std::pair<_Link_type,size_t>
      _M_get_j_max( std::pair<_Link_type,size_t> const node, size_t const level) throw ()
      {
         typedef std::pair<_Link_type,size_t> Result;

        if (_S_is_leaf(node.first))
            return Result(node.first,level);

        _Node_compare compare(node.second % __K,_M_acc);
        Result candidate = node;
        if (_S_left(node.first))
          {
            Result left = _M_get_j_max( Result(_S_left(node.first),node.second), level+1);
            if (compare(candidate.first, left.first))
                candidate = left;
          }
        if (_S_right(node.first))
          {
            Result right = _M_get_j_max(Result(_S_right(node.first),node.second), level+1);
            if (compare(candidate.first, right.first))
                candidate = right;
          }

        if (candidate.first == node.first)
           return Result(candidate.first,level);

        return candidate;
      }


      void
      _M_erase_subtree(_Link_type __n)
      {
        while (__n)
          {
            _M_erase_subtree(_S_right(__n));
            _Link_type __t = _S_left(__n);
            _M_delete_node(__n);
            __n = __t;
          }
      }

      const_iterator
      _M_find(_Link_const_type node, const_reference value, size_t const level) const throw ()
      {
         // be aware! This is very different to normal binary searches, because of the <=
         // relationship used. See top for notes.
         // Basically we have to check ALL branches, as we may have an identical node
         // in different branches.
          const_iterator found = this->end();

        _Node_compare compare(level % __K,_M_acc);
        if (!compare(node,value))   // note, this is a <= test
          {
           // this line is the only difference between _M_find_exact() and _M_find()
            if (_M_matches_node_in_other_ds(node, value, level))
              return const_iterator(node);   // return right away
            if (_S_left(node))
               found = _M_find(_S_left(node), value, level+1);
          }
        if ( _S_right(node) && found == this->end() && !compare(value,node))   // note, this is a <= test
            found = _M_find(_S_right(node), value, level+1);
        return found;
      }

      const_iterator
      _M_find_exact(_Link_const_type node, const_reference value, size_t const level) const throw ()
      {
         // be aware! This is very different to normal binary searches, because of the <=
         // relationship used. See top for notes.
         // Basically we have to check ALL branches, as we may have an identical node
         // in different branches.
          const_iterator found = this->end();

        _Node_compare compare(level % __K,_M_acc);
        if (!compare(node,value))  // note, this is a <= test
        {
           // this line is the only difference between _M_find_exact() and _M_find()
            if (value == *const_iterator(node))
              return const_iterator(node);   // return right away
           if (_S_left(node))
            found = _M_find_exact(_S_left(node), value, level+1);
        }

        // note: no else!  items that are identical can be down both branches
        if ( _S_right(node) && found == this->end() && !compare(value,node))   // note, this is a <= test
            found = _M_find_exact(_S_right(node), value, level+1);
        return found;
      }

      bool
      _M_matches_node_in_d(_Link_const_type __N, const_reference __V,
                           size_t const __L) const throw ()
      {
        _Node_compare compare(__L % __K,_M_acc);
        return !(compare(__N, __V) || compare(__V, __N));
      }

      bool
      _M_matches_node_in_other_ds(_Link_const_type __N, const_reference __V,
                                  size_t const __L = 0) const throw ()
      {
        size_t __i = __L;
        while ((__i = (__i + 1) % __K) != __L % __K)
          if (!_M_matches_node_in_d(__N, __V, __i)) return false;
        return true;
      }

      bool
      _M_matches_node(_Link_const_type __N, const_reference __V,
                      size_t __L = 0) const throw ()
      {
        return _M_matches_node_in_d(__N, __V, __L)
          && _M_matches_node_in_other_ds(__N, __V, __L);
      }

      size_t
        _M_count_within_range(_Link_const_type __N, _Region const& __REGION,
                             _Region const& __BOUNDS,
                             size_t const __L) const throw ()
        {
           size_t count = 0;
          if (__REGION.encloses(_S_value(__N)))
            {
               ++count;
            }
          if (_S_left(__N))
            {
              _Region __bounds(__BOUNDS);
              __bounds.set_high_bound(_S_value(__N), __L);
              if (__REGION.intersects_with(__bounds))
                count += _M_count_within_range(_S_left(__N),
                                     __REGION, __bounds, __L+1);
            }
          if (_S_right(__N))
            {
              _Region __bounds(__BOUNDS);
              __bounds.set_low_bound(_S_value(__N), __L);
              if (__REGION.intersects_with(__bounds))
                count += _M_count_within_range(_S_right(__N),
                                     __REGION, __bounds, __L+1);
            }

          return count;
        }


      template <class Visitor>
        Visitor
        _M_visit_within_range(Visitor visitor,
                             _Link_const_type N, _Region const& REGION,
                             _Region const& BOUNDS,
                             size_t const L) const throw ()
        {
          if (REGION.encloses(_S_value(N)))
            {
              visitor(_S_value(N));
            }
          if (_S_left(N))
            {
              _Region bounds(BOUNDS);
              bounds.set_high_bound(_S_value(N), L);
              if (REGION.intersects_with(bounds))
                visitor = _M_visit_within_range(visitor, _S_left(N),
                                     REGION, bounds, L+1);
            }
          if (_S_right(N))
            {
              _Region bounds(BOUNDS);
              bounds.set_low_bound(_S_value(N), L);
              if (REGION.intersects_with(bounds))
                visitor = _M_visit_within_range(visitor, _S_right(N),
                                     REGION, bounds, L+1);
            }

          return visitor;
        }



      template <typename _OutputIterator>
        _OutputIterator
        _M_find_within_range(_OutputIterator __out,
                             _Link_const_type __N, _Region const& __REGION,
                             _Region const& __BOUNDS,
                             size_t const __L) const throw ()
        {
          if (__REGION.encloses(_S_value(__N)))
            {
              *__out++ = _S_value(__N);
            }
          if (_S_left(__N))
            {
              _Region __bounds(__BOUNDS);
              __bounds.set_high_bound(_S_value(__N), __L);
              if (__REGION.intersects_with(__bounds))
                __out = _M_find_within_range(__out, _S_left(__N),
                                     __REGION, __bounds, __L+1);
            }
          if (_S_right(__N))
            {
              _Region __bounds(__BOUNDS);
              __bounds.set_low_bound(_S_value(__N), __L);
              if (__REGION.intersects_with(__bounds))
                __out = _M_find_within_range(__out, _S_right(__N),
                                     __REGION, __bounds, __L+1);
            }

          return __out;
        }

        // quick little power function
        // next-best is __gnu_cxx::power
        // std::pow() is probably not ideal for simple powers. I forget exact details...
        distance_type _M_square( distance_type x ) const { return x*x; }

       // WARNING: Calculates and RETURNS dist^2 (for speed)
       // NOTE: CENTER is a region of zero area.  It is the point we are aiming for.
       //
       // How it works: Starting with a centerpt (single-pt in a region, with a range
       // attached to it), and bounds, it first calculates the distance to THIS node,
       // and adjusts the center's range DOWN if its closer.  No point looking further
       // than it needs to look.  A form of a dynamic find_within_range.
       // It expands the bounds as usual and sees if it intersects the centerpt+range.
       // And so it goes ...

         // substitude predicate for normal find_nearest()s
         struct always_true
         {
            bool operator()( _Val const& ) const { return true; }
         };

         template <class Predicate>
         std::pair<const_iterator,distance_type>
        _M_find_nearest( _Link_const_type __N, typename _Region::_CenterPt __CENTER,
                             _Region const& __BOUNDS,
                             size_t const __L,
                             Predicate predicate ) const throw ()
        {
           std::pair<const_iterator,distance_type> best(this->end(),__CENTER.second);

           // we ignore this node if the predicate isn't true
           if (predicate(*const_iterator(__N)))
           {
              distance_type dist = 0;
              for ( size_t i = 0; i != __K; ++i )
              {
                 dist += _M_square( __CENTER.first._M_low_bounds[i] - _M_acc(_S_value(__N),i) );
              }
#ifdef KDTREE_CHECK_PERFORMANCE
              ++num_dist_calcs;
#endif
              dist = sqrt(dist);

              best.first = __N;
              best.second = dist;
           }

           // adjust our CENTER target
           __CENTER.second = std::min(__CENTER.second,best.second);

          if (_S_left(__N))
            {
              _Region __bounds(__BOUNDS);
              __bounds.set_high_bound(_S_value(__N), __L);
              if (__bounds.intersects_with(__CENTER))
              {
                 std::pair<const_iterator,distance_type> left =
                    _M_find_nearest( _S_left(__N), __CENTER, __bounds, __L+1, predicate);
                 // check if better than what I found
                 if (left.second < best.second) best = left;
              }
            }

           // adjust our center target (only useful if left found something closer)
           __CENTER.second = std::min(__CENTER.second,best.second);

          if (_S_right(__N))
            {
              _Region __bounds(__BOUNDS);
              __bounds.set_low_bound(_S_value(__N), __L);
              if (__bounds.intersects_with(__CENTER))
              {
                 std::pair<const_iterator,distance_type> right =
                    _M_find_nearest( _S_right(__N), __CENTER, __bounds, __L+1, predicate);
                 // check if better than what I found
                 if (right.second < best.second) best = right;
               }
            }

          return best;
        }

      template <typename _Iter>
        void
        _M_optimise(_Iter const& __A, _Iter const& __B,
                    size_t const __L) throw ()
      {
        if (__A == __B) return;
        _Node_compare compare(__L % __K,_M_acc);
        std::sort(__A, __B, compare);
        _Iter __m = __A + (__B - __A) / 2;
        this->insert(*__m);
        if (__m != __A) _M_optimise(__A, __m, __L+1);
        if (++__m != __B) _M_optimise(__m, __B, __L+1);
      }

      _Link_const_type
      _M_get_root() const
      {
         return static_cast<_Link_const_type>( _M_header->_M_parent );
      }

      _Link_type
      _M_get_root()
      {
         return static_cast<_Link_type>( _M_header->_M_parent );
      }

      void _M_set_root(_Node_base * n)
      {
         _M_header->_M_parent = n;
      }

      _Link_const_type
      _M_get_leftmost() const
      {
        return static_cast<_Link_type>( _M_header->_M_left );
      }

      void
      _M_set_leftmost( _Node_base * a )
      {
         _M_header->_M_left = a;
      }

      _Link_const_type
      _M_get_rightmost() const
      {
        return static_cast<_Link_type>( _M_header->_M_right );
      }

      void
      _M_set_rightmost( _Node_base * a )
      {
         _M_header->_M_right = a;
      }

      static _Link_type
      _S_parent(_Base_ptr N)
      {
        return static_cast<_Link_type>( N->_M_parent );
      }

      static _Link_const_type
      _S_parent(_Base_const_ptr N)
      {
        return static_cast<_Link_const_type>( N->_M_parent );
      }

      static void
      _S_set_parent(_Base_ptr N, _Base_ptr p)
      {
        N->_M_parent = p;
      }

      static void
      _S_set_left(_Base_ptr N, _Base_ptr l)
      {
        N->_M_left = l;
      }

      static _Link_type
      _S_left(_Base_ptr N)
      {
        return static_cast<_Link_type>( N->_M_left );
      }

      static _Link_const_type
      _S_left(_Base_const_ptr N)
      {
        return static_cast<_Link_const_type>( N->_M_left );
      }

      static void
      _S_set_right(_Base_ptr N, _Base_ptr r)
      {
        N->_M_right = r;
      }

      static _Link_type
      _S_right(_Base_ptr N)
      {
        return static_cast<_Link_type>( N->_M_right );
      }

      static _Link_const_type
      _S_right(_Base_const_ptr N)
      {
        return static_cast<_Link_const_type>( N->_M_right );
      }

      static bool
      _S_is_leaf(_Base_const_ptr N)
      {
        return !_S_left(N) && !_S_right(N);
      }

      static const_reference
      _S_value(_Link_const_type N)
      {
        return N->_M_value;
      }

      static const_reference
      _S_value(_Base_const_ptr N)
      {
        return static_cast<_Link_const_type>(N)->_M_value;
      }

      static _Link_const_type
      _S_minimum(_Link_const_type __X)
      {
        return static_cast<_Link_const_type> ( _Node_base::_S_minimum(__X) );
      }

      static _Link_const_type
      _S_maximum(_Link_const_type __X)
      {
        return static_cast<_Link_const_type>( _Node_base::_S_maximum(__X) );
      }

      _Link_type
      _M_new_node(const_reference __V, //  = value_type(),
                  _Base_ptr const __PARENT = NULL,
                  _Base_ptr const __LEFT = NULL,
                  _Base_ptr const __RIGHT = NULL)
      {
        _Link_type __ret = _Base::_M_allocate_node();
        try
          {
            _M_construct_node(__ret, __V, __PARENT, __LEFT, __RIGHT);
          }
        catch(...)
          {
            _M_deallocate_node(__ret);
            __throw_exception_again;
          }
        return __ret;
      }

      /* WHAT was this for?
      _Link_type
      _M_clone_node(_Link_const_type __X)
      {
        _Link_type __ret = _M_allocate_node(__X->_M_value);
        // TODO
        return __ret;
      }
      */

      void
      _M_delete_node(_Link_type __p)
      {
        _M_destroy_node(__p);
        _M_deallocate_node(__p);
      }

      _Link_type _M_header;
      size_type _M_count;
      _Acc _M_acc;

#ifdef KDTREE_DEFINE_OSTREAM_OPERATORS
      friend std::ostream&
      operator<<(std::ostream& o,
                    KDTree<__K, _Val, _Acc, _Cmp, _Alloc> const& tree) throw ()
    {
      o << "meta node:   " << *tree._M_header << std::endl;

      if (tree.empty())
        return o << "[empty " << __K << "d-tree " << &tree << "]";

      o << "nodes total: " << tree.size() << std::endl;
      o << "dimensions:  " << __K << std::endl;

      typedef KDTree<__K, _Val, _Acc, _Cmp, _Alloc> _Tree;
      typedef typename _Tree::_Link_type _Link_type;

      std::stack<_Link_const_type> s;
      s.push(tree._M_get_root());

      while (!s.empty())
        {
          _Link_const_type n = s.top();
          s.pop();
          o << *n << std::endl;
          if (_Tree::_S_left(n)) s.push(_Tree::_S_left(n));
          if (_Tree::_S_right(n)) s.push(_Tree::_S_right(n));
        }

      return o;
    }
#endif

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
 * Parts of this file are (c) 2004-2007 Paul Harris <paulharris@computer.org>.
 *
 * THIS PACKAGE IS PROVIDED "AS IS" AND WITHOUT ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED WARRANTIES
 * OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

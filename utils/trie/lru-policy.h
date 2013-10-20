/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 University of California, Los Angeles
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Alexander Afanasyev <alexander.afanasyev@ucla.edu>
 */

#ifndef LRU_POLICY_H_
#define LRU_POLICY_H_

#include <boost/intrusive/options.hpp>
#include <boost/intrusive/list.hpp>

namespace ns3 {

namespace ndn {
namespace ndnSIM {

/**
 * @brief Traits for Least Recently Used replacement policy
 */
struct lru_policy_traits
{
  /// @brief Name that can be used to identify the policy (for NS-3 object model and logging)
  static std::string GetName () { return "Lru"; }
  
  struct policy_hook_type : public boost::intrusive::list_member_hook<> {};

  template<class Container>
  struct container_hook
  {
    typedef boost::intrusive::member_hook< Container,
                                           policy_hook_type,
                                           &Container::policy_hook_ > type;
  };

  template<class Base,
           class Container,
           class Hook>
  struct policy 
  {
    typedef typename boost::intrusive::list< Container, Hook > policy_container;
    
    // could be just typedef
    class type : public policy_container
    {
    public:
      typedef Container parent_trie;
    
      type (Base &base)
        : base_ (base)
        , max_size_ (100)
      {
      }

      inline void
      update (typename parent_trie::iterator item)
      {
    	  /*lru和fifo的区别就在这里，splice函数参看 http://www.boost.org/doc/libs/1_51_0/doc/html/boost/intrusive/list.html#id1199565-bb
    	   * void splice(const_iterator p, list & x, const_iterator new_ele);
    	   *Requires: p must be a valid iterator of *this. new_ele must point to an element contained in list x.
    	   *Effects: Transfers the value pointed by new_ele, from list x to this list, before the the element pointed by p.
    	   *No destructors or copy constructors are called. If p == new_ele or p == ++new_ele, this function is a null operation.
    	   */
  	//std::cout << "ZhangYu2013-8-9-------------------------------------update" << std::endl;
        // do relocation
        policy_container::splice (policy_container::end (),
                                  *this,
                                  policy_container::s_iterator_to (*item));
      }
  
      inline bool
      insert (typename parent_trie::iterator item)
      {
  	//std::cout << "ZhangYu2013-8-9-------------------------------------insert" << item->payload() << std::endl;

        if (max_size_ != 0 && policy_container::size () >= max_size_)
          {
            base_.erase (&(*policy_container::begin ()));
          }
      
        policy_container::push_back (*item);
        return true;
      }
  
      inline void
      lookup (typename parent_trie::iterator item)
      {
	//std::cout << "ZhangYu2013-8-9-------------------------------------lookup" << std::endl;
        // do relocation
        policy_container::splice (policy_container::end (),
                                  *this,
                                  policy_container::s_iterator_to (*item));
      }
  
      inline void
      erase (typename parent_trie::iterator item)
      {
	//std::cout << "ZhangYu2013-8-9-------------------------------------erase" << std::endl;
        policy_container::erase (policy_container::s_iterator_to (*item));
      }

      inline void
      clear ()
      {
        policy_container::clear ();
      }

      inline void
      set_max_size (size_t max_size)
      {
        max_size_ = max_size;
      }

      inline size_t
      get_max_size () const
      {
        return max_size_;
      }

    private:
      type () : base_(*((Base*)0)) { };

    private:
      Base &base_;
      size_t max_size_;
    };
  };
};

} // ndnSIM
} // ndn
} // ns3

#endif

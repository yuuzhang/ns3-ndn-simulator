/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil -*- */
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
 * Author:  Alexander Afanasyev <alexander.afanasyev@ucla.edu>
 *          Ilya Moiseenko <iliamo@cs.ucla.edu>
 *
 */

#include "ndn-app-face.h"

#include "ns3/log.h"
#include "ns3/packet.h"
#include "ns3/node.h"
#include "ns3/assert.h"
#include "ns3/simulator.h"

#include "ns3/ndn-header-helper.h"
#include "ns3/ndn-app.h"

#include "ndn-interest.h"
#include "ndn-content-object.h"

NS_LOG_COMPONENT_DEFINE ("ndn.AppFace");

namespace ns3 {
namespace ndn {

NS_OBJECT_ENSURE_REGISTERED (AppFace);

TypeId
AppFace::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::ndn::AppFace")
    .SetParent<Face> ()
    .SetGroupName ("Ndn")
    ;
  return tid;
}

AppFace::AppFace (Ptr<App> app)
  : Face (app->GetNode ())
  , m_app (app)
{
  NS_LOG_FUNCTION (this << app);
  
  NS_ASSERT (m_app != 0);
}

AppFace::~AppFace ()
{
  NS_LOG_FUNCTION_NOARGS ();
}

AppFace::AppFace ()
  : Face (0)
{
}

AppFace::AppFace (const AppFace &)
  : Face (0)
{
}

AppFace& AppFace::operator= (const AppFace &)
{
  return *((AppFace*)0);
}


void
AppFace::RegisterProtocolHandler (ProtocolHandler handler)
{
  NS_LOG_FUNCTION (this);

  Face::RegisterProtocolHandler (handler);

  m_app->RegisterProtocolHandler (MakeCallback (&Face::Receive, this));
}

// to pass packets from NDN stacks to the underlying layer(network or application)
bool
AppFace::SendImpl (Ptr<Packet> p)
{
  NS_LOG_FUNCTION (this << p);

  NS_LOG_DEBUG("ZhangYu 2013-8-21 SendImpl p: " << p);

  try
    {
      HeaderHelper::Type type = HeaderHelper::GetNdnHeaderType (p);
      switch (type)
        {
        case HeaderHelper::INTEREST_NDNSIM:
          {
            Ptr<Interest> header = Create<Interest> ();
            p->RemoveHeader (*header);

            if (header->GetNack () > 0)
              m_app->OnNack (header, p);
            else
              m_app->OnInterest (header, p);
          
            break;
          }
          /*在ns3的仿真中，可以产生和实际网络中一样的数据包，参见http://www.nsnam.org/doxygen-release/classns3_1_1_packet.html
           * 每个网络数据包包括： a byte buffer, a set of byte tags, a set of packet tags, and metadat
           */
        case HeaderHelper::CONTENT_OBJECT_NDNSIM:
          {
            static ContentObjectTail tail;
            Ptr<ContentObject> header = Create<ContentObject> ();

            NS_LOG_DEBUG("ZhangYu 2013-8-22 case ::CONTENT_OBJECT_NDNSIM: " << p->GetSize());
            NS_LOG_DEBUG("ZhangYu 2013-8-22 case ::CONTENT_OBJECT_NDNSIM: " << header->GetInstanceTypeId());
            header->SetName("testHead");
            /* 2013-8-22 如果没有上面的句子，下面执行中会报错，而不是编译时报错，是因为Name还没有被设置，是空的，所以获取的时候出错，
             * 但是这样我就不能理解为什么生成一个空的ContentObject，名字没有设置，而可以从p中 remove
             */
            int temp=p->RemoveHeader (*header);
            //temp=p->RemoveHeader (*header);
            NS_LOG_DEBUG("ZhangYu 2013-8-22 case ::CONTENT_OBJECT_NDNSIM: " << p <<"  header   " << header->GetName()<< "  " << temp);

            p->RemoveTrailer (tail);
            m_app->OnContentObject (header, p/*payload*/);
          
            break;
          }
        default:
          NS_FATAL_ERROR ("ccnb support is currently broken");
          break;
        }
      
      return true;
    }
  catch (UnknownHeaderException)
    {
      NS_LOG_ERROR ("Unknown header type");
      return false;
    }
}

std::ostream&
AppFace::Print (std::ostream& os) const
{
  os << "dev=local(" << GetId() << ")";
  return os;
}

} // namespace ndn
} // namespace ns3


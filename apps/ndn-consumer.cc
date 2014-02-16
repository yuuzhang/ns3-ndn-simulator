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
 * Author: Ilya Moiseenko <iliamo@cs.ucla.edu>
 */
//ZhangYu 2014-2-6 为添加DynamicRouting添加的
#include "ns3/ndn-global-routing-helper.h"
#include "ns3/node-list.h"
// --ZhangYu

#include "ndn-consumer.h"
#include "ns3/ptr.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/packet.h"
#include "ns3/callback.h"
#include "ns3/string.h"
#include "ns3/boolean.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"

#include "ns3/ndn-app-face.h"
#include "ns3/ndn-interest.h"
#include "ns3/ndn-content-object.h"
#include "ns3/ndnSIM/utils/ndn-fw-hop-count-tag.h"
#include "ns3/ndnSIM/utils/ndn-rtt-mean-deviation.h"

#include <boost/ref.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/lambda/bind.hpp>

#include "ns3/names.h"

namespace ll = boost::lambda;

NS_LOG_COMPONENT_DEFINE ("ndn.Consumer");

namespace ns3 {
namespace ndn {

NS_OBJECT_ENSURE_REGISTERED (Consumer);

TypeId
Consumer::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::ndn::Consumer")
    .SetGroupName ("Ndn")
    .SetParent<App> ()
    .AddAttribute ("StartSeq", "Initial sequence number",
                   IntegerValue (0),
                   MakeIntegerAccessor(&Consumer::m_seq),
                   MakeIntegerChecker<int32_t>())

    /*下面语句中的MakeNameAccessor是一个宏定义： Make##type##Accessor定义的，in ns-3/src/sore/model/attribute-helper.h
     * NS3中的属性不同与类属性，是一个个基于字符串的命名空间，类似文件系统的路径，支持正则表达式
     */
    .AddAttribute ("Prefix","Name of the Interest",
                   StringValue ("/"),
                   MakeNameAccessor (&Consumer::m_interestName),
                   MakeNameChecker ())

    .AddAttribute ("LifeTime", "LifeTime for interest packet",
                   StringValue ("2s"),
                   MakeTimeAccessor (&Consumer::m_interestLifeTime),
                   MakeTimeChecker ())

    .AddAttribute ("RetxTimer",
                   "Timeout defining how frequent retransmission timeouts should be checked",
                   StringValue ("50ms"),
                   MakeTimeAccessor (&Consumer::GetRetxTimer, &Consumer::SetRetxTimer),
                   MakeTimeChecker ())

    .AddTraceSource ("LastRetransmittedInterestDataDelay", "Delay between last retransmitted Interest and received Data",
                     MakeTraceSourceAccessor (&Consumer::m_lastRetransmittedInterestDataDelay))

    .AddTraceSource ("FirstInterestDataDelay", "Delay between first transmitted Interest and received Data",
                     MakeTraceSourceAccessor (&Consumer::m_firstInterestDataDelay))
    ;
  return tid;
}

Consumer::Consumer ()
  : m_rand (0, std::numeric_limits<uint32_t>::max ())
  , m_seq (0)
  , m_seqMax (0) // don't request anything
{
  NS_LOG_FUNCTION_NOARGS ();

  m_rtt = CreateObject<RttMeanDeviation> ();
  /* 下面的语句不能得到正确的NodeID，猜测是因为还没有执行到设置m_node。因为Consumer是App,初始化完后才装在Node上。所以在主程序ndn-congestion-topology中的获取
   * consumer1->GetId() << std::endl;  consumerHelper.Install (consumer1);
   */
  NS_LOG_DEBUG("2013-8-24 ZhangYu" << " NodeID: "  << this->m_node);
}

void
Consumer::SetRetxTimer (Time retxTimer)
{
  m_retxTimer = retxTimer;
  if (m_retxEvent.IsRunning ())
    {
      // m_retxEvent.Cancel (); // cancel any scheduled cleanup events
      Simulator::Remove (m_retxEvent); // slower, but better for memory
    }

  // schedule even with new timeout
  m_retxEvent = Simulator::Schedule (m_retxTimer,
                                     &Consumer::CheckRetxTimeout, this);
}

Time
Consumer::GetRetxTimer () const
{
  return m_retxTimer;
}

void
Consumer::CheckRetxTimeout ()
{
  Time now = Simulator::Now ();

  Time rto = m_rtt->RetransmitTimeout ();
  // NS_LOG_DEBUG ("Current RTO: " << rto.ToDouble (Time::S) << "s");

  while (!m_seqTimeouts.empty ())
    {
      SeqTimeoutsContainer::index<i_timestamp>::type::iterator entry =
        m_seqTimeouts.get<i_timestamp> ().begin ();
      if (entry->time + rto <= now) // timeout expired?
        {
          uint32_t seqNo = entry->seq;
          m_seqTimeouts.get<i_timestamp> ().erase (entry);
          OnTimeout (seqNo);
        }
      else
        break; // nothing else to do. All later packets need not be retransmitted
    }

  m_retxEvent = Simulator::Schedule (m_retxTimer,
                                     &Consumer::CheckRetxTimeout, this);
}

// Application Methods
void
Consumer::StartApplication () // Called at time specified by Start
{
  NS_LOG_FUNCTION_NOARGS ();

  // do base stuff
  App::StartApplication ();

  ScheduleNextPacket ();

  Ptr<Name> nameWithSequence = Create<Name> (m_interestName);

  //ndn::GlobalRoutingHelper::CalculateNoCommLinkMultiPathRoutes(NodeList::GetNode(6),NodeList::GetNode(7),nameWithSequence);

}

void
Consumer::StopApplication () // Called at time specified by Stop
{
  NS_LOG_FUNCTION_NOARGS ();

  // cancel periodic packet generation
  Simulator::Cancel (m_sendEvent);

  // cleanup base stuff
  App::StopApplication ();
}

void
Consumer::SendPacket ()
{
  if (!m_active) return;

  NS_LOG_FUNCTION_NOARGS ();

  uint32_t seq=std::numeric_limits<uint32_t>::max (); //invalid

  while (m_retxSeqs.size ())
    {
      seq = *m_retxSeqs.begin ();
      m_retxSeqs.erase (m_retxSeqs.begin ());
      break;
    }

  if (seq == std::numeric_limits<uint32_t>::max ())
    {
      if (m_seqMax != std::numeric_limits<uint32_t>::max ())
        {
          if (m_seq >= m_seqMax)
            {
              return; // we are totally done
            }
        }

      seq = m_seq++;
    }


  StringValue tmp;
  this->GetAttribute("Prefix",tmp);
  NS_LOG_DEBUG(this << " 2013-8-23 ZhangYu" << " NodeID: "  << this->m_node->GetId()
		  <<" NodeName: "  <<  Names::FindName(this->m_node)
  	  	  << " produce a interest,  Consumer::m_interestName: " << tmp.Get());

  //
  Ptr<Name> nameWithSequence = Create<Name> (m_interestName);
  (*nameWithSequence) (seq);
  //参考ndn-name.cc，Name支持很灵活的元素添加，实现的是内容的分层命名，2013-8-24

  Interest interestHeader;
  interestHeader.SetNonce               (m_rand.GetValue ());
  interestHeader.SetName                (nameWithSequence);
  interestHeader.SetInterestLifetime    (m_interestLifeTime);

  NS_LOG_DEBUG("2013-8-21 ZhangYu nameWithSequence: "<<*nameWithSequence <<" " << interestHeader.GetName());
  // NS_LOG_INFO ("Requesting Interest: \n" << interestHeader);
  //NS_LOG_INFO ("> Interest for " << seq);

  Ptr<Packet> packet = Create<Packet> ();
  packet->AddHeader (interestHeader);
  NS_LOG_DEBUG ("Interest packet size: " << packet->GetSize ());

  CalculateDynamicRouting(nameWithSequence);

  WillSendOutInterest (seq);  

  FwHopCountTag hopCountTag;
  packet->AddPacketTag (hopCountTag);

  m_transmittedInterests (&interestHeader, this, m_face);
  m_protocolHandler (packet);

  ScheduleNextPacket ();
}

/*
 * ZhangYu 2014-2-10 要实现动态路由，分析后认为应该是在产生了一个interest后，计算路由并为相关节点添加fib，在forwarding-strategy.cc的satisfiyPendingInterest中删除fib
 * 目前来看这种实现是正确的。
 */
void
Consumer::CalculateDynamicRouting(Ptr<Name> nameWithSequence)
{
    for (NodeList::Iterator node = NodeList::Begin (); node != NodeList::End (); node++)
    {
		for(uint32_t appId=0; appId<(*node)->GetNApplications();appId++)
		{
			std::string appTypeStr= (*node)->GetApplication(appId)->GetInstanceTypeId().GetName();
			if(std::string::npos!= appTypeStr.find("Producer"))
			{
				//ZhangYu 2014-2-6 为每个数据包动态计算路由，在产生了一个interest后，发送前为这个包计算路由
				ndn::GlobalRoutingHelper::CalculateNoCommLinkMultiPathRoutes(this->m_node,*node,nameWithSequence);
				//std::cout << "ZhangYu 2014-2-13 ==== 2   " << (*node)->GetId() << std::endl;

			}
		}
    }
}
///////////////////////////////////////////////////
//          Process incoming packets             //
///////////////////////////////////////////////////


void
Consumer::OnContentObject (const Ptr<const ContentObject> &contentObject,
                               Ptr<Packet> payload)
{
  if (!m_active) return;

  App::OnContentObject (contentObject, payload); // tracing inside

  NS_LOG_FUNCTION (this << contentObject << payload);

  // NS_LOG_INFO ("Received content object: " << boost::cref(*contentObject));

  uint32_t seq = boost::lexical_cast<uint32_t> (contentObject->GetName ().GetComponents ().back ());
  NS_LOG_INFO ("< DATA for " << seq);

  int hopCount = -1;
  FwHopCountTag hopCountTag;
  if (payload->RemovePacketTag (hopCountTag))
    {
      hopCount = hopCountTag.Get ();
    }

  SeqTimeoutsContainer::iterator entry = m_seqLastDelay.find (seq);
  if (entry != m_seqLastDelay.end ())
    {
      m_lastRetransmittedInterestDataDelay (this, seq, Simulator::Now () - entry->time, hopCount);
    }

  entry = m_seqFullDelay.find (seq);
  if (entry != m_seqFullDelay.end ())
    {
      m_firstInterestDataDelay (this, seq, Simulator::Now () - entry->time, m_seqRetxCounts[seq], hopCount);
    }

  m_seqRetxCounts.erase (seq);
  m_seqFullDelay.erase (seq);
  m_seqLastDelay.erase (seq);

  m_seqTimeouts.erase (seq);
  m_retxSeqs.erase (seq);

  m_rtt->AckSeq (SequenceNumber32 (seq));
}

void
Consumer::OnNack (const Ptr<const Interest> &interest, Ptr<Packet> origPacket)
{
  if (!m_active) return;

  App::OnNack (interest, origPacket); // tracing inside

  // NS_LOG_DEBUG ("Nack type: " << interest->GetNack ());

  // NS_LOG_FUNCTION (interest->GetName ());

  // NS_LOG_INFO ("Received NACK: " << boost::cref(*interest));
  uint32_t seq = boost::lexical_cast<uint32_t> (interest->GetName ().GetComponents ().back ());
  NS_LOG_INFO ("< NACK for " << seq);
  // std::cout << Simulator::Now ().ToDouble (Time::S) << "s -> " << "NACK for " << seq << "\n";

  // put in the queue of interests to be retransmitted
  // NS_LOG_INFO ("Before: " << m_retxSeqs.size ());
  m_retxSeqs.insert (seq);
  // NS_LOG_INFO ("After: " << m_retxSeqs.size ());

  m_seqTimeouts.erase (seq);

  m_rtt->IncreaseMultiplier ();             // Double the next RTO ??
  ScheduleNextPacket ();
}

void
Consumer::OnTimeout (uint32_t sequenceNumber)
{
  NS_LOG_FUNCTION (sequenceNumber);
   std::cout << Simulator::Now () << ", TO: " << sequenceNumber << ", current RTO: " << m_rtt->RetransmitTimeout ().ToDouble (Time::S) << "s\n";

  m_rtt->IncreaseMultiplier ();             // Double the next RTO
  m_rtt->SentSeq (SequenceNumber32 (sequenceNumber), 1); // make sure to disable RTT calculation for this sample
  m_retxSeqs.insert (sequenceNumber);
  ScheduleNextPacket ();
}

void
Consumer::WillSendOutInterest (uint32_t sequenceNumber)
{

  NS_LOG_DEBUG ("Trying to add " << sequenceNumber << " with " << Simulator::Now () << ". already " << m_seqTimeouts.size () << " items");

  m_seqTimeouts.insert (SeqTimeout (sequenceNumber, Simulator::Now ()));
  m_seqFullDelay.insert (SeqTimeout (sequenceNumber, Simulator::Now ()));

  m_seqLastDelay.erase (sequenceNumber);
  m_seqLastDelay.insert (SeqTimeout (sequenceNumber, Simulator::Now ()));

  m_seqRetxCounts[sequenceNumber] ++;

  m_rtt->SentSeq (SequenceNumber32 (sequenceNumber), 1);
}


} // namespace ndn
} // namespace ns3

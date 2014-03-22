/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011-2012 University of California, Los Angeles
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
// ndn-congestion-topo-plugin.cc
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/ndnSIM-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/point-to-point-layout-module.h"

//ZhangYu 2013-8-16  for ndn::CsTracer
#include <ns3/ndnSIM/utils/tracers/ndn-cs-tracer.h>
// for ndn::L3AggregateTracer
#include <ns3/ndnSIM/utils/tracers/ndn-l3-aggregate-tracer.h>
// for ndn::AppDelayTracer
#include <ns3/ndnSIM/utils/tracers/ndn-app-delay-tracer.h>
// for ndn::L2RateTracer
#include <ns3/ndnSIM/utils/tracers/l2-rate-tracer.h>



// for ndn::L3RateTracer
#include <ns3/ndnSIM/utils/tracers/ndn-l3-rate-tracer.h>
//ZhangYu 2014-2-7 for DynamicRouting，否则不认识Name，试了很多.h才知道要包含ndn-interest.h
#include "ns3/names.h"
#include "ns3/ndn-name.h"
#include "string.h"
#include "ns3/ndn-interest.h"
#include "ns3/ptr.h"
#include <boost/ref.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/lambda/bind.hpp>
//---ZhangYu

using namespace ns3;

/**
 * ZhangYu 2014-3-10，使用BRITE产生的随机网络拓扑，业务量均匀分布在随机的节点对之间
 * 我们这里可以偷懒，因为节点的位置和链路是随机的，所以可以让节点对编号是连续的，效果上应该和均匀分布的随机是一样的。
 * 而且这样拓扑和业务分布可以是固定的，分析数据结果时，可比性强。
 */

int
main (int argc, char *argv[])
{
  CommandLine cmd;
  cmd.Parse (argc, argv);

  AnnotatedTopologyReader topologyReader ("", 25);
  int nodesNumber=2;
  topologyReader.SetFileName ("src/ndnSIM/examples/topologies/2node-result.txt");
  //topologyReader.SetFileName ("src/ndnSIM/examples/topologies/topo-6-node.txt");
  topologyReader.Read ();


  // Install NDN stack on all nodes
  ndn::StackHelper ndnHelper;
  //ndnHelper.SetForwardingStrategy ("ns3::ndn::fw::Flooding");
  ndnHelper.SetForwardingStrategy ("ns3::ndn::fw::BestRoute");
  ndnHelper.SetContentStore ("ns3::ndn::cs::Lru",
                              "MaxSize", "1");
  ndnHelper.InstallAll ();

  topologyReader.ApplyOspfMetric();  //使得链路metric生效

  // Installing global routing interface on all nodes
  ndn::GlobalRoutingHelper ndnGlobalRoutingHelper;
  ndnGlobalRoutingHelper.InstallAll ();

  //设置和安装业务

  ndn::AppHelper consumerHelper ("ns3::ndn::ConsumerCbr");
  consumerHelper.SetAttribute ("Frequency", StringValue ("1000")); // 100 interests a second

  for(int i=0;i<nodesNumber/2;i++)
  {
	  // Getting containers for the consumer/producer
	  Ptr<Node> consumer1 = Names::Find<Node> ("Node"+boost::lexical_cast<std::string> (i));
	  // on the first consumer node install a Consumer application
	  // that will express interests in /dst1 namespace
	  consumerHelper.SetPrefix ("/Node"+boost::lexical_cast<std::string>(i));
	  consumerHelper.Install (consumer1);
	  //std::cout <<"ZhangYu 2014-3-7 consumer1->GetId(): " <<consumer1->GetId() << std::endl;
  }

  ndn::AppHelper producerHelper ("ns3::ndn::Producer");
  producerHelper.SetAttribute ("PayloadSize", StringValue("1024"));

  for(int i=nodesNumber/2;i<nodesNumber;i++)
  {
	  producerHelper.SetPrefix ("/Node"+boost::lexical_cast<std::string>(i-nodesNumber/2));
	  Ptr<Node> producer1 = Names::Find<Node> ("Node"+boost::lexical_cast<std::string> (i));
	  // install producer that will satisfy Interests in /dst1 namespace
	  ndnGlobalRoutingHelper.AddOrigins ("/Node"+boost::lexical_cast<std::string>(i-nodesNumber/2), producer1);
	  producerHelper.Install (producer1);
	  //std::cout <<"ZhangYu 2014-3-7 producer1->GetId(): " <<producer1->GetId() << std::endl;
  }

  // Calculate and install FIBs
  //ndn::GlobalRoutingHelper::CalculateRoutes ();
  ndn::GlobalRoutingHelper::CalculateNoCommLinkMultiPathRoutes();


  Simulator::Stop (Seconds (1.0));

  //ZhangYu Add the trace

  boost::tuple< boost::shared_ptr<std::ostream>, std::list<Ptr<ndn::CsTracer> > >
  csTracers = ndn::CsTracer::InstallAll ("cs-trace.txt", Seconds (1));

  boost::tuple< boost::shared_ptr<std::ostream>, std::list<Ptr<ndn::L3AggregateTracer> > >
  aggTracers = ndn::L3AggregateTracer::InstallAll ("aggregate-trace.txt", Seconds (0.5));

  boost::tuple< boost::shared_ptr<std::ostream>, std::list<Ptr<ndn::L3RateTracer> > >
    rateTracers = ndn::L3RateTracer::InstallAll ("rate-trace.txt", Seconds (10));

  boost::tuple< boost::shared_ptr<std::ostream>, std::list<Ptr<ndn::AppDelayTracer> > >
    tracers = ndn::AppDelayTracer::InstallAll ("app-delays-trace.txt");

  boost::tuple< boost::shared_ptr<std::ostream>, std::list<Ptr<L2RateTracer> > >
   l2tracers = L2RateTracer::InstallAll ("drop-trace.txt", Seconds (0.5));

  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}

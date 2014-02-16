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
// ndn-grid.cc
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/point-to-point-layout-module.h"
#include "ns3/ndnSIM-module.h"

//ZhangYu 2013-8-16  for ndn::CsTracer
#include <ns3/ndnSIM/utils/tracers/ndn-cs-tracer.h>
// for ndn::L3AggregateTracer
#include <ns3/ndnSIM/utils/tracers/ndn-l3-aggregate-tracer.h>

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
 * This scenario simulates a grid topology (using PointToPointGrid module)
 *
 * (consumer) -- ( ) ----- ( )
 *     |          |         |
 *    ( ) ------ ( ) ----- ( )
 *     |          |         |
 *    ( ) ------ ( ) -- (producer)(2,2)
 *
 * All links are 1Mbps with propagation 10ms delay.
 *
 * FIB is populated using NdnGlobalRoutingHelper.
 *
 * Consumer requests data from producer with frequency 100 interests per second
 * (interests contain constantly increasing sequence number).
 *
 * For every received interest, producer replies with a data packet, containing
 * 1024 bytes of virtual payload.
 *
 * To run scenario and see what is happening, use the following command:
 *
 *     NS_LOG=ndn.Consumer:ndn.ConsumerZipfMandelbrot:ndn.Producer ./waf --run=ndn-zipf-mandelbrot
 */

int
main (int argc, char *argv[])
{
    //LogComponentEnable("ndn.CbisGlobalRoutingHelper", LOG_LEVEL_INFO);
    // Setting default parameters for PointToPoint links and channels
    Config::SetDefault ("ns3::PointToPointNetDevice::DataRate", StringValue ("1Mbps"));
    Config::SetDefault ("ns3::PointToPointChannel::Delay", StringValue ("1ms"));
    Config::SetDefault ("ns3::DropTailQueue::MaxPackets", StringValue ("10"));
    
    // Read optional command-line parameters (e.g., enable visualizer with ./waf --run=<> --visualize
    CommandLine cmd;
    cmd.Parse (argc, argv);
    
    int aRowNodes;
    aRowNodes=3;
    
    // Creating 3x3 topology
    PointToPointHelper p2p;
    PointToPointGridHelper grid (aRowNodes, aRowNodes, p2p);
    grid.BoundingBox(100,100,200,200);
    
    // Install CCNx stack on all nodes
    ndn::StackHelper ccnxHelper;
    //ccnxHelper.SetForwardingStrategy ("ns3::ndn::fw::SmartFlooding");
    ccnxHelper.SetForwardingStrategy ("ns3::ndn::fw::Flooding");
    //ccnxHelper.SetForwardingStrategy ("ns3::ndn::fw::BestRoute");
    ccnxHelper.SetContentStore ("ns3::ndn::cs::Lru", "MaxSize", "1");
    ccnxHelper.InstallAll ();
    
    // Installing global routing interface on all nodes
    //ndn::CbisGlobalRoutingHelper ccnxGlobalRoutingHelper;
    ndn::GlobalRoutingHelper ccnxGlobalRoutingHelper;
    ccnxGlobalRoutingHelper.InstallAll ();

    NodeContainer consumerNodes;
    //consumerNodes.Add(grid.GetNode(2,0));
    consumerNodes.Add (grid.GetNode (0,0));
    //consumerNodes.Add (grid.GetNode (1,0));
    //consumerNodes.Add (grid.GetNode (1,2));


    // Install CCNx applications
    std::string prefix = "/prefix";

    /**ZhangYu 2013-12-19
     * Zipf定义了不同ContentObject的数量，使得请求的Content的ID可能存在于Cache中，而在Cbr中，consumer请求的每个content都不相同，HitCache永远为0
    * 应该等效于没有Cache的情况，2013-12-19号下载的新的ndnSIM存放与ndnSIM-1目录中，在cs中有content-store-nocache.cc。可以仿真NoCache的情况
    * 在这个版本中，没有nocache，我曾经修改过empty-policy.h来试图实现nocahe，想把cachesize设置为真正的0，而不是unlimite，但是需要改得较多，未完成放弃
    * 但是如果使用consumerHelper中的 ConsumerCbr，那么就使得每个请求的ContentID都不同，等效于NoCache，似乎结果是一样的。
    */

    //ndn::AppHelper consumerHelper ("ns3::ndn::ConsumerZipfMandelbrot");
    //ZhangYu 2013-10-6 这是关系到hitCache的重要参数，在文章"Optimal Cache Allocation for  Content-Centric Networking" 中有讨论
    //如果下面的NumberOfContents的值设置为10，那么在3x3的grid中，加上--vis可以看出表示流量的绿线很快就没了，估计是每个节点都有了Cache，不需要流量了。
    //consumerHelper.SetAttribute ("NumberOfContents", StringValue ("10")); // 10 different contents
    //consumerHelper.SetAttribute ("Randomize", StringValue ("uniform")); // 100 interests a second

    ndn::AppHelper consumerHelper ("ns3::ndn::ConsumerCbr");

    consumerHelper.SetPrefix (prefix);
    consumerHelper.SetAttribute ("Frequency", StringValue ("1")); // 100 interests a second

    consumerHelper.Install (consumerNodes);

    //ZhangYu 2013-12-30, 添加多个consumer和producer
    //consumerHelper.SetPrefix("/prefix");
    //consumerHelper.Install(grid.GetNode(2,0));


    // Getting containers for the consumer/producer
    //Ptr<Node> producer = grid.GetNode (aRowNodes-1, aRowNodes-1);
    NodeContainer producerNodes;

    producerNodes.Add (grid.GetNode(aRowNodes-1, aRowNodes-1));
    //producerNodes.Add (grid.GetNode(aRowNodes-1, aRowNodes-2));


    ndn::AppHelper producerHelper ("ns3::ndn::Producer");

    //prefix="prefix for producer";
    producerHelper.SetPrefix (prefix);
    producerHelper.SetAttribute ("PayloadSize", StringValue("100"));
    //producerHelper.Install (producer);
    //ccnxGlobalRoutingHelper.AddOrigins (prefix, producer);
    ccnxGlobalRoutingHelper.AddOrigins (prefix, producerNodes);
    producerHelper.Install (producerNodes);
    
    //ZhangYu 2013-12-30, 添加多个consumer和producer
    producerHelper.SetPrefix("/prefixtwo");
    producerHelper.SetAttribute ("PayloadSize", StringValue("100"));
    //ccnxGlobalRoutingHelper.AddOrigins ("/prefixtwo", grid.GetNode(aRowNodes-1, aRowNodes-2));
    //producerHelper.Install(grid.GetNode(aRowNodes-1, aRowNodes-2));

    // Calculate and install FIBs
    //ccnxGlobalRoutingHelper.CalculateRoutes ();
    //ndn::GlobalRoutingHelper::CalculateAllPossibleRoutes();
    //ndn::GlobalRoutingHelper::CalculateZYMultiPathRoutes();
    ndn::GlobalRoutingHelper::CalculateNoCommLinkMultiPathRoutes();
    //ndn::GlobalRoutingHelper::BackupRestoreOriginalMetrics("Backup&Initial");


    //ZhangYu Add the trace

    boost::tuple< boost::shared_ptr<std::ostream>, std::list<Ptr<ndn::CsTracer> > >
    csTracers = ndn::CsTracer::InstallAll ("cs-trace.txt", Seconds (1));


    Simulator::Stop (Seconds (20.0));

    boost::tuple< boost::shared_ptr<std::ostream>, std::list<Ptr<ndn::L3AggregateTracer> > >  aggTracers = ndn::L3AggregateTracer::InstallAll ("aggregate-trace.txt", Seconds (0.5));

    boost::tuple< boost::shared_ptr<std::ostream>, std::list<Ptr<ndn::L3RateTracer> > >
      rateTracers = ndn::L3RateTracer::InstallAll ("rate-trace.txt", Seconds (10));

    Simulator::Run ();
    Simulator::Destroy ();

    return 0;
}

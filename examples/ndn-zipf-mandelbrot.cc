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

  // Creating 3x3 topology
  PointToPointHelper p2p;
  PointToPointGridHelper grid (3, 3, p2p);
  grid.BoundingBox(100,100,200,200);

  // Install CCNx stack on all nodes
  ndn::StackHelper ccnxHelper;
  ccnxHelper.SetForwardingStrategy ("ns3::ndn::fw::SmartFlooding");
  //ccnxHelper.SetForwardingStrategy ("ns3::ndn::fw::BestRoute");
  ccnxHelper.SetContentStore ("ns3::ndn::cs::Lru", "MaxSize", "10");
  ccnxHelper.InstallAll ();

  // Installing global routing interface on all nodes
  //ndn::CbisGlobalRoutingHelper ccnxGlobalRoutingHelper;
  ndn::GlobalRoutingHelper ccnxGlobalRoutingHelper;
  ccnxGlobalRoutingHelper.InstallAll ();

  // Getting containers for the consumer/producer
  Ptr<Node> producer = grid.GetNode (2, 2);
  //NodeContainer producerNodes;
  //producerNodes.Add (grid.GetNode(2,2));

  NodeContainer consumerNodes;
  consumerNodes.Add (grid.GetNode (0,0));

  // Install CCNx applications
  std::string prefix = "/prefix";

  ndn::AppHelper consumerHelper ("ns3::ndn::ConsumerZipfMandelbrot");
  //ndn::AppHelper consumerHelper ("ns3::ndn::ConsumerCbr");
  consumerHelper.SetPrefix (prefix);
  consumerHelper.SetAttribute ("Frequency", StringValue ("100")); // 100 interests a second
  
  //ZhangYu 2013-10-6 这是关系到hitCache的重要参数，在文章"Optimal Cache Allocation for  Content-Centric Networking" 中有讨论
  //如果下面的NumberOfContents的值设置为10，那么在3x3的grid中，加上--vis可以看出表示流量的绿线很快就没了，现在还不能说出为什么， 2013-10-19
  consumerHelper.SetAttribute ("NumberOfContents", StringValue ("10")); // 10 different contents
  //consumerHelper.SetAttribute ("Randomize", StringValue ("uniform")); // 100 interests a second
  consumerHelper.Install (consumerNodes);

  ndn::AppHelper producerHelper ("ns3::ndn::Producer");
  producerHelper.SetPrefix (prefix);
  producerHelper.SetAttribute ("PayloadSize", StringValue("100"));
  producerHelper.Install (producer);
  ccnxGlobalRoutingHelper.AddOrigins (prefix, producer);
  //producerHelper.Install (producerNodes);
  //ccnxGlobalRoutingHelper.AddOrigins (prefix, producerNodes);
  // Calculate and install FIBs
  ccnxGlobalRoutingHelper.CalculateRoutes ();

  //ZhangYu Add the trace

   boost::tuple< boost::shared_ptr<std::ostream>, std::list<Ptr<ndn::CsTracer> > >
   aggTracers = ndn::CsTracer::InstallAll ("cs-trace.txt", Seconds (10));


  Simulator::Stop (Seconds (1.0));

  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}

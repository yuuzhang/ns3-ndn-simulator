/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil -*- */
/*
 * Copyright (c) 2011 UCLA
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
 */
//-----ZhangYu 2013-12-29 为了修改MultiPath添加的头文件
#include "ns3/ndn-app-helper.h"
#include "ns3/ndnSIM-module.h"
#include "ns3/ndn-app.h"
using namespace ns3;
//---------

#include "ndn-global-routing-helper.h"

#include "ns3/ndn-l3-protocol.h"
#include "../model/ndn-net-device-face.h"
#include "../model/ndn-global-router.h"
#include "ns3/ndn-name.h"
#include "ns3/ndn-fib.h"

#include "ns3/node.h"
#include "ns3/node-container.h"
#include "ns3/net-device.h"
#include "ns3/channel.h"
#include "ns3/log.h"
#include "ns3/assert.h"
#include "ns3/names.h"
#include "ns3/node-list.h"
#include "ns3/channel-list.h"
#include "ns3/object-factory.h"

#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>
#include <boost/concept/assert.hpp>
// #include <boost/graph/graph_concepts.hpp>
// #include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
//in order to analysis the shortest paths algorithms, use modified hpp,change the folder from
// /usr/local/include to ndnSIM/boost_1_53_0/boost

#include "boost-graph-ndn-global-routing-helper.h"

#include <math.h>

NS_LOG_COMPONENT_DEFINE ("ndn.GlobalRoutingHelper");

using namespace std;
using namespace boost;

namespace ns3 {
namespace ndn {

void
GlobalRoutingHelper::Install (Ptr<Node> node)
{
  NS_LOG_LOGIC ("Node: " << node->GetId ());

  Ptr<L3Protocol> ndn = node->GetObject<L3Protocol> ();
  NS_ASSERT_MSG (ndn != 0, "Cannot install GlobalRoutingHelper before Ndn is installed on a node");

  Ptr<GlobalRouter> gr = node->GetObject<GlobalRouter> ();
  if (gr != 0)
    {
      NS_LOG_DEBUG ("GlobalRouter is already installed: " << gr);
      return; // already installed
    }

  gr = CreateObject<GlobalRouter> ();
  node->AggregateObject (gr);

  for (uint32_t faceId = 0; faceId < ndn->GetNFaces (); faceId++)
    {
      Ptr<NetDeviceFace> face = DynamicCast<NetDeviceFace> (ndn->GetFace (faceId));
      if (face == 0)
	{
	  NS_LOG_DEBUG ("Skipping non-netdevice face");
	  continue;
	}

      Ptr<NetDevice> nd = face->GetNetDevice ();
      if (nd == 0)
	{
	  NS_LOG_DEBUG ("Not a NetDevice associated with NetDeviceFace");
	  continue;
	}

      Ptr<Channel> ch = nd->GetChannel ();

      if (ch == 0)
	{
	  NS_LOG_DEBUG ("Channel is not associated with NetDevice");
	  continue;
	}

      if (ch->GetNDevices () == 2) // e.g., point-to-point channel
	{
	  for (uint32_t deviceId = 0; deviceId < ch->GetNDevices (); deviceId ++)
	    {
	      Ptr<NetDevice> otherSide = ch->GetDevice (deviceId);
	      if (nd == otherSide) continue;

	      Ptr<Node> otherNode = otherSide->GetNode ();
	      NS_ASSERT (otherNode != 0);

	      Ptr<GlobalRouter> otherGr = otherNode->GetObject<GlobalRouter> ();
	      if (otherGr == 0)
		{
		  Install (otherNode);
		}
	      otherGr = otherNode->GetObject<GlobalRouter> ();
	      NS_ASSERT (otherGr != 0);
	      gr->AddIncidency (face, otherGr);
	    }
	}
      else
	{
	  Ptr<GlobalRouter> grChannel = ch->GetObject<GlobalRouter> ();
	  if (grChannel == 0)
	    {
	      Install (ch);
	    }
	  grChannel = ch->GetObject<GlobalRouter> ();

	  gr->AddIncidency (0, grChannel);
	}
    }
}

void
GlobalRoutingHelper::Install (Ptr<Channel> channel)
{
  NS_LOG_LOGIC ("Channel: " << channel->GetId ());

  Ptr<GlobalRouter> gr = channel->GetObject<GlobalRouter> ();
  if (gr != 0)
    return;

  gr = CreateObject<GlobalRouter> ();
  channel->AggregateObject (gr);

  for (uint32_t deviceId = 0; deviceId < channel->GetNDevices (); deviceId ++)
    {
      Ptr<NetDevice> dev = channel->GetDevice (deviceId);

      Ptr<Node> node = dev->GetNode ();
      NS_ASSERT (node != 0);

      Ptr<GlobalRouter> grOther = node->GetObject<GlobalRouter> ();
      if (grOther == 0)
	{
	  Install (node);
	}
      grOther = node->GetObject<GlobalRouter> ();
      NS_ASSERT (grOther != 0);

      gr->AddIncidency (0, grOther);
    }
}

void
GlobalRoutingHelper::Install (const NodeContainer &nodes)
{
  for (NodeContainer::Iterator node = nodes.Begin ();
       node != nodes.End ();
       node ++)
    {
      Install (*node);
    }
}

void
GlobalRoutingHelper::InstallAll ()
{
  Install (NodeContainer::GetGlobal ());
}


void
GlobalRoutingHelper::AddOrigin (const std::string &prefix, Ptr<Node> node)
{
  Ptr<GlobalRouter> gr = node->GetObject<GlobalRouter> ();
  NS_ASSERT_MSG (gr != 0,
		 "GlobalRouter is not installed on the node");

  Ptr<Name> name = Create<Name> (boost::lexical_cast<Name> (prefix));
  gr->AddLocalPrefix (name);
}

void
GlobalRoutingHelper::AddOrigins (const std::string &prefix, const NodeContainer &nodes)
{
  //一般是在Main函数中调用  ccnxGlobalRoutingHelper.AddOrigins (prefix, producer) ，执行下面的两句运行结果是 1，8，只给producer节点添加了 prefix
    for (NodeContainer::Iterator node = nodes.Begin ();
       node != nodes.End ();
       node++)
    {

      Ptr<GlobalRouter> gr = (*node)->GetObject<GlobalRouter> ();
      NS_LOG_DEBUG("ZhangYu 2013-5-20 Add Origins " << gr->GetId() << endl);
      AddOrigin (prefix, *node);
    }
}

void
GlobalRoutingHelper::AddOrigin (const std::string &prefix, const std::string &nodeName)
{
  Ptr<Node> node = Names::Find<Node> (nodeName);
  NS_ASSERT_MSG (node != 0, nodeName << "is not a Node");

  AddOrigin (prefix, node);
}

void
GlobalRoutingHelper::AddOriginsForAll ()
{
  for (NodeList::Iterator node = NodeList::Begin (); node != NodeList::End (); node ++)
    {
      Ptr<GlobalRouter> gr = (*node)->GetObject<GlobalRouter> ();
      string name = Names::FindName (*node);

      if (gr != 0 && !name.empty ())
        {
          AddOrigin ("/"+name, *node);
        }
    }
}

void
GlobalRoutingHelper::ZYmodifyEdgeMetric()
{
  BOOST_CONCEPT_ASSERT(( IncidenceGraphConcept< NdnGlobalRouterGraph > ));

  NdnGlobalRouterGraph graph;

  for (NodeList::Iterator node=NodeList::Begin();node!=NodeList::End();node++)
  {
    //这部分代码添加是为了设置边的权重，跳过了前面的拓扑定义中或者其他地方关于metric的设置，只是为了直接简单，因为毕竟CaculateRoutes在整个仿真过程中之运行一次
	Ptr<GlobalRouter> source = (*node)->GetObject<GlobalRouter> ();
        if (source == 0)  //注意这里不是判断的节点0,不是source->GetId()==0
          {
            NS_LOG_DEBUG ("Node " << (*node)->GetId () << " does not export GlobalRouter interface");
            continue;
          }
        /* ZhangYu 2013-5-10 in the file boost-graph-ndn-global..,define the following
         * typedef ns3::ndn::GlobalRouter::Incidency edge_descriptor in relax.hpp
         * 遍历当前节点的所有边，
         */
        ns3::ndn::GlobalRouter::IncidencyList edges;
        ns3::ndn::GlobalRouter::Incidency edge;
        edges=source->GetIncidencies();
        //edge= edges.front();

        typedef property_map<NdnGlobalRouterGraph, edge_weight_t>::type WeightMap;
        typedef property_traits<WeightMap>::value_type W;
        property_traits<EdgeWeights>::reference b;

        const WeightMap& weightmap = get(edge_weight, graph);

    BOOST_FOREACH(ns3::ndn::GlobalRouter::Incidency edge, edges)
      {
    	W w_e = get(weightmap, edge);

    	//2013-11-7,通过下面的语句可以更改两个节点间的双向链路，从而控制后面的Dijkstra的计算结果，例如把3x3中的节点1和节点4之间的metric改为2，使得节点4的父节点从1变为3
    	if((source->GetObject<Node>()->GetId()==0) && ((edge.get<2>())->GetObject<Node>()->GetId()==3))
    	(edge.get<1>())->SetMetric(3);
    	if((source->GetObject<Node>()->GetId()==3) && ((edge.get<2>())->GetObject<Node>()->GetId()==0))
        (edge.get<1>())->SetMetric(3);

        std::string fromNode, toNode;
        fromNode=Names::FindName(source->GetObject<Node>());
        toNode=Names::FindName((edge.get<2>())->GetObject<Node>());
        if((fromNode=="Node1") && (toNode=="Node0"))
          (edge.get<1>())->SetMetric(4);
        if((fromNode=="Node0") && (toNode=="Node1"))
          (edge.get<1>())->SetMetric(4);
        //boost::get<1>(source->GetIncidencies().front())->SetMetric(2);
        NS_LOG_DEBUG("ZhangYu 2013-10-24, fromNode: " << fromNode  << "  toNode: " << toNode << "  Metric: " << (edge.get<1>())->GetMetric());

        w_e = get(weightmap, edge);
        //NS_LOG_DEBUG("ZhangYu2013-5-15,  " <<source->GetId() << " " << *w_e.get<0>() << "metric:" <<  w_e.get<1>());

        /*使用下面的语句进行赋值不报错，但是赋值完后，重新执行w_e=get(weightmap,edge）后，之前的值全部丢失，说明这种赋值语句并没有修改weightmap
         * 本来不理解为啥metric是定义在Face中的，可以通过SetMetric来修改更改，发现在property_traits<WeightMap>::value_type中，
         * 定义了Face, uint_16, double，当使用get(weightmap,edge)时，调用在boost-graph-ndn-global-routing-helper.h里的get函数，
         * 返回了edge.get<1>()->GetMetric作为了value_type中的第2项
         */
        //w_e.get<1>()=15; w_e.get<2>()=25.0;

        //NS_LOG_DEBUG("ZhangYu2013-5-9,  " << (boost::get<1>(edge))->GetMetric()<< "   " << boost::get<0>(w_e)<<"   " << boost::get<1>(w_e)<< "   " << boost::get<2>(w_e));
        //NS_LOG_DEBUG("ZhangYu2013-5-9,  " << b.get<1>() << "   " << boost::get<0>(w_e)<<"   " << boost::get<1>(w_e)<< "   " << boost::get<2>(w_e));

        //NS_LOG_DEBUG("ZhangYu2013-5-1, distancesdfgh:asd  " << sizeof(weightmap) << sizeof(zyweightmap) << sizeof(graph));
      }
  }
}

std::vector <std::vector<uint16_t> >  originalMetric;  	//注意这里的> >之间要有空格，否则error: ‘>>’ should be ‘> >’ within a nested template argument list

/*
 * 在这个函数中，执行的是把图中所有的节点端口的Metric备份到 originalMetric或者从其中恢复。在备份的过程中有清空节点fib的语句。
 * 这个清空fib的语句似乎不应该放在备份和恢复函数中，但是因为是从代码直接挪出来的，所以暂时这样 2014-2-1
 */
void
    BackupRestoreOrignalMetrics(const std::string action)
    {
        BOOST_CONCEPT_ASSERT(( VertexListGraphConcept< NdnGlobalRouterGraph > ));
        BOOST_CONCEPT_ASSERT((IncidenceGraphConcept<NdnGlobalRouterGraph>));
        NdnGlobalRouterGraph graph;
        typedef graph_traits<NdnGlobalRouterGraph>::vertex_descriptor vertex_descriptor;
        
        //保存所有边的OriginalMetric，因为找不到graph的一个边的集合，所以还是根据节点来遍历
        originalMetric.resize(NodeList::GetNNodes());

        for(NodeList::Iterator node = NodeList::Begin (); node != NodeList::End (); node++)
        {
            int nodeId=(*node)->GetId();
            Ptr<GlobalRouter> source = (*node)->GetObject<GlobalRouter> ();
            if (source == 0)
            {
                NS_LOG_DEBUG ("Node " << (*node)->GetId () << " does not export GlobalRouter interface");
                continue;
            }

            Ptr<Fib>  fib  = source->GetObject<Fib> ();   //只有这里获取fib，后面添加 Entry
            NS_ASSERT (fib != 0);
            if (action=="Backup&Initial")
            {
			//NS_LOG_DEBUG("ZhangYu ==================================================================================================");
			fib->InvalidateAll ();  //2014-1-8，这一句最终调用的是fib-entry.cc中的，把一个节点的所有端口都设置为如：dev[2]=net(1,2-5)(65535,r,1) 后面的65535，r（表示RED）
			//因为没有只在Backup中执行，在每个节点进行多路径计算时，要恢复一下 metric，如果也运行上面InvaliateAll，会导致前面节点计算后添加的fib entry变为 65535,r。从而出错
            }
            Ptr<L3Protocol> l3 = source->GetObject<L3Protocol> ();
            NS_ASSERT (l3 != 0);

            // remember interface statuses
            originalMetric[nodeId].resize(l3->GetNFaces ());

            for (uint32_t faceId = 0; faceId < l3->GetNFaces (); faceId++)
            {
                if (action=="Restore")
                {
                    l3->GetFace (faceId)->SetMetric (originalMetric[nodeId][faceId]); // value std::numeric_limits<int16_t>::max () MUST NOT be used (reserved)
                    //NS_LOG_DEBUG("ZhangYu 2014-1-2 ===Restore====faceId: " <<faceId << "  Metric: "<< l3->GetFace (faceId)->GetMetric () );
                }
                else if ((action=="Backup")||(action=="Backup&Initial"))
                {
                	originalMetric[nodeId][faceId] = l3->GetFace (faceId)->GetMetric ();
                    //NS_LOG_DEBUG("ZhangYu 2014-1-2 =========================================nodeId: " <<nodeId << "    faceId: " <<faceId << "  Metric: "<< originalMetric[nodeId][faceId] );
                    //NS_LOG_DEBUG("ZhangYu ===============================================" << l3->GetFace (faceId)->GetInstanceTypeId());

                    }
                else
                	NS_LOG_DEBUG("ZhangYu  input a wrong action string for function BackupRestoreOriginalMetrics");
            }

        }
    }
void
    GlobalRoutingHelper::CalculateNoCommLinkMultiPathRoutes(Ptr<Node> srcNode, Ptr<Node> desNode,Ptr<Name> &prefix1)
    {
        uint32_t  multipathNumber=3;    //共计算几条多路径

        BOOST_CONCEPT_ASSERT(( VertexListGraphConcept< NdnGlobalRouterGraph > ));
        BOOST_CONCEPT_ASSERT((IncidenceGraphConcept<NdnGlobalRouterGraph>));
        NdnGlobalRouterGraph graph;
        typedef graph_traits<NdnGlobalRouterGraph>::vertex_descriptor vertex_descriptor;

        BackupRestoreOrignalMetrics("Backup");

        Ptr<GlobalRouter> source = (srcNode)->GetObject<GlobalRouter>();


        //NS_LOG_DEBUG("ZhangYu 2014-1-1 is consumer node Id: " << (*node)->GetId() <<" " << (appTypeStr.find("Consumer")) <<"'  "<< appTypeStr);
        //NS_LOG_DEBUG ("===== Reachability from source Node: " << source->GetObject<Node> ()->GetId () << " (" << Names::FindName (source->GetObject<Node> ()) << ")");


        //计算包含这个consumer的节点到其他节点的最短路
        for(uint32_t pathIndex=0; pathIndex<multipathNumber;pathIndex++)
        {
            DistancesMap    distances;
            PredecessorsMap predecessors;

            dijkstra_shortest_paths (graph, source,
                                     predecessor_map (boost::ref(predecessors))
                                     .
                                     distance_map (boost::ref(distances))
                                     .
                                     distance_inf (WeightInf)
                                     .
                                     distance_zero (WeightZero)
                                     .
                                     distance_compare (boost::WeightCompare ())
                                     .
                                     distance_combine (boost::ZYWeightCombine ())
                                     );

            DistancesMap::iterator des=distances.find(desNode->GetObject<GlobalRouter>());
            BOOST_FOREACH (const Ptr<const Name> &prefix, des->first->GetLocalPrefixes ())
            {
                Ptr<GlobalRouter> curNode =des->first ;
                Ptr<GlobalRouter> preNode;
                NS_LOG_DEBUG("ZhangYu 2014-1-7 producer Node: " << curNode->GetObject<Node>()->GetId() << std::endl);
                
                while (curNode!=source) //回溯到源节点，添加fib，修改链路
                {
                    preNode=predecessors[curNode];
                    Ptr<Fib> fib  = preNode->GetObject<Fib> ();   //这里获取fib，后面添加 Entry
                    NS_ASSERT (fib != 0);   //https://www.nsnam.org/doxygen/group__assert.html#details
                    
                    if(uint16_t( des->second.get<1>()-distances[curNode].get<1> ())>= std::numeric_limits<uint16_t>::max()-1)
                    {
                        std::cout << "ZhangYu 2014-1-8 我认为不应该出现这种情况，出现了是有逻辑错误" << std::endl << std::endl;
                        continue;
                    }
                    Ptr<fib::Entry> entry = fib->Add (prefix, distances[curNode].get<0> (),  des->second.get<1>()-distances[preNode].get<1> ());
                    NS_LOG_DEBUG("ZhangYu 2014-1-8 *entry: " << *entry);
                    
                    entry->SetRealDelayToProducer (distances[curNode].get<0> (), Seconds (des->second.get<2>()-distances[preNode].get<2>()));
                    
                    Ptr<Limits> faceLimits = distances[curNode].get<0> ()->GetObject<Limits> ();
                    Ptr<Limits> fibLimits = entry->GetObject<Limits> ();
                    if (fibLimits != 0)
                    {
                        // if it was created by the forwarding strategy via DidAddFibEntry event
                        fibLimits->SetLimits (faceLimits->GetMaxRate (), 2 *  (des->second.get<2>()-distances[preNode].get<2>())/*exact RTT*/);
                        NS_LOG_DEBUG ("Set limit for prefix " << *prefix << " " << faceLimits->GetMaxRate () << " / " <<
                                      2* (des->second.get<2>()-distances[preNode].get<2>()) << "s (" << faceLimits->GetMaxRate () * 2 *  (des->second.get<2>()-distances[preNode].get<2>())<< ")");
                    }
                    
                    //前面执行完了回溯路径，添加fib，后面的是把这条路径上的Link设置为不可用
                    //更改边的代价时，可以参考CaculateAllPossibleRoutes中的l3->GetFace (faceId),这里更简单的是使用distances[curNode].get<0>()，一样的类型
                    distances[curNode].get<0>()->SetMetric(std::numeric_limits<int16_t>::max ()-1); // value std::numeric_limits<int16_t>::max () MUST NOT be used (reserved)
                    
                    curNode=preNode;
                }
            }
            
        }
        //恢复originalMetric
        BackupRestoreOrignalMetrics("Restore");
        NS_LOG_DEBUG("ZhangYu 2014-1-6 =========================================================================end of CalculateNoCommLinkMultiPathRoutes");
    }

void
    GlobalRoutingHelper::CalculateNoCommLinkMultiPathRoutes()
    {
        uint32_t  multipathNumber=3;    //共计算几条多路径
        
        BOOST_CONCEPT_ASSERT(( VertexListGraphConcept< NdnGlobalRouterGraph > ));
        BOOST_CONCEPT_ASSERT((IncidenceGraphConcept<NdnGlobalRouterGraph>));
        NdnGlobalRouterGraph graph;
        typedef graph_traits<NdnGlobalRouterGraph>::vertex_descriptor vertex_descriptor;

        BackupRestoreOrignalMetrics("Backup&Initial");
        /*ZhangYu 2013-12-31 上面的语句得到的是 ns3::ndn::ConsumerCbr, ns3::Application   ns3::ndn::Producer ns3::Application，为了实现noComLinkMultiPath，要选择consumer节点才进行最短路径的计算
         * 计算出来后一次为所有Path上的节点都添加Fib，这样可以省去为无关的节点也计算最短路，计算一次才为当前计算的节点添加Fib。
         * 为了选择consumer节点，一种方式是在global-routing中(*node)->GetApplication(appId)->GetInstanceTypeId()判断节点的类型，根据字符串开头是consumer的，虽然可以考虑给Node再增加一个属性用来
         * 区分是consumer，这样可以挑出是consumer的节点来进行计算。现有的路由计算中，对source缩小范围，只对属于consumer的source进行计算，所以设置一个属性，不考虑producer，也暂时不考虑一个节点装在了多个consumer的情况
         * 但是增加一个node的属性，需要直接修改NS3代码中的node.cc，影响可能大，所以放弃，只是靠
         */
        for (NodeList::Iterator node = NodeList::Begin (); node != NodeList::End (); node++)
        {
           Ptr<GlobalRouter> source = (*node)->GetObject<GlobalRouter> ();
            if (source == 0)
            {
                NS_LOG_DEBUG ("Node " << (*node)->GetId () << " does not export GlobalRouter interface");
                continue;
            }
            //开始计算最短路
            for(uint32_t appId=0; appId<(*node)->GetNApplications();appId++)
            {
                std::string appTypeStr= (*node)->GetApplication(appId)->GetInstanceTypeId().GetName();
                if(std::string::npos!= appTypeStr.find("Consumer"))
                {
                   	//NS_LOG_DEBUG("ZhangYu 2014-1-1 is consumer node Id: " << (*node)->GetId() <<" " << (appTypeStr.find("Consumer")) <<"'  "<< appTypeStr);
                    NS_LOG_DEBUG ("===== Reachability from source Node: " << source->GetObject<Node> ()->GetId () << " (" << Names::FindName (source->GetObject<Node> ()) << ")");
                    

                    //计算包含这个consumer的节点到其他节点的最短路
                    for(uint32_t pathIndex=0; pathIndex<multipathNumber;pathIndex++)
                    {
                        DistancesMap    distances;
                        PredecessorsMap predecessors;
                        dijkstra_shortest_paths (graph, source,
                                                 predecessor_map (boost::ref(predecessors))
                                                 .
                                                 distance_map (boost::ref(distances))
                                                 .
                                                 distance_inf (WeightInf)
                                                 .
                                                 distance_zero (WeightZero)
                                                 .
                                                 distance_compare (boost::WeightCompare ())
                                                 .
                                                 distance_combine (boost::ZYWeightCombine ())
                                                 );
                        NS_LOG_DEBUG("ZhangYu 2014-2-7 pathIndex: " << pathIndex << endl);
                        for(PredecessorsMap::iterator i=predecessors.begin();i!=predecessors.end();i++)
                        {
                            //NS_LOG_DEBUG("ZhangYu 2013-5-21 predecessors node: " << i->first->GetObject<Node>()->GetId()  <<"  ParentNode: " <<i->second->GetObject<Node>()->GetId());
                        }

                        for (DistancesMap::iterator i = distances.begin (); i != distances.end (); i++)
                        {
                            if (i->first == source)
                                continue;
                            else
                            {
                                if (i->second.get<0> () == 0)
                                {
                                    cout <<"  Node " << i->first->GetObject<Node> ()->GetId () << " is unreachable" << endl;
                                }
                                else
                                {
                                    NS_LOG_DEBUG("ZhangYu 2014-1-3, Node:" << i->first->GetObject<Node>()->GetId()<< "   face:" << *i->second.get<0>()<<"  with distance:" <<i->second.get<1>());
                                    
                                    //下面的语句使得为每个producer的节点的每个应用添加路由fibs，为0就不循环，一个节点有多个Apps时循环（这里循环执行有点冗余，因为步骤一样，只是prefix不同，但是为了代码清爽，就这样了）
                                    NS_LOG_DEBUG("ZhangYu 2014-2-7 i->first->GetLocalPrefixes.size(): " <<i->first->GetLocalPrefixes().size());

                                    BOOST_FOREACH (const Ptr<const Name> &prefix, i->first->GetLocalPrefixes ())
                                    {
                                        Ptr<GlobalRouter> curNode =i->first ;
                                        Ptr<GlobalRouter> preNode;
                                        NS_LOG_DEBUG("ZhangYu 2014-1-7 producer Node: " << curNode->GetObject<Node>()->GetId() );

                                        while (curNode!=source)
                                        {
                                            preNode=predecessors[curNode];
                                            NS_LOG_DEBUG("ZhangYu  2014-1-5 prefix: " << *prefix << "  Node: " << preNode->GetObject<Node>()->GetId()
                                                         << "  reachable via face: " << *distances[curNode].get<0>()
                                                         << "  with distance: " << i->second.get<1>()-distances[preNode].get<1>()
                                                         << "  with delay " << distances[curNode].get<2>());

                                            
                                            Ptr<Fib> fib  = preNode->GetObject<Fib> ();   //这里获取fib，后面添加 Entry
                                            //ZhangYu 2014-1-6，下面的这一句使得每个节点的所有出口都变成值最大，导致传播消息时出错
                                            //fib->InvalidateAll ();
                                            NS_ASSERT (fib != 0);   //2014-1-9现在还不清楚是否可以去掉这句

                                            if(uint16_t( i->second.get<1>()-distances[curNode].get<1> ())== std::numeric_limits<uint16_t>::max()-1)
                                              {
                                                std::cout << "ZhangYu 2014-1-8 我认为不应该出现这种情况，出现了是有逻辑错误" << std::endl << std::endl;
                                              continue;
                                              }
                                        	Ptr<Name> temp=Create<Name> (boost::lexical_cast<string>(*prefix)+"/"+boost::lexical_cast<string>(pathIndex));
                                            //const Ptr<const Name> temp=Create<Name> (boost::lexical_cast<string>(*prefix));
                                            NS_LOG_DEBUG("ZhangYu 2014-1-8 temp: " << *temp);

                                            NS_LOG_DEBUG("ZhangYu 2014-3-19 distances[curNode].get<0>: " << *distances[curNode].get<0>());
                                            NS_LOG_DEBUG("ZhangYu 2014-3-19 i->second.get<1>(): " << i->second.get<1>());
                                            NS_LOG_DEBUG("ZhangYu 2014-3-19 distances[preNode].get<1>(): " << distances[preNode].get<1>());
                                            //Ptr<fib::Entry> entry = fib->Add (temp, distances[curNode].get<0> (),  i->second.get<1>()-distances[preNode].get<1> ());
                                            Ptr<fib::Entry> entry = fib->Add (prefix, distances[curNode].get<0> (),  i->second.get<1>()-distances[preNode].get<1> ());
                                            NS_LOG_DEBUG("ZhangYu 2014-1-8 *entry: " << *entry);

                                            entry->SetRealDelayToProducer (distances[curNode].get<0> (), Seconds (i->second.get<2>()-distances[preNode].get<2>()));

                                            Ptr<Limits> faceLimits = distances[curNode].get<0> ()->GetObject<Limits> ();
                                            Ptr<Limits> fibLimits = entry->GetObject<Limits> ();
                                            if (fibLimits != 0)
                                            {
                                                // if it was created by the forwarding strategy via DidAddFibEntry event
                                                fibLimits->SetLimits (faceLimits->GetMaxRate (), 2 *  (i->second.get<2>()-distances[preNode].get<2>())/*exact RTT*/);
                                                NS_LOG_DEBUG ("Set limit for prefix " << *prefix << " " << faceLimits->GetMaxRate () << " / " <<
                                                              2* (i->second.get<2>()-distances[preNode].get<2>()) << "s (" << faceLimits->GetMaxRate () * 2 *  (i->second.get<2>()-distances[preNode].get<2>())<< ")");
                                            }
                                            
                                            //前面执行完了回溯路径，添加fib，后面的是把这条路径上的Link设置为不可用
                                            //更改边的代价时，可以参考CaculateAllPossibleRoutes中的l3->GetFace (faceId),这里更简单的是使用distances[curNode].get<0>()，一样的类型
                                            //NS_LOG_DEBUG("ZhangYu 2014-1-9 distances[curNode].get<0>()->GetInstanceTypeId()=====" << distances[curNode].get<0>()->GetInstanceTypeId());
                                            distances[curNode].get<0>()->SetMetric(std::numeric_limits<uint16_t>::max ()); // value std::numeric_limits<int16_t>::max () MUST NOT be used (reserved)
                                            //distances[curNode].get<0>()->SetMetric(2000); // value std::numeric_limits<int16_t>::max () MUST NOT be used (reserved)

                                            NS_LOG_DEBUG("ZhangYu 2014-2-9 *entry: " << *entry);
                                            
                                            curNode=preNode;
                                        }
                                        std::cout << "ZhangYu 2014-3-15  predecessors: " << preNode->GetId() << "   source: " << source->GetId() << std::endl;
                                    }
                                }
                            }
                        }
                        
                    }

                    //恢复originalMetric
                    BackupRestoreOrignalMetrics("Restore");
                }
            }
        }
        NS_LOG_DEBUG("ZhangYu 2014-1-6 -end of CalculateNoCommLinkMultiPathRoutes");
    }
    
void
GlobalRoutingHelper::CalculateZYMultiPathRoutes ()
{
    
    /**
     * ZhangYu 2013-12-22, 在AllPossibleRoutes的基础上修改为自己的多路径算法。
     * CalculateAllPossibleRoutes ()的过程是分别以每个节点为 source node，计算其他节点到source的距离，但是只有在碰到装有prefix的节点（也就是producer）时，才为source添加Fib，是到producer的出口。
     * 根据
     */
    
    BOOST_CONCEPT_ASSERT(( VertexListGraphConcept< NdnGlobalRouterGraph > ));
    BOOST_CONCEPT_ASSERT(( IncidenceGraphConcept< NdnGlobalRouterGraph > ));
    
    NdnGlobalRouterGraph graph;
    typedef graph_traits < NdnGlobalRouterGraph >::vertex_descriptor vertex_descriptor;
    
    //ZYmodifyEdgeMetric();
    
    // For now we doing Dijkstra for every node.  Can be replaced with Bellman-Ford or Floyd-Warshall.
    for (NodeList::Iterator node = NodeList::Begin (); node != NodeList::End (); node++)
    {
        Ptr<GlobalRouter> source = (*node)->GetObject<GlobalRouter> ();
        if (source == 0)
        {
            NS_LOG_DEBUG ("Node " << (*node)->GetId () << " does not export GlobalRouter interface");
            continue;
        }

        Ptr<Fib>  fib  = source->GetObject<Fib> ();   //只有这里获取fib，后面添加 Entry
        fib->InvalidateAll ();
        NS_ASSERT (fib != 0);

        NS_LOG_DEBUG ("===== Reachability from source Node: " << source->GetObject<Node> ()->GetId () << " (" << Names::FindName (source->GetObject<Node> ()) << ")");

        Ptr<L3Protocol> l3 = source->GetObject<L3Protocol> ();
        NS_ASSERT (l3 != 0);
        
        //2013-12-31， 为了根据节点装在的App来确定是consumer还是producer
        NS_LOG_DEBUG("ZhangYu 2013-12-31 Applications:  " << (*node)->GetNApplications());

        if(source->GetObject<Node>()->GetId()==0)
        {
            /*
            * 2013-11-8 ZhangYu 计算所有可能的路径的方法是把当前source节点的所有Face（出口）都设置为最大，然后再分别设置为某一个Face为原来拓扑里的值后计算最短路径，
            * 这样就相当于为source计算了每个出口的不同路径。
            */
            // remember interface statuses
            std::vector<uint16_t> originalMetric (l3->GetNFaces ());
            for (uint32_t faceId = 0; faceId < l3->GetNFaces (); faceId++)
            {
              originalMetric[faceId] = l3->GetFace (faceId)->GetMetric ();
              l3->GetFace (faceId)->SetMetric (std::numeric_limits<int16_t>::max ()-1); // value std::numeric_limits<int16_t>::max () MUST NOT be used (reserved)
            }
            // 分别启用一个Face，然后计算最短路径
            for (uint32_t enabledFaceId = 0; enabledFaceId < l3->GetNFaces (); enabledFaceId++)
            {
                if (DynamicCast<ndn::NetDeviceFace> (l3->GetFace (enabledFaceId)) == 0)
                continue;

                // enabling only faceId
                l3->GetFace (enabledFaceId)->SetMetric (originalMetric[enabledFaceId]);

                DistancesMap    distances;
                PredecessorsMap predecessors;

                dijkstra_shortest_paths (graph, source,
                                       predecessor_map (boost::ref(predecessors))
                                       .
                                       distance_map (boost::ref(distances))
                                       .
                                       distance_inf (WeightInf)
                                       .
                                       distance_zero (WeightZero)
                                       .
                                       distance_compare (boost::WeightCompare ())
                                       .
                                       distance_combine (boost::WeightCombine ())
                                       );

                //NS_LOG_DEBUG("ZhangYu 2013-11-8 current enabledFace: " << DynamicCast<ndn::NetDeviceFace> (l3->GetFace (enabledFaceId))->GetInstanceTypeId());
                NS_LOG_DEBUG("ZhangYu 2013-11-12 current enabledFace: " << *l3->GetFace(enabledFaceId));
                for(PredecessorsMap::iterator i=predecessors.begin();i!=predecessors.end();i++)
                {
                    //NS_LOG_DEBUG("ZhangYu 2013-5-21 predecessors node: " << i->first->GetObject<Node>()->GetId()  <<"  ParentNode: " <<i->second->GetObject<Node>()->GetId());
                }
                for (DistancesMap::iterator i = distances.begin (); i != distances.end (); i++)
                {
                    if (i->first == source)
                        continue;
                    else
                    {
                        // cout << "  Node " << i->first->GetObject<Node> ()->GetId ();
                        if (i->second.get<0> () == 0)
                        {
                          // cout << " is unreachable" << endl;
                        }
                        else
                        {
                            //NS_LOG_DEBUG("ZhangYu 2013-12-22 i->first NodeId: " << i->first->GetObject<Node>()->GetId() << "  LocalPrefixes: " << i->first->GetLocalPrefixes().size() );
                            BOOST_FOREACH (const Ptr<const Name> &prefix, i->first->GetLocalPrefixes ())
                            {
                                NS_LOG_DEBUG (" prefix " << *prefix << " reachable via face " << *i->second.get<0> ()
                                            << " with distance " << i->second.get<1> ()
                                            << " with delay " << i->second.get<2> ());
                                //NS_LOG_DEBUG("ZhangYu 2013-11-12 " <<( i->second.get<0>())->GetTypeId());

                                if (i->second.get<0> ()->GetMetric () == std::numeric_limits<uint16_t>::max ()-1)
                                continue;

                                Ptr<fib::Entry> entry = fib->Add (prefix, i->second.get<0> (), i->second.get<1> ());
                                NS_LOG_DEBUG("---ZhangYu 2013-12-22 fib's Node: " << fib->GetObject<Node>()->GetId() << "  fib size: " << fib->GetSize());
                                entry->SetRealDelayToProducer (i->second.get<0> (), Seconds (i->second.get<2> ()));

                                Ptr<Limits> faceLimits = i->second.get<0> ()->GetObject<Limits> ();

                                Ptr<Limits> fibLimits = entry->GetObject<Limits> ();
                                if (fibLimits != 0)
                                {
                                  // if it was created by the forwarding strategy via DidAddFibEntry event
                                  fibLimits->SetLimits (faceLimits->GetMaxRate (), 2 * i->second.get<2> () /*exact RTT*/);
                                  NS_LOG_DEBUG ("Set limit for prefix " << *prefix << " " << faceLimits->GetMaxRate () << " / " <<
                                                2*i->second.get<2> () << "s (" << faceLimits->GetMaxRate () * 2 * i->second.get<2> () << ")");
                                }
                            }
                        }
                    }
                }
                //ZhangYu 2013-12-22，为了查看对Fib的修改，下面添加语句查看
                //NS_LOG_DEBUG("ZhangYu 2013-12-22 fib's Node: " << fib->GetObject<Node>()->GetId() << "  fib size: " << fib->GetSize());
                //BOOST_FOREACH()
                
                // disabling the face again
                l3->GetFace (enabledFaceId)->SetMetric (std::numeric_limits<uint16_t>::max ()-1);
            }
            // recover original interface statuses
            for (uint32_t faceId = 0; faceId < l3->GetNFaces (); faceId++)
            {
                l3->GetFace (faceId)->SetMetric (originalMetric[faceId]);
            }

        }
        else
        {
            //计算除去NodeId为0的点的路由
            
                DistancesMap    distances;
                PredecessorsMap predecessors;
                
                dijkstra_shortest_paths (graph, source,
                                         predecessor_map (boost::ref(predecessors))
                                         .
                                         distance_map (boost::ref(distances))
                                         .
                                         distance_inf (WeightInf)
                                         .
                                         distance_zero (WeightZero)
                                         .
                                         distance_compare (boost::WeightCompare ())
                                         .
                                         distance_combine (boost::WeightCombine ())
                                         );
                
                for(PredecessorsMap::iterator i=predecessors.begin();i!=predecessors.end();i++)
                {
                    //NS_LOG_DEBUG("ZhangYu 2013-5-21 predecessors node: " << i->first->GetObject<Node>()->GetId()  <<"  ParentNode: " <<i->second->GetObject<Node>()->GetId());
                }
                for (DistancesMap::iterator i = distances.begin (); i != distances.end (); i++)
                {
                    if (i->first == source)
                        continue;
                    else
                    {
                        // cout << "  Node " << i->first->GetObject<Node> ()->GetId ();
                        if (i->second.get<0> () == 0)
                        {
                            // cout << " is unreachable" << endl;
                        }
                        else
                        {
                            //NS_LOG_DEBUG("ZhangYu 2013-12-22 i->first NodeId: " << i->first->GetObject<Node>()->GetId() << "  LocalPrefixes: " << i->first->GetLocalPrefixes().size() );
                            BOOST_FOREACH (const Ptr<const Name> &prefix, i->first->GetLocalPrefixes ())
                            {
                                NS_LOG_DEBUG (" prefix " << *prefix << " reachable via face " << *i->second.get<0> ()
                                              << " with distance " << i->second.get<1> ()
                                              << " with delay " << i->second.get<2> ());
                                //NS_LOG_DEBUG("ZhangYu 2013-11-12 " <<( i->second.get<0>())->GetTypeId());
                                
                                if (i->second.get<0> ()->GetMetric () == std::numeric_limits<uint16_t>::max ()-1)
                                    continue;
                                
                                Ptr<fib::Entry> entry = fib->Add (prefix, i->second.get<0> (), i->second.get<1> ());
                                NS_LOG_DEBUG("---ZhangYu 2013-12-22 fib's Node: " << fib->GetObject<Node>()->GetId() << "  fib size: " << fib->GetSize());
                                entry->SetRealDelayToProducer (i->second.get<0> (), Seconds (i->second.get<2> ()));
                                
                                Ptr<Limits> faceLimits = i->second.get<0> ()->GetObject<Limits> ();
                                
                                Ptr<Limits> fibLimits = entry->GetObject<Limits> ();
                                if (fibLimits != 0)
                                {
                                    // if it was created by the forwarding strategy via DidAddFibEntry event
                                    fibLimits->SetLimits (faceLimits->GetMaxRate (), 2 * i->second.get<2> () /*exact RTT*/);
                                    NS_LOG_DEBUG ("Set limit for prefix " << *prefix << " " << faceLimits->GetMaxRate () << " / " <<
                                                  2*i->second.get<2> () << "s (" << faceLimits->GetMaxRate () * 2 * i->second.get<2> () << ")");
                                }
                            }
                        }
                    }
                }
        }
        //--for(NodeList..
    }
    NS_LOG_DEBUG("ZhangYu 2014-1-6 =========================================================================end of route: ");
}


void
GlobalRoutingHelper::CalculateRoutes ()
{
  /**
   * Implementation of route calculation is heavily based on Boost Graph Library
   * See http://www.boost.org/doc/libs/1_49_0/libs/graph/doc/table_of_contents.html for more details
   */

  //BOOST_CONCEPT_ASSERT(( VertexListGraphConcept< NdnGlobalRouterGraph > ));
  BOOST_CONCEPT_ASSERT(( IncidenceGraphConcept< NdnGlobalRouterGraph > ));

  NdnGlobalRouterGraph graph;
  typedef graph_traits < NdnGlobalRouterGraph >::vertex_descriptor vertex_descriptor;
  
  //ZhangYu 把原来里面修改全中的代码挪到一个函数中，因为CaculateRoutes是statics的，必须要属于一个对象，所以把ZYmodifyEdgeMetric也弄成静态的。这样能保证不用传值，修改的是同样的变量
  ZYmodifyEdgeMetric();

  // For now we doing Dijkstra for every node.  Can be replaced with Bellman-Ford or Floyd-Warshall.
  // Other algorithms should be faster, but they need additional EdgeListGraph concept provided by the graph, which
  // is not obviously how implement in an efficient manner
  for (NodeList::Iterator node = NodeList::Begin (); node != NodeList::End (); node++)
    {
      Ptr<GlobalRouter> source = (*node)->GetObject<GlobalRouter> ();
      if (source == 0)  //注意这里不是判断的节点0,不是source->GetId()==0
		{
		  NS_LOG_DEBUG ("Node " << (*node)->GetId () << " does not export GlobalRouter interface");
		  continue;
		}
      DistancesMap    distances;
      //为了搞清楚这个函数，花费了很长时间。2013-5-10，在其中有多个同名函数模板的调用，最终会出现在函数dijkstra_bfs_visitor的relax中，
      /*graph_traits<Graph>::edge_descriptor e
       * put( DistanceMap& d, v=target(e,g), combine( get(d, u), get(property_traits<WeightMap>::value_type & w_e=get(w,e)
       * 送给下面函数和边的权重相关的计算在 boost::WeightCombine中，根据property_traits< EdgeWeights >::reference b, b.get<1>()进行计算
       */
      PredecessorsMap predecessors;	//2013-5-21,为了获取最短路径，使用这个变量
      dijkstra_shortest_paths (graph, source,
			       predecessor_map (boost::ref(predecessors))
			       .
			       distance_map (boost::ref(distances))
			       .
			       //weight_map (weightmap)	//发现是否有这个参数关系不大，因为在combine中调用的时候，会获取
			       //.
			       distance_inf (WeightInf)
			       .
			       distance_zero (WeightZero)
			       .
			       distance_compare (boost::WeightCompare ())
			       .
			       distance_combine (boost::WeightCombine ())
			       );


      // NS_LOG_DEBUG (predecessors.size () << ", " << distances.size ());
      NS_LOG_DEBUG("ZhangYu2013-5-1, distances:" << distances.size());

      Ptr<Fib>  fib  = source->GetObject<Fib> ();
      fib->InvalidateAll ();
      NS_ASSERT (fib != 0);

      NS_LOG_DEBUG (endl << "Reachability from Node: " << source->GetObject<Node> ()->GetId ()<<endl);
      for(PredecessorsMap::iterator i=predecessors.begin();i!=predecessors.end();i++)
        {
          //NS_LOG_DEBUG("ZhangYu 2013-5-21 predecessors node: " << i->first->GetObject<Node>()->GetId() <<"  ParentNode: " <<i->second->GetObject<Node>()->GetId() );
          std::string fromNode, toNode;
          fromNode=Names::FindName(i->first->GetObject<Node>());
          toNode=Names::FindName(i->second->GetObject<Node>());
          /*下面的语句中，i->first得到的是GlobalRouter，如果GetId，得到是错误的节点Id，可以在拓扑定义txt文件中指定Node的systemId，然后就指定了节点的Id。但是
           * 节点的Id不同于GlobalRouter的Id，所以导致节点3和5是互换的。要得到正确的节点Id，应该first后->GetObject<Node>()->GetId，如下面的语句
           */
          NS_LOG_DEBUG("ZhangYu 2013-5-23 " << fromNode  << "(" << i->first->GetObject<Node>()->GetId() << ")  ParentNode: " <<toNode <<"(" <<i->second->GetObject<Node>()->GetId()<<")" );

        }
      for (DistancesMap::iterator i = distances.begin (); i != distances.end (); i++)
        {
          if (i->first == source)
            continue;
          else
            {
              //cout << "  Node " << i->first->GetObject<Node> ()->GetId ();
              if (i->second.get<0> () == 0)
                {
                   //cout << " is unreachable" << endl;
                  NS_LOG_DEBUG("ZhangYu 2013-5-19 Node: " <<i->first->GetId() << " is unreachable" <<endl);
                }
              else
                {
                              NS_LOG_DEBUG("ZhangYu 2013-5-21, Node:" << i->first->GetObject<Node>()->GetId()<< "   face:" << *i->second.get<0>()<<"  with distance:" <<i->second.get<1>());

                  BOOST_FOREACH (const Ptr<const Name> &prefix, i->first->GetLocalPrefixes ())
                    {
                        NS_LOG_DEBUG (" prefix " << prefix << " reachable via face " << *i->second.get<0> ()
                                    << " with distance " << i->second.get<1> ()
                                    << " with delay " << i->second.get<2> ());

                        Ptr<fib::Entry> entry = fib->Add (prefix, i->second.get<0> (), i->second.get<1> ());
                        entry->SetRealDelayToProducer (i->second.get<0> (), Seconds (i->second.get<2> ()));
                        NS_LOG_DEBUG("--ZhangYu 2014-1-6  ==================Seconds(i->second.get<2>()" << Seconds (i->second.get<2>()));

                        Ptr<Limits> faceLimits = i->second.get<0> ()->GetObject<Limits> ();

                        Ptr<Limits> fibLimits = entry->GetObject<Limits> ();
                        if (fibLimits != 0)
                        {
                          // if it was created by the forwarding strategy via DidAddFibEntry event
                          fibLimits->SetLimits (faceLimits->GetMaxRate (), 2 * i->second.get<2> () /*exact RTT*/);
                          NS_LOG_DEBUG ("Set limit for prefix " << *prefix << " " << faceLimits->GetMaxRate () << " / " <<
                                        2*i->second.get<2> () << "s (" << faceLimits->GetMaxRate () * 2 * i->second.get<2> () << ")");
                        }
                    }
                }
            }
        }

	//为了调试循环一次，看节点0计算后的记过，下面使用break打断后面的循环
	//break;
    }
    NS_LOG_DEBUG("ZhangYu 2014-1-6 =========================================================================end of route: ");
}

void
GlobalRoutingHelper::CalculateAllPossibleRoutes ()
{
  /**
   * Implementation of route calculation is heavily based on Boost Graph Library
   * See http://www.boost.org/doc/libs/1_49_0/libs/graph/doc/table_of_contents.html for more details
   */

  BOOST_CONCEPT_ASSERT(( VertexListGraphConcept< NdnGlobalRouterGraph > ));
  BOOST_CONCEPT_ASSERT(( IncidenceGraphConcept< NdnGlobalRouterGraph > ));

  NdnGlobalRouterGraph graph;
  typedef graph_traits < NdnGlobalRouterGraph >::vertex_descriptor vertex_descriptor;

  //ZYmodifyEdgeMetric();

  // For now we doing Dijkstra for every node.  Can be replaced with Bellman-Ford or Floyd-Warshall.
  // Other algorithms should be faster, but they need additional EdgeListGraph concept provided by the graph, which
  // is not obviously how implement in an efficient manner
  for (NodeList::Iterator node = NodeList::Begin (); node != NodeList::End (); node++)
    {
      Ptr<GlobalRouter> source = (*node)->GetObject<GlobalRouter> ();
      if (source == 0)
	{
	  NS_LOG_DEBUG ("Node " << (*node)->GetId () << " does not export GlobalRouter interface");
	  continue;
	}

      Ptr<Fib>  fib  = source->GetObject<Fib> ();
      fib->InvalidateAll ();
      NS_ASSERT (fib != 0);

      NS_LOG_DEBUG ("Reachability from Node: " << source->GetObject<Node> ()->GetId () << " (" << Names::FindName (source->GetObject<Node> ()) << ")");

      Ptr<L3Protocol> l3 = source->GetObject<L3Protocol> ();
      NS_ASSERT (l3 != 0);

      /*
       * 2013-11-8 ZhangYu 这里计算所有可能的路径的方法是把当前节点的所有Face（出口）都设置为最大，然后再分别设置为某一个Face为原来拓扑里的值后计算最短路径，
       * 这样就相当于为source计算了每个出口的不同路径。但是这样并不是计算出真正所有可能的路径，比如节点0，度为2，8到0只有两条路
       */
      // remember interface statuses
      std::vector<uint16_t> originalMetric (l3->GetNFaces ());
      for (uint32_t faceId = 0; faceId < l3->GetNFaces (); faceId++)
        {
          originalMetric[faceId] = l3->GetFace (faceId)->GetMetric ();
          l3->GetFace (faceId)->SetMetric (std::numeric_limits<int16_t>::max ()-1); // value std::numeric_limits<int16_t>::max () MUST NOT be used (reserved)
        }
      // 分别启用一个Face，然后计算最短路径
      for (uint32_t enabledFaceId = 0; enabledFaceId < l3->GetNFaces (); enabledFaceId++)
        {
          if (DynamicCast<ndn::NetDeviceFace> (l3->GetFace (enabledFaceId)) == 0)
            continue;

          // enabling only faceId
          l3->GetFace (enabledFaceId)->SetMetric (originalMetric[enabledFaceId]);

          DistancesMap    distances;
          PredecessorsMap predecessors;

          NS_LOG_DEBUG ("-----------");

          dijkstra_shortest_paths (graph, source,
                                   predecessor_map (boost::ref(predecessors))
                                   .
                                   distance_map (boost::ref(distances))
                                   .
                                   distance_inf (WeightInf)
                                   .
                                   distance_zero (WeightZero)
                                   .
                                   distance_compare (boost::WeightCompare ())
                                   .
                                   distance_combine (boost::WeightCombine ())
                                   );

          // NS_LOG_DEBUG (predecessors.size () << ", " << distances.size ());

          //NS_LOG_DEBUG("ZhangYu 2013-11-8 current enabledFace: " << DynamicCast<ndn::NetDeviceFace> (l3->GetFace (enabledFaceId))->GetInstanceTypeId());
          NS_LOG_DEBUG("ZhangYu 2013-11-12 current enabledFace: " << *l3->GetFace(enabledFaceId));
          for(PredecessorsMap::iterator i=predecessors.begin();i!=predecessors.end();i++)
          {
          	NS_LOG_DEBUG("ZhangYu 2013-5-21 predecessors node: " << i->first->GetObject<Node>()->GetId()  <<"  ParentNode: " <<i->second->GetObject<Node>()->GetId());
          }
          for (DistancesMap::iterator i = distances.begin ();
               i != distances.end ();
               i++)
            {
              if (i->first == source)
                continue;
              else
                {
                  // cout << "  Node " << i->first->GetObject<Node> ()->GetId ();
                  if (i->second.get<0> () == 0)
                    {
                      // cout << " is unreachable" << endl;
                    }
                  else
                    {
                        //NS_LOG_DEBUG("ZhangYu 2013-12-22 i->first NodeId: " << i->first->GetObject<Node>()->GetId() << "  LocalPrefixes: " << i->first->GetLocalPrefixes().size() );
                      BOOST_FOREACH (const Ptr<const Name> &prefix, i->first->GetLocalPrefixes ())
                        {
                          NS_LOG_DEBUG (" prefix " << *prefix << " reachable via face " << *i->second.get<0> ()
                                        << " with distance " << i->second.get<1> ()
                                        << " with delay " << i->second.get<2> ());
                          //NS_LOG_DEBUG("ZhangYu 2013-11-12 " <<( i->second.get<0>())->GetTypeId());

                          if (i->second.get<0> ()->GetMetric () == std::numeric_limits<uint16_t>::max ()-1)
                            continue;

                         Ptr<fib::Entry> entry = fib->Add (prefix, i->second.get<0> (), i->second.get<1> ());
                         NS_LOG_DEBUG("---ZhangYu 2013-12-22 fib's Node: " << fib->GetObject<Node>()->GetId() << "  fib size: " << fib->GetSize());
                         entry->SetRealDelayToProducer (i->second.get<0> (), Seconds (i->second.get<2> ()));

                          Ptr<Limits> faceLimits = i->second.get<0> ()->GetObject<Limits> ();

                          Ptr<Limits> fibLimits = entry->GetObject<Limits> ();
                          if (fibLimits != 0)
                            {
                              // if it was created by the forwarding strategy via DidAddFibEntry event
                              fibLimits->SetLimits (faceLimits->GetMaxRate (), 2 * i->second.get<2> () /*exact RTT*/);
                              NS_LOG_DEBUG ("Set limit for prefix " << *prefix << " " << faceLimits->GetMaxRate () << " / " <<
                                            2*i->second.get<2> () << "s (" << faceLimits->GetMaxRate () * 2 * i->second.get<2> () << ")");
                            }
                        }
                    }
                }
            }
            //ZhangYu 2013-12-22，为了查看对Fib的修改，下面添加语句查看
            //NS_LOG_DEBUG("ZhangYu 2013-12-22 fib's Node: " << fib->GetObject<Node>()->GetId() << "  fib size: " << fib->GetSize());
            //BOOST_FOREACH()
            

          // disabling the face again
          l3->GetFace (enabledFaceId)->SetMetric (std::numeric_limits<uint16_t>::max ()-1);
        }

      // recover original interface statuses
      for (uint32_t faceId = 0; faceId < l3->GetNFaces (); faceId++)
        {
          l3->GetFace (faceId)->SetMetric (originalMetric[faceId]);
        }
    }
}


} // namespace ndn
} // namespace ns3

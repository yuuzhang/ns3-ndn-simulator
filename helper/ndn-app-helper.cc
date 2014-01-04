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
#include "ns3/core-module.h"

#include "ndn-app-helper.h"
#include "ns3/log.h"
#include "ns3/string.h"
#include "ns3/names.h"
#include "ns3/ndn-app.h"

#ifdef NS3_MPI
#include "ns3/mpi-interface.h"
#endif

NS_LOG_COMPONENT_DEFINE ("ndn.AppHelper");

namespace ns3 {
namespace ndn {

AppHelper::AppHelper (const std::string &app)
{
  m_factory.SetTypeId (app);
}

void
AppHelper::SetPrefix (const std::string &prefix)
{
  m_factory.Set ("Prefix", StringValue(prefix));
  //Main函数中的    consumerHelper.SetPrefix (prefix) 和    producerHelper.SetPrefix (prefix) 都会调用此函数
  //NS_LOG_DEBUG("ZhangYu 2013-12-30 prefix: " << prefix);
}

void 
AppHelper::SetAttribute (std::string name, const AttributeValue &value)
{
  m_factory.Set (name, value);
}
    
ApplicationContainer
AppHelper::Install (Ptr<Node> node)
{
  ApplicationContainer apps;
  Ptr<Application> app = InstallPriv (node);
  if (app != 0)
    apps.Add (app);

  NS_LOG_DEBUG("ZhangYu 2013-12-31 Node Id: " << node->GetId() << "  app: " << app->GetInstanceTypeId() << "  " << app->GetTypeId());
  /*ZhangYu 2013-12-31 上面的语句得到的是 ns3::ndn::ConsumerCbr, ns3::Application   ns3::ndn::Producer ns3::Application，为了实现noComLinkMultiPath，要选择consumer节点才进行最短路径的计算
   * 计算出来后一次为所有Path上的节点都添加Fib，这样可以省去为无关的节点也计算最短路，计算一次才为当前计算的节点添加Fib。
   * 为了选择consumer节点，一种方式是在global-routing中(*node)->GetApplication(appId)->GetInstanceTypeId()判断节点的类型，根据字符串开头是consumer的，虽然可以考虑给Node再增加一个属性用来
   * 区分是consumer，这样可以挑出是consumer的节点来进行计算。现有的路由计算中，对source缩小范围，只对属于consumer的source进行计算，所以设置一个属性，不考虑producer，也暂时不考虑一个节点装在了多个consumer的情况
   * 但是增加一个node的属性，需要直接修改NS3代码中的node.cc，影响可能大，所以放弃，只是靠
   */
  return apps;
}
    
ApplicationContainer
AppHelper::Install (std::string nodeName)
{
  Ptr<Node> node = Names::Find<Node> (nodeName);
  return Install (node);
}
    
ApplicationContainer
AppHelper::Install (NodeContainer c)
{
  ApplicationContainer apps;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      Ptr<Application> app = InstallPriv (*i);
      if (app != 0)
        apps.Add (app);
    }
    
  return apps;
}
    
Ptr<Application>
AppHelper::InstallPriv (Ptr<Node> node)
{
#ifdef NS3_MPI
  if (MpiInterface::IsEnabled () &&
      node->GetSystemId () != MpiInterface::GetSystemId ())
    {
      // don't create an app if MPI is enabled and node is not in the correct partition
      return 0;
    }
#endif
  
  Ptr<App> app = m_factory.Create<App> ();        
  node->AddApplication (app);
        
  return app;
}

} // namespace ndn
} // namespace ns3

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

#include "ndn-consumer-window.h"
#include "ns3/ptr.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/packet.h"
#include "ns3/callback.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"

NS_LOG_COMPONENT_DEFINE ("ndn.ConsumerWindow");

namespace ns3 {
namespace ndn {

NS_OBJECT_ENSURE_REGISTERED (ConsumerWindow);

TypeId
ConsumerWindow::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::ndn::ConsumerWindow")
    .SetGroupName ("Ndn")
    .SetParent<Consumer> ()
    .AddConstructor<ConsumerWindow> ()

    .AddAttribute ("Window", "Initial size of the window",
                   StringValue ("1"),
                   MakeUintegerAccessor (&ConsumerWindow::GetWindow, &ConsumerWindow::SetWindow),
                   MakeUintegerChecker<uint32_t> ())

    .AddAttribute ("PayloadSize", "Average size of content object size (to calculate interest generation rate)",
                   UintegerValue (1040),
                   MakeUintegerAccessor (&ConsumerWindow::GetPayloadSize, &ConsumerWindow::SetPayloadSize),
                   MakeUintegerChecker<uint32_t>())
    .AddAttribute ("Size", "Amount of data in megabytes to request (relies on PayloadSize parameter)",
                   DoubleValue (-1), // don't impose limit by default
                   MakeDoubleAccessor (&ConsumerWindow::GetMaxSize, &ConsumerWindow::SetMaxSize),
                   MakeDoubleChecker<double> ())

    .AddAttribute ("InitialWindowOnTimeout", "Set window to initial value when timeout occurs",
                   BooleanValue (true),
                   MakeBooleanAccessor (&ConsumerWindow::m_setInitialWindowOnTimeout),
                   MakeBooleanChecker ())

    .AddTraceSource ("WindowTrace",
                     "Window that controls how many outstanding interests are allowed",
                     MakeTraceSourceAccessor (&ConsumerWindow::m_window))
    .AddTraceSource ("InFlight",
                     "Current number of outstanding interests",
                     MakeTraceSourceAccessor (&ConsumerWindow::m_window))
    ;

  return tid;
}

ConsumerWindow::ConsumerWindow ()
  : m_payloadSize (1040)
  , m_inFlight (0)
{
}

void
ConsumerWindow::SetWindow (uint32_t window)
{
  m_initialWindow = window;
  m_window = m_initialWindow;
}

uint32_t
ConsumerWindow::GetWindow () const
{
  return m_initialWindow;
}

uint32_t
ConsumerWindow::GetPayloadSize () const
{
  return m_payloadSize;
}

void
ConsumerWindow::SetPayloadSize (uint32_t payload)
{
  m_payloadSize = payload;
}

double
ConsumerWindow::GetMaxSize () const
{
  if (m_seqMax == 0)
    return -1.0;

  return m_maxSize;
}

void
ConsumerWindow::SetMaxSize (double size)
{
  m_maxSize = size;
  if (m_maxSize < 0)
    {
      m_seqMax = 0;
      return;
    }

  m_seqMax = floor(1.0 + m_maxSize * 1024.0 * 1024.0 / m_payloadSize);
  NS_LOG_DEBUG ("MaxSeqNo: " << m_seqMax);
  // std::cout << "MaxSeqNo: " << m_seqMax << "\n";
}


void
ConsumerWindow::ScheduleNextPacket ()
{
  if (m_window == static_cast<uint32_t> (0))
    {
      Simulator::Remove (m_sendEvent);

      NS_LOG_DEBUG ("Next event in " << (std::min<double> (0.5, m_rtt->RetransmitTimeout ().ToDouble (Time::S))) << " sec");
      m_sendEvent = Simulator::Schedule (Seconds (std::min<double> (0.5, m_rtt->RetransmitTimeout ().ToDouble (Time::S))),
                                         &Consumer::SendPacket, this);
    }
  else if (m_inFlight >= m_window)
    {
      // simply do nothing
    }
  else
    {
      if (m_sendEvent.IsRunning ())
        {
          Simulator::Remove (m_sendEvent);
        }

      m_sendEvent = Simulator::ScheduleNow (&Consumer::SendPacket, this);
    }
}

///////////////////////////////////////////////////
//          Process incoming packets             //
///////////////////////////////////////////////////

void
ConsumerWindow::OnContentObject (const Ptr<const ContentObject> &contentObject,
                                     Ptr<Packet> payload)
{
  Consumer::OnContentObject (contentObject, payload);

  m_window = m_window + 1;

  if (m_inFlight > static_cast<uint32_t> (0)) m_inFlight--;
  NS_LOG_DEBUG ("Window: " << m_window << ", InFlight: " << m_inFlight);

  ScheduleNextPacket ();
}

void
ConsumerWindow::OnNack (const Ptr<const Interest> &interest, Ptr<Packet> payload)
{
  Consumer::OnNack (interest, payload);

  if (m_inFlight > static_cast<uint32_t> (0)) m_inFlight--;

  if (m_window > static_cast<uint32_t> (0))
    {
      // m_window = 0.5 * m_window;//m_window - 1;
      m_window = std::max<uint32_t> (0, m_window - 1);
    }

  NS_LOG_DEBUG ("Window: " << m_window << ", InFlight: " << m_inFlight);

  ScheduleNextPacket ();
}

void
ConsumerWindow::OnTimeout (uint32_t sequenceNumber)
{
  if (m_inFlight > static_cast<uint32_t> (0)) m_inFlight--;

  if (m_setInitialWindowOnTimeout)
    {
      // m_window = std::max<uint32_t> (0, m_window - 1);
      m_window = m_initialWindow;
    }

  NS_LOG_DEBUG ("Window: " << m_window << ", InFlight: " << m_inFlight);
  Consumer::OnTimeout (sequenceNumber);
}

void
ConsumerWindow::WillSendOutInterest (uint32_t sequenceNumber)
{
  m_inFlight ++;
  Consumer::WillSendOutInterest (sequenceNumber);
}


} // namespace ndn
} // namespace ns3

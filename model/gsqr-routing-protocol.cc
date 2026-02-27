/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "gsqr-routing-protocol.h"
#include "gsqr-routing.h"
#include "gsqr-embedding.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/node.h"
#include "ns3/ipv4-route.h"
#include "ns3/ipv4-interface.h"      
#include "ns3/ipv4-l3-protocol.h"    
#include "ns3/net-device.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/udp-socket.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-mac.h"
#include "ns3/string.h"     
#include "ns3/double.h"

NS_LOG_COMPONENT_DEFINE ("GsqrRoutingProtocol");

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (GsqrRoutingProtocol);

TypeId
GsqrRoutingProtocol::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::GsqrRoutingProtocol")
    .SetParent<Ipv4RoutingProtocol> ()
    .SetGroupName ("Gsqr")
    .AddConstructor<GsqrRoutingProtocol> ()
    .AddAttribute ("HelloInterval", "Interval between hello packets",
                   TimeValue (Seconds (2.0)),
                   MakeTimeAccessor (&GsqrRoutingProtocol::m_helloInterval),
                   MakeTimeChecker ())
    .AddAttribute ("EmbeddingFile", "Path to GraphSAGE embedding file",
                   StringValue (""),
                   MakeStringAccessor (&GsqrRoutingProtocol::m_embeddingFile),
                   MakeStringChecker ())
    .AddAttribute ("LearningRate", "Q-learning learning rate (alpha)",
                   DoubleValue (0.1),
                   MakeDoubleAccessor (&GsqrRoutingProtocol::SetLearningRate),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("DiscountFactor", "Q-learning discount factor (gamma)",
                   DoubleValue (0.9),
                   MakeDoubleAccessor (&GsqrRoutingProtocol::SetDiscountFactor),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("EnergyWeight", "Weight for energy consumption (lambda)",
                   DoubleValue (0.01),
                   MakeDoubleAccessor (&GsqrRoutingProtocol::SetEnergyWeight),
                   MakeDoubleChecker<double> ())
  ;
  return tid;
}

GsqrRoutingProtocol::GsqrRoutingProtocol ()
  : m_ipv4 (0),
    m_routing (0),
    m_embedding (0),
    m_helloInterval (Seconds (2.0)),
    m_nodeId (0),
    m_controlPacketsSent (0),   
    m_controlBytesSent (0)       
{
  NS_LOG_FUNCTION (this);
}

GsqrRoutingProtocol::~GsqrRoutingProtocol ()
{
  NS_LOG_FUNCTION (this);
  
  delete m_routing;
  delete m_embedding;
}

void
GsqrRoutingProtocol::SetIpv4 (Ptr<Ipv4> ipv4)
{
  NS_LOG_FUNCTION (this << ipv4);
  m_controlPacketsSent = 0;
  m_controlBytesSent = 0;
  
  m_ipv4 = ipv4;
  
  if (m_ipv4) {
    m_nodeId = m_ipv4->GetObject<Node> ()->GetId ();
    
    m_routing = new GsqrRouting ();
    m_embedding = new GsqrEmbedding ();
    
    // Important: Check and start the interface manually
    for (uint32_t i = 0; i < m_ipv4->GetNInterfaces (); i++) {
      if (i == 0) continue; 
      
      if (m_ipv4->IsUp (i)) {
        //NotifyInterfaceUp
        NotifyInterfaceUp (i);
      } else {
        // Monitor interface status changes 
        // implement a status monitoring mechanism
      }
    }
    
    // Alternative: Start the Hello timer directly 
    //(try again later if the interface is not ready)
    if (!m_helloEvent.IsRunning ()) {
      m_helloEvent = Simulator::Schedule (Seconds (1.0), &GsqrRoutingProtocol::SendHello, this);
    }
  }
}

Ptr<Ipv4Route>
GsqrRoutingProtocol::RouteOutput (Ptr<Packet> p, const Ipv4Header &header,
                                  Ptr<NetDevice> oif, Socket::SocketErrno &sockerr)
{
  NS_LOG_FUNCTION (this << p << header << oif);
  
  if (!m_ipv4) {
    sockerr = Socket::ERROR_NOROUTETOHOST;
    return 0;
  }
  
  Ipv4Address dest = header.GetDestination ();
  sockerr = Socket::ERROR_NOTERROR;
  
  Ptr<Ipv4Route> route = Create<Ipv4Route> ();
  route->SetDestination (dest);
  
  // Simplification: Find the next hop
  // Method 1: Use the first non-loopback interface
  for (uint32_t i = 0; i < m_ipv4->GetNInterfaces (); ++i) {
    // Skip the loopback and down interfaces
    if (i == 0) continue; // loopback
    if (!m_ipv4->IsUp (i)) continue;
    
    uint32_t numAddr = m_ipv4->GetNAddresses (i);
    if (numAddr == 0) continue;
    
    Ipv4InterfaceAddress ifAddr = m_ipv4->GetAddress (i, 0);
    
    route->SetGateway (Ipv4Address::GetZero ()); 
    route->SetSource (ifAddr.GetLocal ());
    route->SetOutputDevice (m_ipv4->GetNetDevice (i));
    
    NS_LOG_LOGIC ("RouteOutput: Using interface " << i 
                  << " with address " << ifAddr.GetLocal ());
    break;
  }
  
  return route;
}

bool
GsqrRoutingProtocol::RouteInput (Ptr<const Packet> p, const Ipv4Header &header,
                                 Ptr<const NetDevice> idev,
                                 const UnicastForwardCallback &ucb,
                                 const MulticastForwardCallback &mcb,
                                 const LocalDeliverCallback &lcb,
                                 const ErrorCallback &ecb)
{
  NS_LOG_FUNCTION (this << p << header << idev);
  
  if (!m_ipv4) {
    return false;
  }
  
  // Check if it is a local address
  Ipv4Address destAddr = header.GetDestination ();
  
  // Method 1: Check all interfaces for this address
  for (uint32_t i = 0; i < m_ipv4->GetNInterfaces (); ++i) {
    for (uint32_t j = 0; j < m_ipv4->GetNAddresses (i); ++j) {
      Ipv4InterfaceAddress ifAddr = m_ipv4->GetAddress (i, j);
      
      if (ifAddr.GetLocal () == destAddr) {
        // This is a local address, delivered to the upper layer
        if (!lcb.IsNull ()) {
          lcb (p, header, header.GetProtocol ());
        }
        return true;
      }
      
      // Check broadcast address
      if (destAddr == ifAddr.GetBroadcast () || destAddr == Ipv4Address ("255.255.255.255")) {
        if (!lcb.IsNull ()) {
          lcb (p, header, header.GetProtocol ());
        }
        return true;
      }
    }
  }
  
  // Not a local address. Forwarding is required 
  // (simplified implementation)
  
  
  return false;
}

void
GsqrRoutingProtocol::NotifyInterfaceUp (uint32_t interface)
{
  NS_LOG_FUNCTION (this << interface);
  
  if (interface > 0) { 
    NS_LOG_INFO ("Interface " << interface << " is up, starting GSQR protocol");
    
    if (m_ipv4->GetNAddresses (interface) > 0) {
      Ipv4InterfaceAddress addr = m_ipv4->GetAddress (interface, 0);
      NS_LOG_INFO ("  Address: " << addr.GetLocal () 
                    << "/" << addr.GetMask ().GetPrefixLength ());
    }
    
    // +++ Hello Socket +++
    if (!m_socket) {
      m_helloPort = 6543; // Fixed port, or configurable parameter
      
      // 1. Crreate UDP Socket
      m_socket = Socket::CreateSocket (GetObject<Node> (), UdpSocketFactory::GetTypeId ());
      
      // 2. Bind to local IP and Hello port
      Ipv4Address localAddress = m_ipv4->GetAddress (interface, 0).GetLocal ();
      InetSocketAddress local = InetSocketAddress (localAddress, m_helloPort);
      m_socket->Bind (local);
      
      // 3. Set broadcast permissions
      m_socket->SetAllowBroadcast (true);
      
      // 4. Set callback
      //m_socket->SetRecvCallback (MakeCallback (&GsqrRoutingProtocol::ReceiveHelloPacket, this));
      m_socket->SetRecvCallback (MakeCallback (&GsqrRoutingProtocol::ReceiveHello, this));
      //m_socket->SetRecvCallback (MakeCallback (&GsqrRoutingProtocol::ReceiveHello, this, std::placeholders::_1));
      
      NS_LOG_INFO ("GSQR Node " << m_nodeId << " Hello socket bound to " << localAddress << ":" << m_helloPort);
    }
    // Start the Hello timer (if not already started)
    if (!m_helloEvent.IsRunning ()) {
      m_helloEvent = Simulator::Schedule (Seconds (0.5), &GsqrRoutingProtocol::SendHello, this);
    }
  }
}

void
GsqrRoutingProtocol::NotifyInterfaceDown (uint32_t interface)
{
  NS_LOG_FUNCTION (this << interface);
}

void
GsqrRoutingProtocol::NotifyAddAddress (uint32_t interface, Ipv4InterfaceAddress address)
{
  NS_LOG_FUNCTION (this << interface << address);
}

void
GsqrRoutingProtocol::NotifyRemoveAddress (uint32_t interface, Ipv4InterfaceAddress address)
{
  NS_LOG_FUNCTION (this << interface << address);
}

void
GsqrRoutingProtocol::PrintRoutingTable (Ptr<OutputStreamWrapper> stream, Time::Unit unit) const
{
  NS_LOG_FUNCTION (this << stream << unit);
  
  *stream->GetStream () << "GSQR Routing Table for Node " << m_nodeId << std::endl;
  *stream->GetStream () << "========================================" << std::endl;
  
  for (const auto& neighbor : m_neighbors) {
    *stream->GetStream () << "Neighbor " << neighbor.first 
                         << " (last seen: " << neighbor.second.lastSeen.GetSeconds () << "s)"
                         << std::endl;
  }
  
  *stream->GetStream () << std::endl;
}

void
GsqrRoutingProtocol::NotifyAddRoute (Ipv4Address dst, Ipv4Mask mask, uint32_t ifIndex,
                                     Ipv4Address nextHop, uint32_t metric)
{
  NS_LOG_FUNCTION (this << dst << mask << ifIndex << nextHop << metric);
}

void
GsqrRoutingProtocol::NotifyRemoveRoute (Ipv4Address dst, Ipv4Mask mask, uint32_t ifIndex,
                                        Ipv4Address nextHop, uint32_t metric)
{
  NS_LOG_FUNCTION (this << dst << mask << ifIndex << nextHop << metric);
}

void
GsqrRoutingProtocol::SetLearningRate (double alpha)
{
  NS_LOG_FUNCTION (this << alpha);
  if (m_routing) {
    // m_routing->SetLearningRate (alpha);
    NS_LOG_INFO ("Learning rate set to: " << alpha);
  }
}

void
GsqrRoutingProtocol::SetDiscountFactor (double gamma)
{
  NS_LOG_FUNCTION (this << gamma);
  if (m_routing) {
    // m_routing->SetDiscountFactor (gamma);
    NS_LOG_INFO ("Discount factor set to: " << gamma);
  }
}

void
GsqrRoutingProtocol::SetEnergyWeight (double lambda)
{
  NS_LOG_FUNCTION (this << lambda);
  if (m_routing) {
    // m_routing->SetEnergyWeight (lambda);
    NS_LOG_INFO ("Energy weight set to: " << lambda);
  }
}


void
GsqrRoutingProtocol::SendHello (void)
{
    NS_LOG_FUNCTION (this);
    
    // If the Socket does not exist, try to create it
    if (!m_socket) {
      InitializeHelloSocket ();
    }
    
    if (!m_socket || !m_ipv4) {
      NS_LOG_WARN ("GSQR Node " << m_nodeId << " cannot send Hello: socket/IPv4 not ready");
      ScheduleNextHello ();
      return;
    }
    
    // check
    if (!m_socket || !m_ipv4) {
      NS_LOG_WARN ("GSQR Node " << m_nodeId << " cannot send Hello: socket/IPv4 not ready");
      ScheduleNextHello();
      return;
    }
    

    // 1. Create a Hello Header with data
    GsqrHelloHeader helloHeader;
    helloHeader.SetNodeId (m_nodeId);
    helloHeader.SetTimestamp (Simulator::Now ().GetSeconds ());
    helloHeader.SetMeanETX (1.5);
    helloHeader.SetResidualEnergy (95.0);
    helloHeader.SetQueueLength (0.1);
    
    // 2. Create Packet
    Ptr<Packet> packet = Create<Packet> (0);
    packet->AddHeader (helloHeader);
    uint32_t pktSize = packet->GetSize ();
    
    // 3.send to broadcast Address
    Ipv4Address broadcastAddr ("255.255.255.255");
    InetSocketAddress remote = InetSocketAddress (broadcastAddr, m_helloPort);
    
    int bytesSent = m_socket->SendTo (packet, 0, remote);
    
    // 4. Statistical control packet 
    //(counted only when sent successfully)
    NS_LOG_INFO("Before sending - Hello count: " << m_controlPacketsSent 
                << ", Control bytes: " << m_controlBytesSent);
    if (bytesSent > 0) {
        m_controlPacketsSent++;        
        m_controlBytesSent += pktSize;   
        
        NS_LOG_INFO("After sending - Hello count: " << m_controlPacketsSent 
                    << ", Control bytes: " << m_controlBytesSent
                    << ", This packet size: " << pktSize);
    } else {
        NS_LOG_WARN ("GSQR Node " << m_nodeId << " failed to send Hello");
    }
    
    // 5. Schedule Next Hello
    ScheduleNextHello();
   
}

void
GsqrRoutingProtocol::InitializeHelloSocket (void)
{
  NS_LOG_FUNCTION (this);
  
  if (m_socket || !m_ipv4) {
    return;
  }
  
  // Find the first non-loopback UP interface
  for (uint32_t i = 0; i < m_ipv4->GetNInterfaces (); i++) {
    if (i == 0) continue; 
    
    if (m_ipv4->IsUp (i) && m_ipv4->GetNAddresses (i) > 0) {
      m_helloPort = 6543;
      
      // 1. Create UDP Socket
      m_socket = Socket::CreateSocket (GetObject<Node> (), UdpSocketFactory::GetTypeId ());
      
      // 2. Bind to local IP and Hello port
      Ipv4Address localAddress = m_ipv4->GetAddress (i, 0).GetLocal ();
      InetSocketAddress local = InetSocketAddress (localAddress, m_helloPort);
      m_socket->Bind (local);
      
      // 3. Set broadcast permissions
      m_socket->SetAllowBroadcast (true);
      
      // 4. Set callback
      m_socket->SetRecvCallback ([this](Ptr<Socket> socket) {
        this->ReceiveHello (socket);
      });
      
      NS_LOG_INFO ("GSQR Node " << m_nodeId << " Hello socket bound to " 
                  << localAddress << ":" << m_helloPort);
      break;
    }
  }
}

void
GsqrRoutingProtocol::ScheduleNextHello()
{
  m_helloEvent = Simulator::Schedule (m_helloInterval, &GsqrRoutingProtocol::SendHello, this);
}

void
GsqrRoutingProtocol::ReceiveHello (Ptr<Socket> socket)
{
  NS_LOG_FUNCTION (this << socket);
  
  Address fromAddr;
  Ptr<Packet> packet = socket->RecvFrom (fromAddr);
  
  if (!packet || packet->GetSize () == 0) {
    return;
  }
  
  // 1. Get Hello Header
  GsqrHelloHeader helloHeader;
  packet->PeekHeader (helloHeader);
  
  uint32_t neighborId = helloHeader.GetNodeId ();
  
  // 2. Get Sender IP Address
  InetSocketAddress inetFromAddr = InetSocketAddress::ConvertFrom (fromAddr);
  Ipv4Address senderIp = inetFromAddr.GetIpv4 ();
  
  // 3. Update or create neighbor information
  NeighborInfo& neighbor = m_neighbors[neighborId];
  neighbor.nodeId = neighborId;
  neighbor.lastSeen = Simulator::Now ();
  neighbor.meanETX = helloHeader.GetMeanETX ();
  neighbor.residualEnergy = helloHeader.GetResidualEnergy ();
  neighbor.queueLength = helloHeader.GetQueueLength ();
  
  // The record is received from which interface (for routing decisions)
  for (uint32_t i = 0; i < m_ipv4->GetNInterfaces (); ++i) {
    if (m_ipv4->GetAddress (i, 0).GetLocal () == senderIp) {
      neighbor.interface = i;
      break;
    }
  }
  
  NS_LOG_DEBUG ("GSQR Node " << m_nodeId << " received Hello from Node " 
                << neighborId << " via " << senderIp 
                << ", ETX=" << neighbor.meanETX);
}

void
GsqrRoutingProtocol::CleanupNeighbors (void)
{
  NS_LOG_FUNCTION (this);

}

std::vector<uint32_t>
GsqrRoutingProtocol::GetCurrentNeighbors (void) const
{
  std::vector<uint32_t> neighbors;
  for (const auto& entry : m_neighbors) {
    neighbors.push_back (entry.first);
  }
  return neighbors;
}

// GsqrHelloHeader
TypeId
GsqrHelloHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::GsqrHelloHeader")
    .SetParent<Header> ()
    .SetGroupName ("Gsqr")
    .AddConstructor<GsqrHelloHeader> ()
  ;
  return tid;
}

TypeId
GsqrHelloHeader::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

uint32_t
GsqrHelloHeader::GetSerializedSize (void) const
{
  return sizeof(m_nodeId) + sizeof(m_timestamp) + 
         sizeof(m_meanETX) + sizeof(m_residualEnergy) + 
         sizeof(m_queueLength);
}

void
GsqrHelloHeader::Serialize (Buffer::Iterator start) const
{
  start.WriteHtonU32 (m_nodeId);
  start.WriteHtonU64 (static_cast<uint64_t>(m_timestamp));
  start.WriteHtonU64 (static_cast<uint64_t>(m_meanETX));
  start.WriteHtonU64 (static_cast<uint64_t>(m_residualEnergy));
  start.WriteHtonU64 (static_cast<uint64_t>(m_queueLength));
}

uint32_t
GsqrHelloHeader::Deserialize (Buffer::Iterator start)
{
  m_nodeId = start.ReadNtohU32 ();
  
  uint64_t temp;
    
    temp = start.ReadNtohU64 ();
    memcpy(&m_timestamp, &temp, sizeof(double));
    
    temp = start.ReadNtohU64 ();
    memcpy(&m_meanETX, &temp, sizeof(double));
    
    temp = start.ReadNtohU64 ();
    memcpy(&m_residualEnergy, &temp, sizeof(double));
    
    temp = start.ReadNtohU64 ();
    memcpy(&m_queueLength, &temp, sizeof(double));
    
    return GetSerializedSize ();
}

void
GsqrHelloHeader::Print (std::ostream &os) const
{
  os << "GsqrHelloHeader [Node=" << m_nodeId 
     << ", Timestamp=" << m_timestamp
     << ", ETX=" << m_meanETX
     << ", Energy=" << m_residualEnergy
     << ", Queue=" << m_queueLength << "]";
}

GsqrHelloHeader::GsqrHelloHeader ()
  : m_nodeId (0),
    m_timestamp (0.0),
    m_meanETX (0.0),
    m_residualEnergy (0.0),
    m_queueLength (0.0)
{
}

} // namespace ns3
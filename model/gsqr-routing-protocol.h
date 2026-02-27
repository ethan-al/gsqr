/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#ifndef GSQR_ROUTING_PROTOCOL_H
#define GSQR_ROUTING_PROTOCOL_H

#include "ns3/ipv4-routing-protocol.h"
#include "ns3/ipv4-interface.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/ipv4-route.h"
#include "ns3/ipv4.h"
#include "ns3/packet.h"
#include "ns3/nstime.h"
#include "ns3/event-id.h"
#include "ns3/header.h"
#include "ns3/string.h"     
#include "ns3/double.h"     
#include "ns3/type-id.h"    
#include "ns3/attribute.h"  
#include <map>
#include <vector>

namespace ns3
{

class GsqrRouting;
class GsqrEmbedding;

/**
 * \brief GSQR Hello packet header
 */
class GsqrHelloHeader : public Header
{
public:
  GsqrHelloHeader ();
  virtual ~GsqrHelloHeader () {}

  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);
  virtual void Print (std::ostream &os) const;
  
  void SetNodeId (uint32_t id) { m_nodeId = id; }
  uint32_t GetNodeId (void) const { return m_nodeId; }
  
  void SetTimestamp (double ts) { m_timestamp = ts; }
  double GetTimestamp (void) const { return m_timestamp; }
  
  void SetMeanETX (double etx) { m_meanETX = etx; }
  double GetMeanETX (void) const { return m_meanETX; }
  
  void SetResidualEnergy (double energy) { m_residualEnergy = energy; }
  double GetResidualEnergy (void) const { return m_residualEnergy; }
  
  void SetQueueLength (double length) { m_queueLength = length; }
  double GetQueueLength (void) const { return m_queueLength; }

private:
  uint32_t m_nodeId;
  double m_timestamp;
  double m_meanETX;
  double m_residualEnergy;
  double m_queueLength;
};

/**
 * \brief GSQR Routing Protocol
 * 
 * Implements the GSQR routing protocol for UAV networks.
 * Uses GraphSAGE embeddings for Q-value approximation.
 */
class GsqrRoutingProtocol : public Ipv4RoutingProtocol
{
public:
  static TypeId GetTypeId (void);
  
  GsqrRoutingProtocol ();
  virtual ~GsqrRoutingProtocol ();
  
  // From Ipv4RoutingProtocol
  virtual Ptr<Ipv4Route> RouteOutput (Ptr<Packet> p, const Ipv4Header &header,
                                      Ptr<NetDevice> oif, Socket::SocketErrno &sockerr);
  virtual bool RouteInput (Ptr<const Packet> p, const Ipv4Header &header,
                           Ptr<const NetDevice> idev, const UnicastForwardCallback &ucb,
                           const MulticastForwardCallback &mcb, const LocalDeliverCallback &lcb,
                           const ErrorCallback &ecb);
  virtual void NotifyInterfaceUp (uint32_t interface);
  virtual void NotifyInterfaceDown (uint32_t interface);
  virtual void NotifyAddAddress (uint32_t interface, Ipv4InterfaceAddress address);
  virtual void NotifyRemoveAddress (uint32_t interface, Ipv4InterfaceAddress address);
  virtual void SetIpv4 (Ptr<Ipv4> ipv4);
  virtual void PrintRoutingTable (Ptr<OutputStreamWrapper> stream, Time::Unit unit = Time::S) const;
  virtual void NotifyAddRoute (Ipv4Address dst, Ipv4Mask mask, uint32_t ifIndex,
                               Ipv4Address nextHop, uint32_t metric);
  virtual void NotifyRemoveRoute (Ipv4Address dst, Ipv4Mask mask, uint32_t ifIndex,
                                  Ipv4Address nextHop, uint32_t metric);
  
  // GSQR-specific methods
  void SetLearningRate (double alpha);
  void SetDiscountFactor (double gamma);
  void SetEnergyWeight (double lambda);
  
  uint32_t GetControlPacketsSent (void) const{return m_controlPacketsSent;}
  uint64_t GetControlBytesSent () const{ return m_controlBytesSent;}
  
private:
  struct NeighborInfo
  {
    uint32_t nodeId;
    Time lastSeen;
    double meanETX;
    double residualEnergy;
    double queueLength;
    uint32_t interface;
  };
  
  void SendHello (void);
  void ScheduleNextHello();
  void ReceiveHello (Ptr<Socket> socket);
  void InitializeHelloSocket (void);
  
  void SendPacket (Ptr<Packet> packet, Ipv4Address dest, uint32_t interface);
  void PacketSent (Ptr<const Packet> packet, const Ipv4Header& header,
                   Ptr<Ipv4Interface> interface, uint32_t ifIndex);
  std::vector<uint32_t> GetCurrentNeighbors (void) const;
  void CleanupNeighbors (void);
  
  Ptr<Ipv4> m_ipv4;
  
  GsqrRouting* m_routing;        
  GsqrEmbedding* m_embedding;    
  
  Time m_helloInterval;
  std::string m_embeddingFile;
  uint32_t m_nodeId;
  
  std::map<uint32_t, NeighborInfo> m_neighbors;
  EventId m_helloEvent;
  EventId m_cleanupEvent;
  uint32_t m_controlPacketsSent;    
  uint64_t m_controlBytesSent;    
 
  Ptr<Socket> m_socket;          
  uint16_t m_helloPort;          
};

} // namespace ns3

#endif /* GSQR_ROUTING_PROTOCOL_H */
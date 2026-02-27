/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "gsqr-helper.h"
#include "ns3/gsqr-routing-protocol.h"
#include "ns3/node.h"
#include "ns3/log.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("GsqrHelper");

GsqrHelper::GsqrHelper()
{
    m_factory.SetTypeId("ns3::GsqrRoutingProtocol");
}

GsqrHelper::~GsqrHelper()
{
    NS_LOG_FUNCTION(this);
}

GsqrHelper* 
GsqrHelper::Copy() const
{
    NS_LOG_FUNCTION(this);
    return new GsqrHelper(*this);
}

Ptr<Ipv4RoutingProtocol>
GsqrHelper::Create(Ptr<Node> node) const
{
    NS_LOG_FUNCTION(this << node);
    
    Ptr<GsqrRoutingProtocol> protocol = m_factory.Create<GsqrRoutingProtocol>();
    
    node->AggregateObject(protocol);
    
    NS_LOG_INFO("GSQR routing protocol created for node " << node->GetId());
    
    return protocol;
}

void GsqrHelper::Install(NodeContainer nodes) const
{
    NS_LOG_FUNCTION(this);
    
    for (NodeContainer::Iterator i = nodes.Begin(); i != nodes.End(); ++i)
    {
        Install(*i);
    }
}

void GsqrHelper::Install(Ptr<Node> node) const
{
    NS_LOG_FUNCTION(this << node);
    
    Create(node);
    
    NS_LOG_INFO("GSQR routing protocol installed on node " << node->GetId());
}

void GsqrHelper::SetLearningRate(double alpha)
{
    NS_LOG_FUNCTION(this << alpha);
    m_factory.Set("LearningRate", DoubleValue(alpha));
}

void GsqrHelper::SetDiscountFactor(double gamma)
{
    NS_LOG_FUNCTION(this << gamma);
    m_factory.Set("DiscountFactor", DoubleValue(gamma));
}

void GsqrHelper::SetEnergyWeight(double lambda)
{
    NS_LOG_FUNCTION(this << lambda);
    m_factory.Set("EnergyWeight", DoubleValue(lambda));
}

void GsqrHelper::SetUpdateInterval(double seconds)
{
    NS_LOG_FUNCTION(this << seconds);
    m_factory.Set("UpdateInterval", DoubleValue(seconds));
}

void GsqrHelper::SetEmbeddingFile(const std::string &filename)
{
    NS_LOG_FUNCTION(this << filename);
    m_factory.Set("EmbeddingFile", StringValue(filename));
}

void GsqrHelper::SetHelloInterval(Time interval)
{
    NS_LOG_FUNCTION(this << interval);
    m_factory.Set("HelloInterval", TimeValue(interval));
}

} // namespace ns3
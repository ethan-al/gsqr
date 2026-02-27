/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#ifndef GSQR_HELPER_H
#define GSQR_HELPER_H

#include "ns3/ipv4-routing-helper.h" 
#include "ns3/ptr.h"
#include "ns3/object-factory.h"
#include "ns3/node-container.h"
#include <string>

namespace ns3
{

class GsqrHelper : public Ipv4RoutingHelper
{
public:
    GsqrHelper();
    virtual ~GsqrHelper();
    
    virtual GsqrHelper* Copy() const;
    virtual Ptr<Ipv4RoutingProtocol> Create(Ptr<Node> node) const;
    
    void Install(NodeContainer nodes) const;
    void Install(Ptr<Node> node) const;
    
    void SetLearningRate(double alpha);
    void SetDiscountFactor(double gamma);
    void SetEnergyWeight(double lambda);
    void SetUpdateInterval(double seconds);
    void SetEmbeddingFile(const std::string &filename);
    void SetHelloInterval(Time interval);

private:
    ObjectFactory m_factory;
};

} // namespace ns3

#endif // GSQR_HELPER_H
#include "gsqr-routing.h"
#include "ns3/log.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <numeric>
#include <cmath>

NS_LOG_COMPONENT_DEFINE ("GsqrRouting");

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (GsqrRouting);

TypeId 
GsqrRouting::GetTypeId (void)
{
    static TypeId tid = TypeId ("ns3::GsqrRouting")
        .SetParent<Object> ()
        .SetGroupName ("Gsqr")
        .AddConstructor<GsqrRouting> ()
        .AddAttribute ("LearningRate",
                       "Learning rate (alpha)",
                       DoubleValue (0.1),
                       MakeDoubleAccessor (&GsqrRouting::m_alpha),
                       MakeDoubleChecker<double> (0.0, 1.0))
        .AddAttribute ("DiscountFactor",
                       "Discount factor (gamma)",
                       DoubleValue (0.9),
                       MakeDoubleAccessor (&GsqrRouting::m_gamma),
                       MakeDoubleChecker<double> (0.0, 1.0))
        .AddAttribute ("EnergyWeight",
                       "Energy weight (lambda)",
                       DoubleValue (0.01),
                       MakeDoubleAccessor (&GsqrRouting::m_lambda),
                       MakeDoubleChecker<double> (0.0, 1.0))
        .AddAttribute ("UpdateInterval",
                       "Update interval in seconds",
                       DoubleValue (2.0),
                       MakeDoubleAccessor (&GsqrRouting::m_updateInterval),
                       MakeDoubleChecker<double> (0.1, 10.0))
        ;
    return tid;
}

GsqrRouting::GsqrRouting ()
    : m_nodeId (0),
      m_alpha (0.1),
      m_gamma (0.9),
      m_lambda (0.01),
      m_updateInterval (2.0)
{
    NS_LOG_FUNCTION (this);
}

GsqrRouting::~GsqrRouting ()
{
    NS_LOG_FUNCTION (this);
}

void GsqrRouting::Initialize(uint32_t nodeId, const std::string &embeddingFile)
{
    NS_LOG_FUNCTION (this << nodeId << embeddingFile);
    m_nodeId = nodeId;
    
    if (!embeddingFile.empty()) {
        LoadEmbeddingsFromFile(embeddingFile);
    } else {
        // Initialize default embedding
        for (uint32_t i = 0; i <= 50; ++i) { // Assume up to 50 nodes
            Embedding emb;
            emb.h = std::vector<double>(16, 0.0);
            emb.b = 0.0;
            m_embeddings[i] = emb;
        }
    }
}

double GsqrRouting::ComputeQValue(uint32_t destId, uint32_t neighborId) const
{
    NS_LOG_FUNCTION (this << destId << neighborId);
    
    auto destIt = m_embeddings.find(destId);
    auto neighborIt = m_embeddings.find(neighborId);
    
    if (destIt == m_embeddings.end() || neighborIt == m_embeddings.end()) {
        NS_LOG_WARN ("Embedding not found for dest " << destId << " or neighbor " << neighborId);
        return 0.0;
    }
    
    const auto& h_d = destIt->second.h;
    const auto& h_nh = neighborIt->second.h;
    double b_nh = neighborIt->second.b;
    
    //: h_d^T · h_nh
    double dot = DotProduct(h_d, h_nh);
    
    // bias: + b_nh
    return dot + b_nh;
}

uint32_t GsqrRouting::SelectNextHop(uint32_t destNodeId, uint32_t currentNodeId)
{
    NS_LOG_FUNCTION (this << destNodeId << currentNodeId);
    
    auto neighborIt = m_neighbors.find(currentNodeId);
    if (neighborIt == m_neighbors.end() || neighborIt->second.empty()) {
        NS_LOG_WARN ("No neighbors for node " << currentNodeId);
        return currentNodeId; 
    }
    
    // Calculate the Q value for each neighbor
    std::vector<std::pair<double, uint32_t>> qValues;
    for (uint32_t neighborId : neighborIt->second) {
        double q = ComputeQValue(destNodeId, neighborId);
        qValues.emplace_back(q, neighborId);
        NS_LOG_DEBUG ("Q(" << currentNodeId << "->" << neighborId << ", dest=" << 
                     destNodeId << ") = " << q);
    }
    
    // Select the neighbor with the maximum Q value
    auto bestIt = std::max_element(qValues.begin(), qValues.end(),
        [](const auto& a, const auto& b) { return a.first < b.first; });
    
    if (bestIt != qValues.end()) {
        NS_LOG_DEBUG ("Selected next hop: " << bestIt->second << " with Q=" << bestIt->first);
        return bestIt->second;
    }
    
    return currentNodeId;
}

void GsqrRouting::ReceiveAck(uint32_t neighborId, uint32_t destId, 
                            double reward, double delay, double energy)
{
    NS_LOG_FUNCTION (this << neighborId << destId << reward << delay << energy);
    
    // Calculate rewards: r = -T_delay - λ·E_per_bit
    double r = -delay - m_lambda * energy;
    
    // Calculate the current Q value
    double qCurrent = ComputeQValue(destId, neighborId);
    
    // Calculate the maximum Q value for the next hop (simplified: assume 0)
    double qNextMax = 0.0; // need to know the next hop information in practice
    
    // TDerror: δ = r + γ * max_{n′} Q̂(v′,d,n′) - Q̂(v,d,nh)
    double tdError = r + m_gamma * qNextMax - qCurrent;
    
    // Find embedding vector
    auto destIt = m_embeddings.find(destId);
    auto neighborIt = m_embeddings.find(neighborId);
    
    if (destIt == m_embeddings.end() || neighborIt == m_embeddings.end()) {
        NS_LOG_WARN ("Cannot update: embedding not found");
        return;
    }
    
    // Update embed: h_nh = h_nh + α * δ * h_d
    const auto& h_d = destIt->second.h;
    auto& h_nh = neighborIt->second.h;
    
    for (size_t i = 0; i < 16; ++i) {
        h_nh[i] += m_alpha * tdError * h_d[i];
    }
    
    // Update bias: b_nh = b_nh + α * δ
    neighborIt->second.b += m_alpha * tdError;
    
    NS_LOG_DEBUG ("Updated embedding for neighbor " << neighborId << 
                 ", tdError=" << tdError);
}

void GsqrRouting::UpdateNeighborList(uint32_t nodeId, const std::vector<uint32_t> &neighbors)
{
    NS_LOG_FUNCTION (this << nodeId << "neighbors count: " << neighbors.size());
    m_neighbors[nodeId] = neighbors;
}

std::vector<double> GsqrRouting::GetNodeFeatures(uint32_t nodeId) const
{
    NS_LOG_FUNCTION (this << nodeId);
    
    auto it = m_nodeFeatures.find(nodeId);
    if (it != m_nodeFeatures.end()) {
        return {it->second.meanETX, it->second.residualEnergy, it->second.queueLength};
    }
    
    // Return default features
    return {1.0, 1.0, 0.0}; // mean_ETX=1.0, E_r=1.0, q=0.0
}

std::vector<double> GsqrRouting::GenerateEmbedding(uint32_t nodeId)
{
    NS_LOG_FUNCTION (this << nodeId);
    
    // Simplified version: Return the current embedding or generate a new one
    auto it = m_embeddings.find(nodeId);
    if (it != m_embeddings.end()) {
        return it->second.h;
    }
    
    // generate random embedding
    std::vector<double> embedding(16, 0.0);
    for (double& val : embedding) {
        val = (std::rand() / (double)RAND_MAX) * 2.0 - 1.0; // [-1, 1]
    }
    
    Embedding emb;
    emb.h = embedding;
    emb.b = 0.0;
    m_embeddings[nodeId] = emb;
    
    return embedding;
}

double GsqrRouting::GetMemoryUsageKB() const
{
    size_t numNodes = m_embeddings.size();
    size_t totalBytes = numNodes * (16 * sizeof(double) + sizeof(double)); // h + b
    return totalBytes / 1024.0;
}

void GsqrRouting::LoadEmbeddingsFromFile(const std::string &filename)
{
    NS_LOG_FUNCTION (this << filename);
    
    std::ifstream file(filename);
    if (!file.is_open()) {
        NS_LOG_ERROR ("Cannot open embedding file: " << filename);
        return;
    }
    
    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string item;
        
        // Node ID
        if (!std::getline(ss, item, ',')) continue;
        uint32_t nodeId = std::stoul(item);
        
        // 16 Dim
        std::vector<double> embedding;
        for (int i = 0; i < 16; ++i) {
            if (!std::getline(ss, item, ',')) break;
            embedding.push_back(std::stod(item));
        }
        
        // bias
        if (!std::getline(ss, item, ',')) continue;
        double bias = std::stod(item);
        
        Embedding emb;
        emb.h = embedding;
        emb.b = bias;
        m_embeddings[nodeId] = emb;
        
        NS_LOG_DEBUG ("Loaded embedding for node " << nodeId);
    }
    
    file.close();
    NS_LOG_INFO ("Loaded " << m_embeddings.size() << " embeddings from " << filename);
}

void GsqrRouting::SaveEmbeddingsToFile(const std::string &filename) const
{
    NS_LOG_FUNCTION (this << filename);
    
    std::ofstream file(filename);
    if (!file.is_open()) {
        NS_LOG_ERROR ("Cannot create file: " << filename);
        return;
    }
    
    for (const auto& entry : m_embeddings) {
        file << entry.first;
        for (double val : entry.second.h) {
            file << "," << val;
        }
        file << "," << entry.second.b << "\n";
    }
    
    file.close();
    NS_LOG_INFO ("Saved " << m_embeddings.size() << " embeddings to " << filename);
}

double GsqrRouting::DotProduct(const std::vector<double> &a, const std::vector<double> &b) const
{
    if (a.size() != b.size()) {
        return 0.0;
    }
    
    double result = 0.0;
    for (size_t i = 0; i < a.size(); ++i) {
        result += a[i] * b[i];
    }
    return result;
}

std::vector<double> GsqrRouting::VectorAdd(const std::vector<double> &a,
                                          const std::vector<double> &b) const
{
    std::vector<double> result(a.size());
    for (size_t i = 0; i < a.size(); ++i) {
        result[i] = a[i] + b[i];
    }
    return result;
}

std::vector<double> GsqrRouting::VectorScale(const std::vector<double> &v, double scalar) const
{
    std::vector<double> result(v.size());
    for (size_t i = 0; i < v.size(); ++i) {
        result[i] = v[i] * scalar;
    }
    return result;
}

} // namespace ns3
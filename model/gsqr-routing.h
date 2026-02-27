#ifndef GSQR_ROUTING_H
#define GSQR_ROUTING_H

#include "ns3/object.h" 
#include "ns3/ptr.h"
#include <vector>
#include <map>
#include <cstdint>
#include <string>

namespace ns3
{

class GsqrRouting : public Object
{
public:
    static TypeId GetTypeId (void); 
    
    GsqrRouting();
    virtual ~GsqrRouting();

    // Core GSQR Method
    void Initialize(uint32_t nodeId, const std::string &embeddingFile = "");

    // online Q approximation algorithm
    // Q̂(v,d,nh) = h_d^T h_nh + b_nh, nh ∈ N(v)
    uint32_t SelectNextHop(uint32_t destNodeId, uint32_t currentNodeId);

    // reinforcement learning update
    // h_nh ← h_nh + α δ h_d, b_nh ← b_nh + α δ
    void ReceiveAck(uint32_t neighborId, uint32_t destId, double reward,
                    double delay, double energy);

    // Neighborhood Management
    void UpdateNeighborList(uint32_t nodeId, const std::vector<uint32_t> &neighbors);

    // Node Features: x_v = [mean_ETX, E_r, q] ∈ ℝ³
    std::vector<double> GetNodeFeatures(uint32_t nodeId) const;

    // GraphSAGE Embedding（simple）
    std::vector<double> GenerateEmbedding(uint32_t nodeId);

    void SetLearningRate(double alpha) { m_alpha = alpha; }
    void SetDiscountFactor(double gamma) { m_gamma = gamma; }
    void SetEnergyWeight(double lambda) { m_lambda = lambda; }
    void SetUpdateInterval(double seconds) { m_updateInterval = seconds; }

    size_t GetEmbeddingDimension() const { return m_embeddingDim; }
    size_t GetNumNodes() const { return m_embeddings.size(); }
    double GetMemoryUsageKB() const;

    double ComputeQValue(uint32_t destId, uint32_t neighborId) const;
    
private:
    struct Embedding
    {
        std::vector<double> h; 
        double b;              
    };

    struct NodeFeatures
    {
        double meanETX;        
        double residualEnergy;
        double queueLength;    
    };

    
    uint32_t m_nodeId;
    std::map<uint32_t, Embedding> m_embeddings;
    std::map<uint32_t, NodeFeatures> m_nodeFeatures;
    std::map<uint32_t, std::vector<uint32_t>> m_neighbors;

    
    double m_alpha;          
    double m_gamma;          
    double m_lambda;         
    double m_updateInterval; // Δt = 2s

    static const size_t m_embeddingDim = 16; // 16Dim GraphSAGE
    static const size_t m_featureDim = 3;    // 3Dim Node featrue

    void LoadEmbeddingsFromFile(const std::string &filename);
    void SaveEmbeddingsToFile(const std::string &filename) const;

    double DotProduct(const std::vector<double> &a, const std::vector<double> &b) const;
    std::vector<double> VectorAdd(const std::vector<double> &a,
                                  const std::vector<double> &b) const;
    std::vector<double> VectorScale(const std::vector<double> &v, double scalar) const;
};

} // namespace ns3

#endif // GSQR_ROUTING_H
/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#ifndef GSQR_EMBEDDING_H
#define GSQR_EMBEDDING_H

#include "ns3/object.h"
#include "ns3/ptr.h"
#include <vector>
#include <map>
#include <string>
#include <cstdint>

namespace ns3 {

/**
 * \ingroup gsqr
 * \brief Classes for Managing GraphSAGE Embedded Vectors
 * 
 * Responsible for loading/saving embedded vectors from CSV files 
 * and providing embedded query and update interfaces
 */
class GsqrEmbedding : public Object
{
public:
    static TypeId GetTypeId (void);
    
    GsqrEmbedding ();
    virtual ~GsqrEmbedding ();
    
    bool LoadFromCSV (const std::string& filename);
    bool SaveToCSV (const std::string& filename);
    
    const std::vector<double>& GetEmbedding (uint32_t nodeId) const;
    double GetBias (uint32_t nodeId) const;
    
    void SetEmbedding (uint32_t nodeId, const std::vector<double>& embedding);
    void SetBias (uint32_t nodeId, double bias);
    
    void UpdateEmbedding (uint32_t nodeId, const std::vector<double>& gradient, double learningRate);
    void UpdateBias (uint32_t nodeId, double gradient, double learningRate);
    
    size_t GetEmbeddingDimension () const { return m_dimension; }
    size_t GetNumNodes () const { return m_embeddings.size (); }
    
private:
    size_t m_dimension {16};  
    std::map<uint32_t, std::vector<double>> m_embeddings;
    std::map<uint32_t, double> m_biases;
};

} // namespace ns3

#endif // GSQR_EMBEDDING_H
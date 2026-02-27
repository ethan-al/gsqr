/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "gsqr-embedding.h"
#include "ns3/log.h"
#include "ns3/uinteger.h"      
#include "ns3/double.h"       
#include "ns3/string.h"       
#include <fstream>
#include <sstream>
#include <iostream>

NS_LOG_COMPONENT_DEFINE ("GsqrEmbedding");

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (GsqrEmbedding);

TypeId 
GsqrEmbedding::GetTypeId (void)
{
    static TypeId tid = TypeId ("ns3::GsqrEmbedding")
        .SetParent<Object> ()
        .SetGroupName ("Gsqr")
        .AddConstructor<GsqrEmbedding> ()
        .AddAttribute ("Dimension",
                       "Dimension of embedding vectors",
                       UintegerValue (16),
                       MakeUintegerAccessor (&GsqrEmbedding::m_dimension),
                       MakeUintegerChecker<uint32_t> (1, 64))  
        ;
    return tid;
}

GsqrEmbedding::GsqrEmbedding ()
{
    NS_LOG_FUNCTION (this);
}

GsqrEmbedding::~GsqrEmbedding ()
{
    NS_LOG_FUNCTION (this);
}

bool GsqrEmbedding::LoadFromCSV(const std::string& filename)
{
    NS_LOG_FUNCTION (this << filename);
    
    std::ifstream file(filename);
    if (!file.is_open()) {
        NS_LOG_ERROR ("Cannot open embedding file: " << filename);
        return false;
    }
    
    std::string line;
    m_embeddings.clear();
    m_biases.clear();
    
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string item;
        std::vector<double> embedding;
        
        if (!std::getline(ss, item, ',')) {
            NS_LOG_WARN ("Invalid line format: " << line);
            continue;
        }
        
        uint32_t nodeId;
        try {
            nodeId = std::stoul(item);
        } catch (...) {
            NS_LOG_WARN ("Invalid node ID: " << item);
            continue;
        }
        
        embedding.clear();
        for (int i = 0; i < 16; ++i) {
            if (!std::getline(ss, item, ',')) {
                NS_LOG_WARN ("Incomplete embedding for node " << nodeId);
                break;
            }
            try {
                embedding.push_back(std::stod(item));
            } catch (...) {
                NS_LOG_WARN ("Invalid embedding value: " << item);
                embedding.push_back(0.0);
            }
        }
        
        if (embedding.size() != 16) {
            NS_LOG_WARN ("Embedding dimension mismatch for node " << nodeId);
            continue;
        }
        
        if (!std::getline(ss, item, ',')) {
            NS_LOG_WARN ("Missing bias for node " << nodeId);
            continue;
        }
        
        double bias;
        try {
            bias = std::stod(item);
        } catch (...) {
            NS_LOG_WARN ("Invalid bias value: " << item);
            bias = 0.0;
        }
        
        m_embeddings[nodeId] = embedding;
        m_biases[nodeId] = bias;
        
        NS_LOG_DEBUG ("Loaded embedding for node " << nodeId << 
                     ", bias: " << bias);
    }
    
    file.close();
    NS_LOG_INFO ("Loaded " << m_embeddings.size() << " embeddings from " << filename);
    return true;
}

bool GsqrEmbedding::SaveToCSV(const std::string& filename)
{
    NS_LOG_FUNCTION (this << filename);
    
    std::ofstream file(filename);
    if (!file.is_open()) {
        NS_LOG_ERROR ("Cannot create file: " << filename);
        return false;
    }
    
    for (const auto& entry : m_embeddings) {
        uint32_t nodeId = entry.first;
        const auto& embedding = entry.second;
        
        if (embedding.size() != 16) {
            NS_LOG_WARN ("Embedding dimension mismatch for node " << nodeId);
            continue;
        }
        
        file << nodeId;
        for (double val : embedding) {
            file << "," << val;
        }
        
        auto biasIt = m_biases.find(nodeId);
        double bias = (biasIt != m_biases.end()) ? biasIt->second : 0.0;
        file << "," << bias << "\n";
    }
    
    file.close();
    NS_LOG_INFO ("Saved " << m_embeddings.size() << " embeddings to " << filename);
    return true;
}

const std::vector<double>& GsqrEmbedding::GetEmbedding(uint32_t nodeId) const
{
    auto it = m_embeddings.find(nodeId);
    if (it != m_embeddings.end()) {
        return it->second;
    }
    
    // Returns a zero vector if the node has no pre-trained embeddings
    NS_LOG_WARN ("No embedding found for node " << nodeId << ", using zero vector");
    static const std::vector<double> zeroVector(16, 0.0);
    return zeroVector;
}

double GsqrEmbedding::GetBias(uint32_t nodeId) const
{
    auto it = m_biases.find(nodeId);
    if (it != m_biases.end()) {
        return it->second;
    }
    
    NS_LOG_WARN ("No bias found for node " << nodeId << ", using 0.0");
    return 0.0;
}

void GsqrEmbedding::SetEmbedding(uint32_t nodeId, const std::vector<double>& embedding)
{
    if (embedding.size() != 16) {
        NS_LOG_ERROR ("Embedding dimension must be 16, got " << embedding.size());
        return;
    }
    m_embeddings[nodeId] = embedding;
}

void GsqrEmbedding::SetBias(uint32_t nodeId, double bias)
{
    m_biases[nodeId] = bias;
}

void GsqrEmbedding::UpdateEmbedding(uint32_t nodeId, 
                                   const std::vector<double>& gradient, 
                                   double learningRate)
{
    auto it = m_embeddings.find(nodeId);
    if (it == m_embeddings.end()) {
        // Create a zero vector if the node has no initial embedding
        m_embeddings[nodeId] = std::vector<double>(16, 0.0);
        it = m_embeddings.find(nodeId);
    }
    
    // Check gradient dimensions
    if (gradient.size() != 16) {
        NS_LOG_ERROR ("Gradient dimension must be 16, got " << gradient.size());
        return;
    }
    
    //  h = h + α * gradient
    for (size_t i = 0; i < 16; ++i) {
        it->second[i] += learningRate * gradient[i];
    }
    
    NS_LOG_DEBUG ("Updated embedding for node " << nodeId);
}

void GsqrEmbedding::UpdateBias(uint32_t nodeId, double gradient, double learningRate)
{
    m_biases[nodeId] += learningRate * gradient;
    NS_LOG_DEBUG ("Updated bias for node " << nodeId << " to " << m_biases[nodeId]);
}

} // namespace ns3
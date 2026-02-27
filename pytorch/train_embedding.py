#!/usr/bin/env python3
"""
GraphSAGE Embed training script
Generate 16-dimensional node embedding for GSQR
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch_geometric.nn import SAGEConv
import numpy as np
import networkx as nx
from sklearn.preprocessing import StandardScaler
import pandas as pd
import matplotlib.pyplot as plt
import os

class GraphSAGE(nn.Module):
    """The GraphSAGE model generates a 16-dimensional embedding."""
    def __init__(self, in_channels, hidden_channels, out_channels):
        super(GraphSAGE, self).__init__()
        self.conv1 = SAGEConv(in_channels, hidden_channels)
        self.conv2 = SAGEConv(hidden_channels, out_channels)
        
    def forward(self, x, edge_index):
        x = self.conv1(x, edge_index)
        x = F.relu(x)
        x = F.dropout(x, p=0.2, training=self.training)
        x = self.conv2(x, edge_index)
        return x

def generate_uav_network(num_nodes=30, area_size=1000):
    """Generate drone network diagram"""
    print(f"Generate {num_nodes} nodes UAV network...")
    
    # 1. Generate random positions (3D space)
    positions = np.random.rand(num_nodes, 3) * [area_size, area_size, 150]
    
    # 2. Constructing adjacency matrix based on distance
    adjacency = np.zeros((num_nodes, num_nodes))
    communication_range = 250  # Communication range: 250 meters
    
    for i in range(num_nodes):
        for j in range(i+1, num_nodes):
            dist = np.linalg.norm(positions[i] - positions[j])
            if dist < communication_range:
                adjacency[i][j] = 1
                adjacency[j][i] = 1
    
    # 3. Construct node features [ETX, remaining energy, queue length]
    features = np.zeros((num_nodes, 3))
    for i in range(num_nodes):
        features[i, 0] = np.random.uniform(0.5, 2.0)  # ETX: 0.5-2.0
        features[i, 1] = np.random.uniform(0.3, 1.0)  # remaining energy: 30%-100%
        features[i, 2] = np.random.uniform(0.0, 0.8)  # queue length: 0%-80%
    
    # 4. Convert to NetworkX diagram for visualization
    G = nx.Graph()
    for i in range(num_nodes):
        G.add_node(i, pos=positions[i], features=features[i])
    
    for i in range(num_nodes):
        for j in range(i+1, num_nodes):
            if adjacency[i][j] == 1:
                G.add_edge(i, j, weight=1.0)
    
    print(f"  Average degree: {np.mean(np.sum(adjacency, axis=1)):.2f}")
    print(f"  Connect components: {nx.number_connected_components(G)}")
    
    return positions, adjacency, features, G

def train_embeddings(num_nodes=30, num_epochs=50):
    """Training GraphSAGE embeddings"""
    print("\n=== Start GraphSAGE training ===")
    
    # parameter
    input_dim = 3      # ETX, E_r, q
    hidden_dim = 32
    output_dim = 16    # 16 Dim
    learning_rate = 0.01
    
    # Generate training data
    positions, adjacency, features, G = generate_uav_network(num_nodes)
    
    # Convert to PyTorch Geometric format
    edge_index = []
    for i in range(num_nodes):
        for j in range(num_nodes):
            if adjacency[i][j] == 1:
                edge_index.append([i, j])
    
    if len(edge_index) == 0:
        print("Error: The image has no edges!")
        return None
    
    edge_index = torch.tensor(edge_index, dtype=torch.long).t()
    features = torch.tensor(features, dtype=torch.float)
    
    # standardized feature
    scaler = StandardScaler()
    features_np = features.numpy()
    features_np = scaler.fit_transform(features_np)
    features = torch.tensor(features_np, dtype=torch.float)
    
    # initialization model
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"Device: {device}")
    
    model = GraphSAGE(input_dim, hidden_dim, output_dim).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)
    
    # Move data to the device
    features = features.to(device)
    edge_index = edge_index.to(device)
    
    # Training cycle
    losses = []
    model.train()
    
    for epoch in range(num_epochs):
        optimizer.zero_grad()
        
        # forward 
        embeddings = model(features, edge_index)
        
        # Use negative sampling loss (simplified)
        # Positive sample: The actual connected node pair
        # Negative sample: Randomly sampled node pairs
        
        pos_samples = edge_index.t() 
        num_pos = pos_samples.size(0)
        
        # Generate negative samples 
        # (randomly disconnected node pairs)
        neg_samples = []
        while len(neg_samples) < num_pos:
            i = np.random.randint(0, num_nodes)
            j = np.random.randint(0, num_nodes)
            if i != j and adjacency[i][j] == 0:
                neg_samples.append([i, j])
        
        neg_samples = torch.tensor(neg_samples, dtype=torch.long).to(device)
        
        # Calculate the similarity of positive samples
        pos_similarity = torch.sum(
            embeddings[pos_samples[:, 0]] * embeddings[pos_samples[:, 1]], 
            dim=1
        )
        
        # Calculate negative sample similarity
        neg_similarity = torch.sum(
            embeddings[neg_samples[:, 0]] * embeddings[neg_samples[:, 1]], 
            dim=1
        )
        
        # Loss function: Maximize positive sample similarity 
        # and minimize negative sample similarity
        loss = -torch.log(torch.sigmoid(pos_similarity)).mean() \
               - torch.log(1 - torch.sigmoid(neg_similarity)).mean()
        
        loss.backward()
        optimizer.step()
        
        losses.append(loss.item())
        
        if (epoch + 1) % 10 == 0:
            print(f'Epoch {epoch+1}/{num_epochs}, Loss: {loss.item():.4f}')
    
    # Get final embedding
    model.eval()
    with torch.no_grad():
        final_embeddings = model(features, edge_index).cpu().numpy()
    
    # Generate bias (based on node features)
    biases = np.random.randn(num_nodes) * 0.1
    
    print(f"Training completed, final loss: {losses[-1]:.4f}")
    
    # training loss function
    plt.figure(figsize=(10, 6))
    plt.plot(losses)
    plt.xlabel('Epoch')
    plt.ylabel('Loss')
    plt.title('GraphSAGE Training Loss')
    plt.grid(True, alpha=0.3)
    plt.savefig('training_loss.png', dpi=150, bbox_inches='tight')
    plt.close()
    
    return final_embeddings, biases, G

def save_embeddings(embeddings, biases, filename='emb_16.csv'):
    """Save embedded in CSV file"""
    df = pd.DataFrame(embeddings)
    df.columns = [f'h{i}' for i in range(16)]
    df['bias'] = biases
    df.index.name = 'node_id'
    
    # Ensure the outputs directory exists
    os.makedirs('outputs', exist_ok=True)
    filepath = f'outputs/{filename}'
    
    df.to_csv(filepath)
    print(f"\nEmbedded saved to: {filepath}")
    print(f"  Number of nodes: {len(df)}")
    print(f"  Embedding dimension: 16")
    print(f"  file size: {os.path.getsize(filepath) / 1024:.1f} KB")
    
    # Show the first few rows
    print("\nExample of embedding the first three nodes:")
    print(df.head(3).round(4))
    
    return filepath

def visualize_embeddings(embeddings, G, filename='embedding_visualization.png'):
    """Visualization Embedding (2D PCA)"""
    from sklearn.decomposition import PCA
    
    # Use PCA to reduce dimensions to 2D
    pca = PCA(n_components=2)
    embeddings_2d = pca.fit_transform(embeddings)
    
    plt.figure(figsize=(12, 10))
    
    # Draw nodes
    scatter = plt.scatter(embeddings_2d[:, 0], embeddings_2d[:, 1], 
                         c=range(len(embeddings)), cmap='viridis', 
                         s=100, alpha=0.8, edgecolors='black')
    
    # Add node label
    for i in range(len(embeddings)):
        plt.annotate(str(i), (embeddings_2d[i, 0], embeddings_2d[i, 1]),
                    fontsize=8, ha='center', va='center')
    
    # draw networkx edges
    pos_dict = {i: embeddings_2d[i] for i in range(len(embeddings))}
    nx.draw_networkx_edges(G, pos_dict, alpha=0.2, edge_color='gray')
    
    plt.colorbar(scatter, label='Node ID')
    plt.xlabel('PCA Component 1')
    plt.ylabel('PCA Component 2')
    plt.title('GraphSAGE Embeddings (2D PCA Projection)')
    plt.grid(True, alpha=0.3)
    
    filepath = f'outputs/{filename}'
    plt.savefig(filepath, dpi=150, bbox_inches='tight')
    plt.close()
    
    print(f"Embedded visualization saved to: {filepath}")
    print(f"  PCA explained variance ratio: {pca.explained_variance_ratio_.sum():.3f}")

if __name__ == "__main__":
    print("=" * 60)
    print("GSQR - GraphSAGE training embedding")
    print("=" * 60)
    
    np.random.seed(42)
    torch.manual_seed(42)
    
    # training parameters
    num_nodes = 30      
    num_epochs = 50     
    
    try:
        # training embedding
        embeddings, biases, G = train_embeddings(num_nodes, num_epochs)
        
        if embeddings is not None:
            # Save embedded
            csv_file = save_embeddings(embeddings, biases)
            
            # Visualization embedding
            visualize_embeddings(embeddings, G)
            
            print("\n✅ Training completed")
            print("next step:")
            print("1. Used in ns-3 simulation outputs/emb_16.csv")
        else:
            print("❌ Training failed")
            
    except Exception as e:
        print(f"❌ Error during training: {e}")
        import traceback
        traceback.print_exc()

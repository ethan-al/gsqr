# Lightweight Graph Embedding for Adaptive Routing in Resource-Constrained UAV Networks

[![NS-3](https://img.shields.io/badge/NS--3-3.41-blue)](https://www.nsnam.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
<!-- [![Preprint](https://img.shields.io/badge/Preprint-TechRxiv-orange)](https://doi.org/your-doi) -->

**Associated manuscript under review at IEEE Signal Processing Letters (2026).**  
Preprint will be available on TechRxiv soon.

This repository contains the source code for the paper

> **"Lightweight Graph Embedding for Adaptive Routing in Resource-Constrained UAV Networks"**  

## 📁 Repository Structure

```
gsqr/
├── model/          # GSQR protocol core implementation
├── helper/         # NS-3 helper classes
├── examples/       # Simulation script (gsqr-comparison.cc)
├── pytorch/        # Optional pre-training scripts
└── results/        # Output files (auto-generated)
```

## 🔧 Requirements

- **NS-3.41** (released December 2023)  
  ⚠️ Other versions (ns-3.40, ns-3-dev) may cause build failures due to API changes.

Download NS-3.41 from:
- [GitLab repository](https://gitlab.com/nsnam/ns-3.41)
- [Official release tarball](https://www.nsnam.org/releases/ns-allinone-3.41.tar.bz2)

## 🚀 Quick Start

### 1. Clone the repository into NS-3 source

```bash
cd /path/to/ns-3.41/src
git clone https://github.com/ethan-al/gsqr.git
# After cloning, you should have: /path/to/ns-3.41/src/gsqr/
```

### 2. Build NS-3 with the module

```bash
cd /path/to/ns-3.41
./ns3 configure --enable-examples
./ns3 build
```

### 3. Run simulations

```bash
# Run the submission version (v1.0)
git checkout v1.0-submission
./ns3 run "gsqr-comparison"

# Note: v2.0-enhanced is under development and not yet available.
# Only v1.0-submission is currently released.

# Basic run (30 nodes, 1 seed, 60 seconds)
./ns3 run "gsqr-comparison"

# Custom parameters
./ns3 run "gsqr-comparison --maxNodes=10 --seeds=5 --time=30"

# Quick mode (only N=30, for testing)
./ns3 run "gsqr-comparison --quick"
```

### Available parameters

You can customize the simulation with the following command-line parameters:

| Parameter | Description | Default |
|-----------|-------------|---------|
| `--maxNodes=N` | Maximum number of UAVs | 30 |
| `--seeds=S` | Number of random seeds | 1 |
| `--time=T` | Simulation time per run (seconds) | 60 |
| `--quick` | Quick mode (only runs N=30) | false |

## 📊 Output

Results are automatically saved in:

```
results/
├── gsqr_results.txt      # Main numerical results (PDR, delay, NRO, etc.)
└── gsqr_simulation.log   # Detailed simulation log
```

The `results/` folder is created automatically—no need to create it manually.
To reproduce paper results, run with default parameters.

## 🧠 PyTorch Training Code (Historical / Optional)

This directory contains the original PyTorch scripts used in early development (v0.5) to generate GraphSAGE embeddings (`emb_16.csv`) for warm-start initialization.

However, we discovered that **GSQR achieves superior performance through pure online learning** — initializing embeddings randomly and updating them during simulation. As a result, the final v1.0 implementation **does not require or use any pre-trained files**.

The scripts are retained for transparency and potential future comparison.

### Files
- `pytorch/train_embedding.py` – Training script (outputs to `outputs/`)
- `pytorch/requirements.txt` – Python dependencies

## 📌 Versioning

- **v1.0-submission** – Exact code used to generate results in the submitted manuscript (SPL-46114)
- **v2.0-enhanced** – *(Coming soon)* Improved version with accurate convergence time measurement

To switch between versions:
```bash
git checkout v1.0-submission   # for submission results
git checkout v2.0-enhanced      # for enhanced features
```
⚠️ v2.0-enhanced is under development — use v1.0-submission to reproduce the submitted paper results.

## 📝 Important Notes

- **Do not modify files outside the `gsqr/` folder**—this ensures compatibility with NS-3.
- This implementation is validated on **NS-3.41 only**.
- This implementation is distributed under the MIT License, compatible with both academic and commercial use.

## 📚 Citation

If you use this code in your research, please cite:

```
@article{wang2026lightweight,
  title={Lightweight Graph Embedding for Adaptive Routing in Resource-Constrained UAV Networks},
  author={Wang, Haoyu},
  journal={IEEE Signal Processing Letters},
  year={2026},
  note={Submitted. Preprint: \url{https://doi.org/...}}
}
```

## 🤝 Acknowledgments

This work uses NS-3 (https://www.nsnam.org/), which is distributed under the GNU GPLv2 license.

---

**For questions or issues, please open an issue on GitHub.**

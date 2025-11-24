<p align="left">
  <img src="./src/image/ezreal.jpg" width="120" style="vertical-align: middle; margin-right: 15px;" />
  <span style="font-size: 28px; font-weight: bold;">
    EZREAL: Enhancing Zero-Shot Outdoor Robot Navigation toward Distant Targets under Varying Visibility
  </span>
  <br>
</p>
---

# ⭐️ **Latest Update**
🔥 **The Core System has been fully implemented and pushed.**  
👉 **Please check the [`master`](https://github.com/louiszengCN/EzReal/tree/master) branch for the latest code.**
- [🌐 Project Page](https://tianlezeng.github.io/EzReal/)  
- [📄 Paper on arXiv](https://arxiv.org/abs/2509.13720)

---

## 🔓 Open-Source Plan (Updated)

We will open-source EZREAL in two stages:

---

### 1️⃣ ~~Core System (Releasing Soon)~~ **✔️ Core System Released**

The following core modules have already been released on the `master` branch:

- **Multi-scale tile perception**
- **Saliency-enhanced distant target detection**
- **Visibility-robust heading estimation**
- **ROS-ready implementation for simulation and real robots**
- **Real-time advanced frontier detector (>100Hz in test)**

This part is **fully functional** and supports complete evaluation of the paper’s main capability:

🚀 **Zero-Shot Outdoor Navigation (ZSON)**  
📍 **Robust navigation toward distant targets**  
🌫 **Stable performance under varying visibility & occlusion**

We also provide visualization tools that clearly illustrate each module of the system.

---

### 2️⃣ Planner Module (Coming Soon)

We are currently improving the planning and active search module.

The following components will be released later:

- **Active search strategy**
- **Fallback navigation for long-term occlusion**

These modules are **independent** of the core system and **not required** to test the main results.

The core system already works with **any existing planner** that subscribes to the chosen *frontier goal* topic.

---

**Updating date: 11.24**

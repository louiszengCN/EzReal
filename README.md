<p align="left">
  <img src="./src/image/ezreal.jpg"   width="120" style="vertical-align: middle; margin-right: 15px;" />
  <span style="font-size: 28px; font-weight: bold;">
    EZREAL: Enhancing Zero-Shot Outdoor Robot Navigation toward Distant Targets under Varying Visibility
  </span>
</p>

---
## 🔓 Open-Source Plan

We will open-source EZREAL in two parts:

### 1️⃣ Core System (Releasing Soon)
We will first release the **core implementation** used in the paper, including:

- Multi-scale tile perception  
- Saliency-enhanced distant target detection  
- Visibility-robust heading estimation  
- ROS-ready implementation for simulation and real robots  
- Real-time advanced frontier detector (>100Hz in our test)

This part is **fully functional** and supports complete testing of the paper’s main capability:  
**Zero-Shot Outdoor Navigation (ZSON) toward distant targets under varying visibility**. \
We also provide extensive visualization tools that clearly illustrate each core module of the system.

### 2️⃣ Planner Module (Released Later)
We are currently improving the planner.  
Therefore, modules related to:

- **Active search**
- **Fallback navigation under long-term occlusion**


will be released a bit later.

These planner components are *independent of the core system* and **do not affect** testing the main results.  
The core system can work with **any external planner** that subscribes to frontier point (ROS topic with coordinate)

More updates will follow.

Upating date: 11.21

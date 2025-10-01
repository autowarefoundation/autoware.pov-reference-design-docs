# Evaluation and Testing

(To be completed)

The evaluation for each model can be found on the GitHub repo of the PoV. 

- SceneSeg: [link](https://github.com/autowarefoundation/autoware.privately-owned-vehicles/tree/main/SceneSeg#performance-results)

- DomainSeg: [link](https://github.com/autowarefoundation/autoware.privately-owned-vehicles/tree/main/DomainSeg#performance-results)

- Scene3D: [link](https://github.com/autowarefoundation/autoware.privately-owned-vehicles/tree/main/Scene3D)

- EgoPath: [link](https://github.com/autowarefoundation/autoware.privately-owned-vehicles/tree/main/EgoPath)

## System Evaluation

The section shows the benchmark results of the VisionPilot model on different hardware environment as the references. There are two procedures to conduct the benchmark:

- Just-based: [link](https://autowarefoundation.github.io/autoware.pov-reference-design-docs/main/software-configuration/zenoh/#usage)

- Make-base: [link](https://github.com/NEWSLabNTU/2025-vision-pilot-benchmark)

### ADLINK AVA-3510

#### Hardware Spec


- **CPU**:  Intel Xeon E-2278GE (16 cores)
- **GPU**: NVIDIA Quadro RTX 5000.
- **Memory**: 64GB LPDDR5/16G on RTX 5000
- **Driver**: 
- **ROS**: ROS Humble with Autoware recommended Cyclone DDS settings.
- **Runtime**: TensorRT
- **OS**: Ubuntu 24.04.3

#### Benchmark Result

- Zenoh:


| Model  | CPU Utilization | GPU Utilization | Peak Memory Usage  | Frame Rate  |
|:---:|:---:|:---:|:---:|:---:|
| SceneSeg   |   |   |   | 58  |
| DomainSeg  |   |   |   | 58  |
| Scene3D  |   |   |   | 57  |
| EgoSpace |   |   |   |   |

    - SceneSeg
  
    ```raw
    Current FPS: 58
    --- Per-frame Timing (microseconds) ---
    * Total processing time: 17229 us
    * Preprocessing time: 1200 us
    * Inference time: 15829 us      
    * Output time: 199 us
    ```

    - DomainSeg

    ```raw
    Current FPS: 58
    --- Per-frame Timing (microseconds) ---
    * Total processing time: 16625 us
    * Preprocessing time: 1203 us
    * Inference time: 15250 us
    * Output time: 171 us
    ```

    - Scene3D

    ```raw
    Current FPS: 57
    --- Per-frame Timing (microseconds) ---
    * Total processing time: 17484 us
    * Preprocessing time: 204 us
    * Inference time: 16817 us
    * Output time: 462 us
    ```

- ROS 2:

| Model  | CPU Utilization | GPU Utilization | Peak Memory Usage  | Frame Rate  |
|:---:|:---:|:---:|:---:|:---:|
| SceneSeg   |   |   |   | 60  |
| DomainSeg  |   |   |   | 60  |
| Scene3D  |   |   |   | 60  |
| EgoSpace |   |   |   |   |

    - SceneSeg
  
    ```raw
    * Current FPS: 60
    --- Per-frame Timing (microseconds) --- 
    * Total processing time: 16272 us
    * Preprocessing time: 178 us
    * Inference time: 15713 us
    * Output time: 381 us
    ```

    - DomainSeg

    ```raw
    * Current FPS: 60.00
    --- Per-frame Timing (microseconds) --- 
    * Total processing time: 15393 us
    * Preprocessing time: 253 us
    * Inference time: 14672 us
    * Output time: 467 us
    -------------------------- 
    ```

### ARM processors and nVidia AGX Orin
#### Hardware spec:
- **CPU**: 12-core ARM Cortex-A78AE CPU at 2.2GHz.
- **GPU**: NVIDIA Ampere GPU with 2048 CUDA Cores.
- **Memory**: 64GB LPDDR5. The system and GPU memories are shared.
- **Driver**: The NVIDIA JetPack 6.0 (Ubuntu 22.04 LTS based) was used.
- **ROS**: ROS Humble with Autoware recommended Cyclone DDS settings.
- **Runtime**: ONNX runtime  1.19.0 or TensorRT

#### Benchmark results: 

| Model  | CPU Utilization | GPU Utilization | Peak Memory Usage  | Frame Rate  |
|:---:|:---:|:---:|:---:|:---:|
| SceneSeg  <br> (ONNX runtime)| 91%  ~ 99%  | 99%  | 45G <br> including network model (~30G) + other process (15G)  | 8  |
| SceneSeg  <br> (TensorRT runtime)  |   |   |   |   |
| DomainSeg  |   |   |   |   |
| Scene3D  |   |   |   |   |
| EgoSpace |   |   |   |   |


  - Demo Video: [link](https://drive.google.com/file/d/1P6NPrnKex2EkNgzlvM20Ap6YoPxFmVgl/view?usp=drive_link)


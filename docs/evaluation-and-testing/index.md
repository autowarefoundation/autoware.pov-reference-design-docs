# Evaluation and Testing

The evaluation for each model can be found on the GitHub repo of the PoV. 

- SceneSeg: [link](https://github.com/autowarefoundation/autoware.privately-owned-vehicles/tree/main/SceneSeg#performance-results)

- DomainSeg: [link](https://github.com/autowarefoundation/autoware.privately-owned-vehicles/tree/main/DomainSeg#performance-results)

- Scene3D: [link](https://github.com/autowarefoundation/autoware.privately-owned-vehicles/tree/main/Scene3D)

- EgoPath: [link](https://github.com/autowarefoundation/autoware.privately-owned-vehicles/tree/main/EgoPath)

## System Evaluation Results

The section shows the benchmark results of the VisionPilot model on different hardware environment as the references. There are two procedures to conduct the benchmark:

- Just-based: [link](https://autowarefoundation.github.io/autoware.pov-reference-design-docs/main/software-configuration/zenoh/#usage)

- Make-base: [link](https://github.com/NEWSLabNTU/2025-vision-pilot-benchmark)

Two sets of computation configurations are used to benchmark the pipeline:

- X86-based Computer: [link](index.md#adlink-adm-al30)
- ARM-based Computer: [link](index.md#arm-processors-and-nvidia-agx-orin)

### ADLINK ADM-AL30

#### Hardware Spec 

- **CPU**:  Intel 12th Gen Core i7-12700E (12 cores / 20 threads)
- **GPU**: NVIDIA RTX 4000 SFF Ada (20 GB VRAM)
- **Memory**: 128 GB DDR5 ECC
- **Driver**: Driver Version: 580.95.05 & CUDA Version: 13.0
- **ROS**: ROS Humble & Zenoh
- **Runtime**: TensorRT
- **OS**: Ubuntu 22.04.5

[link to AL30 (
Autonomous Driving Solutions)](https://www.adlinktech.com/products/Automotive-Computing/Autonomous-Driving/ADM-AL30?lang=en)

#### Benchmark Result

- Zenoh:

    | Model  | CPU Utilization | GPU Utilization | Peak Memory Usage  | Frame Rate  |
    |:---:|:---:|:---:|:---:|:---:|
    | SceneSeg   | 7%  | 37%  | 3.33G  | 53.19  |
    | DomainSeg  | 7%  | 40%  | 3.37G  | 53.33  |
    | Scene3D  | 10%  | 38%  |  3.22G | 43.87  |
    | EgoSpace |   |   |   |   |

    - SceneSeg
  
    ```raw
    * Current FPS: 53.19
    --- Per-frame Timing (microseconds) ---
    * Total processing time: 23336 us
    * Preprocessing time: 496 us
    * Inference time: 22028 us
    * Output time: 811 us
    ```

    - DomainSeg

    ```raw
    * Current FPS: 53.33
    --- Per-frame Timing (microseconds) ---
    * Total processing time: 22492 us
    * Preprocessing time: 585 us
    * Inference time: 21522 us
    * Output time: 384 us
    ```

    - Scene3D

    ```raw
    * Current FPS: 43.87
    --- Per-frame Timing (microseconds) ---
    * Total processing time: 22202 us
    * Preprocessing time: 797 us
    * Inference time: 20115 us
    * Output time: 1289 us
    ```

- ROS 2:

    | Model  | CPU Utilization | GPU Utilization | Peak Memory Usage  | Frame Rate  |
    |:---:|:---:|:---:|:---:|:---:|
    | SceneSeg   | 7% | 39% | 2.89G | 46.67  |
    | DomainSeg  | 7% | 41% | 2.98G | 53.85  |
    | Scene3D  | 6% | 40% | 2.76G | 50  |
    | EgoSpace |   |   |   |   |

    - SceneSeg
  
    ```raw
    * Current FPS: 46.67
    --- Per-frame Timing (microseconds) --- 
    * Total processing time: 23795 us
    * Preprocessing time: 739 us
    * Inference time: 22058 us
    * Output time: 996 us
    ```

    - DomainSeg

    ```raw
    * Current FPS: 53.85
    --- Per-frame Timing (microseconds) --- 
    * Total processing time: 17951 us
    * Preprocessing time: 746 us
    * Inference time: 16390 us
    * Output time: 814 us
    ```

    - Scene3D

    ```raw
    * Current FPS: 50.00
    --- Per-frame Timing (microseconds) ---
    * Total processing time: 12981 us
    * Preprocessing time: 211 us
    * Inference time: 11026 us
    * Output time: 1743 us
    ```

### ARM processors and nVidia AGX Orin
#### Hardware spec:
- **CPU**: 12-core ARM Cortex-A78AE CPU at 2.2GHz.
- **GPU**: NVIDIA Ampere GPU with 2048 CUDA Cores.
- **Memory**: 64GB LPDDR5. The system and GPU memories are shared.
- **Driver**: The NVIDIA JetPack 6.0 (Ubuntu 22.04 LTS based) was used.
- **ROS**: ROS Humble with Autoware recommended Cyclone DDS settings.
- **Runtime**: ONNX runtime  1.19.0 or TensorRT

[link to nVidia Jetson Orin AGX](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/)

#### Benchmark results: 

| Model  | CPU Utilization | GPU Utilization | Peak Memory Usage  | Frame Rate  |
|:---:|:---:|:---:|:---:|:---:|
| SceneSeg  <br> (ONNX runtime)| 91%  ~ 99%  | 99%  | 45G <br> including network model (~30G) + other process (15G)  | 8  |
| SceneSeg  <br> (TensorRT runtime - FP16)  |  57 ~ 66 %  |74 %	  |0.8 % (~0.50 GB)  | 29.12  |
| DomainSeg <br> (TensorRT runtime - FP16) | 56 ~ 60 % | 88 %  | 0.8 % (~0.50 GB)  | 29.85  |
| Scene3D <br> (TensorRT runtime - FP16) | 53 ~ 56 % |82 %  | 0.6 % (~0.38 GB)  | 29.90  |
| SceneSeg <br> (TensorRT runtime - FP32)  |  42 ~ 49 % |  99 % | 0.6 % (~0.38 GB)  | 17.10  |
| DomainSeg <br> (TensorRT runtime - FP32) | 43 ~ 47 %  | 99 %  | 0.6 % (~0.38 GB)  |  17.07  |
| Scene3D <br> (TensorRT runtime - FP32) | 44 ~ 46 %  |99 %  | 0.6 % (~0.38 GB)  |  17.03  |


[link](https://github.com/NEWSLabNTU/2025-vision-pilot-benchmark/tree/main) to the instructions and complete results.


- Demo Video: [link](https://drive.google.com/file/d/1P6NPrnKex2EkNgzlvM20Ap6YoPxFmVgl/view?usp=drive_link)

### Advantech AFE-R750
#### Hardware spec:
- **CPU**: 8-core NVIDIA Arm® Cortex A78AE v8.2.
- **GPU**: 1792-core NVIDIA Ampere GPU with 56 Tensor Cores.
- **Memory**: 32GB LPDDR5.
- **Driver**: The NVIDIA JetPack 6.1.
- **ROS**: ROS Humble.
- **Runtime**: TensorRT

[link to Advantech AFE-R750](https://www.advantech.com/zh-tw/products/8d5aadd0-1ef5-4704-a9a1-504718fb3b41/afe-r750/mod_779d2a74-61d9-4d78-a4e0-2ca07afbd64b)

#### Benchmark results: 

| Model  | CPU Utilization | GPU Utilization | Peak Memory Usage  | Frame Rate  |
|:---:|:---:|:---:|:---:|:---:|
| SceneSeg  <br> (TensorRT runtime - FP16)  |  45 ~ 50 %  |80 %	  |< 0.50 GB  | 21  |
| DomainSeg <br> (TensorRT runtime - FP16) | 55 ~ 60 % | 90 %  | < 0.50 GB  | 21  |
| Scene3D <br> (TensorRT runtime - FP16) | 40 ~ 45 % | 85 %  | < 0.40 GB  | 22  |
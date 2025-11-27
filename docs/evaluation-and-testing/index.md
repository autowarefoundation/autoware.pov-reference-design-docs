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

Two sets of computation configurations are used to benchmark the pipile:

- X86-based Computer: [link](index.md#adlink-ava-3510)
- ARM-based Computer: [link](index.md#arm-processors-and-nvidia-agx-orin)

### ADLINK AVA-3510

#### Hardware Spec 

- **CPU**:  Intel Xeon E-2278GE (16 cores)
- **GPU**: NVIDIA Quadro RTX 5000.
- **Memory**: 64GB LPDDR5/16G on RTX 5000
- **Driver**: Driver Version: 580.82.07 & CUDA Version: 13.0
- **ROS**: ROS Jazzy & Zenoh
- **Runtime**: TensorRT
- **OS**: Ubuntu 24.04.3

[link to 3510 (discontinued)](https://www.adlinktech.com/products/automotive-computing/autonomous-driving/ava-3510?lang=en)

[link to AL30 (
Autonomous Driving Solutions)](https://www.adlinktech.com/products/Automotive-Computing/Autonomous-Driving/ADM-AL30?lang=en)

#### Benchmark Result

- Zenoh:

    | Model  | CPU Utilization | GPU Utilization | Peak Memory Usage  | Frame Rate  |
    |:---:|:---:|:---:|:---:|:---:|
    | SceneSeg   | 13%  | 65%  | 20G  | 58  |
    | DomainSeg  | 14%  | 62%  | 21G  | 58  |
    | Scene3D  | 15%  | 69%  |  19G | 57  |
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
    | SceneSeg   | 17% | 72% | 6G | 60  |
    | DomainSeg  | 16% | 68% | 6G | 60  |
    | Scene3D  | 16% | 67% | 5.9G | 60  |
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
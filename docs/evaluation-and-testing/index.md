# Evaluation and Testing

(To be completed)

## Model Evaluation

- SceneSeg: https://github.com/autowarefoundation/autoware.privately-owned-vehicles/tree/main/SceneSeg#performance-results
- DomainSeg: https://github.com/autowarefoundation/autoware.privately-owned-vehicles/tree/main/DomainSeg#performance-results

## System Evaluation

The overall performance for the whole pipeline.

### ADLINK AVA-3510

#### Hardware Spec

- OS: Ubuntu 24.04.3
- CPU: Intel Xeon E-2278GE (16 cores)
- GPU: NVIDIA Quadro RTX 5000
- Memory: 64 GB

#### Benchmark Result

- Zenoh:

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

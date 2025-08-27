# Scene3D

## Overview

Scene3D is able process monocular camera images to produce high resolution depth maps with sharp object boundaries, visible on the leaves of trees, thin structures such as poles, and on the edges of foreground objects - helping self-driving cars understand the dynamic driving scene in real-time. Scene3D enables important downstream perception tasks such as foreground obstacle detection, and is robust to changes in object appearance, size, shape and type, addressing 'long-tail' edge case scenarios. The current release of Scene3D estimates per-pixel relative depth, indicating which objects are nearer vs further away from the camera. Scene3D is part of the AutoSeg Foundation Model which forms the core of the vision-pipeline of the Autoware Autonomous Highway Pilot System.

## Requirements
```
dgp==0.1.3
matplotlib==3.5.3
Models==0.9.7
numpy==2.2.5
opencv_contrib_python==4.10.0.84
opencv_python==4.10.0.84
opencv_python_headless==4.11.0.86
Pillow==11.3.0
```

## Demo
Please see the [Models](https://github.com/autowarefoundation/autoware.privately-owned-vehicles/tree/main/Models) folder to access the pre-trained network weights for Scene3D as well as scripts for network training, inference and visualization of network predictions.

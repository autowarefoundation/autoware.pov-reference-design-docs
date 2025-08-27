# SceneSeg

## Overview

 SceneSeg is a neural network that is able to segment all important foreground objects, irrespective of what that object is. SceneSeg is able to implicitly learn the visual features of foreground objects such as cars, buses, vans, pedestrians, cyclists, animals, rickshaws, trucks and other similar objects, even though it has not been explicitly trained to detect these object types. SceneSeg is also able to detect objects that are outside of its training data, such as tyres rolling down a highway, or a loose trailer. SceneSeg can also detect objects in unusual presentations that it hasn't seen during training. 

## Requirements

```
Models==0.9.7
Pillow==11.3.0
```

## Demo

![Demo-1](images/SceneSeg_GIF.gif)

![Demo-2](images/SceneSeg_GIF_Rain.gif)

<img src="images/SceneSeg_GIF.gif" alt="Demo-1" width="300" height="200">



Please click the video link to play - [Video link](https://drive.google.com/file/d/1riGlT3Ct-O1Y2C0DqxemwWS233dJrY7F/view?usp=sharing)

Please see the [Models](https://github.com/autowarefoundation/autoware.privately-owned-vehicles/tree/main/Models) folder to access the pre-trained network weights for SceneSeg as well as scripts for network training, inference and visualization of network predictions.


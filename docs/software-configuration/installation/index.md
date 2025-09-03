# Software installation

We suggest to use Ubuntu 22.04 for the software stack.
Since VisionPilot requires AI packages, we need to install the following packages first.

## Prerequisites

* **CUDA**: Optional for GPU processing.
* **OpenCV**: For image and video processing.
    * Ubuntu: `sudo apt install libopencv-dev`
* **ONNX Runtime**: For model inference.
    * Download from [the GitHub release](https://github.com/microsoft/onnxruntime/releases)
* **LibTorch**: For tensor manipulation capabilities.
    * Download from [the PyTorch website](https://pytorch.org/get-started/locally/)

## Models

The following models will be used in the pipeline.

* SceneSeg - segmentation of foreground objects:
    * [Link to Download Pytorch Model Weights *.pth](https://drive.google.com/file/d/1vCZMdtd8ZbSyHn1LCZrbNKMK7PQvJHxj/view?usp=sharing)
    * [Link to Download Traced Pytorch Model *.pt](https://drive.google.com/file/d/1G2pKrjEGLGY1ouQdNPh11N-5LlmDI7ES/view?usp=drive_link)
    * [Link to Download ONNX FP32 Weights *.onnx](https://drive.google.com/file/d/1l-dniunvYyFKvLD7k16Png3AsVTuMl9f/view?usp=drive_link)
* Scene3D - monocular depth estimation
    * [Link to Download Pytorch Model Weights *.pth](https://drive.google.com/file/d/1MrKhfEkR0fVJt-SdZEc0QwjwVDumPf7B/view?usp=sharing)
    * [Link to Download Traced Pytorch Model *.pt](https://drive.google.com/file/d/1-LO3j2YCvwxeNLzyLrnzEwalTrYUZgK0/view?usp=drive_link)
    * [Link to Download ONNX FP32 Weights *.onnx](https://drive.google.com/file/d/19gMPt_1z4eujo4jm5XKuH-8eafh-wJC6/view?usp=drive_link)
* DomainSeg - segmentation of construction objects
    * [Link to Download Pytorch Model Weights *.pth](https://drive.google.com/file/d/1sYa2ltivJZEWMsTFZXAOaHK--Ovnadu2/view?usp=drive_link)
    * [Link to Download Traced Pytorch Model *.pt](https://drive.google.com/file/d/12fLHpx3IZDglRJaDZT9kMhsV-f6ZTyks/view?usp=drive_link)
    * [Link to Download ONNX FP32 Weights *.onnx](https://drive.google.com/file/d/1zCworKw4aQ9_hDBkHfj1-sXitAAebl5Y/view?usp=drive_link)

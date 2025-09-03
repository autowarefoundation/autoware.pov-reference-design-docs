# Zenoh

## Installation

We need install zenoh and some dependencies first.

* **Zenoh C library**: Required for the transportation.
  * Download from [the GitHub release](https://github.com/eclipse-zenoh/zenoh-c/releases)
  * You can also add the Eclipse repository for apt server.
  
    ```shell
    curl -L https://download.eclipse.org/zenoh/debian-repo/zenoh-public-key | sudo gpg --dearmor --yes --output /etc/apt/keyrings/zenoh-public-key.gpg
    echo "deb [signed-by=/etc/apt/keyrings/zenoh-public-key.gpg] https://download.eclipse.org/zenoh/debian-repo/ /" | sudo tee /etc/apt/sources.list.d/zenoh.list > /dev/null
    sudo apt update
    sudo apt install libzenohc-dev
    ```

* **CLI11**: Used for the command line interface.
  * Ubuntu: `sudo apt install libcli11-dev`

## Build

* Environment setup

```shell
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
```

* Configure with cmake

```shell
mkdir build && cd build
cmake .. \
    -DLIBTORCH_INSTALL_ROOT=/path/to/libtorch/ \
    -DONNXRUNTIME_ROOTDIR=/path/to/onnxruntime-linux-x64-gpu-1.22.0 \
    -DUSE_CUDA_BACKEND=True
```

* Build

```shell
make
```

## Usage

After a successful build, you will find two executables in the `build` directory.

### Video Visualization

Subscribe a video from a Zenoh publisher and then publish it to a Zenoh Subscriber.

* Usage the video publisher and subscriber

```bash
# Terminal 1
./video_publisher -k video/input
# Terminal 2
./run_model SceneSeg_FP32.onnx -i video/input -o video/output
./run_model DomainSeg_FP32.onnx -i video/input -o video/output
# Terminal 3
./video_subscriber -k video/output
```

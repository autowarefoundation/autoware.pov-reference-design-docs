# Zenoh

## Dependencies

We need install zenoh and some dependencies first.

* **Zenoh C library**: Required for the transportation.
    * Download from [the GitHub release](https://github.com/eclipse-zenoh/zenoh-c/releases)
    * You can also add the Eclipse repository for apt server.
  
    ```bash
    curl -L https://download.eclipse.org/zenoh/debian-repo/zenoh-public-key | sudo gpg --dearmor --yes --output /etc/apt/keyrings/zenoh-public-key.gpg
    echo "deb [signed-by=/etc/apt/keyrings/zenoh-public-key.gpg] https://download.eclipse.org/zenoh/debian-repo/ /" | sudo tee /etc/apt/sources.list.d/zenoh.list > /dev/null
    sudo apt update
    sudo apt install libzenohc-dev
    ```

* **CLI11**: Used for the command line interface.
    * Ubuntu: `sudo apt install libcli11-dev`

* **just**: Simplify the command.

  ```bash
  # If you are using Ubuntu 22.04
  wget -qO - 'https://proget.makedeb.org/debian-feeds/prebuilt-mpr.pub' | gpg --dearmor | sudo tee /usr/share/keyrings/prebuilt-mpr-archive-keyring.gpg 1> /dev/null
  echo "deb [arch=all,$(dpkg --print-architecture) signed-by=/usr/share/keyrings/prebuilt-mpr-archive-keyring.gpg] https://proget.makedeb.org prebuilt-mpr $(lsb_release -cs)" | sudo tee /etc/apt/sources.list.d/prebuilt-mpr.list
  sudo apt update
  # If not, just install it directly
  sudo apt install just
  ```

* **parallel**: Run commands in parallel

  ```bash
  sudo apt install parallel
  ```

## Build

* Note that you need to export the path you install libraries first

```bash
export LIBTORCH_INSTALL_ROOT=/path/to/libtorch/
export ONNXRUNTIME_ROOTDIR=/path/to/onnxruntime-linux-x64-gpu-1.22.0
cd Zenoh
just all
```

## Usage

```bash
# Original video pub/sub
just run_video_pubsub
# SceneSeg
just run_sceneseg
# DomainSeg
just run_domainseg
# Scene3D
just run_scene3d
```

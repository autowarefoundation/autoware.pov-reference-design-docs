# System Configuration

(To be completed)

The section describes the system configuration of the PoV vehicles. It shows the options to design your own PoV vehicles. The configuration consists of ECU selection, development approach, middleware candidates, etc.

## Version

There are 4 versions on the roadmap.

![version](images/Roadmap.jpg)

## Hardware Selection

The PoV software stack heavily relies on the inference models, which means high performant GPU is required.

Here is the rough GPU requirements:

- Vision Pilot: 50-80 TOPS
- Vision Pilot Plus: 150 TOPS
- 500 TOPS: Vision Pilot Pro
- 1000 TOPS: Vision Drive

NVIDIA GPU is well supported. AMD, Qualcomm and other vendors (e.g. NPU Halio-8) are on the roadmap.

## Development Approach

- **Native Installation**: Direct installation on hardware for maximum performance
- **Containerized Development**: Reproducible environments with easier team collaboration (on the roadmap)

## Middleware Choice

Now the PoV supports two kinds of pipeline:

- **ROS**: Able to bridge with the existing Autoware system.
- **Zenoh**: High performant middleware.

# System Configuration

(To be completed)

The section describes the system configuration of the PoV vehicles. It shows the options to design your own PoV vehicles. The configuration consists of ECU selection, development approach, middleware candidates, etc.

## Hardware Selection

The PoV software stack heavily relies on the inference models, which means high performant GPU is required.

TODO: Prefer x86 or ARM

TODO: The spec of the GPU requirement

## Development Approach

- **Native Installation**: Direct installation on hardware for maximum performance
- **Containerized Development**: Reproducible environments with easier team collaboration (recommended for ARM)

## Middleware Choice

Now the PoV supports two kinds of pipeline:

- **ROS**: Able to bridge with the existing Autoware system.
- **Zenoh**: High performant middleware.

## Version

There are 4 versions on the roadmap.

![version](images/Roadmap.jpg)

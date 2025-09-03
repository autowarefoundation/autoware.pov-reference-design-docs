# Hardware Configuration

(To be completed)

This section describes the hardware configurations for PoV vehicles, including

- [Sensors and Actuators](Sensors-and-Actuators/index.md): describes the sensors and actuators used in the reference design.
- [ECUs](ECUs/index.md): describes the ECUs used in the reference design.

The sensors requirements are different based on different PoV version.

- Vision Pilot:
  - Two front-facing cameras:
    - a main camera: 8MP RGB camera with 120 degree horizontal FoV
    - a long-range camera: 8MP RGB camera with 30 degree horizontal FoV
  - One front-facing 4D RADAR.

- Vision Pilot Plus:
  - Two front-facing cameras:
    - a main camera: 8MP RGB camera with 120 degree horizontal FoV
    - a long-range camera: 8MP RGB camera with 30 degree horizontal FoV
  - Two 4D RADAR: front and rear
  - Four blindspot Radar

- Vision Pilot Pro:
  - Two front-facing cameras:
    - a main camera: 8MP RGB camera with 120 degree horizontal FoV
    - a long-range camera: 8MP RGB camera with 30 degree horizontal FoV
  - Two 4D RADAR: front and rear
  - Four blindspot Radar
  - Four side camera: 8MP RGB camera with 100 degree horizontal FoV
  - Sat-Nav 2D map

- Vision Drive:
  - Two front-facing cameras:
    - a main camera: 8MP RGB camera with 120 degree horizontal FoV
    - a long-range camera: 8MP RGB camera with 30 degree horizontal FoV
  - One LWIR Camera: 60 degree horizontal FoV
  - One SWIR Camera: 60 degree horizontal FoV
  - Two 4D RADAR: front and rear
  - Four blindspot Radar
  - Four side camera: 8MP RGB camera with 100 degree horizontal FoV
  - Sat-Nav 2D map

The locations of the sensors are shown in the figure below.
![Sensor](assets/images/roadmap.jpg)

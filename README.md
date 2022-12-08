# mujoco_contact_surfaces

Implementation of hydroelastic contact surfaces as a mujoco plugin. This plugin can be used with https://github.com/ubi-agni/mujoco_ros_pkgs.

```sh
# Download and extract Drake
cd <folder to install drake into>
curl -SL https://github.com/RobotLocomotion/drake/releases/download/v1.8.0/drake-20220919-focal.tar.gz | tar -xz
export drake_DIR=$PWD/drake  # Tell cmake where to find Drake
sudo drake/share/drake/setup/install_prereqs  # Install Drake dependencies
```
## Hydroelastic contact surfaces
For more information refer to:
- https://drake.mit.edu/
- [Hydroelastic Contact in Drake Blog Post](https://medium.com/toyotaresearch/rethinking-contact-simulation-for-robot-manipulation-434a56b5ec88)
- https://arxiv.org/pdf/1904.11433.pdf
- https://arxiv.org/pdf/2110.04157.pdf

## Tactile sensor simulation
This plugin can also be used to simulate tactile sensor arrays like the [Myrmex sensor](https://www.researchgate.net/profile/Carsten-Schuermann/publication/229035606_Modular_high_speed_tactile_sensor_system_with_video_interface/links/541affca0cf25ebee988df89/Modular-high-speed-tactile-sensor-system-with-video-interface.pdf).

https://user-images.githubusercontent.com/10142376/203995262-40b0a764-0a63-4793-a65e-4875b55b6245.mp4

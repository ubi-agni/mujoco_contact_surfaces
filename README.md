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

## Cite

If you are using this repository in your reasearch, please cite the following works in your publications accordingly:

```bibtex

@online{
  leinsHydroelasticTouchSimulationTactile2025,
  title = {{{HydroelasticTouch}}: {{Simulation}} of {{Tactile Sensors}} with {{Hydroelastic Contact Surfaces}}},
  shorttitle = {{{HydroelasticTouch}}},
  author = {Leins, David P. and Patzelt, Florian and Haschke, Robert},
  date = {2025-01-14},
  eprint = {2501.08077},
  eprinttype = {arXiv},
  eprintclass = {cs},
  doi = {10.48550/arXiv.2501.08077},
  url = {http://arxiv.org/abs/2501.08077},
  pubstate = {prepublished},
}

@inproceedings{
  leinsMuJoCoROSIntegrating2025,
  title = {{{MuJoCo ROS}}: {{Integrating ROS}} with the {{MuJoCo}} Engine for Accurate and Scalable Robotic Simulation},
  booktitle = {2025 {{IEEE}} International Conference on Simulation, Modeling, and Programming for Autonomous Robots ({{SIMPAR}})},
  author = {Leins, David P. and Haschke, Robert and Ritter, Helge},
  date = {2025},
  publisher = {IEEE. in press}
}

@inproceedings{
  patzeltCurvedTactileSensor2023,
  title = {Curved {{Tactile Sensor Simulation}} with {{Hydroelastic Contacts}} in {{MuJoCo}}},
  author = {Patzelt, Florian and Leins, David and Haschke, Robert},
  year = {2023},
  booktitle = {NeurIPS 2023 Workshop on Touch Processing: a new Sensing Modality for AI},
  url = {https://www.touchprocessing.org/2023/#accepted-papers},
}

@inproceedings{
    leins2023more,
    title={More Accurate Tactile Sensor Simulation with Hydroelastic Contacts in MuJoCo},
    author={David Leins and Florian Patzelt and Robert Haschke},
    booktitle={IROS 2023 Workshop on Leveraging Models for Contact-Rich Manipulation},
    year={2023},
    url={https://openreview.net/forum?id=gCdemyFIVe}
}
```

## Tactile sensor simulation
This plugin can also be used to simulate tactile sensor arrays like the [Myrmex sensor](https://www.researchgate.net/profile/Carsten-Schuermann/publication/229035606_Modular_high_speed_tactile_sensor_system_with_video_interface/links/541affca0cf25ebee988df89/Modular-high-speed-tactile-sensor-system-with-video-interface.pdf).

https://user-images.githubusercontent.com/10142376/203995262-40b0a764-0a63-4793-a65e-4875b55b6245.mp4

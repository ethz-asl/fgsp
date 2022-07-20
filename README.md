# Collaborative Robot Mapping using Spectral Graph Analysis

This work entitled FGSP (short for Factor Graph Signal Processing) deals with the problem of creating globally consistent pose graphs in a centralized multi-robot SLAM framework.

## Overview

FGSP mainly relies on two separate components for synchronization, the `graph_monitor` (server) and the `graph_client` (robots).
The `graph_monitor` reads the optimized server graph published by a separate system.
This is generally agnostic to the mapping server as long as the specific messages
are used.
The message definitions can be obtained from [maplab_msgs](https://github.com/ethz-asl/maplab_msgs) and an example server is the `maplab_server` which can be found [here](https://github.com/ethz-asl/maplab/tree/develop).

The `graph_client` finds the discrepancies and publishes the required constraints to obtain consistent pose graphs on server and robot.
To do so, it listens to the graph of published by the `graph_monitor` as well as the onboard state estimation.
Similar to the `graph_monitor`, this is also agnostic to the used framework as long as the state estimation is published on the correct topics.

## Getting started

There are mainly two options to build FGSP:
 * Regular `colcon` build
 * Deployment build with `docker`


 For the former, the following dependencies need to be installed on the host machine:
  * [liegroups](https://github.com/utiasSTARS/liegroups)
  * [maplab_msgs](https://github.com/ethz-asl/maplab_msgs)
  * [pygsp](https://github.com/epfl-lts2/pygsp)

A submodule of each dependency can be found in the in the `dependencies/` directory.
Other dependencies are `numpy`, `pandas`, `scipy` and need to be installed with, e.g., `pip`.

### Colcon Build
Having all dependencies installed, FGSP can be built with
```
colcon build
```

The server then runs `graph_monitor.py` (see `launch/graph_monitor.launch` for an example) while the clients runs `graph_client.py` (see `launch/robots/local_client.launch` for an example).
For actual robot deployments, the client configuration needs to be adapted per robot.

### Docker Build

Ensure that you have `docker` installed
```
sudo apt install docker.io docker

```

Run the deployment script that builds the image and makes it available to the local Docker daemon:
```
./deploy/build_docker.sh
```

Run files can be found in the `script/` folder.
For example, for running the graph monitor on the server side:
```
./script/run_graph_monitor
```

## Reference

Our paper is available at

*Bernreiter, Lukas, Shehryar Khattak, Lionel Ott, Roland Siegwart, Marco Hutter, and Cesar Cadena. "Collaborative Robot Mapping using Spectral Graph Analysis." In 2022 IEEE International Conference on Robotics and Automation (ICRA), IEEE, 2022.* [[Link](https://ieeexplore.ieee.org/document/9812102)] [[ArXiv](https://arxiv.org/abs/2203.00308)]


BibTex:
```
@inproceedings{bernreiter2022collaborative,
  author={Bernreiter, Lukas and Khattak, Shehryar and Ott, Lionel and Siegwart, Roland and Hutter, Marco and Cadena, Cesar},
  booktitle={2022 International Conference on Robotics and Automation (ICRA)}, 
  title={Collaborative Robot Mapping using Spectral Graph Analysis}, 
  year={2022},  
  pages={3662-3668},
  doi={10.1109/ICRA46639.2022.9812102}
}
```

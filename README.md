# Factor Graph Signal Processing

This work deals with the problem of creating globally consistent pose graphs in a centralized multi-robot SLAM framework.

## Getting started

Ensure that you have `docker` installed
```
sudo apt install docker.io docker

```

Run the deployment script that builds the image and makes it available to the local Docker daemon:
```
./deploy/build_docker.sh
```

Run files can be found in the `script/` folder.
For example, for running the graph monitor:
```
./script/run_graph_monitor
```

## Reference

Our paper is available at

*Bernreiter, Lukas, Shehryar Khattak, Lionel Ott, Roland Siegwart, Marco Hutter, and Cesar Cadena. "Collaborative Robot Mapping using Spectral Graph Analysis." In 2022 IEEE International Conference on Robotics and Automation (ICRA), IEEE, 2022.* [[ArXiv](https://arxiv.org/abs/2203.00308)]


BibTex:
```
@INPROCEEDINGS{bernreiter2022collaborative,
 author={Bernreiter, Lukas and Khattak, Shehryar and Ott, Lionel and Siegwart, Roland and Hutter, Marco and Cadena, Cesar},
 booktitle={2022 International Conference on Robotics and Automation (ICRA)},
 title={Collaborative Robot Mapping using Spectral Graph Analysis},
 year={2022},
 volume={},
 number={},
 pages={},
 doi={}}
```

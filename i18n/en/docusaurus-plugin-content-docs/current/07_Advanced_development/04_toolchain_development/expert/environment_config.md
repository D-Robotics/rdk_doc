---
sidebar_position: 1
---
# Environment Dependency

This section introduces the environment dependency requirements for D-Robotics Plugin Pytorch. We recommend using the Docker environment provided by D-Robotics, and you can refer to the [**Machine Deployment**](/toolchain_development/intermediate/environment_config#machine_deploy) documentation for instructions on how to obtain it.

|             | gpu                      | cpu         |
| ----------- | ------------------------ | ----------- |
| os          | Ubuntu20.04              | Ubuntu20.04 |
| cuda        | 11.6                     | N/A         |
| python      | 3.8                      | 3.8         |
| torch       | 1.13.0+cu116             | 1.13.0+cpu  |
| torchvision | 0.14.0+cu116             | 0.14.0+cpu  |
| Recommended GPUs    | titan v/2080ti/v100/3090 | N/A          |
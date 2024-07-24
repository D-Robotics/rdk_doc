---
sidebar_position: 1
---

# 环境依赖

本节为您介绍 Horizon Plugin Pytorch 所需的环境依赖条件；建议使用地平线提供的Docker环境，获取方式可参考 [**开发机环境部署**](/toolchain_development/intermediate/environment_config#machine_deploy) 文档内容。

|             | gpu                      | cpu         |
| ----------- | ------------------------ | ----------- |
| os          | Ubuntu20.04              | Ubuntu20.04 |
| cuda        | 11.6                     | N/A         |
| python      | 3.8                      | 3.8         |
| torch       | 1.13.0+cu116             | 1.13.0+cpu  |
| torchvision | 0.14.0+cu116             | 0.14.0+cpu  |
| 推荐显卡    | titan v/2080ti/v100/3090 | N/A          |
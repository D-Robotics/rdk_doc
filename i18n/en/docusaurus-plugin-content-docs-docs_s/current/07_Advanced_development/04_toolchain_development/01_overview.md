---
sidebar_position: 1
---

# 7.4.1 Algorithm Toolchain

## Version: 3.7.0

:::info Version Notes
This version corresponds to system software **version 4.0.5**. Users can run the `cat /etc/version` command on the device to confirm the system software version.
:::

## Toolkit Download

### OE Development Toolkit

**Download URL:**
```bash
wget https://d-robotics-aitoolchain.oss-cn-beijing.aliyuncs.com/oe/3.7.0/oe-package-3.7.0-s100-s600.tgz
```

### OE User Manual

**Online Reading URL:**
👉 [https://toolchain.d-robotics.cc/](https://toolchain.d-robotics.cc/)

**Download URL:**
<!-- ```bash
wget -c ftp://oeftp@sdk.d-robotics.cc/oe_v3.2.0/s100-3.2.0-oe-doc.zip --ftp-password=Oeftp~123$%
```   -->

👉 [OE User Manual Version 3.7.0 Download](https://archive.d-robotics.cc/toolchain/oe-doc-3.7.0-s100-s600.zip)

## Docker Images

### CPU Docker

**Download URL:**

Method 1. Log in to the D-Robotics Registry server to pull the image online

```bash
docker login -u "ccr\$deliver-ronly" registry.d-robotics.cc -p 'VLaeatrjF9yGf6I44trT74zKhUpZSVlr'
docker pull registry.d-robotics.cc/deliver/ai_toolchain_ubuntu_22_s100_s600_cpu:v3.7.0
```
Method 2. Download the offline tar package

```bash
wget https://d-robotics-aitoolchain.oss-cn-beijing.aliyuncs.com/oe/3.7.0/ai_toolchain_ubuntu_22_s100_s600_cpu_v3.7.0.tar
```

### GPU Docker

**Download URL:**

Method 1. Log in to the D-Robotics Registry server to pull the image online
```bash
docker login -u "ccr\$deliver-ronly" registry.d-robotics.cc -p 'VLaeatrjF9yGf6I44trT74zKhUpZSVlr'
docker pull registry.d-robotics.cc/deliver/ai_toolchain_ubuntu_22_s100_s600_gpu:v3.7.0
```
Method 2. Download the offline tar package
```bash
wget https://d-robotics-aitoolchain.oss-cn-beijing.aliyuncs.com/oe/3.7.0/ai_toolchain_ubuntu_22_s100_s600_gpu_v3.7.0.tar
```
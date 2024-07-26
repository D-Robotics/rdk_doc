# Monocular Elevation Network

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Introduction

elevation_net is a monocular elevation network detection algorithm example developed based on the hobot_dnn package. It uses an elevation network model and indoor data with BPU for model inference on the horizon RDK to obtain algorithm inference results.

Code Repository:  (https://github.com/D-Robotics/elevation_net>

Application Scenarios: The monocular elevation network detection algorithm parses images to obtain depth and height information of pixels, mainly used in autonomous driving, smart homes, intelligent transportation, and other fields.

## Supported Platforms

| Platform                | System    | Function                        |
| ----------------------- | ------------ | --------------------------------------- |
| RDK X3, RDK X3 Module   | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) | · Start local data offline and save inference rendering results locally |

## Preparation

### Horizon RDK

1. Horizon RDK has burned the  Ubuntu 20.04/22.04 system image provided by Horizon.

2. TogetheROS.Bot has been successfully installed on Horizon RDK.

## Usage

The monocular elevation network detection algorithm example package uses the form of reading local images. After algorithm inference, it detects depth and height information based on pixels of the Image. At the same time, the package processes the depth and height information and publishes PointCloud2 topic data. Users can subscribe to PointCloud2 data for application development.

### Horizon RDK

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```shell
# Configure the tros.b environment
source /opt/tros/setup.bash

# Copy the configuration file required for running the example from the installation path of tros.b.
cp -r /opt/tros/${TROS_DISTRO}/lib/elevation_net/config/ .

# Start the launch file
ros2 launch elevation_net elevation_net.launch.py
```

</TabItem>

<TabItem value="humble" label="Humble">

```shell
# Configure the tros.b environment
source /opt/tros/humble/setup.bash

# Copy the configuration file required for running the example from the installation path of tros.b.
cp -r /opt/tros/${TROS_DISTRO}/lib/elevation_net/config/ .

# Start the launch file
ros2 launch elevation_net elevation_net.launch.py
```

</TabItem>

</Tabs>

## Result Analysis

The package outputs the following information:
 
```shell
[16:15:17:520]root@ubuntu:/userdata# ros2 run elevation_net elevation_net
[16:15:18:976][WARN] [1655108119.406738772] [example]: This is dnn node example!
[16:15:19:056][WARN] [1655108119.475098438] [elevation_dection]: Parameter:
[16:15:19:056]config_file_path_:./config
[16:15:19:056] model_file_name_: ./config/elevation.hbm
[16:15:19:058]feed_image:./config/images/charging_base.png
[16:15:19:058][INFO] [1655108119.475257138] [dnn]: Node init.
[16:15:19:058][INFO] [1655108119.475309553] [elevation_dection]: Set node para.
[16:15:19:058][INFO] [1655108119.475370258] [dnn]: Model init.
[16:15:19:058][BPU_PLAT]BPU Platform Version(1.3.1)!
[16:15:19:095][HBRT] set log level as 0. version = 3.13.27
[16:15:19:095][DNN] Runtime version = 1.8.4_(3.13.27 HBRT)
[16:15:19:133][000:000] (model.cpp:244): Empty desc, model name: elevation, input branch:0, input name:inputquanti-_output
[16:15:19:133][000:000] (model.cpp:244): Empty desc, model name: elevation, input branch:1, input name:inputquanti2-_output
[16:15:19:134][000:000] (model.cpp:313): Empty desc, model name: elevation, output branch:0, output name:output_block1quanticonvolution0_conv_output
[16:15:19:134][INFO] [1655108119.528437276] [dnn]: The model input 0 width is 960 and height is 512
[16:15:19:134][INFO] [1655108119.528535271] [dnn]: The model input 1 width is 960 and height is 512
[16:15:19:134][INFO] [1655108119.528598393] [dnn]: Task init.
[16:15:19:135][INFO] [1655108119.530435806] [dnn]: Set task_num [2]
[16:15:19:135][INFO] [1655108119.530549051] [elevation_dection]: The model input width is 960 and height is 512
[16:15:19:158][INFO] [1655108119.559583836] [elevation_dection]: read image: ./config/images/charging_base.png to detect
[16:15:19:299][WARN] [1655108119.731084555] [elevation_dection]: start success!!!
[16:15:19:351][INFO] [1655108119.779924566] [elevation_net_parser]: fx_inv_: 0.000605
[16:15:19:383][INFO] [1655108119.780357879] [elevation_net_parser]: fy_inv_: 0.000604
[16:15:19:383][INFO] [1655108119.780576493] [elevation_net_parser]: cx_inv_: -0.604389
[16:15:19:383][INFO] [1655108119.780654031] [elevation_net_parser]: cy_inv_: -0.318132
[16:15:19:384][INFO] [1655108119.780751527] [elevation_net_parser]: nx_: 0.000000
[16:15:19:384][INFO] [1655108119.780858063] [elevation_net_parser]: ny_: 0.000000
[16:15:19:384][INFO] [1655108119.780962558] [elevation_net_parser]: nz_: 1.000000
[16:15:19:384][INFO] [1655108119.781067928] [elevation_net_parser]: camera_height: 1.000000
[16:15:19:385][INFO] [1655108119.781833267] [elevation_net_parser]: model out width: 480, height: 256
```[16:15:19:416][INFO] [1655108119.808395254] [elevation_net_parser]: depth: 998.000000
[16:15:19:416][INFO] [1655108119.808593786] [elevation_net_parser]: height: -42.699909
[16:15:19:416][INFO] [1655108119.808644533] [elevation_net_parser]: depth: 998.000000
[16:15:19:416][INFO] [1655108119.808692531] [elevation_net_parser]: height: -25.339746
[16:15:19:416][INFO] [1655108119.808739279] [elevation_net_parser]: depth: 998.000000
[16:15:19:416][INFO] [1655108119.808785527] [elevation_net_parser]: height: -22.111366
[16:15:19:416][INFO] [1655108119.808832774] [elevation_net_parser]: depth: 998.000000
[16:15:19:416][INFO] [1655108119.808878606] [elevation_net_parser]: height: -25.339746
[16:15:19:416][INFO] [1655108119.808925645] [elevation_net_parser]: depth: 998.000000
[16:15:19:416][INFO] [1655108119.808971809] [elevation_net_parser]: height: -21.989540
[16:15:19:416][INFO] [1655108119.809017516] [elevation_net_parser]: depth: 998.000000
[16:15:19:416][INFO] [1655108119.809063138] [elevation_net_parser]: height: -48.303890
[16:15:19:416][INFO] [1655108119.809109678] [elevation_net_parser]: depth: 998.000000
[16:15:19:416][INFO] [1655108119.809155592] [elevation_net_parser]: height: -32.527466
[16:15:19:416][INFO] [1655108119.809202548] [elevation_net_parser]: depth: 998.000000
[16:15:19:416][INFO] [1655108119.809247880] [elevation_net_parser]: height: -32.710201
[16:15:19:416][INFO] [1655108119.809294669] [elevation_net_parser]: depth: 998.000000
[16:15:19:416][INFO] [1655108119.809340542] [elevation_net_parser]: height: -33.014767
[16:15:19:417][INFO] [1655108119.809387165] [elevation_net_parser]: depth: 998.000000
[16:15:19:417][INFO] [1655108119.809433454] [elevation_net_parser]: height: -35.451283
[16:15:19:417][INFO] [1655108119.809480202] [elevation_net_parser]: depth: 998.000000
[16:15:19:417][INFO] [1655108119.809527158] [elevation_net_parser]: height: -38.192360
[16:15:19:417][INFO] [1655108119.809573906] [elevation_net_parser]: depth: 998.000000
[16:15:19:417][INFO] [1655108119.809619820] [elevation_net_parser]: height: -34.233025
[16:15:19:417][INFO] [1655108119.809667235] [elevation_net_parser]: depth: 998.000000
[16:15:19:417][INFO] [1655108119.809713357] [elevation_net_parser]: height: -34.233025
[16:15:19:417][INFO] [1655108119.809759397] [elevation_net_parser]: depth: 998.000000
[16:15:19:417][INFO] [1655108119.809805686] [elevation_net_parser]: height: -33.014767
[16:15:19:417][INFO] [1655108119.809852643] [elevation_net_parser]: depth: 998.000000
[16:15:19:417][INFO] [1655108119.809899307] [elevation_net_parser]: height: -34.354851
[16:15:19:417][INFO] [1655108119.809945930] [elevation_net_parser]: depth: 998.000000
[16:15:19:417][INFO] [1655108119.809991844] [elevation_net_parser]: height: -35.024891
[16:15:19:417][INFO] [1655108119.810038384] [elevation_net_parser]: depth: 998.000000
[16:15:19:417][INFO] [1655108119.810084715] [elevation_net_parser]: height: -41.298916
[16:15:19:417][INFO] [1655108119.810131296] [elevation_net_parser]: depth: 998.000000
[16:15:19:417][INFO] [1655108119.810268706] [elevation_net_parser]: height: -33.745720
[16:15:19:417][INFO] [1655108119.810317745] [elevation_net_parser]: depth: 998.000000
[16:15:19:417][INFO] [1655108119.810364285] [elevation_net_parser]: height: -32.710201
[16:15:19:417][INFO] [1655108119.810410741] [elevation_net_parser]: depth: 998.000000
```

The log displays the depth and height information of the image.

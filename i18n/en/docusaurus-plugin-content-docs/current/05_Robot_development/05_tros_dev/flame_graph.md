---
sidebar_position: 4
---

# 5.5.4 Performance Flame Graph

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Introduction

A flame graph is a graphical representation of performance data collected by tools like perf. It provides statistical analysis of the data to identify performance hotspots.

![http-bw](./image/flame_graph/flamegraph.png "flame graph")

Each box in the flame graph represents a function, with the length of the box representing its execution time. Therefore, wider boxes indicate longer execution times.

The height of the flame graph represents the depth of function calls. The topmost functions are the leaf functions.

Code repository: [https://github.com/brendangregg/FlameGraph.git](https://github.com/brendangregg/FlameGraph.git)

## Supported Platforms

| Platform       | System     | Function |
| -------------- | ----------- | --------------------- |
| RDK X3, RDK X3 Module  | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) | Demonstrating how to generate a flame graph |

## Preparation

### Horizon RDK

1. The flamegraph tool consists of two parts. The first part is the executable program "perf". Perf is a software performance analysis tool used to collect and output function call information in the system or a specific process. Perf is already included in the Horizon RDK operating system, so it can be used directly.

2. The second part of the flamegraph tool is a script used to parse the text output of perf and generate an SVG-formatted flame graph for easy observation and analysis.

## Usage

1. Use the perf record tool to sample the function calls in the Horizon RDK system and generate a perf.data file.

    ```shell
    root@ubuntu:~# perf record -F 99 -a -g -- sleep 60
    ```

2. Use the perf script tool to parse the perf.data file and generate out.perf.

    ```shell
    root@ubuntu:~# perf script > out.perf
    ```

3. Clone the FlameGraph repository on your PC or Horizon RDK: `git clone https://github.com/brendangregg/FlameGraph.git`. Navigate to the "flamegraph" directory and copy the out.perf file generated in step 2 to the "flamegraph" directory. Use the stackcollapse-perf.pl script provided in the flamegraph package to unfold the out.perf file and generate out.folded.

    ```shell
    ./stackcollapse-perf.pl out.perf > out.folded
    ```
4. Generate an SVG flame graph using flamegraph.pl:

```shell
./flamegraph.pl out.folded > flame.svg
```

Steps 1 and 2 are completed on the Horizon RDK, while steps 3 and 4 are completed on either a PC or the Horizon RDK.

## Results Analysis

After following the workflow in the previous section, the recorded function calls within the Horizon RDK system are shown in the following image:

![](./image/flame_graph/flame_graph_result.png)
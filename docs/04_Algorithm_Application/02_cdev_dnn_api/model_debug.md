---
sidebar_position: 8
---

# 模型推理DEBUG方法


## 错误码

    HB_DNN_SUCCESS = 0                              // 执行成功
    HB_DNN_INVALID_ARGUMENT = -6000001              // 非法参数
    HB_DNN_INVALID_MODEL = -6000002                 // 非法模型
    HB_DNN_MODEL_NUMBER_EXCEED_LIMIT = -6000003     // 模型个数超过限制
    HB_DNN_INVALID_PACKED_DNN_HANDLE = -6000004     // 非法packed handle
    HB_DNN_INVALID_DNN_HANDLE = -6000005            // 非法handle
    HB_DNN_CAN_NOT_OPEN_FILE = -6000006             // 文件不存在
    HB_DNN_OUT_OF_MEMORY = -6000007                 // 没有足够的内存
    HB_DNN_TIMEOUT = -6000008                       // 超时
    HB_DNN_TASK_NUM_EXCEED_LIMIT = -6000009         // 任务数量超限制
    HB_DNN_TASK_BATCH_SIZE_EXCEED_LIMIT = -6000010  // 多任务处理数量超限制
    HB_DNN_INVALID_TASK_HANDLE = -6000011           // 非法task handle
    HB_DNN_RUN_TASK_FAILED = -6000012               // 任务执行失败
    HB_DNN_MODEL_IS_RUNNING = -6000013              // 任务执行中
    HB_DNN_INCOMPATIBLE_MODEL = -6000014            // 不兼容的模型
    HB_DNN_API_USE_ERROR = -6000015                 // 接口使用错误
    HB_DNN_MULTI_PROGRESS_USE_ERROR = -6000016      // 多进程使用错误

    HB_SYS_SUCCESS = 0                              // 执行成功
    HB_SYS_INVALID_ARGUMENT = -6000129              // 非法参数
    HB_SYS_OUT_OF_MEMORY = -6000130                 // 没有足够的内存
    HB_SYS_REGISTER_MEM_FAILED = -6000131           // 注册内存失败

## 配置信息{#configuration_information}

1. 日志等级。 ``dnn`` 中的日志主要分为4个等级，：

   - ``HB_DNN_LOG_NONE = 0``，不输出日志；
   - ``HB_DNN_LOG_WARNING = 3``，该等级主要用来输出代码中的告警信息；
   - ``HB_DNN_LOG_ERROR = 4``，该等级主要用来输出代码中的报错信息；
   - ``HB_DNN_LOG_FATAL = 5``，该等级主要用来输出代码中的导致退出的错误信息。

2. 日志等级设置规则：

   若发生的LOG等级 >= 设置的等级，则该LOG可以被打印，反之被屏蔽；设置的LOG等级越小，打印信息越多（等级0除外，0不输出日志）。
   例如：设置LOG等级为3，即为 ``WARNING`` 级别，则3,4,5等级的LOG均可以被打印；
   预测库默认LOG等级为 ``HB_DNN_LOG_WARNING`` ，则以下LOG级别的信息可以被打印： 
   ``WARNING`` 、 ``ERROR`` 、 ``FATAL``。

3. 日志等级设置方式：
   可通过环境变量 ``HB_DNN_LOG_LEVEL`` 设置日志等级。
   比如： ``export HB_DNN_LOG_LEVEL=3``，则输出 ``WARNING`` 级以上级别的日志。

4. 常用环境变量

        HB_DNN_LOG_LEVEL                // 设置日志等级。
        HB_DNN_CONV_MAP_PATH            // 模型卷积层配置文件路径；编译参数layer_out_dump为true时产生的json文件。
        HB_DNN_DUMP_PATH                // 模型卷积层结果输出路径，与HB_DNN_CONV_MAP_PATH配合使用。
        HB_DNN_PLUGIN_PATH              // 自定义CPU算子动态链接库所在目录。
        HB_DNN_PROFILER_LOG_PATH        // 模型运行各阶段耗时统计信息dump路径。
        HB_DNN_SIM_PLATFORM             // x86模拟器模拟平台设置，可设置为BERNOULLI、BERNOULLI2、BAYES。
        HB_DNN_SIM_BPU_MEM_SIZE         // x86模拟器设置BPU内存大小，单位MB。
        HB_DNN_ENABLE_DSP               // 使能DSP模块，仅 RDK Ultra 可用。

## 开发机模拟器使用注意事项


1. 开发机模拟器在使用时，可以通过设置环境变量 ``HB_DNN_SIM_PLATFORM`` 来指定需要模拟的处理器架构，可执行如下命令：

   - ``export HB_DNN_SIM_PLATFORM=BERNOULLI``，为 ``BERNOULLI`` 架构，模拟地平线 ``xj2`` 平台；
   - ``export HB_DNN_SIM_PLATFORM=BERNOULLI2``，为 ``BERNOULLI2`` 架构，模拟地平线 ``x3`` 平台, **RDK X3** 可使用；
   - ``export HB_DNN_SIM_PLATFORM=BAYES``，为 ``BAYES`` 架构，模拟地平线 ``xj5`` 平台， **RDK Ultra** 可使用。

2. 如果不设置 ``HB_DNN_SIM_PLATFORM`` 环境变量，会根据第一次加载的模型架构来设置模拟器平台，例如：第一次加载的模型是 ``BERNOULLI2`` 架构，则程序默认设置的平台为 ``x3``。

3. 在开发机模拟器中执行 ``resize`` 相关操作之前，需要通过 ``HB_DNN_SIM_PLATFORM`` 环境变量指定平台。

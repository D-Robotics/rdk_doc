---
sidebar_position: 9
---
# 4.2.9 Model Inference DEBUG Method

## Error Codes

    HB_DNN_SUCCESS = 0                              // Execution succeeded
    HB_DNN_INVALID_ARGUMENT = -6000001              // Invalid argument
    HB_DNN_INVALID_MODEL = -6000002                 // Invalid model
    HB_DNN_MODEL_NUMBER_EXCEED_LIMIT = -6000003     // Number of models exceeds limit
    HB_DNN_INVALID_PACKED_DNN_HANDLE = -6000004     // Invalid packed handle
    HB_DNN_INVALID_DNN_HANDLE = -6000005            // Invalid handle
    HB_DNN_CAN_NOT_OPEN_FILE = -6000006             // File does not exist
    HB_DNN_OUT_OF_MEMORY = -6000007                 // Not enough memory
    HB_DNN_TIMEOUT = -6000008                       // Timeout
    HB_DNN_TASK_NUM_EXCEED_LIMIT = -6000009         // Number of tasks exceeds limit
    HB_DNN_TASK_BATCH_SIZE_EXCEED_LIMIT = -6000010  // Number of tasks exceeds batch size limit
    HB_DNN_INVALID_TASK_HANDLE = -6000011           // Invalid task handle
    HB_DNN_RUN_TASK_FAILED = -6000012               // Failed to execute task
    HB_DNN_MODEL_IS_RUNNING = -6000013              // Model is running
    HB_DNN_INCOMPATIBLE_MODEL = -6000014            // Incompatible model
    HB_DNN_API_USE_ERROR = -6000015                 // API usage error
    HB_DNN_MULTI_PROGRESS_USE_ERROR = -6000016      // Multi-process usage error

    HB_SYS_SUCCESS = 0                              // Execution succeeded
    HB_SYS_INVALID_ARGUMENT = -6000129              // Invalid argument
    HB_SYS_OUT_OF_MEMORY = -6000130                 // Not enough memory
    HB_SYS_REGISTER_MEM_FAILED = -6000131           // Failed to register memory

## Configuration Information

1. Log levels. The logs in ``dnn`` are divided into four levels:

   - ``HB_DNN_LOG_NONE = 0``, no log output;
   - ``HB_DNN_LOG_WARNING = 3``, used to output warning messages in the code;
   - ``HB_DNN_LOG_ERROR = 4``, used to output error messages in the code;
   - ``HB_DNN_LOG_FATAL = 5``, used to output fatal error messages that cause program termination.

2. Log level setting rules:

   If the log level that occurs is greater than or equal to the set level, the log can be printed; otherwise, it is blocked. The smaller the set log level, the more print information there is (except for level 0, which does not output logs). For example, if the log level is set to 3, which is the ``WARNING`` level, logs of level 3, 4, and 5 can be printed. The default log level of the prediction library is ``HB_DNN_LOG_WARNING``, so log information of the following levels can be printed: ``WARNING``, ``ERROR``, ``FATAL``.

3. Log level setting method:
   The log level can be set using the environment variable ``HB_DNN_LOG_LEVEL``. For example :`export HB_DNN_LOG_LEVEL=3`, output logs at or above the level of `Warning`

4. Common Environment Variables
   ```
   HB_DNN_LOG_LEVEL                 // Set the log level.
   HB_DNN_CONV_MAP_PATH             // Path to the model convolution layer configuration file; generated json file when the compilation parameter layer_out_dump is true.
   HB_DNN_DUMP_PATH                 // Path to output the model convolution layer results, used in conjunction with HB_DNN_CONV_MAP_PATH.
   HB_DNN_PLUGIN_PATH               // Directory where the custom CPU operator dynamic link libraries are located.
   HB_DNN_PROFILER_LOG_PATH         // Path to dump the timing statistics of each stage during model running.
   HB_DNN_SIM_PLATFORM              // Setting for the x86 simulator simulation platform, can be set to BERNOULLI, BERNOULLI2, BAYES.
   HB_DNN_SIM_BPU_MEM_SIZE          // Setting for the x86 simulator BPU memory size, in MB.
   HB_DNN_ENABLE_DSP                // Enables the DSP module, only available for RDK Ultra.
   ```
   
## Considerations for Development Machine Simulator

1. When using the development machine simulator, you can specify the simulated processor architecture by setting the environment variable `HB_DNN_SIM_PLATFORM`. You can execute the following commands:

   - `export HB_DNN_SIM_PLATFORM=BERNOULLI2` for the `BERNOULLI2` architecture, simulating the D-Robotics `x3` platform, **RDK X3** can be used;
   - `export HB_DNN_SIM_PLATFORM=BAYES` for the `BAYES` architecture, simulating the D-Robotics `x5` platform, **RDK X5** can be used.

2. If the `HB_DNN_SIM_PLATFORM` environment variable is not set, the simulator platform will be set based on the architecture of the first loaded model. For example, if the first loaded model is of `BERNOULLI2` architecture, the program will default to the `x3` platform.

3. Before executing any operations related to `resize` in the development machine simulator, you need to specify the platform by using the `HB_DNN_SIM_PLATFORM` environment variable.
---
sidebar_position: 5
---

# BPU（算法推理模块）API

`BPU` API提供了以下的接口：

| 函数 | 功能 |
| ---- | ----- |
| sp_init_bpu_module | **初始化算法推理模块对象，创建算法推理任务** |
| sp_bpu_start_predict | **进行AI算法推理，获得推理结果** |
| sp_release_bpu_module | **关闭算法推理任务** |
| sp_init_bpu_tensors | **分配tensor内存** |
| sp_deinit_bpu_tensor | **销毁tensor内存** |


## sp_init_bpu_module  

**【函数原型】**  

`bpu_module *sp_init_bpu_module(const char *model_file_name)`

**【功能描述】**  

打开`model_file_name`算法模型，初始化一个算法推理任务。

**【参数】**

- `model_file_name`： 算法模型文件，需要是经过地平线AI算法工具链转换的或者训练得到的定点模型。

**【返回类型】** 

AI算法推理任务对象。

## sp_bpu_start_predict  

**【函数原型】**  

`int32_t sp_bpu_start_predict(bpu_module *bpu_handle, char *addr)`

**【功能描述】**  

传入图像数据完成AI算法推理，返回算法结果。

**【参数】**

- `bpu_handle`： 算法推理任务对象
- `addr`：图像数据输入

**【返回类型】** 

无。

## sp_init_bpu_tensors 

**【函数原型】**  

` int32_t sp_init_bpu_tensors(bpu_module *bpu_handle, hbDNNTensor *output_tensors)`

**【功能描述】**  

初始化并分配内存给传入的`tensor`。

**【参数】**

- `bpu_handle`： 算法推理任务对象
- `output_tensors`：`tensor`地址

**【返回类型】** 

无。

## sp_deinit_bpu_tensor 

**【函数原型】**  

` int32_t sp_deinit_bpu_tensor(hbDNNTensor *tensor, int32_t len)`

**【功能描述】**  

将传入的`tensor`释放并回收内存。

**【参数】**

- `tensor`： 带出来`tensor`指针
- `output_tensors`：`tensor`地址

**【返回类型】** 

无。


## sp_release_bpu_module  

**【函数原型】**  

`int32_t sp_release_bpu_module(bpu_module *bpu_handle)`

**【功能描述】**  

关闭算法推理任务。

**【参数】**

- `bpu_handle`： 算法推理任务对象

**【返回类型】** 

成功返回 0，失败返回 -1。

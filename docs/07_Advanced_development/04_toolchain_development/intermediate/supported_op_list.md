---
sidebar_position: 3
---

# 模型算子支持列表{#supported_op_list_and_restrictions}


## 使用限制说明

本章节主要介绍D-Robotics 处理器支持的 `Caffe` 和 `ONNX` 算子情况，其他未列出的算子因D-Robotics 处理器 bpu硬件限制，暂不支持。

**术语概念：**

-    BPU加速   ：D-Robotics 处理器可以进行加速的算子（一定约束条件下），如果不满足约束条件，则会在CPU进行计算

-    CPU计算   ：当前已经在D-Robotics ARM CPU上进行优化的算子，支持onnx opset10与opset11。

-    CPU计算※   ：暂时未集成的CPU算子。


**其他注意事项：**

-   RDK X3所有BPU上运行的算子均遵守一般限制：input_batch ≤ 128。 

-   RDK Ultra 和 RDK X5所有BPU上运行的算子均遵守一般限制：1. 输入输出维度均为4，对于支持非四维情况的op，会在约束中显性标识； 2. shape：H,W,C ∈ [1, 65536]，`N <= 4096；3. N x C x H x W <= 1G bytes`。

-   支持 ``Caffe 1.0`` 基础算子以及常用扩展算子，支持onnx ``opset10`` 和 ``opset11`` 算子，对于无法满足BPU加速约束条件的算子将会退化到ARM CPU进行计算。

-   ``Cast`` , ``Constant`` , ``Dropout`` , ``Reshape`` , ``Squeeze`` , ``Unsqueeze`` , ``Shape`` 这些算子(OP)无法直接运行在BPU上，但在一些情况下（常量折叠）算法工具链会将其优化掉进而实现支持的效果。

-   标记为PyTorch的算子(OP)为官方的opset11不包含的算子，D-Robotics 算法工具链提供了导出脚本可以将其从PyTorch导出到D-Robotics 自定义的onnx OP中。

-   基于tensorlfow-onnx（https://github.com/onnx/tensorflow-onnx）转换工具，支持将 ``tensorlfow1.*`` 版本的算子稳定的转换到opset6-opset11版本的ONNX模型格式，但是 ``Tensroflow2.*`` 当前支持还属于实验版本。

-   关于OP主动量化被动量化的说明：一个符合本章节约束条件的OP仍然运行在CPU的主要原因是该OP属于被动量化OP，算法工具链会根据OP的计算特性和BPU底层逻辑等多方面考虑设计量化逻辑，当前量化逻辑分为：主动量化，被动量化，手动量化。量化逻辑更多信息请阅读：[**算法工具链中的主动量化和被动量化逻辑**](https://developer.d-robotics.cc/forumDetail/118364000835765793) 章节。


## RDK X3支持的Caffe算子列表

| **caffe算子名称**       | **CPU计算/BPU加速** | **X3 BPU支持约束** | **CPU支持约束** |
| ---------------------- | ----------------- | ----------------- | -------------- |
| Convolution | BPU加速 | Kernel宽高取值范围：HxW=[1,7]x[1,7] <br/> 输入输出Channel取值范围 `(one group) <= 2048`（对于非dilated、group、depthwise conv等普通卷积，可以放宽至 `<=4096`）。<br/> stride无限制。<br/> Dilation取值范围：只支持设置为2的幂次方，且必须能够被stride整除。<br/> h_dilated 和 w_dilated 可以不同但要求 `h_diated <= w_dilated`。<br/> 单个 Kernel 总体大小限制：`HxWxC <= 32768`。<br/> 不支持配置axis，默认为1 | 仅支持4维Conv计算。<br/> auto_pad 属性不支持。<br/> type约束支持：float, int32, int8。<br/> pads 属性约束``[Hstart, Wstart, Hend, Wend]`（pads长度等于4）并且 Hstart == Hend，Wstart == Wend。 | 
| Deconvolution | BPU加速 | Kernel 宽高取值范围：HxW=[2,14]x[2,14]。  <br/> 输入输出Channel数值取值范围：`C <= 2048`。  <br/>Padding宽高取值范围： <br/>HxW=[0,(Kernel_H-1)/2]x[0,(Kernel_W-1)/2] 。 <br/>Stride取值范围：Stride ∈ {2, 4} 。 <br/> `stride_h ≦ stride_w` 。 <br/>Dilation ∈ {(1, 1)}。  <br/>不支持配置axis属性。 | 不支持output_shape和output_padding参数；  <br/>auto_pad参数只支持NOTSET模式；  <br/>不支持axis |
| MaxUnpool                                                    | CPU计算 | --- | from_type支持：   <br/>- X：type约束：仅支持float类型。  <br/>- I：Tensor（int64）。  <br/>to_type支持：type约束：仅支持float类型。                                       |
| Pooling      | BPU加速 | 共有四种Pooling算子即MaxPooling，AveragePooling，GlobalMaxPooling，GlobalAveragePooling。 <br/>对四种Pooling的约束分别为：   <br/>MaxPooling： <br/> Kernel宽高的取值范围为：[1,64]x[1,64] 。 <br/>Stride取值范围为：[1,185]。 <br/> Padding值需要大于等于零。 <br/>AveragePooling： <br/> Kernel HxW=[1, 7]x[1, 7], Stride ∈{1, 185}。  <br/>GlobalAveragePooling：  <br/>假设输入shape为NCHW， 则输入宽高需满足 `HxW <= 8192` 。 <br/>GlobalMaxPooling：  <br/>假设输入shape为NCHW，则输入宽高取值范围为HxW=[1,1024]x[1,1024]。 | 无  |
| SPP          | CPU计算 | 不支持                                                       | 支持pyramid_height，2^n 次pooling, `n<7` ; <br/> pooling kernel 小于等于 255；  <br/> 支持pool，配置可选值为 `{0，1}` |
| InnerProduct | BPU加速 | InnerProduct将被转化为Conv实现。  <br/>假设InnerProduct的输入feature map的shape为NCHW ： <br/>1. 如果HW均小于等于7，则Gemm的限制等同于Conv。  <br/>2. 如果H和W均为1，那么C的限制为` <= 16384`；否则 C的大小限制为 `<= 2048`。  <br/>3. 如果Gemm后是一个BPU支持的节点，Gemm会进行低精度int8输出，此时的输入宽高限制为: H x W/8 x C/4 `<=` 1024。  <br/>4. 如果Gemm后是一个非BPU支持的节点，Gemm会进行高精度int32输出，此时的输入宽高限制为: H x W/8 x C/4 < 2048 。 <br/> 不支持配置axis属性 | 无                                                           |
| LRN  | CPU计算 | 不支持 | local_size 支持、 <br/>alpha支持、 <br/>beta 支持、 <br/>norm_region 支持，配置可选值`{ACROSS_CHANNELS, WITHIN_CHANNEL }`、 <br/>k 支持 |
| MVN  | CPU计算 | 不支持 | normalize_variance支持，配置可选值为{0, 1}、 <br/>across_channels支持，配置可选值为{0, 1}、 <br/>仅支持Float32类型的计算。 |
| BatchNorm                                                    | BPU加速 | 无限制 | 无                                         |
| ELU                                                          | CPU计算 | 不支持 | 无                                         |
| BNLL                                                         | CPU计算 | 不支持 | 无                                         |
| PReLU                                                        | BPU加速 | 无限制 | 无                                         |
| ReLU/LeakyRelu                                               | BPU加速 | 无限制 | 无                                         |
| Sigmoid                                                      | BPU加速 | 对于一个输入维度为1CHW的tensor，仅支持min(8W4C对齐后的shape，32C对齐后的shape) `<=8192`的情况。 <br/>8W4C：实际运行时tensor的W维度padding至8的整数倍，C维度padding至4的整数倍。 <br/>32C：实际运行时tensor的C维度padding至32的整数倍。 <br/>在两个对齐方式中取对齐后shape最小值，判断是否`<=8192`。 | 无                                         |
| TanH                                                         | BPU加速 | 无限制 | 无                                         |
| Eltwise                                                      | BPU加速 | operation目前支持Add和Mul，暂不支持减。  <br/>Add：  <br/>输入channel大小 `M<= 2048`  <br/>支持以下几种情况： <br/> 1. Add的两个输入shape为NCHW和NCHW；  <br/>2. Add的两个输入shape为NCHW和NC11（Add的两个输入都需要是其它op的输出） <br/> Mul： <br/> Mul的两个输入都需要是四维并且C的大小需要 `<= 2048`。 <br/> 同时仅支持如下shape的相乘：  <br/>1. (1xCxHxW vs 1xCxHxW)。  <br/>2. (1xCxHxW vs 1xCx1x1)。  <br/>3. (1xCxHxW vs 1x1x1x1)。 | 无                                                           |
| Bias                                                         | BPU加速 | 参考Eltwise等于Add的情况                                     | 无                                                           |
| Scale                                                        | BPU加速 | 参考Eltwise等于Mul的情况                                     | 无                                                           |
| AbsVal                                                       | CPU计算 | 不支持                                                       | 无                                                           |
| Exp                                                          | BPU加速 | 无限制                                                       | 无                                                           |
| Log                                                          | CPU计算 | 不支持                                                       | 无                                                           |
| Power                                                        | BPU加速 | 无限制                                                       | 无                                                           |
| Threshold                                                    | CPU计算 | 不支持                                                       | 无                                                           |
| Reduction                                                    | CPU计算 | 不支持                                                       | operation 支持 SUM、ASUM、 SUMSQ、MEAN ； <br/>axis 支持；  <br/> 仅支持Float32类型的计算。 |
| Softmax                                                      | CPU计算 | 不支持                                                       | 无                                                           |
| ArgMax                                                       | BPU加速 | 仅支持 `axis=1，c<=64` 。 <br/>不支持配置top_k != 1                    | 无                                                           |
| Concat                                                       | BPU加速 | 输入输出Channel：`C<=2048 `                                       | 无                                                           |
| Split                                                        | BPU加速 | 无限制                                                       | 无                                                           |
| Slice                                                        | BPU加速 | 无限制                                                       | 无                                                           |
| Reshape                                                      | CPU计算 | 不支持（一些场景下可以融合）                                 | shape 支持[1,4]个 shape_dim 配置 ； <br/> axis 支持[-4,3]范围内可配，不支 持 N 维度，默认值 0，遵循 caffe 规则 ； <br/> num_axes 支持[-1,3]范围内可配，默认 值-1 表示对 axis 起始的所有 轴进行变换 |
| Flatten                                                      | CPU计算 | 不支持（一些场景下可以融合）                                 | axis 取值范围[-4,3]，默认值 为 1，-4 与 0 含义相同。  <br/>只支持End_axis == -1。 |
| Crop                                                         | CPU计算 | 不支持                                                       | 无                                                           |
| Dropout                                                      | BPU加速 | 无限制                                                       | 无                                                           |
| LSTM                                                         | BPU加速 | 仅支持batch=1                                                | --                                                           |
| Normalize                                                    | CPU计算 | 不支持                                                       | type约束：仅支持float类型。                                 |
| PassThrough                                                  | BPU加速 | 支持mode=DCR 和 mode=CRD。 <br/>仅支持H和W方向的重新排列，并且仅支持blocksize=2的重排列。 <br/>举例：NxCxHxW -> Nx(4C)x(H/2)x(W/2)。  | type约束：仅支持float类型。  |
| CReLU                                                         | CPU计算 | 不支持                                               | type约束：仅支持float类型。                                 |
| RReLU                                                         | CPU计算 | 不支持                                               | 无                                                          |
| Permute                                                         | CPU计算 | 不支持     | - 支持nhwc2nchw，perm：[0, 3, 1, 2]。  <br/> - 支持nchw2nhwc，perm：[0, 2, 3, 1]。 <br/> - 支持指定perm维度转换，数据类型仅支持float，int8，int32。 |
| MatMul                                                         | BPU加速 | 对于两个输入分别为featuremap和weight的场景（即featuremap与常量相乘） <br/> 其中第一个输入是featuremap，第二个输入是weight，以下几种场景均可优化到BPU上运行： <br/>- K vs KxN、K vs 1xKxN、K vs 1x1xKxN  <br/>- MxK vs K、MxK vs KxN、MxK vs 1x1xKxN  <br/>- 1xMxK vs K、1xMxK vs 1xKxN  <br/>- 1x1xMxK vs K、1x1xMxK vs 1xKxN、1x1xMxK vs 1x1xKxN  <br/>- BxMxK vs KxN （B>=1）  <br/>- 1xBxMxK vs KxN （B>=1） <br/>- AxBxMxK vs KxN (A>1，B>1)  <br/>- 其中第一个输入是weight，第二个输入是featuremap，以下场景可优化到BPU上运行： <br/>- 1xBxMxK vs 1x1xKxN (B>1)  <br/>对于两个输入均为featuremap的场景（即两个featuremap相乘），以下场景可优化到BPU上运行： <br/>- 1xBxMxK vs 1x1xKxN （B>=1）  | type约束：仅支持float类型。                                                       |
| Upsample                                                     | BPU加速 | 输入featuremap需为四维NCHW，并且只支持在H和W维度上进行resize；  <br/> 放大系数factor支持2的幂数倍如2，4，8，16，32等； <br/> 支持H维度和W维度的放大系数不同但需要满足 `H_factor <= W_factor` | 无                                                           |
| ROIPooling                                                   | CPU计算 | 不支持                                                       | 无                                                           |
| PSROIPooling                                                 | CPU计算 | 不支持                                                       | 无                                                           |



## RDK X3支持的ONNX算子列表

| **ONNX算子名称** | **CPU计算/BPU加速** | **X3 BPU支持约束** | **CPU支持约束** |
| ------------ | --------------- | --------------- | ----------- |
| Abs                       | CPU计算         | --          | type约束：仅支持float类型。   |  
| Acos                      | CPU计算         | --          | type约束：仅支持float类型。 |    
| Acosh                     | CPU计算         | --     | type约束：仅支持float类型。      |    
| Add                       | BPU加速         | 输入channel大小 `M<= 2048` 支持以下几种情况： <br/> 1. Add的两个输入shape为NCHW和NCHW；  <br/>2. Add的两个输入shape为NCHW和NC11（Add的两个输入都需要是其它op的输出）； <br/>3.作为resnet中的short-cut子结构的Add，会被融合到上一个conv中加速计算。 | - 支持相同输入shape计算。 <br/>- 支持输入1是标量或者输入2是标量的计算。 <br/>- 支持broadcast计算，最大维度是5。 |          
| And                       | CPU计算         | --  | - 支持相同输入shape计算。 <br/>- 支持输入1是标量或者输入2是标量的计算。 <br/>- 支持broadcast计算，最大维度是5。 |                      
| ArgMax                    | BPU加速         | 1. 输入维度为四维输入NCHW。  <br/>2. 仅支持沿C维度进行argmax，即axis=1。 <br/> 3. `C <= 64 `| type约束：仅支持float类型。      |                       
| ArgMin                    | CPU计算         | --                                                           | type约束：仅支持float类型。    |                         
| Asin                      | CPU计算         | --                                                           | type约束：仅支持float类型。     |                         
| Asinh                     | CPU计算         | --                                                           | type约束：仅支持float类型。    |                        
| Atan                      | CPU计算         | --                                                           | type约束：仅支持float类型。     |                       
| Atanh                     | CPU计算         | --                                                           | type约束：仅支持float类型。     |                        
| AveragePool               | BPU加速         | Kernel HxW=[1, 7]x[1, 7], Stride ∈{1, 185}                   | auto_pad 属性不支持。 <br/>仅支持四维Tensor计算。     |                      
| BatchNormalization        | BPU加速         | 优化阶段会被融合到上一个conv中支持                               | type约束：仅支持float类型。  <br/>支持第1个维度是channel的数据排布方式计算。   |         
| BitShift                  | CPU计算※        | --                                                           | --                                                           |     
| Cast                      | CPU计算         | --                                                           | from_type支持double, float, bool, int64, uint32, int32, uint16, int16, uint8, int8。<br/>to_type支持double, float, bool, int64, uint32, int32, uint16, int16, uint8, int8。|    
| Ceil                      | CPU计算         | --                                                           | type约束：仅支持float类型。     | 
| Clip                      | BPU加速         | 无限制。                                                      | type约束：仅支持float类型。 <br/>仅有2个输入时，默认为min参数。 | 
| Compress                  | CPU计算※        | --                                                           | --                                                           |  
| Concat                    | BPU加速         | 输入输出Channel：`C<=2048`。                                    | --                                                          |  
| ConcatFromSequence        | CPU计算※        | --                                                           | --                                                           |   
| Constant                  | BPU加速         | 会通过常量折叠将其优化为数值存储                                | 目前不支持sparse_tensor属性。 <br/> type约束：仅支持float类型。                    |    
| ConstantOfShape           | BPU加速         | 会通过常量折叠将其优化为数值存储                                 | type约束支持：float,int32,int8。   |    
| Conv                      | BPU加速         | Kernel宽高取值范围：HxW=[1,7]x[1,7]。 <br/> 输入输出Channel取值范围`(one group) <= 2048`（对于非dilated、group、depthwise conv等普通卷积，可以放宽至`<=4096`）。 <br/> stride无限制，，但对于Conv后接Add(resnet shortcut-connecting) Stride取值范围为：{1, 2}。 <br/> Dilation取值范围：只支持设置为2的幂次方，且必须能够被stride整除。 <br/>h_dilated和w_dilated可以不同但要求`h_diated<=w_dilated` 。 <br/> 单个Kernel总体大小限制: `HxWxC <= 32768` | 仅支持4维Conv计算。 <br/>auto_pad 属性不支持。 <br/>type约束支持：float,int32,int8。 <br/>pads属性约束：[Hstart, Wstart, Hend, Wend]（pads长度等于4）并且Hstart==Hend，Wstart==Wend。|   
| ConvInteger               | CPU计算※        | --                                                           | --                                                           | 
| ConvTranspose             | BPU加速         | Kernel 宽高取值范围：HxW=[2,14]x[2,14]。 <br/> 输入输出Channel数值取值范围：`C <= 2048` 。 <br/> Padding宽高取值范围：HxW=[0,(Kernel_H-1)/2]x[0,(Kernel_W-1)/2]。 <br/> Stride取值范围：`Stride ∈ {2, 4}`。 <br/> `stride_h ≦ stride_w`。 <br/> `Dilation ∈ {(1, 1)}` | auto_pad属性不支持。  <br/>type约束支持：float,int32,int8。 |  
| Cos                       | BPU加速         | 对于一个输入维度为1CHW的tensor，仅支持 `CxHxW <= 8192`的情况        | type约束：仅支持float类型。            |   
| Cosh                      | CPU计算         | --                                                           | type约束：仅支持float类型。      |    
| CumSum                    | CPU计算         | --                                                           | from_type： <br/>x：type约束仅支持float类型。 <br/>axis：type约束仅支持int32类型。 <br/>to_type：type约束仅支持float类型。 |   
| DepthToSpace              | BPU加速         | 支持mode=DCR 和 mode=CRD。 <br/> 仅支持H和W方向的重新排列，并且仅支持blocksize=2的重排列。  <br/>举例：NxCxHxW -> Nx(C/4)x(2H)x(2W) | from_type支持： <br/>- type约束仅支持float类型。 <br/>- 仅支持4维度Tensor计算。 <br/>to_type支持： <br/>- type约束仅支持float类型。 <br/>- 仅支持4维度Tensor计算。  | 
| DequantizeLinear          | CPU计算        | --                                                           | --                                                           |   
| Det                       | CPU计算※        | --                                                           | --                                                           |   
| Div                       | BPU加速         | 1. 只支持两个输入均为featuremap（不支持输入来自于常量）；  <br/>2. 对input shape的约束请参考Mul算子    | - 支持相同输入shape计算。 <br/>- 支持输入1是标量或者输入2是标量的计算。 <br/>- 支持broadcast计算，最大维度是5。 |  
| Dropout                   | BPU加速         | 该算子推理阶段不参加计算， 会被移除优化                                   | --                                                           |   
| Einsum                    | CPU计算※        | --                                                           | --                                                           |  
| Elu                       | CPU计算         | --                                                           | type约束：仅支持float类型。   |  
| Equal                     | CPU计算         | --                                                           | - 支持相同输入shape计算。 <br/>- 支持输入1是标量或者输入2是标量的计算。 <br/>- 支持broadcast计算，最大维度是5。   |    
| Erf                       | CPU计算        | --                                                           | type约束：支持float、double数据类型。                               |     
| Exp                       | BPU加速         | --                                                           | type约束：仅支持float类型。        | 
| Expand                    | CPU计算         | --                                                           | --                                                           | 
| EyeLike                   | CPU计算        | --                                                           | --                                                           |  
| Flatten                   | CPU计算         | --                                                           | --               | 
| Floor                     | CPU计算         | --                                                           | type约束：仅支持float类型。   | 
| GRU                       | CPU计算         | --                                                           | - direction属性仅支持forward类型。 <br/>- type约束：仅支持float类型。 <br/>- 仅支持输入个数是3、4、6。 <br/>- 输出个数是2。  | 
| Gather                    | CPU计算         | --                                                           | from_type支持： <br/>- input：type约束支持： <br/>float,int64,int32,int8,uint64,uint32,uint8。 <br/>- indices：type约束支持int32, int64。  <br/>to_type支持：type约束支持： <br/>float,int64,int32,int8,uint64,uint32,uint8。 |
| GatherElements            | CPU计算        | --                                                           | --                                                      |   
| GatherND                  | CPU计算         | --                          | from_type支持： <br/>- input：type约束支持float,int32,int8。 <br/>- indices：tensor(int64)。 <br/>to_type支持：type约束支持float,int32,int8。|    
| Gemm                      | BPU加速         | Gemm将被转化为Conv实现。 <br/> 假设Gemm的输入feature map的shape为NCHW： <br/> 1. 如果HW均小于等于7，则Gemm的限制等同于Conv。 <br/> 2. 如果H和W均为1，那么C的限制为 `<= 16384`；否则 C的大小限制为 `<= 2048`。 <br/> 3. 如果Gemm后是一个BPU支持的节点，Gemm会进行低精度int8输出，此时的输入宽高限制为: `H x W/8 x C/4 <= 1024`。 <br/> 4. 如果Gemm后是一个非BPU支持的节点，Gemm会进行高精度int32输出，此时的输入宽高限制为: `H x W/8 x C/4 < 2048`。 | type约束：仅支持float类型。    |  
| GlobalAveragePool         | BPU加速         | 假设输入shape为NCHW， 则输入宽高需满足 `HxW <= 8192 `          | 无                                                           | 
| GlobalLpPool              | CPU计算        | --                                                           | - type约束：支持float和double类型。 <br/> - 仅支持四维Tensor计算。 |  
| GlobalMaxPool             | BPU加速         | 假设输入shape为NCHW， 则输入宽高取值范围为HxW=[1,1024]x[1,1024] | - type约束仅支持float类型。 <br/>- 仅支持四维Tensor。 |   
| Greater                   | CPU计算         | --                                                           | - 支持相同输入shape计算。 <br/>- 支持输入1是标量或者输入2是标量的计算。 <br/>- 支持broadcast计算，最大维度是5。 | 
| HardSigmoid               | CPU计算         | --                                                           | type约束仅支持float类型。  |   
| Hardmax                   | CPU计算※        | --                                                           | --                                                           |  
| Identity                  | CPU计算         | --                                                           | --                                                         |    
| If                        | CPU计算※        | --                                                           | --                                                           |  
| InstanceNormalization     | CPU计算         | --                                                           |- type约束仅支持float类型。 <br/>- 支持第1个维度是channel的数据排布方式计算。   |  
| IsInf                     | CPU计算※        | --                                                           | --                                                           |   
| IsNaN                     | CPU计算※        | --                                                           | --                                                           |   
| LRN                       | CPU计算         | --                                                           | - type约束仅支持float类型。 <br/>- 仅支持四维Tensor。  | 
| LSTM                      | BPU加速         | 仅支持batch_size=1                                           | - 不支持属性设置。 <br/>- type约束仅支持float类型。 <br/>- 仅支持输入个数是3、4、8。 <br/>- 输出个数是2。 |   
| LeakyRelu                 | BPU加速         | 无                                                           | 无                                                           | 
| Less                      | CPU计算        | --                                                           | - 支持相同输入shape计算。 <br/>- 支持输入1是标量或者输入2是标量的计算。 <br/>- 支持broadcast计算，最大维度是5。| 
| LessOrEqual               | CPU计算         |                                                              |- 支持相同输入shape计算。 <br/>- 支持输入1是标量或者输入2是标量的计算。 <br/>- 支持broadcast计算，最大维度是5。  |  
| Log                       | CPU计算         | --                                                           | type约束：仅支持float类型。    | 
| LogSoftmax                | CPU计算         | --                                                           | type约束：仅支持float类型。    | 
| Loop                      | CPU计算※        | --                                                           | --                                                           |  
| LpNormalization           | CPU计算        | --                                                           | - p范数仅支持1或者2。 <br/>- type约束支持double类型和float类型。|  
| LpPool                    | CPU计算        | --                                                           | - auto_pad属性不支持。 <br/>- type约束支持double类型和float类型。 <br/>- 仅支持4维计算。 |  
| MatMulInteger             | CPU计算※        | --                                                           | --                                                           |    
| MatMul                    | BPU加速         | 对于两个输入分别为featuremap和weight的场景（即featuremap与常量相乘） <br/> 其中第一个输入是featuremap，第二个输入是weight，以下几种场景均可优化到BPU上运行： <br/>- K vs KxN、K vs 1xKxN、K vs 1x1xKxN  <br/>- MxK vs K、MxK vs KxN、MxK vs 1x1xKxN  <br/>- 1xMxK vs K、1xMxK vs 1xKxN  <br/>- 1x1xMxK vs K、1x1xMxK vs 1xKxN、1x1xMxK vs 1x1xKxN  <br/>- BxMxK vs KxN （B>=1）  <br/>- 1xBxMxK vs KxN （B>=1） <br/>- AxBxMxK vs KxN (A>1，B>1)  <br/>- 其中第一个输入是weight，第二个输入是featuremap，以下场景可优化到BPU上运行： <br/>- 1xBxMxK vs 1x1xKxN (B>1)  <br/>对于两个输入均为featuremap的场景（即两个featuremap相乘），以下场景可优化到BPU上运行： <br/>- 1xBxMxK vs 1x1xKxN （B>=1） | type约束：仅支持float类型。  |  
| Max                       | CPU计算         | --                                                           | - 支持1-∞个输入。 <br/>- 支持相同输入shape计算。 <br/>- 支持输入1是标量或者输入2是标量的计算。 <br/>- 支持broadcast计算，最大维度是5。 |  
| MaxPool                   | BPU加速         | Kernel宽高的取值范围为：[1, 64]x[1, 64]。 <br/> Stride取值范围为：[1,185]。 <br/>Padding值需要大于等于零。 <br/>MaxPool不支持dilation。 | 1. dilation只支持1x1。 <br/>2. 只支持数据行优先存储。 <br/>3. auto_pad属性不支持。 <br/>4. storage_order属性不支持。 <br/>5. 仅支持四维Tensor计算。 |   
| MaxRoiPool                | CPU计算         | --                                                           | 无                                                           |   
| Mean                      | CPU计算※        | --                                                           | --                                                           | 
| Min                       | CPU计算        | --                                                           | - 支持1-∞个输入。 <br/>- 支持相同输入shape计算。 <br/>- 支持输入1是标量或者输入2是标量的计算。 <br/>- 支持broadcast计算，最大维度是5。 |  
| Mod                       | CPU计算※        | --                                                           | --                                                           | 
| Mul                       | BPU加速         | Mul的两个输入都需要是四维并且C的大小需要 `<= 2048`。  <br/>同时仅支持如下shape的相乘：  <br/>1. (1xCxHxW vs 1xCxHxW)。  <br/>2. (1xCxHxW vs 1xCx1x1)。 <br/> 3. (1xCxHxW vs 1x1x1x1) 。 <br/>注意：输入的取值不能为0。 | - 支持相同输入shape计算。 <br/>- 支持输入1是标量或者输入2是标量的计算。 <br/>- 支持broadcast计算，最大维度是5。 |    
| Multinomial               | CPU计算※        | --                                                           | --                                                           |   
| Neg                       | CPU计算        | --                                                           | --                                                        |  
| NonZero                   | CPU计算         | --                                                           | - type约束支持：float,int32,int8。 <br/>- 支持1维计算。 <br/>- 支持4维计算。 |   
| Not                       | CPU计算        | --                                                           | --                                                           |  
| OneHot                    | CPU计算        | --                                                           | --                                                        |   
| Or                        | CPU计算         | --                                                           | - 支持相同输入shape计算。 <br/>- 支持输入1是标量或者输入2是标量的计算。  <br/>- 支持broadcast计算，最大维度是5。  |   
| PRelu                     | BPU加速         | --                                                        | - type约束支持：仅支持float类型。 <br/>- from_type：X和slope。 <br/>- to_type：Y。 <br/>- X的shape为data_shape，slope的为slope_shape ，shape约束如下：   <br/>- data_shape == slope_shape。    <br/>- slope_shape.ProdSize() == 1。    <br/>- X和slope仅支持NCHW排布的4维度计算，并且N、C维度值相等。      <br/>- HxW 与1x1（ slope_shape ）。      <br/>- HxW与Hx1（ slope_shape ）。      <br/>- HxW与1xW（ slope_shape ）。  <br/>- X是4维度 && slope是3维度 && data_shape[1] == slope_shape [0] && slope_shape [1] == 1 && slope_shape [2] == 1。  |                         |
| Pad                       | BPU加速         | 支持mode = Constant。 <br/>仅支持H，W维度的pad。   | Pad-10： <br/>- type约束仅支持float类型。 <br/>- 仅支持NCHW排布的4维Tensor。 <br/>- 属性pads的约束如下：   <br/>- len(pads) == 8 && pads[i] >=0 && pads[0] == 0 && pads[1] == 0 && pads[4] == 0 && pads[5] == 0。  <br/>Pad-11： <br/>- from_type支持：   <br/>- data：type约束仅支持float类型。   <br/>- pads : tensor(int64)。   <br/>- constant_value (optional)：type约束仅支持float类型。 <br/>- to_type支持：type约束仅支持float类型。 <br/>- 仅支持4维Tensor。 <br/>- 仅支持2/3维度填充。 |   
| Pow                       | BPU加速         | 只支持第二个输入（exponent）为单个值。                 | - type约束支持：double, float，int64, int32。 <br/>- 支持相同输入shape的计算。 <br/>- 支持输入1是标量或者输入2是标量的计算。 <br/>- 支持broadcast计算，最大维度是5。 <br/>- 仅支持X和Y相同type。 |  
| QLinearConv               | CPU计算※        | --                                                           | --                                                           |   
| QLinearMatMul             | CPU计算※        | --                                                           | --                                                           |
| QuantizeLinear            | CPU计算          | --                                                           | --                                                           |
| RNN                       | CPU计算         | --                                                           | - type约束：仅支持float类型。 <br/>- 属性约束：direction属性仅支持forward。 <br/>- 输入约束：仅支持X、W、R输入，不支持可选输入B、sequence_lens、initial_h设置。  <br/>- 输出约束：仅支持Y_h的输出，shape [num_directions, batch_size, hidden_size]。 |
| RandomNormal              | CPU计算※        | --                                                           | --                                                           |
| RandomNormalLike          | CPU计算※        | --                                                           | --                                                           |
| RandomUniform             | CPU计算         | --                                                           | --                                                           |
| RandomUniformLike         | CPU计算         | --                                                           | --                                                           |
| Range                     | CPU计算         | --                                                           |type约束支持：float,int64,int32,int16。            |
| Reciprocal                | BPU加速         | --                                                           | --                                                           |
| ReduceL1                  | CPU计算         | --                                                           | --                              |
| ReduceL2                  | CPU计算         | --                                                           | --       |
| ReduceLogSum              | CPU计算         | --                                                           | 仅支持float、double数据类型                           |
| ReduceLogSumExp           | CPU计算         | --                                                           | type约束支持float、double数据类型。               |
| ReduceMax                 | CPU计算         | --                                                           | axes支持0, 1或者等于输入数据的维数                           |
| ReduceMean                | BPU加速         | input featuremap需为四维，并且axes=[2, 3]                    | axes支持0, 1或者等于输入数据的维数                           |
| ReduceMin                 | CPU计算         | --                                                           | --                                                           |
| ReduceProd                | CPU计算         | --                                                           | --                                                           |
| ReduceSum                 | CPU计算         | --                                                           | axes支持0, 1或者等于输入数据的维数                           |
| ReduceSumSquare           | CPU计算         | --                                                           | axes支持0, 1或者等于输入数据的维数                           |
| Relu                      | BPU加速         | 会被融合到前一个conv中                                         | type约束：仅支持float类型。      |  
| Reshape                   | CPU计算         | --                                                           | --                                                         |
| Resize                    | BPU加速         | 1. 输入featuremap需为四维NCHW，并且只支持在H和W维度上进行resize，onnx opset=11时支持roi输入（pytorch转换的模型需手动修改算子添加roi输入，roi只支持常量输入），roi输入只支持H和W维度，roi输入只在tf_crop_and_resize模式下起作用。 <br/>2. 属性mode支持nearest和linear两种模式。 <br/>3. 支持放大和缩小。 <br/>4. 对于mode=nearest，放大系数factor支持2的幂数倍如2，4，8，16，32等；支持H维度和W维度的放大系数不同但需要满足 `H_factor <= W_factor`。 <br/>5. 对于onnx opset=11，属性coordinate_transformation_mode支持half_pixel，pytorch_half_pixel, asymmetric，align_corners和tf_crop_and_resize，当coordinate_transformation_mode=tf_crop_and_resize时，需要保证roi输入转换得到的边界坐标为整数。 | resize-10  <br/>- 输入等于2时，使用opset10。 <br/>- 输入数据是4维Tensor。  <br/>resize-11   <br/>- 输入大于2时，使用opset11。 <br/>- 输入数据是4维Tensor。 <br/>- coordinate_transformation_mode在nearest, linear模式下支持half_pixel, asymmetric, align_corners和pytorch_half_pixel四种，在cubic模式下只支持half_pixel。 <br/>- extrapolation_value属性不支持。 |
| ReverseSequence           | CPU计算         | --                                                           | --                                                           |
| RoiAlign                  | CPU计算         | --                                                           | --                                                           |
| Round                     | CPU计算        | --                                                           | --                                                           |
| Scan                      | CPU计算※        | --                                                           | --                                                           |
| Scatter (deprecated)      | CPU计算※        | --                                                           | --                                                           |
| ScatterElements           | CPU计算         | --                                                           | from_type支持： <br/>- data：type约束支持：float,int32,int8。 <br/>- indices：type约束仅支持int32类型。 <br/>- updates：type约束支持：float,int32,int8。 <br/>to_type支持：type约束支持：float,int32,int8。  |
| ScatterND                 | CPU计算         | --                                                           | from_type支持： <br/>- data：type约束支持：float,int32,int8。 <br/>- updates : type约束支持：float,int32,int8。 <br/>to_type支持：type约束支持：float,int32,int8。   |
| Selu                      | CPU计算         | --                                                           | type约束：仅支持float类型。    |
| SequenceAt                | CPU计算※        | --                                                           | --                                                           |
| SequenceConstruct         | CPU计算※        | --                                                           | --                                                           |
| SequenceEmpty             | CPU计算※        | --                                                           | --                                                           |
| SequenceErase             | CPU计算※        | --                                                           | --                                                           |
| SequenceInsert            | CPU计算※        | --                                                           | --                                                           |
| SequenceLength            | CPU计算※        | --                                                           | --                                                           |
| Shape                     | BPU加速         | 会通过常量折叠将其优化为数值存储                               | --  |
| Shrink                    | CPU计算※        | --                                                           | --                                                           |
| Sigmoid                   | BPU加速         | 对于一个输入维度为1CHW的tensor，仅支持min(8W4C对齐后的shape，32C对齐后的shape) `<=8192`的情况。 <br/>8W4C：实际运行时tensor的W维度padding至8的整数倍，C维度padding至4的整数倍。 <br/>32C：实际运行时tensor的C维度padding至32的整数倍。 <br/>在两个对齐方式中取对齐后shape最小值，判断是否`<=8192`。    | type约束：仅支持float类型。   |
| Sign                      | CPU计算         | --                                                           | 无                                                           |
| Sin                       | BPU加速         | 对于一个输入维度为1CHW的tensor，仅支持`CxHxW <= 8192`的情况       | type约束：仅支持float类型。     |
| Sinh                      | CPU计算         | --                                                           | type约束：仅支持float类型。     |
| Size                      | BPU加速         | 会通过常量折叠将其优化为数值存储                            | --                                                           |
| Slice                     | BPU加速         | 无限制                                                       | 无                                                           |
| Softmax                   | BPU加速         | 默认运行在CPU上，当该op输入为四维且axis=1，并且作为模型输出节点时，可以通过run_on_bpu指定该节点将其运行在BPU上。 | type约束：仅支持float类型。  |
| Softplus                  | BPU加速         | 对于一个输入维度为1CHW的tensor，仅支持`CxHxW <= 8192`的情况    | type约束：仅支持float类型。  |
| Softsign                  | CPU计算         | --                                                           | type约束：仅支持float类型。       |
| SpaceToDepth              | BPU加速         | 支持mode=DCR 和 mode=CRD。 <br/> 仅支持H和W方向的重新排列，并且仅支持blocksize=2的重排列。  <br/>举例：NxCxHxW -> Nx(4C)x(H/2)x(W/2) | type约束：仅支持float类型。    |
| Split                     | BPU加速         | 1. 只支持输入大小为NCHW；  <br/>2. 原始输入的长度必须是每个被切分的tensor长度的倍数；  <br/>3. 只支持沿着C，H，W维度的切分，也就是axis支持等于1，2，3；  <br/>4. split数应可以整除 | type约束：仅支持float类型。    |
| SplitToSequence           | CPU计算※        | --                                                           | --                                                           |
| Sqrt                      | BPU加速         | 对于一个输入维度为1CHW的tensor，仅支持`CxHxW <= 8192`的情况            |type约束：仅支持float类型。   |
| Squeeze                   | CPU计算         | 如果该op出现在模型中的常量计算子结构中，会被常量折叠优化删除掉，不参与推理      | --                                                          |            |
| StringNormalizer          | CPU计算※        | --                                                           | --                                                           |
| Sub                       | CPU计算         | --                                                           | - 支持相同输入shape计算。 <br/>- 支持输入1是标量或者输入2是标量的计算。 <br/>- 支持broadcast计算，最大维度是5。  |
| Sum                       | BPU加速         | 限制条件等同于Add                                            | type约束：仅支持float类型。   |   
| Tan                       | CPU计算         | --                                                           | type约束：仅支持float类型。   |
| Tanh                      | BPU加速         | 对于一个输入维度为1CHW的tensor，仅支持`CxHxW <= 8192`的情况       | type约束：仅支持float类型。   |
| TfIdfVectorizer           | CPU计算※        | --                                                           | --                                                           |
| ThresholdedRelu           | CPU计算         | --                                                           | type约束：仅支持float类型。  |
| Tile                      | CPU计算         | --                                                           | type约束：仅支持float,int64,int32,uint64,uint32类型。   |
| TopK                      | CPU计算         | --                                                           | - type约束：仅支持float类型。  <br/>- 仅支持opset-10。  |
| Transpose                 | CPU计算         | --                                                            | - 支持nhwc2nchw，perm：[0, 3, 1, 2]。 <br/>- 支持nchw2nhwc，perm：[0, 2, 3, 1]。 <br/>- 支持指定perm维度转换，数据类型仅支持float，int8，int32。   |
| Unique                    | CPU计算※        | --                                                           | --                                                           |
| Unsqueeze                 | CPU计算         | 如果该op出现在模型中的常量计算子结构中，会被常量折叠优化删除掉，不参与推理        | --                                                          |
| Upsample (resize替代)     | BPU加速          | --                                                           | Upsample-(resize-10)  <br/>- 输入等于2时，使用opset10。 <br/>- 输入数据是4维Tensor。  <br/>Upsample-(resize-11)   <br/>- 输入大于2时，使用opset11。 <br/>- 输入数据是4维Tensor。 <br/>- coordinate_transformation_mode在nearest, linear模式下支持half_pixel, asymmetric, align_corners和pytorch_half_pixel四种，在cubic模式下只支持half_pixel。 <br/>- extrapolation_value属性不支持。  |
| Where                     | CPU计算         | --                                                           | type约束支持float和int64类型。 <br/> condition的shape为cond_shape，X的shape为x_shape，Y的shape为y_shape ，output的shape为o_shape，shape约束如下： <br/>- 仅支持cond_shape == o_shape情况下：   <br/>- x_shape == o_shape的broadcast。   <br/>- y_shape == o_shape的broadcast。 <br/>- 仅支持cond_shape.NDim() == 4 && o_shape.NDim() == 4 && N维度值相同 && C维度值相同：   <br/>- 1x1（cond_shape）与HxW （o_shape）。   <br/>- Hx1（cond_shape）与HxW（o_shape）。   <br/>- 1xW（cond_shape）与HxW（o_shape）。 |
| Xor                       | CPU计算※        | --                                                           | --                                                           |
| Function                  | CPU计算※        | --                                                           | --                                                           |
| Celu                      | CPU计算※        | --                                                           | --                                                           |
| DynamicQuantizeLinear     | CPU计算※        | --                                                           | --                                                           |
| GreaterOrEqual            | CPU计算        | --    | - 支持相同输入shape计算。 <br/>- 支持输入1是标量或者输入2是标量的计算。 <br/>- 支持broadcast计算，最大维度是5。 |
| MeanVarianceNormalization | CPU计算※        | --                                                           | --                                                           |
| GridSample（PyTorch）     | CPU计算※         | --                                                           |                                                              |


## RDK X5支持的Caffe算子列表

| **caffe算子名称**       | **CPU计算/BPU加速** | **X5 BPU支持约束** | **CPU支持约束** |
| ------------------- | --------------- | --------------- | ----------- |
| Convolution   | BPU加速 | 限制条件等同于 ONNX Conv | 支持conv1d、conv2d、conv3d。<br/>type约束支持：float，int32，int8。<br/>auto_pad属性不支持。<br/>pads属性约束：<br/> - conv1d： [Dstart，Dend]，pads长度等于2，并且Dstart = Dend。<br/> - conv2d：[Hstart，Wstart，Hend，Wend]，pads长度等于4 ，并且Hstart==Hend，Wstart==Wend。<br/>- conv3d：[Dstart，Hstart，Wstart，Dend，Hend，Wend]，pads长度等于6 ，并且Dstart = Dend，Hstart==Hend，Wstart==Wend。                           |
| Deconvolution | BPU加速 | 限制条件等同于 ONNX ConvTranspose | shape约束：仅支持4维Tensor计算。 <br/>type约束：仅支持float类型。 <br/>attribute约束：<br/>- 仅支持dilations、group、output_padding、 pads 、strides 属性。<br/>- pads属性约束：[hstart, wstart, hend, wend]必须满足(hstart==hend and wstart==wend)。 |
| MaxUnpool| CPU计算 | --- | from_type支持：  <br/>- X：type约束：仅支持float类型。<br/>- I：Tensor（int64）。<br/>to_type支持：type约束：仅支持float类型。                                       |
| Pooling      | BPU加速 | 共有四种Pooling算子即MaxPooling，AveragePooling，GlobalMaxPooling，GlobalAveragePooling。对四种Pooling的约束分别为：<br/>- MaxPooling：<br/>该算子支持int16输入输出。<br/>kernel `<=` 256；<br/>stride `<=` 256；<br/>padding `<=` 256。<br/>MaxPooling不支持dilation。<br/>- AveragePooling：<br/>限制条件等同于 ONNX AveragePool <br/>- GlobalAveragePooling：<br/>无限制。<br/>- GlobalMaxPooling：<br/>H, W ∈ [1, 256]。 | 无  |
| SPP          | CPU计算 | 不支持  | 支持pyramid_height，`2^n` 次pooling, n `< 7`;<br/>pooling kernel 小于等于 255； <br/>支持pool，配置可选值为 `{0，1}` |
| InnerProduct | BPU加速 | InnerProduct将被转化为Conv实现，边界约束参考Conv。 <br/>不支持配置axis属性。 | 无                                                           |
| LRN  | CPU计算 | 不支持 | local_size 支持。<br/>alpha支持。<br/>beta 支持。<br/>norm_region 支持，配置可选值`{ACROSS_CHANNELS, WITHIN_CHANNEL }`。<br/>k 支持。 |
| MVN  | CPU计算 | 不支持 | normalize_variance支持，配置可选值为{0, 1}。<br/>across_channels支持，配置可选值为{0, 1}。<br/>仅支持Float32类型的计算。 |
| BatchNorm                                                    | BPU加速 | 无限制 | 无                                         |
| ELU                                                          | BPU加速 | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维，最高维 ∈ [1, 4096]，其它维 ∈ [1, 65536]。 | 无                                         |
| BNLL                                                         | CPU计算 | 不支持 | 无                                         |
| PReLU                                                        | BPU计算 | 1. 该算子仅支持int8输入输出。<br/> 2. 输入输出仅支持4维。 | - type约束支持：仅支持float类型。<br/>- from_type：X和slope。<br/>- to_type：Y。<br/>- X的shape为data_shape，slope的为slope_shape ，shape约束如下：<br/>  - data_shape == slope_shape 。<br/>  - slope_shape.ProdSize() == 1 。<br/>  - X和slope仅支持NCHW排布的4维度计算，并且N、C维度值相等。 <br/>    - HxW 与1x1（ slope_shape ）。 <br/>    - HxW与Hx1（ slope_shape ）。 <br/>    - HxW与1xW（ slope_shape ） 。<br/>  - X是4维度 && slope是3维度 && data_shape[1] == slope_shape [0] && slope_shape [1] == 1 && slope_shape [2] == 1。                         |
| ReLU/LeakyRelu                                               | BPU加速 | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维 | 无                                         |
| Sigmoid                                                      | BPU加速 | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维 | 无                                         |
| TanH                                                         | BPU加速 | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维 | 无                                         |
| Eltwise                                                      | BPU加速 | 目前支持的operation包括Add、Sub、Mul。<br/>1. 该算子支持int16输入输出。<br/>2. 输入类型支持featurmap和常量，且最多支持一个常量输入。<br/>3. 支持所有维度的广播，支持两个输入之间的互相广播，例如NH1C和N1WC。<br/>4. 输入输出维度支持1-10维，大小为一般限制（见备注）。支持两个输入维度不同，输入大于4维时可通过合并相邻维度降维到4维（包括N），合并规则是：<br/>(1)将输出dim为1的维度去除，例如[1, 2, 3, 4] [1, 2, 1, 4]->[1, 2, 3, 4]可看为[2, 3, 4],[2, 1, 4]->[2, 3,4]。<br/>(2)相邻的非广播维度可以合并，如[2, 5, 4, 5, 3] [2, 5, 1, 5, 3], 2, 5可以合并。<br/>(3)相邻的同一Tensor的广播维度可以合并: 如[2, 5, 4, 5, 2] [1, 1, 1, 5, 2] 2，5，4可以合并。<br/>(4)广播维度不能和相邻非广播维度合并，如[2, 5, 4, 5, 2] [2, 1, 4, 1, 2]不能合并；非同一Tensor的广播维度不能合并 [2, 1, 4, 1, 2] [1, 5, 1, 5, 1]。 | 无                                                           |
| Bias                                                         | BPU加速 | 参考Eltwise等于Add的情况                                     | 无                                                           |
| Scale                                                        | BPU加速 | 参考Eltwise等于Mul的情况                                     | 无                                                           |
| AbsVal                                                       | BPU加速 | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维| 无  |
| Exp                                                          | BPU加速 | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维| 无  |
| Log                                                          | BPU加速 | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维| 无  |
| Power                                                        | BPU加速 | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维<br/>3. 第二个输入只支持标量。| 无  |
| Threshold    | CPU计算 | 不支持      | 无                                                           |
| Reduction                                                    | CPU计算 | 不支持  | operation 支持 SUM、ASUM、 SUMSQ、MEAN、Max、LogSum、Min、Prod； <br/>axis 支持； <br/> 仅支持Float32类型的计算。 |
| Softmax                                                      | BPU加速 | 1. 该算子支持int16输入输出。<br/>2. 默认运行在CPU上，当该op输入为四维并且axis=1,2,3时，可以通过run_on_bpu指定该节点将其运行在BPU上。 | 无 |
| ArgMax                                                       | BPU加速 | 1. 仅支持 axis=1，c`<=`64。<br/>2. 不支持配置top_k != 1。<br/>3. 该算子支持int16输入输出。| 无  |
| Concat                                                       | BPU加速 | 1. 该算子支持int16输入输出。<br/>2. 不支持N维度concat。 | 无                                                           |
| Split                                                        | BPU加速 | 1. 该算子支持int16输入输出。<br/>2. 原始输入的长度必须是每个被切分的tensor长度的倍数。<br/>3. 支持除N维度以外的任意维度。<br/>4. split数应可以整除。<br/>5. 支持非四维输入输出。| 无 |
| Slice                                                        | BPU加速 | 1. 该算子支持int16输入输出。<br/>2. 无限制，支持非四维输入输出。 | 无                                                           |
| Reshape                                                      | BPU加速 | 1. 该算子支持int16输入输出。<br/>2. 支持1-10维输入输出。| shape 支持`[1,4]`个 shape_dim 配置 ；<br/> axis 支持`[-4,3]`范围内可配，不支 持 N 维度，默认值 0，遵循 caffe 规则 ；<br/> num_axes 支持`[-1,3]`范围内可配，默认 值 -1 表示对 axis 起始的所有 轴进行变换 |
| Flatten                                                      | CPU计算 | 不支持（一些场景下可以融合）                                 | axis 取值范围`[-4,3]`，默认值 为 1，-4 与 0 含义相同。 <br/>只支持End_axis == -1。 |
| Crop    | CPU计算 | 不支持           | 无                                                           |
| Dropout  | BPU加速 | 无限制        | 无                                                           |
| LSTM     | BPU加速 | 仅支持batch=1     | --                                                           |
| Normalize   | CPU计算 | 不支持        | type约束：仅支持float类型。                                 |
| PassThrough  | BPU加速 | 支持mode=DCR 和 mode=CRD。<br/>仅支持H和W方向的重新排列，并且仅支持blocksize=2的重排列。<br/>举例：NxCxHxW -> Nx(4C)x(H/2)x(W/2)。  | type约束：仅支持float类型。  |
| CReLU | CPU计算 | 不支持       | type约束：仅支持float类型。                                 |
| RReLU  | CPU计算 | 不支持     | 无                                                          |
| Permute                                                       | BPU加速 | 1. 支持任意输入维度。<br/>2. 除batch维度（第一维）以外，支持任意其它维度的转换。 | - 支持nhwc2nchw，perm：[0, 3, 1, 2]。 <br/> - 支持nchw2nhwc，perm：[0, 2, 3, 1]。<br/> - 支持指定perm维度转换，数据类型仅支持float，int8，int32。 |
| MatMul                                                         | BPU加速 | C = MatMul(A，B)，对输入A和输入B有以下维度限制：<br/>- A和B均支持非四维输入但需满足约束：<br/>  - A和B的维度必须相同。<br/>  - A和B的最低两个维度M, K ∈ [1, 8192]，其他更高维度∈[1, 4096]。    <br/>  注：HDMK vs HDKN，MK/KN即为最低两个维度。<br/>- 支持的broadcast需满足以下条件：<br/>  - A 跟B两个输入，除开最低两维的其他维度全是1或者全是不需要广播的值。<br/>    - 此场景支持的例子：HDMK vs H1KN<br/>    - 此场景不支持反例：H1MK vs 1DKN<br/>  - A除了最低两个维度，其他维度不能即有需要广播的值也有不需要广播的值。<br/>    - 此场景支持的例子：11MK vs HDKN<br/>    - 此场景不支持反例：H1MK vs HDKN<br/>  - B除了最低两个维度，如果其他维度即有需要广播的值也有不需要广播的值，那么不需要广播的值只能在连续的高维度上。<br/>    - 此场景支持的例子：BHDMK vs B11KN<br/>    - 此场景不支持反例：BHDMK vs B1DKN  <br/>  注：需要广播的值和不需要广播的值：<br/>   <br/>- 如果A和B在对应维度轴上的两个值，一个为1，另一个为非1，那么1就是需要广播的值，非1就是不需要广播的值；<br/>    - 如果A和B在对应维度轴上的两个值相等，那么这两个值都是不需要广播的值（如HDMK vs H1KN，1是需要广播的值，H是不需要广播的值）。 | type约束：仅支持float类型。                                                       |
| Upsample                                                     | BPU加速 | 输入featuremap需为四维NCHW，并且只支持在H和W维度上进行resize； <br/> 放大系数factor不能同时小于2。 | 无  |
| ROIPooling  | CPU计算 | 不支持       | 无                                                           |
| PSROIPooling  | CPU计算 | 不支持     | 无                                                           |



## RDK X5支持的ONNX算子列表

| **ONNX算子名称** | **CPU计算/BPU加速** | **X5 BPU支持约束** | **CPU支持约束** |
| ------------ | --------------- | --------------- | ----------- |
| Abs  | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维| type约束：仅支持float类型。   |  
| Acos  | BPU加速         |1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维| type约束支持float和double类型。 |    
| Acosh | BPU加速         |1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维| type约束支持float和double类型。|    
| Add                       | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入类型支持featurmap和常量，且最多支持一个常量输入。<br/>3. 支持所有维度的广播，支持两个输入之间的互相广播，例如NH1C和N1WC。<br/>4. 输入输出维度支持1-10维，大小为一般限制（见备注）。支持两个输入维度不同，输入大于4维时可通过合并相邻维度降维到4维（包括N），合并规则是：<br/>(1)将输出dim为1的维度去除，例如[1, 2, 3, 4] [1, 2, 1, 4]->[1, 2, 3, 4]可看为[2, 3, 4],[2, 1, 4]->[2, 3,4]。<br/>(2)相邻的非广播维度可以合并，如[2, 5, 4, 5, 3] [2, 5, 1, 5, 3], 2, 5可以合并。<br/>(3)相邻的同一Tensor的广播维度可以合并: 如[2, 5, 4, 5, 2] [1, 1, 1, 5, 2] 2，5，4可以合并。<br/>(4)广播维度不能和相邻非广播维度合并：如[2, 5, 4, 5, 2] [2, 1, 4, 1, 2]不能合并；非同一Tensor的广播维度不能合并 [2, 1, 4, 1, 2] [1, 5, 1, 5, 1]。<br/>5. 作为resnet中的short-cut子结构的Add，会被融合到上一个conv中加速计算。 | - 支持相同输入shape计算。<br/>- 支持输入1是标量或者输入2是标量的计算。<br/>- 支持broadcast计算，最大维度是8。 |          
| And                       | CPU计算         | --  | - 支持相同输入shape计算。<br/>- 支持输入1是标量或者输入2是标量的计算。<br/>- 支持broadcast计算，最大维度是8。 |                      
| ArgMax                    | BPU加速         | 1. 输入维度为四维输入NCHW。<br/>2. N ∈ [1, 4096]，H,W ∈ [1, 65536] ，C ∈ [1, 8191]。<br/>3. 该算子支持int16输入输出。<br/>4. 仅支持沿C维度进行argmax/argmin，即axis=1。| type约束：仅支持float类型。|
| ArgMin                    | BPU加速         | 1. 输入维度为四维输入NCHW。<br/>2. N ∈ [1, 4096]，H,W ∈ [1, 65536] ，C ∈ [1, 8191]。<br/>3. 该算子支持int16输入输出。<br/>4. 仅支持沿C维度进行argmax/argmin，即axis=1。| type约束：仅支持float类型。| 
| Asin                      | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维。| type约束支持float和double类型。 |                         
| Asinh                     | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维。| type约束支持float和double类型。|                        
| Atan                      | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维。| type约束支持float和double类型。|                       
| Atanh                     | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维。| type约束支持float和double类型。|                       
| AveragePool               | BPU加速         | kernel: H,W ∈ [1, 256] <br/> H * W `<=` 8192 ，H * W > 1 <br/>stride: H,W ∈ [1, 256]< br/>padding: H,W ∈ [0, 255] | 输入和输出支持4维和5维。|                      
| BatchNormalization        | BPU加速         | 无限制。  | type约束：仅支持float类型。 <br/>支持第1个维度是channel的数据排布方式计算。   |         
| BitShift                  | CPU计算※        | --                                                           | --                                                           |     
| Cast                      | CPU计算         | --                                                           | from_type支持double, float, bool, int64, uint32, int32, uint16, int16, uint8, int8。<br/>to_type支持double, float, bool, int64, uint32, int32, uint16, int16, uint8, int8。|    
| Ceil                      | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维| type约束支持double类型和float类型。     | 
| Clip                      | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维| opset6: <br/>min, max作为属性值，dtype仅支持float类型;<br/>opset11: <br/>min, max作为输入，仅有两个输入时，第二个为min；dtype支持float, double类型。 | 
| Compress                  | CPU计算※        | --                                                           | --                                                           |  
| Concat                    | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 不支持N维度concat。 | --                                                          |  
| ConcatFromSequence        | CPU计算※        | --                                                           | --                                                           |   
| Constant                  | BPU加速         | 会通过常量折叠将其优化为数值存储                                | 目前不支持sparse_tensor属性。 |    
| ConstantOfShape           | BPU加速         | 会通过常量折叠将其优化为数值存储                                 | type约束支持：float,int32,int8。   |    
| Conv                      | BPU加速         | 支持四维输入（conv2d）和五维输入（conv3d）。<br/>四维输入（conv2d）：<br/>Kernel shape范围：N,C ∈ [1, 8192]; H,W ∈ [1, 31]。C * H * W < = 32767。<br/>输入输出Channel取值范围(one group) `<=` 8192，如果Conv是量化子图的最后一个算子，取值范围`<=` 65536。<br/>stride取值范围：H,W ∈ [1, 256]，但对于Conv后接Add(resnet shortcut-connecting) Stride取值范围为：{1, 2}，对dilated>1的conv，stride只支持=1。<br/>Dilation取值范围：H,W∈ [1, 16]，H或W大于1时，只支持输出int8，且输入Tensor的H必须能被dilation的H整除，输入Tensor的W必须能被dilation的W整除。<br/>padding取值范围：H,W ∈ [0, 256]。<br/>五维输入（conv3d）：<br/>输入大小NCDHW：N ∈ [1, 128]; H,W,D,C ∈ [1, 65536]。<br/>kernel大小NCDHW：N,C ∈ [1, 65536]; H,W ∈ [1, 31], D ∈ [1, 8191]。<br/>padding大小DHW：H,W ∈ [0, 256], D ∈ [0, kernel_d/2]。<br/>stride取值范围：H, W同为1或H, W同为2。<br/>group，dilation暂不支持。<br/>Size: 1G bytes；当D * C > 4096时, H * alignCeil(W, 256) * D * C < 1G。<br/>weight的D * 输入的C `<=` 8192。 | 支持conv1d、conv2d、conv3d。<br/>type约束支持：float，int32，int8。<br/>auto_pad属性不支持。<br/>pads属性约束：<br/>- conv1d： [Dstart，Dend]，pads长度等于2，并且Dstart = Dend。<br/>- conv2d：[Hstart，Wstart，Hend，Wend]，pads长度等于4 ，并且Hstart==Hend，Wstart==Wend。<br/>- conv3d：[Dstart，Hstart，Wstart，Dend，Hend，Wend]，pads长度等于6 ，并且Dstart = Dend，Hstart==Hend，Wstart==Wend。|   
| ConvInteger               | CPU计算※        | --                      | --  |
| ConvTranspose             | BPU加速         | 输入输出featuremap大小限制：<br/>N ∈ [1, 128]。<br/>H,W ∈ [1, 65536]。 <br/>C ∈ [1, 2048] 。<br/>Size: 1G bytes。<br/>weight大小限制：<br/>N,C ∈ [1, 2048]。 <br/>H,W ∈ [1, 14]且HW不同时为1。<br/>Size: <br/>psh = padding.h % stride.h;<br/>psw = padding.w % stride.w;<br/>ksh = (kernel.h - 1 + psh) / stride.h +1<br/>ksw = (kernel.w - 1 + psw) / stride.w + 1<br/>group_num = fout.c / kernel.c<br/>ksc = fin.c / group_num<br/>kernel_size = ksh * ksw * ksc<br/>kernel_size ∈ [1, 32767] <br/>padding取值范围：<br/>stride为奇数时，H,W ∈ [0, kernel / stride)。<br/>stride为偶数，H,W ∈ [0, kernel / stride]。<br/>out_pad取值范围：H,W ∈ {0,1}。<br/>stride >= 1 && stride `<=`14 但不支持stride_h和stride_w同时等于1。<br/>Dilation ∈ {(1, 1)}。| shape约束：仅支持4维Tensor计算。<br/>type约束：仅支持float类型。<br/>attribute约束：<br/>- 仅支持dilations、group、output_padding、 pads 、strides 属性。<br/>- pads属性约束：[hstart, wstart, hend, wend]必须满足(hstart==hend and wstart==wend)。|  
| Cos                       | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维| type约束支持float类型。|   
| Cosh                      | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维| type约束支持float类型。|   
| CumSum                    | CPU计算         | --                                                           | axis：type约束仅支持int32类型。 |   
| DepthToSpace              | BPU加速         | 该算子支持int16输入输出。<br/> 支持mode=DCR 和 mode=CRD。<br/> 仅支持H和W方向的重新排列，并且仅支持blocksize=2的重排列。 <br/>举例：NxCxHxW -> Nx(C/4)x(2H)x(2W) 输出的channel必须是4的倍数。| from_type支持：<br/>- type约束仅支持float类型。<br/>- 仅支持4维度Tensor计算。<br/>to_type支持：<br/>- type约束仅支持float类型。<br/>- 仅支持4维度Tensor计算。  | 
| DequantizeLinear          | CPU计算        | --       | --         |   
| Det                       | CPU计算※        | --  | --       |   
| Div                       | BPU加速         | 对input shape的约束请参考Mul算子。  | - 支持相同输入shape计算。<br/>- 支持输入1是标量或者输入2是标量的计算。<br/>- 支持broadcast计算，最大维度是8。 |  
| Dropout                   | BPU加速         | 该算子推理阶段不参加计算， 会被移除优化                                   | --                                                           |   
| Einsum                    | CPU计算※        | --                                                           | --                                                           |  
| Elu                       | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维| type约束：仅支持float类型。   |  
| Equal                     | BPU加速         | 1. 该算子支持int16输入。<br/>2. 支持所有维度的广播，支持fin0或fin1其中一个输入的广播，不能支持互相广播。<br/>3. 输入输出维度支持1-10维，大小为一般限制（见备注）。输入大于4维时可通过合并相邻维度降维到4维（包括N），合并规则是：<br/>(1)将输出dim为1的维度去除，例如[1, 2, 3, 4] [1, 2, 1, 4]->[1, 2, 3, 4]可看为[2, 3, 4],[2, 1, 4]->[2, 3,4]。<br/>(2)相邻的非广播维度可以合并，如[2, 5, 4, 5, 3] [2, 5, 1, 5, 3], 2, 5可以合并。<br/>(3)相邻的同一Tensor的广播维度可以合并: 如[2, 5, 4, 5, 2] [1, 1, 1, 5, 2] 2，5，4可以合并。<br/>(4)广播维度不能和相邻非广播维度合并：如[2, 5, 4, 5, 2] [2, 1, 4, 1, 2]不能合并。<br/>4. 默认运行在CPU上，可以通过run_on_bpu指定该节点将其运行在BPU上。 | - 支持相同输入shape计算。<br/>- 支持输入1是标量或者输入2是标量的计算。<br/>- 支持broadcast计算，最大维度是8。   |    
| Erf                       | BPU加速        | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维 | type约束：支持float数据类型。                              |     
| Exp                       | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维 | type约束：仅支持float类型。        | 
| Expand                    | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维，输入与输出仅支持有一个维度上的数值不同。<br/>3. 输入与输出仅允许有一个维度上数值不同。 | --    | 
| EyeLike                   | CPU计算        | --                                                           | --                                                           |  
| Flatten                   | BPU加速         | 限制条件等同于Reshape。 | --               | 
| Floor                     | BPU加速         |  1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维 | type约束：仅支持float类型。   | 
| GRU                       | CPU计算         | --                                                           | - direction属性仅支持forward类型。<br/>- type约束：仅支持float类型。  | 
| Gather                    | BPU加速         | 1. input/output/indices 的rank都要小于等于4。<br/>2. indices支持：<br/>    - indices是feature（其他op输出）时，type约束仅支持int32类型。<br/>    - indices是weight（模型保存的常量）时，type约束支持int32和int64类型。 | from_type支持：<br/>- input：type约束支持：<br/>float,int64,int32,int8,uint64,uint32,uint8。<br/>- indices：type约束支持int32, int64。 <br/>to_type支持：type约束支持：<br/>float,int64,int32,int8,uint64,uint32,uint8。 |
| GatherElements            | BPU加速        | 1. 该算子支持int16输入输出。<br/>2. input/indices/output维度支持1-10维。<br/>3. 当输入维度 i != axis时，要求indices.shape[i] `<=` input.shape[i]。 | --     |   
| GatherND                  | CPU计算         | --                          | from_type支持：<br/>- input：type约束支持float,int32,int8。<br/>- indices：tensor(int64)。<br/>to_type支持：type约束支持float,int32,int8。|    
| Gemm                      | BPU加速         | Gemm将被转化为Conv实现，边界约束参考Conv。 | type约束：仅支持float类型。    |  
| GlobalAveragePool         | BPU加速         | 无限制。           | - type约束：仅支持float类型。<br/>- 仅支持四维Tensor。| 
| GlobalLpPool              | CPU计算        | --                                                           | - type约束：支持float和double类型。<br/> - 仅支持四维Tensor计算。 |  
| GlobalMaxPool             | BPU加速         | H, W ∈ [1, 256]。 | - type约束仅支持float类型。<br/>- 仅支持四维Tensor。 |   
| Greater                   | BPU加速         | 1. 该算子支持int16输入。<br/>2. 支持所有维度的广播，支持fin0或fin1其中一个输入的广播，不能支持互相广播。<br/>3. 输入输出维度支持1-10维，大小为一般限制（见备注）。输入大于4维时可通过合并相邻维度降维到4维（包括N），合并规则是：<br/>(1)将输出dim为1的维度去除，例如[1, 2, 3, 4] [1, 2, 1, 4]->[1, 2, 3, 4]可看为[2, 3, 4],[2, 1, 4]->[2, 3,4]。<br/>(2)相邻的非广播维度可以合并，如[2, 5, 4, 5, 3] [2, 5, 1, 5, 3], 2, 5可以合并。<br/>(3)相邻的同一Tensor的广播维度可以合并：如[2, 5, 4, 5, 2] [1, 1, 1, 5, 2] 2，5，4可以合并。<br/>(4)广播维度不能和相邻非广播维度合并：如[2, 5, 4, 5, 2] [2, 1, 4, 1, 2]不能合并。<br/>4. 默认运行在CPU上，可以通过run_on_bpu指定该节点将其运行在BPU上。 | - 支持相同输入shape计算。<br/>- 支持输入1是标量或者输入2是标量的计算。<br/>- 支持broadcast计算，最大维度是8。 | 
| HardSigmoid               | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维 | type约束仅支持float类型。  |   
| Hardmax                   | CPU计算※        | --                                                           | --                                                           |  
| Identity                  | CPU计算         | --                                                           | --                                                         |    
| If                        | CPU计算※        | --                                                           | --                                                           |  
| InstanceNormalization     | CPU计算         | --                                                           |- type约束仅支持float类型。<br/>- 支持第1个维度是channel的数据排布方式计算。   |  
| IsInf                     | CPU计算※        | --                                                           | --                                                           |   
| IsNaN                     | CPU计算※        | --                                                           | --                                                           |   
| LRN                       | CPU计算         | --                                                           | - type约束仅支持float类型。<br/>- 仅支持四维Tensor。  | 
| LSTM                      | BPU加速         | 仅支持batch_size=1，如果需要配置多batch，需要在导出onnx时保证LSTM的batch为1并在yaml中配置参数input_batch=1。 | - type约束仅支持float类型。<br/>- 属性约束：direction属性仅支持forward。<br/>- 输入约束：<br/>   - 支持X、W、R输入配置；<br/>   - 支持X、W、R、B输入配置（sequence_lens为空或默认值）；<br/>   -  支持X、W、R、B、sequence_lens、initial_h、initial_c、P输入配置（sequence_lens为空或者默认值）。 |   
| LeakyRelu                 | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维 | type约束：仅支持float类型。  | 
| Less                      | BPU加速        | 1. 该算子支持int16输入。<br/>2. 支持所有维度的广播，支持fin0或fin1其中一个输入的广播，不能支持互相广播。<br/>3. 输入输出维度支持1-10维，大小为一般限制（见备注）。输入大于4维时可通过合并相邻维度降维到4维（包括N），合并规则是：<br/>(1)将输出dim为1的维度去除，例如[1, 2, 3, 4] [1, 2, 1, 4]->[1, 2, 3, 4]可看为[2, 3, 4],[2, 1, 4]->[2, 3,4]。<br/>(2)相邻的非广播维度可以合并，如[2, 5, 4, 5, 3] [2, 5, 1, 5, 3], 2, 5可以合并。<br/>(3)相邻的同一Tensor的广播维度可以合并：如[2, 5, 4, 5, 2] [1, 1, 1, 5, 2] 2，5，4可以合并。<br/>(4)广播维度不能和相邻非广播维度合并：如[2, 5, 4, 5, 2] [2, 1, 4, 1, 2]不能合并。<br/>4. 默认运行在CPU上，可以通过run_on_bpu指定该节点将其运行在BPU上。 | - 支持相同输入shape计算。<br/>- 支持输入1是标量或者输入2是标量的计算。<br/>- 支持broadcast计算，最大维度是8。| 
| LessOrEqual               | BPU加速         |opset11 不支持单个LessOrEqual算子，支持拆分后的算子Greater+Not运行在BPU上，限制条件与Greater相同。|- 支持相同输入shape计算。<br/>- 支持输入1是标量或者输入2是标量的计算。<br/>- 支持broadcast计算，最大维度是8。  |  
| Log                       | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维 | type约束：仅支持float类型。    | 
| LogSoftmax                | CPU计算         | --                                                           | type约束：仅支持float类型。    | 
| Loop                      | CPU计算※        | --                                                           | --                                                           |  
| LpNormalization           | CPU计算        | --                                                           | - p范数仅支持1或者2。<br/>- type约束支持float类型。|  
| LpPool                    | CPU计算        | --                                                           | - auto_pad属性不支持。<br/>- type约束支持float类型。<br/>- 仅支持4维计算。 |  
| MatMulInteger             | CPU计算※        | --                                                           | --                                                           |    
| MatMul                    | BPU加速         | C = MatMul(A，B)，对输入A和输入B有以下维度限制：<br/>- A和B均支持非四维输入但需满足约束：<br/>  - A和B的维度必须相同。<br/>  - A和B的最低两个维度M, K∈[1, 8192]，其他更高维度∈[1, 4096]。    <br/>  注：HDMK vs HDKN，MK/KN即为最低两个维度。<br/>- 支持的broadcast需满足以下条件：<br/>  - A跟B两个输入，除开最低两维的其他维度全是1或者全是不需要广播的值。<br/>    - 此场景支持的例子：HDMK vs H1KN<br/>    - 此场景不支持反例：H1MK vs 1DKN<br/>  - A除了最低两个维度，其他维度不能即有需要广播的值也有不需要广播的值。<br/>    - 此场景支持的例子：11MK vs HDKN<br/>    - 此场景不支持反例：H1MK vs HDKN<br/>  - B除了最低两个维度，如果其他维度即有需要广播的值也有不需要广播的值，那么不需要广播的值只能在连续的高维度上。<br/>    - 此场景支持的例子：BHDMK vs B11KN<br/>    - 此场景不支持反例：BHDMK vs B1DKN  <br/>  注：需要广播的值和不需要广播的值：<br/>    - 如果A和B在对应维度轴上的两个值，一个为1，另一个为非1，那么1就是需要广播的值，非1就是不需要广播的值；<br/>    - 如果A和B在对应维度轴上的两个值相等，那么这两个值都是不需要广播的值（如HDMK vs H1KN，1是需要广播的值，H是不需要广播的值） | type约束：仅支持float类型。  |  
| Max                       | BPU加速         | 1.该算子支持int16输入输出。<br/>2.支持所有维度的广播，支持两个输入之间的互相广播，例如NH1C和N1WC。<br/>3.输入输出维度支持1-10维，大小为一般限制（见备注）。支持两个输入维度不同，输入大于4维时可通过合并相邻维度降维到4维（包括N），合并规则是：<br/>(1)将输出dim为1的维度去除，例如[1, 2, 3, 4] [1, 2, 1, 4]->[1, 2, 3, 4]可看为[2, 3, 4],[2, 1, 4]->[2, 3,4]。<br/>(2)相邻的非广播维度可以合并，如[2, 5, 4, 5, 3] [2, 5, 1, 5, 3], 2, 5可以合并。<br/>(3)相邻的同一Tensor的广播维度可以合并：如[2, 5, 4, 5, 2] [1, 1, 1, 5, 2] 2，5，4可以合并。<br/>(4)广播维度不能和相邻非广播维度合并：如[2, 5, 4, 5, 2] [2, 1, 4, 1, 2]不能合并；非同一Tensor的广播维度不能合并 [2, 1, 4, 1, 2] [1, 5, 1, 5, 1]。| - 支持1-∞个输入。<br/>- 支持相同输入shape计算。<br/>- 支持输入1是标量或者输入2是标量的计算。<br/>- 支持broadcast计算，最大维度是8。 |  
| MaxPool                   | BPU加速         | 该算子支持int16输入输出。<br/>kernel `<=` 256。<br/>stride `<=` 256。<br/>padding `<=` 256。<br/>MaxPool不支持dilation。 | 1. dilation只支持1x1。<br/>2. 只支持数据行优先存储。<br/>3. auto_pad属性不支持。<br/>4. storage_order属性不支持。<br/>5. 输入和输出支持4维和5维。|   
| MaxRoiPool                | CPU计算         | --                                                           | 无                                                           |   
| Mean                      | CPU计算※        | --                                                           | --                                                           | 
| Min                       | BPU加速        | 1.该算子支持int16输入输出。<br/>2.支持所有维度的广播，支持两个输入之间的互相广播，例如NH1C和N1WC。<br/>3.输入输出维度支持1-10维，大小为一般限制（见备注）。支持两个输入维度不同，输入大于4维时可通过合并相邻维度降维到4维（包括N），合并规则是：<br/>(1)将输出dim为1的维度去除，例如[1, 2, 3, 4] [1, 2, 1, 4]->[1, 2, 3, 4]可看为[2, 3, 4],[2, 1, 4]->[2, 3,4]。<br/>(2)相邻的非广播维度可以合并，如[2, 5, 4, 5, 3] [2, 5, 1, 5, 3], 2, 5可以合并。<br/>(3)相邻的同一Tensor的广播维度可以合并：如[2, 5, 4, 5, 2] [1, 1, 1, 5, 2] 2，5，4可以合并。<br/>(4)广播维度不能和相邻非广播维度合并：如[2, 5, 4, 5, 2] [2, 1, 4, 1, 2]不能合并；非同一Tensor的广播维度不能合并 [2, 1, 4, 1, 2] [1, 5, 1, 5, 1]。<br/>4. 默认运行在CPU上，可以通过run_on_bpu指定该节点将其运行在BPU上。| - 支持1-∞个输入。<br/>- 支持相同输入shape计算。<br/>- 支持输入1是标量或者输入2是标量的计算。<br/>- 支持broadcast计算，最大维度是8。 |  
| Mod                       | CPU计算        | --                                                           | --                                                           | 
| Mul                       | BPU加速         | 1. 该算子支持int16输入输出。<br/>2.输入类型支持featurmap和常量，且最多支持一个常量输入。<br/>3. 支持所有维度的广播，支持两个输入之间的互相广播，例如NH1C和N1WC。<br/>4. 输入输出维度支持1-10维，大小为一般限制（见备注）。支持两个输入维度不同，输入大于4维时可通过合并相邻维度降维到4维（包括N），合并规则是：<br/>(1)将输出dim为1的维度去除，例如[1, 2, 3, 4] [1, 2, 1, 4]->[1, 2, 3, 4]可看为[2, 3, 4],[2, 1, 4]->[2, 3,4]。<br/>(2)相邻的非广播维度可以合并，如[2, 5, 4, 5, 3] [2, 5, 1, 5, 3], 2, 5可以合并。<br/>(3)相邻的同一Tensor的广播维度可以合并: 如[2, 5, 4, 5, 2] [1, 1, 1, 5, 2] 2，5，4可以合并。<br/>(4)广播维度不能和相邻非广播维度合并：如[2, 5, 4, 5, 2] [2, 1, 4, 1, 2]不能合并；非同一Tensor的广播维度不能合并 [2, 1, 4, 1, 2] [1, 5, 1, 5, 1]。| - 支持相同输入shape计算。<br/>- 支持输入1是标量或者输入2是标量的计算。<br/>- 支持broadcast计算，最大维度是8。 |    
| Multinomial               | CPU计算※        | --                                                           | --                                                           |   
| Neg                       | CPU计算        | --                                                           | --                                                        |  
| Not                       | CPU计算        | --                                                           | --                                                           |  
| OneHot                    | CPU计算        | --                                                           | --                                                        |   
| Or                        | CPU计算         | --                                     | - 支持相同输入shape计算。<br/>- 支持输入1是标量或者输入2是标量的计算。 <br/>- 支持broadcast计算，最大维度是8。|   
| PRelu                     | BPU加速         | 1. 该算子仅支持int8输入输出。<br/>2. 输入输出仅支持4维。| - type约束支持：仅支持float类型。<br/>- from_type：X和slope。<br/>- to_type：Y。<br/>- X的shape为data_shape，slope的为slope_shape ，shape约束如下：  <br/>- data_shape == slope_shape。   <br/>- slope_shape.ProdSize() == 1。   <br/>- X和slope仅支持NCHW排布的4维度计算，并且N、C维度值相等。     <br/>- HxW 与1x1（ slope_shape ）。     <br/>- HxW与Hx1（ slope_shape ）。     <br/>- HxW与1xW（ slope_shape ）。 <br/>- X是4维度 && slope是3维度 && data_shape[1] == slope_shape [0] && slope_shape [1] == 1 && slope_shape [2] == 1。  |                         |
| Pad                       | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 支持mode = Constant。<br/>3. 支持所有维度的Pad。 | Pad-10：<br/>- type约束仅支持float类型。<br/>- 仅支持NCHW排布的4维Tensor。<br/>- 属性pads的约束如下：  <br/>- len(pads) == 8 && pads[i] >=0 && pads[0] == 0 && pads[1] == 0 && pads[4] == 0 && pads[5] == 0。 <br/>Pad-11：<br/>- from_type支持：  <br/>- data：type约束仅支持float类型。  <br/>- pads : tensor(int64)。  <br/>- constant_value (optional)：type约束仅支持float类型。<br/>- to_type支持：type约束仅支持float类型。<br/>- 输入和输出支持4维，仅支持2/3维度填充。<br/>- 输入和输出支持5维，仅支持2/3/4维度填充。|   
| Pow                       | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维 <br/>3. 第二个输入只支持标量。  | - type约束支持：double, float，int64, int32。<br/>- 支持相同输入shape的计算。<br/>- 支持输入1是标量或者输入2是标量的计算。<br/>- 支持broadcast计算，最大维度是5。<br/>- 仅支持X和Y相同type。 |  
| QLinearConv               | CPU计算※        | --                                                           | --                                                           |   
| QLinearMatMul             | CPU计算※        | --                                                           | --                                                           |
| QuantizeLinear            | CPU计算          | --                                                           | --                                                           |
| RNN                       | CPU计算         | --                                                           | - type约束：仅支持float类型。<br/>- 属性约束：direction属性仅支持forward。<br/>- 输入约束：仅支持X、W、R输入，不支持可选输入B、sequence_lens、initial_h设置。 <br/>- 输出约束：仅支持Y_h的输出，shape [num_directions, batch_size, hidden_size]。 |
| RandomNormal              | CPU计算※        | --                                                           | --                                                           |
| RandomNormalLike          | CPU计算※        | --                                                           | --                                                           |
| RandomUniform             | CPU计算         | --                                                           | --                                                           |
| RandomUniformLike         | CPU计算         | --                                                           | --                                                           |
| Range                     | CPU计算         | --                                                           |type约束支持：float,int64,int32,int16。            |
| Reciprocal                | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维  | --                                                           |
| ReduceL1                  | CPU计算         | --                                                           | --                              |
| ReduceL2                  | CPU计算         | --                                                           | --       |
| ReduceLogSum              | CPU计算         | --                                                           | --                         |
| ReduceLogSumExp           | CPU计算         | --                                                           | type约束支持float、double数据类型。               |
| ReduceMax                 | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入支持2-5维，需要指定axes属性，指定的axes数量为1，不支持沿大于1个维度进行reduce操作。<br/>3. reduce维度对应的轴的size ∈ [1, 8192]。<br/>4. 仅支持keepdims == 1。 | axes支持0, 1或者等于输入数据的维数                           |
| ReduceMean                | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入支持2-5维，需要指定axes属性，指定的axes数量为1，不支持沿大于1个维度进行reduce操作。<br/>3. 当reduce维度=2时，支持同时沿HW维度进行reduce。<br/>4. 仅支持keepdims == 1。 | axes支持0, 1或者等于输入数据的维数                           |
| ReduceMin                 | CPU计算         | --                                                           | --                                                           |
| ReduceProd                | CPU计算         | --                                                           | --                                                           |
| ReduceSum                 | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入支持2-5维，需要指定axes属性，指定的axes数量为1，不支持沿大于1个维度进行reduce操作。 | axes支持0, 1或者等于输入数据的维数  |
| ReduceSumSquare           | CPU计算         | --                                                           | axes支持0, 1或者等于输入数据的维数                           |
| Relu                      | BPU加速         | 无限制                                      | type约束：仅支持float类型。      |  
| Reshape                   | BPU加速         |1. 该算子支持int16输入输出。<br/>2. 支持1-10维输入输出。  | --                                                         |
| Resize                    | BPU加速         | 1. 输入featuremap需为四维NCHW，并且只支持在H和W维度上进行resize，onnx opset=11时支持roi输入（pytorch转换的模型需手动修改算子添加roi输入，roi只支持常量输入），roi输入只支持H和W维度，roi输入只在tf_crop_and_resize模式下起作用。<br/>2. 属性mode支持nearest和linear两种模式。<br/>3. 支持放大和缩小。<br/>4. 对于mode=nearest，放大系数factor支持2的幂数倍如2，4，8，16，32等；支持H维度和W维度的放大系数不同但需要满足H_factor `<=` W_factor。<br/>5. 对于onnx opset=11，属性coordinate_transformation_mode支持half_pixel，pytorch_half_pixel, asymmetric，align_corners和tf_crop_and_resize，当coordinate_transformation_mode=tf_crop_and_resize时，需要保证roi输入转换得到的边界坐标为整数。 | resize-10 <br/>- 输入等于2时，使用opset10。<br/>- 输入数据是4维Tensor。 <br/>resize-11  <br/>- 输入大于2时，使用opset11。<br/>- 输入数据是4维Tensor。<br/>- coordinate_transformation_mode在nearest, linear模式下支持half_pixel, asymmetric, align_corners和pytorch_half_pixel四种，在cubic模式下只支持half_pixel。<br/>- extrapolation_value属性不支持。 |
| ReverseSequence           | CPU计算         | --                                                           | --                                                           |
| RoiAlign                  | CPU计算         | --                                                           | --                                                           |
| Round                     | BPU加速        | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维。| --                                                           |
| Scan                      | CPU计算※        | --                                                           | --                                                           |
| Scatter (deprecated)      | CPU计算※        | --                                                           | --                                                           |
| ScatterElements           | CPU计算         | --                                                           | from_type支持：<br/>- data：type约束支持：float,int32,int8。<br/>- indices：type约束仅支持int32类型。<br/>- updates：type约束支持：float,int32,int8。<br/>to_type支持：type约束支持：float,int32,int8。  |
| ScatterND                 | CPU计算         | --                                                           | from_type支持：<br/>- data：type约束支持：float,int32,int8。<br/>- updates : type约束支持：float,int32,int8。<br/>to_type支持：type约束支持：float,int32,int8。   |
| Selu                      | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维。 | type约束：仅支持float类型。    |
| SequenceAt                | CPU计算※        | --                                                           | --                                                           |
| SequenceConstruct         | CPU计算※        | --                                                           | --                                                           |
| SequenceEmpty             | CPU计算※        | --                                                           | --                                                           |
| SequenceErase             | CPU计算※        | --                                                           | --                                                           |
| SequenceInsert            | CPU计算※        | --                                                           | --                                                           |
| SequenceLength            | CPU计算※        | --                                                           | --                                                           |
| Shape                     | BPU加速         | 会通过常量折叠将其优化为数值存储                               | --                                                             |
| Shrink                    | CPU计算※        | --                                                           | --                                                           |
| Sigmoid                   | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维  | type约束：仅支持float类型。   |
| Sign                      | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维 | type约束：仅支持float类型。                |
| Sin                       | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维  | type约束支持float和double类型。 |
| Sinh                      | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维 | type约束支持float类型。 |
| Size                      | BPU加速         | 会通过常量折叠将其优化为数值存储                            | --                                                           |
| Slice                     | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 无限制，支持非四维输入输出。  | 无                                                           |
| Softmax                   | BPU加速         | - 该算子支持int16输入输出。<br/>- 默认运行在CPU上，由于onnx::softmax和pytorch::softmax计算存在区别，分以下两种情况：<br/>1. 对于onnx::softmax，当该op输入为四维并且axis=3时，可以通过run_on_bpu指定该节点将其运行在BPU上。<br/>2. 对于pytorch::softmax, 当该op输入为四维并且axis=1,2,3时，可以通过run_on_bpu指定该节点将其运行在BPU上。 | type约束：仅支持float类型。  |
| Softplus                  | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维 | type约束：仅支持float类型。  |
| Softsign                  | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维 | type约束：仅支持float类型。  |
| SpaceToDepth              | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 支持mode=DCR 和 mode=CRD。<br/> 仅支持H和W方向的重新排列，并且仅支持blocksize=2的重排列。 <br/>举例：NxCxHxW -> Nx(4C)x(H/2)x(W/2) | type约束：仅支持float类型。    |
| Split                     | BPU加速         |1. 该算子支持int16输入输出。<br/>2. 原始输入的长度必须是每个被切分的tensor长度的倍数。<br/>3. 支持除N维度以外的任意维度。<br/>4. split数应可以整除。<br/>5. 支持非四维输入输出。 | type约束：仅支持float类型。    |
| SplitToSequence           | CPU计算※        | --                                                           | --                                                           |
| Sqrt                      | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维 |type约束：仅支持float类型。   |
| Squeeze                   | BPU加速         | 该op会被转换成Reshape，BPU约束详见Reshape op。     | --                                                          |            |
| StringNormalizer          | CPU计算※        | --                                                           | --                                                           |
| Sub                       | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入类型支持featurmap和常量，且最多支持一个常量输入。<br/>3. 支持所有维度的广播，支持两个输入之间的互相广播，例如NH1C和N1WC。<br/>4. 输入输出维度支持1-10维，大小为一般限制（见备注）。支持两个输入维度不同，输入大于4维时可通过合并相邻维度降维到4维（包括N），合并规则是：<br/>(1)将输出dim为1的维度去除，例如[1, 2, 3, 4] [1, 2, 1, 4]->[1, 2, 3, 4]可看为[2, 3, 4],[2, 1, 4]->[2, 3,4]。<br/>(2)相邻的非广播维度可以合并，如[2, 5, 4, 5, 3] [2, 5, 1, 5, 3], 2, 5可以合并。<br/>(3)相邻的同一Tensor的广播维度可以合并: 如[2, 5, 4, 5, 2] [1, 1, 1, 5, 2] 2，5，4可以合并。<br/>(4)广播维度不能和相邻非广播维度合并：如[2, 5, 4, 5, 2] [2, 1, 4, 1, 2]不能合并；非同一Tensor的广播维度不能合并 [2, 1, 4, 1, 2] [1, 5, 1, 5, 1]。 | - 支持相同输入shape计算。<br/>- 支持输入1是标量或者输入2是标量的计算。<br/>- 支持broadcast计算，最大维度是8。  |
| Sum                       | BPU加速         | 限制条件等同于Add                                            | type约束：仅支持float类型。   |   
| Tan                       | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维  | type约束：支持float类型。 |
| Tanh                      | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维 | type约束：仅支持float类型。   |
| TfIdfVectorizer           | CPU计算※        | --                                                           | --                                                           |
| ThresholdedRelu           | CPU计算         | --                                                           | type约束：仅支持float类型。  |
| Tile                      | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入与输出仅允许有一个维度上数值不同。| type约束：仅支持float,int64,int32,uint64,uint32类型。   |
| TopK                      | BPU加速 | 1. 该算子支持int16输入输出。<br/>2. input/indices/output维度支持1-10维。<br/>3. indices type约束支持int16/int32/int64。<br/>4. 参数sorted只支持true。 | - type约束：仅支持float类型。|
| Transpose                 | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 支持任意输入维度。 | - 支持nhwc2nchw，perm：[0, 3, 1, 2]。<br/>- 支持nchw2nhwc，perm：[0, 2, 3, 1]。<br/>- 支持指定perm维度转换，数据类型仅支持float，int8，int32。   |
| Unique                    | CPU计算※        | --                                                           | --                                                           |
| Unsqueeze                 | BPU加速         | 该op会被转换成Reshape，BPU约束详见Reshape op。       | --                                                          |
| Upsample (resize替代)     | BPU加速          | --                                                           | Upsample-(resize-10) <br/>- 输入等于2时，使用opset10。<br/>- 输入数据是4维Tensor。 <br/>Upsample-(resize-11)  <br/>- 输入大于2时，使用opset11。<br/>- 输入数据是4维Tensor。<br/>- coordinate_transformation_mode在nearest, linear模式下支持half_pixel, asymmetric, align_corners和pytorch_half_pixel四种，在cubic模式下只支持half_pixel。<br/>- extrapolation_value属性不支持。  |
| Where                     | CPU计算         | --                                                           | type约束支持float和int64类型。<br/> condition的shape为cond_shape，X的shape为x_shape，Y的shape为y_shape ，output的shape为o_shape，shape约束如下：<br/>- 仅支持cond_shape == o_shape情况下：  <br/>- x_shape == o_shape的broadcast。  <br/>- y_shape == o_shape的broadcast。<br/>- 仅支持cond_shape.NDim() == 4 && o_shape.NDim() == 4 && N维度值相同 && C维度值相同：  <br/>- 1x1（cond_shape）与HxW （o_shape）。  <br/>- Hx1（cond_shape）与HxW（o_shape）。  <br/>- 1xW（cond_shape）与HxW（o_shape）。 |
| Xor                       | CPU计算※        | --                                                           | --                                                           |
| Function                  | CPU计算※        | --                                                           | --                                                           |
| Celu                      | CPU计算※        | --                                                           | --                                                           |
| DynamicQuantizeLinear     | CPU计算※        | --                                                           | --                                                           |
| GreaterOrEqual            | BPU加速        | opset11 不支持单个GreaterOrEqual算子，支持拆分后的算子Less+Not运行在BPU上，限制条件与Less相同。| - 支持相同输入shape计算。<br/>- 支持输入1是标量或者输入2是标量的计算。<br/>- 支持broadcast计算，最大维度是5。 |
| MeanVarianceNormalization | CPU计算※        | --                                                           | --                                                           |
| GridSample（PyTorch）     | BPU加速         | 1. 输入维度仅支持四维，第一个输入需满足N ∈ [1, 4096]； C ∈ [1, 65536]； H,W ∈ [1, 1024] 且 H * W `<=` 512 * 512。<br/>2. mode只支持'bilinear'、'nearest'。<br/>3. padding_mode只支持'zeros'、'border'。<br/>4. 该算子为opset16的onnx算子，为在opset11支持，工具链以自定义算子的方式提供导出，导出包含该算子的onnx模型请使用horizon_nn.torch.export_onnx接口替换torch.onnx.export，接口传参相同，示例代码如下：<br/>from horizon_nn.torch import export_onnx<br/>    ...    <br/>    export_onnx(...)|  |


## RDK Ultra支持的Caffe算子列表

| **caffe算子名称**       | **CPU计算/BPU加速** | **RDK Ultra BPU支持约束** | **CPU支持约束** |
| ------------------- | --------------- | --------------- | ----------- |
| Convolution   | BPU加速 | `Kernel宽高限制：<=32`。 <br/> 输入输出Channel取值范围`(one group) <= 8192`，如果Conv是量化子图的最后一个算子，取值范围`<= 65536`。 <br/> stride无限制，但对于Conv后接Add(resnet shortcut-connecting) Stride取值范围为：{1, 2}。 <br/> Dilation取值限制：`<=16`。<br/>当dilation != 1时，stride只支持为1。 <br/> 不支持配置axis，默认为1。 | 仅支持4维Conv计算。<br/>auto_pad 属性不支持。<br/>type约束支持：float,int32,int8。<br/>pads属性约束：[Hstart, Wstart, Hend, Wend]（pads长度等于4）并且Hstart==Hend，Wstart==Wend。                                |
| Deconvolution | BPU加速 | kernel >= stride。 <br/>输入输出`featuremap大小 <= 2048`。 <br/> `pad <= kernel / stride`。<br/> `out_pad < 2` 。<br/>`stride >= 1 && stride <=14` 但不支持stride_h和stride_w同时等于1。 <br/>不支持配置axis属性。 | shape约束：仅支持4维Tensor计算。 <br/>type约束：仅支持float类型。 <br/>attribute约束：<br/>- 仅支持dilations、group、output_padding、 pads 、strides 属性。<br/>- pads属性约束：[hstart, wstart, hend, wend]必须满足(hstart==hend and wstart==wend)。 |
| MaxUnpool                                                    | CPU计算 | --- | from_type支持：  <br/>- X：type约束：仅支持float类型。 <br/>- I：Tensor（int64）。 <br/>to_type支持：type约束：仅支持float类型。                                       |
| Pooling      | BPU加速 | 共有四种Pooling算子即MaxPooling，AveragePooling，GlobalMaxPooling，GlobalAveragePooling。<br/>对四种Pooling的约束分别为：  <br/>MaxPooling：<br/> 该算子支持int16输入输出。<br/>`kernel <= 256；stride <= 256；padding <= 256`。 <br/> MaxPooling不支持dilation。<br/>AveragePooling：<br/> `kernel <= 256; stride <= 256；padding <= 256`。 <br/>GlobalAveragePooling： <br/>无限制。<br/>GlobalMaxPooling： <br/>H, W ∈ [1, 256]。 | 无  |
| SPP          | CPU计算 | 不支持                                                       | 支持pyramid_height，2^n 次pooling, `n<7`;<br/>pooling kernel 小于等于 255； <br/>支持pool，配置可选值为`{0，1}` |
| InnerProduct | BPU加速 | InnerProduct将被转化为Conv实现，边界约束参考Conv。 <br/>假不支持配置axis属性。 | 无                                                           |
| LRN  | CPU计算 | 不支持 | local_size 支持。<br/>alpha支持。<br/>beta 支持。<br/>norm_region 支持，配置可选值`{ACROSS_CHANNELS, WITHIN_CHANNEL }`。<br/>k 支持。 |
| MVN  | CPU计算 | 不支持 | normalize_variance支持，配置可选值为{0, 1}。<br/>across_channels支持，配置可选值为{0, 1}。<br/>仅支持Float32类型的计算。 |
| BatchNorm                                                    | BPU加速 | 无限制 | 无                                         |
| ELU                                                          | BPU加速 | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维，最高维 ∈ [1, 4096]，其它维 ∈ [1, 65536]。 | 无                                         |
| BNLL                                                         | CPU计算 | 不支持 | 无                                         |
| PReLU                                                        | CPU计算 | 不支持 | - type约束支持：仅支持float类型。<br/>- from_type：X和slope。<br/>- to_type：Y。<br/>- X的shape为data_shape，slope的为slope_shape ，shape约束如下：<br/>  - data_shape == slope_shape 。<br/>  - slope_shape.ProdSize() == 1 。<br/>  - X和slope仅支持NCHW排布的4维度计算，并且N、C维度值相等。 <br/>    - HxW 与1x1（ slope_shape ）。 <br/>    - HxW与Hx1（ slope_shape ）。 <br/>    - HxW与1xW（ slope_shape ） 。<br/>  - X是4维度 && slope是3维度 && data_shape[1] == slope_shape [0] && slope_shape [1] == 1 && slope_shape [2] == 1。                                     |
| ReLU/LeakyRelu                                               | BPU加速 | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维，最高维 ∈ [1, 4096]，其它维 ∈ [1, 65536]。 | 无                                         |
| Sigmoid                                                      | BPU加速 | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维，最高维 ∈ [1, 4096]，其它维 ∈ [1, 65536]。 | 无                                         |
| TanH                                                         | BPU加速 | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维，最高维 ∈ [1, 4096]，其它维 ∈ [1, 65536]。 | 无                                         |
| Eltwise                                                      | BPU加速 | 目前支持的operation包括Add、Sub、Mul。<br/>1. 该算子支持int16输入输出。<br/>2. 输入类型支持featurmap和常量，且最多支持一个常量输入；<br/>3. 支持除第一维外的广播，支持两个输入之间的互相广播，例如NH1C和N1WC；<br/>4. 输入输出维度支持2维、3维、4维和5维，大小为一般限制（见备注）。支持两个输入维度不同，输入为5维时需要满足以下限制：<br/>(1)首先可以通过合并相邻维度降维到4维，例如NHWD1和N1WDC可以合并W维和D维来降维；<br/>(2)其次广播的维度不能和相邻维度合并，例如NHWD1和N11DC因为H维、W维和C维都是广播的维度，无法通过合并相邻维度降维，所以无法支持。 | 无                                                           |
| Bias                                                         | BPU加速 | 参考Eltwise等于Add的情况                                     | 无                                                           |
| Scale                                                        | BPU加速 | 参考Eltwise等于Mul的情况                                     | 无                                                           |
| AbsVal                                                       | BPU加速 | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维，最高维 ∈ [1, 4096]，其它维 ∈ [1, 65536]。| 无  |
| Exp                                                          | BPU加速 | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维，最高维 ∈ [1, 4096]，其它维 ∈ [1, 65536]。| 无  |
| Log                                                          | BPU加速 | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维，最高维 ∈ [1, 4096]，其它维 ∈ [1, 65536]。| 无  |
| Power                                                        | BPU加速 | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维，最高维 ∈ [1, 4096]，其它维 ∈ [1, 65536]。<br/>3. 第二个输入只支持标量。| 无  |
| Threshold                                                    | CPU计算 | 不支持                                                       | 无                                                           |
| Reduction                                                    | CPU计算 | 不支持  | operation 支持 SUM、ASUM、 SUMSQ、MEAN、Max、LogSum、Min、Prod； <br/>axis 支持； <br/> 仅支持Float32类型的计算。 |
| Softmax                                                      | BPU加速 | 1. 该算子支持int16输入输出。<br/>2. 默认运行在CPU上，当该op输入为四维并且axis=1,2,3时，可以通过run_on_bpu指定该节点将其运行在BPU上。 | 无 |
| ArgMax                                                       | BPU加速 | 1. 仅支持 axis=1，`c<=64`。<br/>2. 不支持配置top_k != 1。<br/>3. 该算子支持int16输入输出。| 无  |
| Concat                                                       | BPU加速 | 1. 该算子支持int16输入输出。<br/>2. 不支持N维度concat。 | 无                                                           |
| Split                                                        | BPU加速 | 1. 该算子支持int16输入输出。<br/>2. 原始输入的长度必须是每个被切分的tensor长度的倍数。<br/>3. 支持除N维度以外的任意维度。<br/>4. split数应可以整除。<br/>5. 支持非四维输入输出。| 无 |
| Slice                                                        | BPU加速 | 1. 该算子支持int16输入输出。<br/>2. 无限制，支持非四维输入输出。 | 无                                                           |
| Reshape                                                      | BPU加速 | 1. 该算子支持int16输入输出。<br/>2. 支持1-10维输入输出。| shape 支持[1,4]个 shape_dim 配置 ；<br/> axis 支持[-4,3]范围内可配，不支 持 N 维度，默认值 0，遵循 caffe 规则 ；<br/> num_axes 支持[-1,3]范围内可配，默认 值-1 表示对 axis 起始的所有 轴进行变换 |
| Flatten                                                      | CPU计算 | 不支持（一些场景下可以融合）                                 | axis 取值范围[-4,3]，默认值 为 1，-4 与 0 含义相同。 <br/>只支持End_axis == -1。 |
| Crop                                                         | CPU计算 | 不支持                                                       | 无                                                           |
| Dropout                                                      | BPU加速 | 无限制                                                       | 无                                                           |
| LSTM                                                         | BPU加速 | 仅支持batch=1                                                | --                                                           |
| Normalize                                                    | CPU计算 | 不支持                                                       | type约束：仅支持float类型。                                 |
| PassThrough                                                  | BPU加速 | 支持mode=DCR 和 mode=CRD。<br/>仅支持H和W方向的重新排列，并且仅支持blocksize=2的重排列。<br/>举例：NxCxHxW -> Nx(4C)x(H/2)x(W/2)。  | type约束：仅支持float类型。  |
| CReLU                                                         | CPU计算 | 不支持                                               | type约束：仅支持float类型。                                 |
| RReLU                                                         | CPU计算 | 不支持                                               | 无                                                          |
| Permute                                                       | BPU加速 | 1. 支持任意输入维度。<br/>2. 除batch维度（第一维）以外，支持任意其它维度的转换。 | - 支持nhwc2nchw，perm：[0, 3, 1, 2]。 <br/> - 支持nchw2nhwc，perm：[0, 2, 3, 1]。<br/> - 支持指定perm维度转换，数据类型仅支持float，int8，int32。 |
| MatMul                                                         | BPU加速 | C = MatMul(A，B)，对输入A和输入B有以下维度限制：<br/>- A和B均支持非四维输入但需满足约束：<br/>  - A和B的维度必须相同。<br/>  - A和B的最低两个维度M, K ∈ [1, 8192]，其他更高维度∈[1, 4096]。    <br/>  注：HDMK vs HDKN，MK/KN即为最低两个维度。<br/>- 支持的broadcast需满足以下条件：<br/>  - A 跟B两个输入，除开最低两维的其他维度全是1或者全是不需要广播的值。<br/>    - 此场景支持的例子：HDMK vs H1KN<br/>    - 此场景不支持反例：H1MK vs 1DKN<br/>  - A除了最低两个维度，其他维度不能即有需要广播的值也有不需要广播的值。<br/>    - 此场景支持的例子：11MK vs HDKN<br/>    - 此场景不支持反例：H1MK vs HDKN<br/>  - B除了最低两个维度，如果其他维度即有需要广播的值也有不需要广播的值，那么不需要广播的值只能在连续的高维度上。<br/>    - 此场景支持的例子：BHDMK vs B11KN<br/>    - 此场景不支持反例：BHDMK vs B1DKN  <br/>  注：需要广播的值和不需要广播的值：<br/>   <br/>- 如果A和B在对应维度轴上的两个值，一个为1，另一个为非1，那么1就是需要广播的值，非1就是不需要广播的值；<br/>    - 如果A和B在对应维度轴上的两个值相等，那么这两个值都是不需要广播的值（如HDMK vs H1KN，1是需要广播的值，H是不需要广播的值）。 | type约束：仅支持float类型。                                                       |
| Upsample                                                     | BPU加速 | 输入featuremap需为四维NCHW，并且只支持在H和W维度上进行resize； <br/> 放大系数factor不能同时小于2。 | 无  |
| ROIPooling                                                   | CPU计算 | 不支持                                                       | 无                                                           |
| PSROIPooling                                                 | CPU计算 | 不支持                                                       | 无                                                           |




## RDK Ultra支持的ONNX算子列表

| **ONNX算子名称** | **CPU计算/BPU加速** | **RDK Ultra BPU支持约束** | **CPU支持约束** |
| ------------ | --------------- | --------------- | ----------- |
| Abs                       | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维，最高维 ∈ [1, 4096]，其它维 ∈ [1, 65536]。| type约束：仅支持float类型。   |  
| Acos                      | CPU计算         | --          | type约束：仅支持float类型。 |    
| Acosh                     | CPU计算         | --     | type约束：仅支持float类型。      |    
| Add                       | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入类型支持featurmap和常量，且最多支持一个常量输入。<br/>3. 支持除第一维外的广播，支持两个输入之间的互相广播，例如NH1C和N1WC。<br/>4. 输入输出维度支持2维、3维、4维和5维，大小为一般限制（见备注）。支持两个输入维度不同，输入为5维时需要满足以下限制：<br/>(1)首先可以通过合并相邻维度降维到4维，例如NHWD1和N1WDC可以合并W维和D维来降维；<br/>(2)其次广播的维度不能和相邻维度合并，例如NHWD1和N11DC因为H维、W维和C维都是广播的维度，无法通过合并相邻维度降维，所以无法支持。<br/>5. 作为resnet中的short-cut子结构的Add，会被融合到上一个conv中加速计算。 | - 支持相同输入shape计算。<br/>- 支持输入1是标量或者输入2是标量的计算。<br/>- 支持broadcast计算，最大维度是5。 |          
| And                       | CPU计算         | --  | - 支持相同输入shape计算。<br/>- 支持输入1是标量或者输入2是标量的计算。<br/>- 支持broadcast计算，最大维度是5。 |                      
| ArgMax                    | BPU加速         | 1. 输入维度为四维输入NCHW。 <br/>2. 仅支持沿C维度进行argmax，即axis=1。<br/> 3. `C <= 64` 。 <br/>4. 该算子支持int16输入输出。| type约束：仅支持float类型。|
| ArgMin                    | BPU加速         | 1. 输入维度为四维输入NCHW。 <br/>2. 仅支持沿C维度进行argmax，即axis=1。<br/> 3. `C <= 64` 。 <br/>4. 该算子支持int16输入输出。| type约束：仅支持float类型。| 
| Asin                      | CPU计算         | --                                                           | type约束：仅支持float类型。     |                         
| Asinh                     | CPU计算         | --                                                           | type约束：仅支持float类型。    |                        
| Atan                      | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维，最高维 ∈ [1, 4096]，其它维 ∈ [1, 65536]。 | type约束：仅支持float类型。     |
| Atanh                     | CPU计算         | --                                                           | type约束：仅支持float类型。     |                        
| AveragePool               | BPU加速         | `kernel <= 256`。<br/>`stride <= 256`。<br/>`padding <= 256`。 | auto_pad 属性不支持。<br/>仅支持四维Tensor计算。     |                      
| BatchNormalization        | BPU加速         | 无限制。  | type约束：仅支持float类型。 <br/>支持第1个维度是channel的数据排布方式计算。   |         
| BitShift                  | CPU计算※        | --                                                           | --                                                           |     
| Cast                      | CPU计算         | --                                                           | from_type支持double, float, bool, int64, uint32, int32, uint16, int16, uint8, int8。<br/>to_type支持double, float, bool, int64, uint32, int32, uint16, int16, uint8, int8。|    
| Ceil                      | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维，最高维 ∈ [1, 4096]，其它维 ∈ [1, 65536]。 | type约束：仅支持float类型。     | 
| Clip                      | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维，最高维 ∈ [1, 4096]，其它维 ∈ [1, 65536]。 | opset6: <br/>min, max作为属性值，dtype仅支持float类型;<br/>opset11: <br/>min, max作为输入，仅有两个输入时，第二个为min；dtype支持float, double类型。 | 
| Compress                  | CPU计算※        | --                                                           | --                                                           |  
| Concat                    | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 不支持N维度concat。 | --                                                          |  
| ConcatFromSequence        | CPU计算※        | --                                                           | --                                                           |   
| Constant                  | BPU加速         | 会通过常量折叠将其优化为数值存储                                | 目前不支持sparse_tensor属性。 |    
| ConstantOfShape           | BPU加速         | 会通过常量折叠将其优化为数值存储                                 | type约束支持：float,int32,int8。   |    
| Conv                      | BPU加速         | 支持四维输入（conv2d）和五维输入（conv3d）。<br/>四维输入（conv2d）：<br/>Kernel shape范围：`N,C ∈ [1, 8192]`; `H,W ∈ [1, 31]`。`C*H*W < = 65535`。<br/>输入输出Channel取值范围 `(one group) <= 8192`，如果Conv是量化子图的最后一个算子，取值范围 `<= 65536`。<br/>stride取值范围：`H,W ∈ [1, 256]`，但对于Conv后接Add(resnet shortcut-connecting) Stride取值范围为：`{1, 2}`，对dilated>1的conv，stride只支持=1。<br/>Dilation取值范围：`H,W∈ [1, 16]`，H或W大于1时，只支持输出int8，且输入Tensor的H必须能被dilation的H整除，输入Tensor的W必须能被dilation的W整除。<br/>padding取值范围：`H,W ∈ [0, 256]`。<br/>五维输入（conv3d）：<br/>输入大小NCDHW：`N ∈ [1, 128]`; `H,W,D,C ∈ [1, 65536]`。<br/>kernel大小NCDHW：`N,C ∈ [1, 65536]; H,W ∈ [1, 31], D ∈ [1, 8191]`。<br/>padding大小DHW：`H,W ∈ [0, 256]`, `D ∈ [0, kernel_d/2]`。<br/>stride取值范围：H, W同为1或H, W同为2。<br/>group，dilation暂不支持。<br/> Size: 1G bytes；当 `D * C > 4096`时, `H * alignCeil(W, 256) * D * C < 1G`。<br/>weight的D * 输入的`C <= 8192`。 | 仅支持4维Conv计算。<br/>auto_pad 属性不支持。<br/>type约束支持：float,int32,int8。<br/>pads属性约束：[Hstart, Wstart, Hend, Wend]（pads长度等于4）并且Hstart==Hend，Wstart==Wend。|  
| ConvInteger               | CPU计算※        | --                                                           | --                                                           | 
| ConvTranspose             | BPU加速         | 输入输出featuremap大小限制：<br/>N ∈ [1, 128]。<br/>H,W ∈ [1, 65536]。 <br/>C ∈ [1, 2048] 。<br/>Size: 1G bytes。<br/>weight大小限制：<br/>N,C ∈ [1, 2048]。 <br/>H,W ∈ [1, 14]且HW不同时为1。<br/>Size: [1, 65535]。<br/>padding取值范围：<br/>stride为奇数时，`H,W ∈ [0, kernel / stride)`。<br/>stride为偶数，H,W ∈ [0, kernel / stride]。<br/>out_pad取值范围：H,W ∈ {0,1}。<br/>stride >= 1 && `stride <=14 `但不支持stride_h和stride_w同时等于1。<br/>`Dilation ∈ {(1, 1)}`。 | shape约束：仅支持4维Tensor计算。<br/>type约束：仅支持float类型。<br/>attribute约束：<br/>- 仅支持dilations、group、output_padding、 pads 、strides 属性。<br/>- pads属性约束：[hstart, wstart, hend, wend]必须满足(hstart==hend and wstart==wend)。|  
| Cos                       | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维，最高维 ∈ [1, 4096]，其它维 ∈ [1, 65536]。 | type约束：仅支持float类型。            |   
| Cosh                      | CPU计算         | --                                                           | type约束：仅支持float类型。      |    
| CumSum                    | CPU计算         | --                                                           | axis：type约束仅支持int32类型。 |   
| DepthToSpace              | BPU加速         | 支持mode=DCR 和 mode=CRD。<br/> 仅支持H和W方向的重新排列，并且仅支持blocksize=2的重排列。 <br/>举例：NxCxHxW -> Nx(C/4)x(2H)x(2W) 输出的channel必须是4的倍数。| from_type支持：<br/>- type约束仅支持float类型。<br/>- 仅支持4维度Tensor计算。<br/>to_type支持：<br/>- type约束仅支持float类型。<br/>- 仅支持4维度Tensor计算。  | 
| DequantizeLinear          | CPU计算        | --                                                           | --                                                           |   
| Det                       | CPU计算※        | --                                                           | --                                                           |   
| Div                       | BPU加速         | 1. 只支持两个输入均为featuremap（不支持输入来自于常量）； <br/>2. 对input shape的约束请参考Mul算子    | - 支持相同输入shape计算。<br/>- 支持输入1是标量或者输入2是标量的计算。<br/>- 支持broadcast计算，最大维度是5。 |  
| Dropout                   | BPU加速         | 该算子推理阶段不参加计算， 会被移除优化                                   | --                                                           |   
| Einsum                    | CPU计算※        | --                                                           | --                                                           |  
| Elu                       | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维，最高维 ∈ [1, 4096]，其它维 ∈ [1, 65536]。 | type约束：仅支持float类型。   |  
| Equal                     | BPU加速         | 1. 该算子支持int16输入。<br/>2. 输入输出维度支持2-5维。<br/>3. 支持所有维度的广播，支持fin0或fin1其中一个输入的广播，不能支持互相广播，5维广播时有以下限制：<br/>（1）需要能够合并相邻维度降到4维（包括维度N），例如NHWDC和NH1D1可以合并NH维来降维。<br/>（2）广播的维度不能和相邻维度合并，例如NHWDC和N1W1C因为无法合并相邻维度，所以无法支持。<br/>4. 默认运行在CPU上，可以通过run_on_bpu指定该节点将其运行在BPU上。 | - 支持相同输入shape计算。<br/>- 支持输入1是标量或者输入2是标量的计算。<br/>- 支持broadcast计算，最大维度是5。   |    
| Erf                       | CPU计算        | --                                                           | type约束：支持float、double数据类型。                               |     
| Exp                       | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维，最高维 ∈ [1, 4096]，其它维 ∈ [1, 65536]。 | type约束：仅支持float类型。        | 
| Expand                    | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维，输入与输出仅支持有一个维度上的数值不同。<br/>3. 输入与输出仅允许有一个维度上数值不同。 | --    | 
| EyeLike                   | CPU计算        | --                                                           | --                                                           |  
| Flatten                   | BPU加速         | 限制条件等同于Reshape。 | --               | 
| Floor                     | BPU加速         |  1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维，最高维 ∈ [1, 4096]，其它维 ∈ [1, 65536]。 | type约束：仅支持float类型。   | 
| GRU                       | CPU计算         | --                                                           | - direction属性仅支持forward类型。<br/>- type约束：仅支持float类型。  | 
| Gather                    | BPU加速         | 1. input/output/indices 的rank都要小于等于4。<br/>2. indices支持：<br/>    - indices是feature（其他op输出）时，type约束仅支持int32类型。<br/>    - indices是weight（模型保存的常量）时，type约束支持int32和int64类型。 | from_type支持：<br/>- input：type约束支持：<br/>float,int64,int32,int8,uint64,uint32,uint8。<br/>- indices：type约束支持int32, int64。 <br/>to_type支持：type约束支持：<br/>float,int64,int32,int8,uint64,uint32,uint8。 |
| GatherElements            | BPU加速        | 1. 该算子支持int16输入输出。<br/>2. input/indices/output维度支持1-10维。<br/>3. indices type约束支持int16/int32/int64。 | --     |   
| GatherND                  | CPU计算         | --                          | from_type支持：<br/>- input：type约束支持float,int32,int8。<br/>- indices：tensor(int64)。<br/>to_type支持：type约束支持float,int32,int8。|    
| Gemm                      | BPU加速         | Gemm将被转化为Conv实现，边界约束参考Conv。 | type约束：仅支持float类型。    |  
| GlobalAveragePool         | BPU加速         | 无限制。           | - type约束：仅支持float类型。<br/>- 仅支持四维Tensor。| 
| GlobalLpPool              | CPU计算        | --                                                           | - type约束：支持float和double类型。<br/> - 仅支持四维Tensor计算。 |  
| GlobalMaxPool             | BPU加速         | H, W ∈ [1, 256]。 | - type约束仅支持float类型。<br/>- 仅支持四维Tensor。 |   
| Greater                   | BPU加速         | 1. 该算子支持int16输入。<br/>2. 输入输出维度支持2-5维。<br/>3. 支持所有维度的广播，支持fin0或fin1其中一个输入的广播，不能支持互相广播，5维广播时有以下限制：<br/>（1）需要能够合并相邻维度降到4维（包括维度N），例如NHWDC和NH1D1可以合并NH维来降维。<br/>（2）广播的维度不能和相邻维度合并，例如NHWDC和N1W1C因为无法合并相邻维度，所以无法支持。<br/>4. 默认运行在CPU上，可以通过run_on_bpu指定该节点将其运行在BPU上。 | - 支持相同输入shape计算。<br/>- 支持输入1是标量或者输入2是标量的计算。<br/>- 支持broadcast计算，最大维度是5。 | 
| HardSigmoid               | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维，最高维 ∈ [1, 4096]，其它维 ∈ [1, 65536]。 | type约束仅支持float类型。  |   
| Hardmax                   | CPU计算※        | --                                                           | --                                                           |  
| Identity                  | CPU计算         | --                                                           | --                                                         |    
| If                        | CPU计算※        | --                                                           | --                                                           |  
| InstanceNormalization     | CPU计算         | --                                                           |- type约束仅支持float类型。<br/>- 支持第1个维度是channel的数据排布方式计算。   |  
| IsInf                     | CPU计算※        | --                                                           | --                                                           |   
| IsNaN                     | CPU计算※        | --                                                           | --                                                           |   
| LRN                       | CPU计算         | --                                                           | - type约束仅支持float类型。<br/>- 仅支持四维Tensor。  | 
| LSTM                      | BPU加速         | 仅支持batch_size=1，如果需要配置多batch，需要在导出onnx时保证LSTM的batch为1并在yaml中配置参数input_batch=1。 | - type约束仅支持float类型。<br/>- 属性约束：direction属性仅支持forward。<br/>- 输入约束：<br/>   - 支持X、W、R输入配置；<br/>   - 支持X、W、R、B输入配置（sequence_lens为空或默认值）；<br/>   -  支持X、W、R、B、sequence_lens、initial_h、initial_c、P输入配置（sequence_lens为空或者默认值）。 |   
| LeakyRelu                 | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维，最高维 ∈ [1, 4096]，其它维 ∈ [1, 65536]。| type约束：仅支持float类型。  | 
| Less                      | BPU加速        | 1. 该算子支持int16输入。<br/>2. 输入输出维度支持2-5维。<br/>3. 默认运行在CPU上，可以通过run_on_bpu指定该节点将其运行在BPU上。 | - 支持相同输入shape计算。<br/>- 支持输入1是标量或者输入2是标量的计算。<br/>- 支持broadcast计算，最大维度是5。| 
| LessOrEqual               | BPU加速         |opset11 不支持单个LessOrEqual算子，支持拆分后的算子Greater+Not运行在BPU上，限制条件与Greater相同。|- 支持相同输入shape计算。<br/>- 支持输入1是标量或者输入2是标量的计算。<br/>- 支持broadcast计算，最大维度是5。  |  
| Log                       | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维，最高维 ∈ [1, 4096]，其它维 ∈ [1, 65536]。 | type约束：仅支持float类型。    | 
| LogSoftmax                | CPU计算         | --                                                           | type约束：仅支持float类型。    | 
| Loop                      | CPU计算※        | --                                                           | --                                                           |  
| LpNormalization           | CPU计算        | --                                                           | - p范数仅支持1或者2。<br/>- type约束支持double类型和float类型。|  
| LpPool                    | CPU计算        | --                                                           | - auto_pad属性不支持。<br/>- type约束支持double类型和float类型。<br/>- 仅支持4维计算。 |  
| MatMulInteger             | CPU计算※        | --                                                           | --                                                           |    
| MatMul                    | BPU加速         | C = MatMul(A，B)，对输入A和输入B有以下维度限制：<br/>- A和B均支持非四维输入但需满足约束：<br/>  - A和B的维度必须相同。<br/>  - A和B的最低两个维度M, K∈[1, 8192]，其他更高维度∈[1, 4096]。    <br/>  注：HDMK vs HDKN，MK/KN即为最低两个维度。<br/>- 支持的broadcast需满足以下条件：<br/>  - A跟B两个输入，除开最低两维的其他维度全是1或者全是不需要广播的值。<br/>    - 此场景支持的例子：HDMK vs H1KN<br/>    - 此场景不支持反例：H1MK vs 1DKN<br/>  - A除了最低两个维度，其他维度不能即有需要广播的值也有不需要广播的值。<br/>    - 此场景支持的例子：11MK vs HDKN<br/>    - 此场景不支持反例：H1MK vs HDKN<br/>  - B除了最低两个维度，如果其他维度即有需要广播的值也有不需要广播的值，那么不需要广播的值只能在连续的高维度上。<br/>    - 此场景支持的例子：BHDMK vs B11KN<br/>    - 此场景不支持反例：BHDMK vs B1DKN  <br/>  注：需要广播的值和不需要广播的值：<br/>    - 如果A和B在对应维度轴上的两个值，一个为1，另一个为非1，那么1就是需要广播的值，非1就是不需要广播的值；<br/>    - 如果A和B在对应维度轴上的两个值相等，那么这两个值都是不需要广播的值（如HDMK vs H1KN，1是需要广播的值，H是不需要广播的值） | type约束：仅支持float类型。  |  
| Max                       | BPU加速         | 1.该算子支持int16输入输出。<br/>2.输入输出维度支持2-5维。<br/>3.支持所有维度的广播，支持fin0或fin1其中一个输入的广播，不能支持互相广播，5维广播时有以下限制：<br/>（1）需要能够合并相邻维度降到4维（包括维度N），例如NHWDC和NH1D1可以合并NH维来降维。<br/>（2）广播的维度不能和相邻维度合并，例如NHWDC和N1W1C因为无法合并相邻维度，所以无法支持。 | - 支持1-∞个输入。<br/>- 支持相同输入shape计算。<br/>- 支持输入1是标量或者输入2是标量的计算。<br/>- 支持broadcast计算，最大维度是5。 |  
| MaxPool                   | BPU加速         | 该算子支持int16输入输出。<br/>`kernel <= 256`。<br/>`stride <= 256`。<br/>`padding <= 256`。<br/>MaxPool不支持dilation。 | 1. dilation只支持1x1。<br/>2. 只支持数据行优先存储。<br/>3. auto_pad属性不支持。<br/>4. storage_order属性不支持。<br/>5.仅支持四维Tensor计算。|   
| MaxRoiPool                | CPU计算         | --                                                           | 无                                                           |   
| Mean                      | CPU计算※        | --                                                           | --                                                           | 
| Min                       | BPU加速        | 1.该算子支持int16输入输出。<br/>2.输入输出维度支持2-5维。<br/>3.支持所有维度的广播，支持fin0或fin1其中一个输入的广播，不能支持互相广播，5维广播时有以下限制：<br/>（1）需要能够合并相邻维度降到4维（包括维度N），例如NHWDC和NH1D1可以合并NH维来降维。<br/>（2）广播的维度不能和相邻维度合并，例如NHWDC和N1W1C因为无法合并相邻维度，所以无法支持。<br/>4. 默认运行在CPU上，可以通过run_on_bpu指定该节点将其运行在BPU上。| - 支持1-∞个输入。<br/>- 支持相同输入shape计算。<br/>- 支持输入1是标量或者输入2是标量的计算。<br/>- 支持broadcast计算，最大维度是5。 |  
| Mod                       | CPU计算※        | --                                                           | --                                                           | 
| Mul                       | BPU加速         | 1. 该算子支持int16输入输出。<br/>2.输入类型支持featurmap和常量，且最多支持一个常量输入。<br/>3.支持除第一维外的广播，支持两个输入之间的互相广播，例如NH1C和N1WC。<br/>4.输入输出维度支持2维、3维、4维和5维，大小为一般限制（见备注）。支持两个输入维度不同，输入为5维时需要满足以下限制：<br/>(1)首先可以通过合并相邻维度降维到4维，例如NHWD1和N1WDC可以合并W维和D维来降维；<br/>(2)其次广播的维度不能和相邻维度合并，例如NHWD1和N11DC因为H维、W维和C维都是广播的维度，无法通过合并相邻维度降维，所以无法支持。 | - 支持相同输入shape计算。<br/>- 支持输入1是标量或者输入2是标量的计算。<br/>- 支持broadcast计算，最大维度是5。 |    
| Multinomial               | CPU计算※        | --                                                           | --                                                           |   
| Neg                       | CPU计算        | --                                                           | --                                                        |  
| Not                       | CPU计算        | --                                                           | --                                                           |  
| OneHot                    | CPU计算        | --                                                           | --                                                        |   
| Or                        | CPU计算         | --                                     | - 支持相同输入shape计算。<br/>- 支持输入1是标量或者输入2是标量的计算。 <br/>- 支持broadcast计算，最大维度是5。|   
| PRelu                     | CPU计算         | --                                                        | - type约束支持：仅支持float类型。<br/>- from_type：X和slope。<br/>- to_type：Y。<br/>- X的shape为data_shape，slope的为slope_shape ，shape约束如下：  <br/>- data_shape == slope_shape。   <br/>- slope_shape.ProdSize() == 1。   <br/>- X和slope仅支持NCHW排布的4维度计算，并且N、C维度值相等。     <br/>- HxW 与1x1（ slope_shape ）。     <br/>- HxW与Hx1（ slope_shape ）。     <br/>- HxW与1xW（ slope_shape ）。 <br/>- X是4维度 && slope是3维度 && data_shape[1] == slope_shape [0] && slope_shape [1] == 1 && slope_shape [2] == 1。  |                         |
| Pad                       | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 支持mode = Constant。<br/>3. 支持所有维度的Pad。 | Pad-10：<br/>- type约束仅支持float类型。<br/>- 仅支持NCHW排布的4维Tensor。<br/>- 属性pads的约束如下：  <br/>- len(pads) == 8 && pads[i] >=0 && pads[0] == 0 && pads[1] == 0 && pads[4] == 0 && pads[5] == 0。 <br/>Pad-11：<br/>- from_type支持：  <br/>- data：type约束仅支持float类型。  <br/>- pads : tensor(int64)。  <br/>- constant_value (optional)：type约束仅支持float类型。<br/>- to_type支持：type约束仅支持float类型。<br/>- 仅支持4维Tensor。<br/>- 仅支持2/3维度填充。 |   
| Pow                       | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维，最高维 ∈ [1, 4096]，其它维 ∈ [1, 65536]。<br/>3. 第二个输入只支持标量。  | - type约束支持：double, float，int64, int32。<br/>- 支持相同输入shape的计算。<br/>- 支持输入1是标量或者输入2是标量的计算。<br/>- 支持broadcast计算，最大维度是5。<br/>- 仅支持X和Y相同type。 |  
| QLinearConv               | CPU计算※        | --                                                           | --                                                           |   
| QLinearMatMul             | CPU计算※        | --                                                           | --                                                           |
| QuantizeLinear            | CPU计算          | --                                                           | --                                                           |
| RNN                       | CPU计算         | --                                                           | - type约束：仅支持float类型。<br/>- 属性约束：direction属性仅支持forward。<br/>- 输入约束：仅支持X、W、R输入，不支持可选输入B、sequence_lens、initial_h设置。 <br/>- 输出约束：仅支持Y_h的输出，shape [num_directions, batch_size, hidden_size]。 |
| RandomNormal              | CPU计算※        | --                                                           | --                                                           |
| RandomNormalLike          | CPU计算※        | --                                                           | --                                                           |
| RandomUniform             | CPU计算         | --                                                           | --                                                           |
| RandomUniformLike         | CPU计算         | --                                                           | --                                                           |
| Range                     | CPU计算         | --                                                           |type约束支持：float,int64,int32,int16。            |
| Reciprocal                | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维，最高维 ∈ [1, 4096]，其它维 ∈ [1, 65536]。 | --                                                           |
| ReduceL1                  | CPU计算         | --                                                           | --                              |
| ReduceL2                  | CPU计算         | --                                                           | --       |
| ReduceLogSum              | CPU计算         | --                                                           | --                         |
| ReduceLogSumExp           | CPU计算         | --                                                           | type约束支持float、double数据类型。               |
| ReduceMax                 | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入支持2-5维，需要指定axes属性，指定的axes数量为1，不支持沿大于1个维度进行reduce操作。<br/>3. reduce维度对应的轴的size ∈ [1, 8192]。<br/>4. 仅支持keepdims == 1。 | axes支持0, 1或者等于输入数据的维数                           |
| ReduceMean                | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入支持2-5维，需要指定axes属性，指定的axes数量为1，不支持沿大于1个维度进行reduce操作。<br/>3. 当reduce维度=2时，支持同时沿HW维度进行reduce。<br/>4. 仅支持keepdims == 1。 | axes支持0, 1或者等于输入数据的维数                           |
| ReduceMin                 | CPU计算         | --                                                           | --                                                           |
| ReduceProd                | CPU计算         | --                                                           | --                                                           |
| ReduceSum                 | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入支持2-5维，需要指定axes属性，指定的axes数量为1，不支持沿大于1个维度进行reduce操作。 | axes支持0, 1或者等于输入数据的维数  |
| ReduceSumSquare           | CPU计算         | --                                                           | axes支持0, 1或者等于输入数据的维数                           |
| Relu                      | BPU加速         | 无限制                                      | type约束：仅支持float类型。      |  
| Reshape                   | BPU加速         |1. 该算子支持int16输入输出。<br/>2. 支持1-10维输入输出。  | --                                                         |
| Resize                    | BPU加速         | 1. 输入featuremap需为四维NCHW，并且只支持在H和W维度上进行resize，onnx opset=11时支持roi输入（pytorch转换的模型需手动修改算子添加roi输入，roi只支持常量输入），roi输入只支持H和W维度，roi输入只在tf_crop_and_resize模式下起作用。<br/>2. 属性mode支持nearest和linear两种模式。<br/>3. 支持放大和缩小。<br/>4. 对于mode=nearest，放大系数factor支持2的幂数倍如2，4，8，16，32等；支持H维度和W维度的放大系数不同但需要满足`H_factor <= W_factor`。<br/>5. 对于onnx opset=11，属性coordinate_transformation_mode支持half_pixel，pytorch_half_pixel, asymmetric，align_corners和tf_crop_and_resize，当coordinate_transformation_mode=tf_crop_and_resize时，需要保证roi输入转换得到的边界坐标为整数。 | resize-10 <br/>- 输入等于2时，使用opset10。<br/>- 输入数据是4维Tensor。 <br/>resize-11  <br/>- 输入大于2时，使用opset11。<br/>- 输入数据是4维Tensor。<br/>- coordinate_transformation_mode在nearest, linear模式下支持half_pixel, asymmetric, align_corners和pytorch_half_pixel四种，在cubic模式下只支持half_pixel。<br/>- extrapolation_value属性不支持。 |
| ReverseSequence           | CPU计算         | --                                                           | --                                                           |
| RoiAlign                  | CPU计算         | --                                                           | --                                                           |
| Round                     | CPU计算        | --                                                           | --                                                           |
| Scan                      | CPU计算※        | --                                                           | --                                                           |
| Scatter (deprecated)      | CPU计算※        | --                                                           | --                                                           |
| ScatterElements           | CPU计算         | --                                                           | from_type支持：<br/>- data：type约束支持：float,int32,int8。<br/>- indices：type约束仅支持int32类型。<br/>- updates：type约束支持：float,int32,int8。<br/>to_type支持：type约束支持：float,int32,int8。  |
| ScatterND                 | CPU计算         | --                                                           | from_type支持：<br/>- data：type约束支持：float,int32,int8。<br/>- updates : type约束支持：float,int32,int8。<br/>to_type支持：type约束支持：float,int32,int8。   |
| Selu                      | CPU计算         | --                                                           | type约束：仅支持float类型。    |
| SequenceAt                | CPU计算※        | --                                                           | --                                                           |
| SequenceConstruct         | CPU计算※        | --                                                           | --                                                           |
| SequenceEmpty             | CPU计算※        | --                                                           | --                                                           |
| SequenceErase             | CPU计算※        | --                                                           | --                                                           |
| SequenceInsert            | CPU计算※        | --                                                           | --                                                           |
| SequenceLength            | CPU计算※        | --                                                           | --                                                           |
| Shape                     | BPU加速         | 会通过常量折叠将其优化为数值存储                               | --                                                             |
| Shrink                    | CPU计算※        | --                                                           | --                                                           |
| Sigmoid                   | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维，最高维 ∈ [1, 4096]，其它维 ∈ [1, 65536]。    | type约束：仅支持float类型。   |
| Sign                      | CPU计算         | --                                                           | type约束：仅支持float类型。                |
| Sin                       | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维，最高维 ∈ [1, 4096]，其它维 ∈ [1, 65536]。 | type约束：仅支持float类型。     |
| Sinh                      | CPU计算         | --                                                           | type约束：仅支持float类型。     |
| Size                      | BPU加速         | 会通过常量折叠将其优化为数值存储                            | --                                                           |
| Slice                     | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 无限制，支持非四维输入输出。  | 无                                                           |
| Softmax                   | BPU加速         | - 该算子支持int16输入输出。<br/>- 默认运行在CPU上，由于onnx::softmax和pytorch::softmax计算存在区别，分以下两种情况：<br/>1. 对于onnx::softmax，当该op输入为四维并且axis=3时，可以通过run_on_bpu指定该节点将其运行在BPU上。<br/>2. 对于pytorch::softmax, 当该op输入为四维并且axis=1,2,3时，可以通过run_on_bpu指定该节点将其运行在BPU上。 | type约束：仅支持float类型。  |
| Softplus                  | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维，最高维 ∈ [1, 4096]，其它维 ∈ [1, 65536]。 | type约束：仅支持float类型。  |
| Softsign                  | CPU计算         | --                                                           | type约束：仅支持float类型。       |
| SpaceToDepth              | BPU加速         | 支持mode=DCR 和 mode=CRD。<br/> 仅支持H和W方向的重新排列，并且仅支持blocksize=2的重排列。 <br/>举例：NxCxHxW -> Nx(4C)x(H/2)x(W/2) | type约束：仅支持float类型。    |
| Split                     | BPU加速         |1. 该算子支持int16输入输出。<br/>2. 原始输入的长度必须是每个被切分的tensor长度的倍数。<br/>3. 支持除N维度以外的任意维度。<br/>4. split数应可以整除。<br/>5. 支持非四维输入输出。 | type约束：仅支持float类型。    |
| SplitToSequence           | CPU计算※        | --                                                           | --                                                           |
| Sqrt                      | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维，最高维 ∈ [1, 4096]，其它维 ∈ [1, 65536]。|type约束：仅支持float类型。   |
| Squeeze                   | BPU加速         | 该op会被转换成Reshape，BPU约束详见Reshape op。     | --                                                          |            |
| StringNormalizer          | CPU计算※        | --                                                           | --                                                           |
| Sub                       | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入类型支持featurmap和常量，且最多支持一个常量输入。<br/>3. 支持除第一维外的广播，支持两个输入之间的互相广播，例如NH1C和N1WC。<br/>4. 输入输出维度支持2维、3维、4维和5维，大小为一般限制（见备注）。支持两个输入维度不同，输入为5维时需要满足以下限制：<br/>(1)首先可以通过合并相邻维度降维到4维，例如NHWD1和N1WDC可以合并W维和D维来降维；<br/>(2)其次广播的维度不能和相邻维度合并，例如NHWD1和N11DC因为H维、W维和C维都是广播的维度，无法通过合并相邻维度降维，所以无法支持。 | - 支持相同输入shape计算。<br/>- 支持输入1是标量或者输入2是标量的计算。<br/>- 支持broadcast计算，最大维度是5。  |
| Sum                       | BPU加速         | 限制条件等同于Add                                            | type约束：仅支持float类型。   |   
| Tan                       | CPU计算         | --                                                           | type约束：仅支持float类型。   |
| Tanh                      | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入输出支持1-10维，最高维 ∈ [1, 4096]，其它维 ∈ [1, 65536]。| type约束：仅支持float类型。   |
| TfIdfVectorizer           | CPU计算※        | --                                                           | --                                                           |
| ThresholdedRelu           | CPU计算         | --                                                           | type约束：仅支持float类型。  |
| Tile                      | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 输入与输出仅允许有一个维度上数值不同。| type约束：仅支持float,int64,int32,uint64,uint32类型。   |
| TopK                      | BPU加速 | 1. 该算子支持int16输入输出。<br/>2. input/indices/output维度支持1-10维。<br/>3. indices type约束支持int16/int32/int64。<br/>4. 参数sorted只支持true。 | - type约束：仅支持float类型。|
| Transpose                 | BPU加速         | 1. 该算子支持int16输入输出。<br/>2. 支持任意输入维度。 | - 支持nhwc2nchw，perm：[0, 3, 1, 2]。<br/>- 支持nchw2nhwc，perm：[0, 2, 3, 1]。<br/>- 支持指定perm维度转换，数据类型仅支持float，int8，int32。   |
| Unique                    | CPU计算※        | --                                                           | --                                                           |
| Unsqueeze                 | BPU加速         | 该op会被转换成Reshape，BPU约束详见Reshape op。       | --                                                          |
| Upsample (resize替代)     | BPU加速          | --                                                           | Upsample-(resize-10) <br/>- 输入等于2时，使用opset10。<br/>- 输入数据是4维Tensor。 <br/>Upsample-(resize-11)  <br/>- 输入大于2时，使用opset11。<br/>- 输入数据是4维Tensor。<br/>- coordinate_transformation_mode在nearest, linear模式下支持half_pixel, asymmetric, align_corners和pytorch_half_pixel四种，在cubic模式下只支持half_pixel。<br/>- extrapolation_value属性不支持。  |
| Where                     | CPU计算         | --                                                           | type约束支持float和int64类型。<br/> condition的shape为cond_shape，X的shape为x_shape，Y的shape为y_shape ，output的shape为o_shape，shape约束如下：<br/>- 仅支持cond_shape == o_shape情况下：  <br/>- x_shape == o_shape的broadcast。  <br/>- y_shape == o_shape的broadcast。<br/>- 仅支持cond_shape.NDim() == 4 && o_shape.NDim() == 4 && N维度值相同 && C维度值相同：  <br/>- 1x1（cond_shape）与HxW （o_shape）。  <br/>- Hx1（cond_shape）与HxW（o_shape）。  <br/>- 1xW（cond_shape）与HxW（o_shape）。 |
| Xor                       | CPU计算※        | --                                                           | --                                                           |
| Function                  | CPU计算※        | --                                                           | --                                                           |
| Celu                      | CPU计算※        | --                                                           | --                                                           |
| DynamicQuantizeLinear     | CPU计算※        | --                                                           | --                                                           |
| GreaterOrEqual            | BPU加速        | opset11 不支持单个GreaterOrEqual算子，支持拆分后的算子Less+Not运行在BPU上，限制条件与Less相同。| - 支持相同输入shape计算。<br/>- 支持输入1是标量或者输入2是标量的计算。<br/>- 支持broadcast计算，最大维度是5。 |
| MeanVarianceNormalization | CPU计算※        | --                                                           | --                                                           |
| GridSample（PyTorch）     | BPU加速         | 1. 输入维度仅支持四维，第一个输入需满足N ∈ [1, 4096]； C ∈ [1, 65536]； H,W ∈ [1, 1024] 且 `H*W <= 720*1024`。<br/>2. mode只支持'bilinear'、'nearest'。<br/>3. padding_mode只支持'zeros'、'border'。<br/>4. 该算子为opset16的onnx算子，为在opset11支持，工具链以自定义算子的方式提供导出，导出包含该算子的onnx模型请使用horizon_nn.torch.export_onnx接口替换torch.onnx.export，接口传参相同，示例代码如下：<br/>from horizon_nn.torch import export_onnx    <br/>...    <br/>    export_onnx(...)|                                                              |

---
sidebar_position: 3
---


## Supported Operator Lists and Restrictions{#supported_op_list_and_restrictions}

### Limitations and Notes

This section primarily covers the operators supported by the D-Robotics Processor for both `Caffe` and `ONNX`. Operators not listed are currently unsupported due to hardware limitations on the BPU.

**Terminology:**

- **BPU Acceleration**: Operators that the D-Robotics Processor can accelerate under certain constraints; if not met, they will be computed on the CPU.
- **CPU Computation**: Operators already optimized on D-Robotics's ARM CPU, supporting ONNX opsets 10 and 11.
- **CPU Computation※**: Temporary CPU operators not yet integrated.

**Additional Considerations:**

- For all BPU in RDK X3, there is a general restriction: input_batch ≤ 128.

- On RDK Ultra BPU, restrictions apply:
  1. Input and output dimensions must be 4D; support for non-four-dimensional ops is indicated explicitly.
  2. Shape: H, W, C ∈ [1, 65536], N ≤ 4096; and N x C x H x W ≤ 1GB.
  3. Supports Caffe 1.0 base operators and common extended operators, as well as ONNX opsets 10 and 11. Ops not meeting BPU acceleration constraints fallback to ARM CPU.

- Operators like `Cast`, `Constant`, `Dropout`, `Reshape`, `Squeeze`, `Unsqueeze`, and `Shape` (OPs) cannot run directly on the BPU, but algorithmic toolchains may optimize them in some cases (e.g., constant folding) for support.

- Operators marked as PyTorch are officially unsupported opsets 11 ops, which D-Robotics's algorithm toolchain provides a script to export from PyTorch to custom ONNX ops.

- Tensorflow-onnx conversion tool (https://github.com/onnx/tensorflow-onnx) supports converting TensorFlow 1.x operators to stable ONNX opsets 6-11, but TensorFlow 2.x support is still experimental.

- **Quantization Details**: A compliant operator may still run on CPU due to being a passively quantized OP. The algorithm toolchain designs quantization logic based on the OP's computation characteristics and BPU low-level logic. For more information on active, passive, and manual quantization, see the "[Quantization Logic in Algorithm Toolchain](https://developer.d-robotics.cc/forumDetail/118364000835765793)" chapter.

## RDK X3 List of supported Caffe operators

| **Caffe Operator Name**       | **CPU Computing/BPU Acceleration** | **X3 BPU Constraints** | **CPU Constraints** |
| - | ----------------- | --------------- | ----------- |
| Convolution   | BPU Acceleration | Kernel Size: HxW = [1, 7]x[1, 7] for BPU<br/> `Channel limit: <= 2048 (for non-dilated, group, depthwise conv), or <= 4096 for standard convs`<br/> No stride limit<br/> Dilation: only powers of 2 allowed, divisible by stride<br/> `h_dilated <= w_dilated`<br/> `Total kernel size: HxWxC <= 32768`<br/> axis not supported (default: 1) | 4D Conv only<br/> auto_pad not supported<br/>Type constraints: float, int32, int8<br/>Pads constraint: [Hstart, Wstart, Hend, Wend] (4 elements) with Hstart == Hend and Wstart == Wend |
| Deconvolution | BPU Acceleration | Kernel Size: HxW = [2, 14]x[2, 14]<br/>`Channel limit: C <= 2048`<br/>Padding: HxW = [0, (Kernel_H-1)/2]x[0, (Kernel_W-1)/2]<br/>`Stride: Stride ∈ {2, 4}, stride_h ≤ stride_w`<br/>Dilation: (1, 1)<br/>No axis support | output_shape and output_padding unsupported<br/>auto_pad: NOTSET only<br/>No axis support |
| MaxUnpool | CPU Computing | --- | from_type constraints: X - float only, I - Tensor(int64)<br/>to_type constraints: float only |
| Pooling      | BPU Acceleration | Four types: MaxPooling, AveragePooling, GlobalMaxPooling, GlobalAveragePooling<br/>Constraints: <br/>`MaxPooling: Kernel Size = [1, 64]x[1, 64], Stride = [1, 185], Padding >= 0`<br/>AveragePooling: HxW = [1, 7]x[1, 7], Stride ∈ {1, 185}<br/>`GlobalAveragePooling: HxW <= 8192 for NCHW input`<br/>GlobalMaxPooling: HxW = [1, 1024]x[1, 1024] for NCHW input | None |
| SPP          | CPU Computing | Not supported |`pyramid_height: 2^n pooling, n < 7`<br/>`pooling kernel size <= 255`<br/>pool option: {0, 1} |
| InnerProduct | BPU Acceleration | Converted to Conv<br/>Constraints: <br/>`For NCHW input, if HW < 7, Gemm limits same as Conv`<br/>`H = W = 1: C limit <= 16384; otherwise, C limit <= 2048`<br/>`Low-precision int8 output after BPU node: H x W/8 x C/4 ≤ 1024`<br/>`High-precision int32 output: H x W/8 x C/4 < 2048`<br/>No axis support | None |
| LRN          | CPU Computing | Not supported | local_size supported<br/>alpha, beta supported<br/>norm_region: ACROSS_CHANNELS, WITHIN_CHANNEL (optional)<br/>k supported |
| MVN          | CPU Computing | Not supported |` normalize_variance: {0, 1} (optional)`<br/>`across_channels: {0, 1} (optional)`<br/>Float32 only |
| BatchNorm    | BPU Acceleration | Unlimited | None |
| ELU          | CPU Computing | Not supported | None |
| BNLL         | CPU Computing | Not supported | None |
| PReLU        | BPU Acceleration | Unlimited | None |
| ReLU/LeakyReLu | BPU Acceleration | Unlimited | None |
| Sigmoid      | BPU Acceleration | `For 1CHW tensor: min(8W4C-aligned shape, 32C-aligned shape) ≤ 8192`<br/>8W4C: pad W to multiples of 8, C to multiples of 4<br/>32C: pad C to multiples of 32<br/>Use the smaller aligned shape | None |
| TanH         | BPU Acceleration | Unlimited | None |
| Eltwise      | BPU Acceleration | Operation supports Add and Mul, no Sub<br/>`Add: M ≤ 2048 channels`<br/>Supported cases: <br/>1. NCHW vs NCHW<br/>2. NCHW vs NC11 (inputs must be op outputs)<br/>`Mul: Both inputs must be 4D, C ≤ 2048`<br/>Supported shapes: <br/>1. (1xCxHxW vs 1xCxHxW)<br/>2. (1xCxHxW vs 1xCx1x1)<br/>3. (1xCxHxW vs 1x1x1x1) | None |
| Bias         | BPU Acceleration | Refer to Eltwise (Add) constraints | None |
| Scale        | BPU Acceleration | Refer to Eltwise (Mul) constraints | None |
| AbsVal       | CPU Computing | Not supported | None |
| Exp          | BPU Acceleration | Unlimited | None |
| Log          | CPU Computing | Not supported | None |
| Power            | BPU             | Unlimited                                                                                         | None                                                                                     |
| Threshold        | CPU              | Not supported                                                                                     | None                                                                                     |
| Reduction        | CPU              | Not supported                                                                                     | Operation supports SUM, ASUM, SUMSQ, MEAN. <br/>Axis supports. <br/>Only supports Float32 calculations. |
| Softmax          | CPU              | Not supported                                                                                     | None                                                                                     |
| ArgMax           | BPU             | `Only supports axis=1 and c<=64.` <br/>Does not support top_k != 1                                      | None                                                                                     |
| Concat           | BPU             | Input/Output Channel: `C<=2048`                                                                       | None                                                                                     |
| Split            | BPU             | Unlimited                                                                                         | None                                                                                     |
| Slice            | BPU             | Unlimited                                                                                         | None                                                                                     |
| Reshape          | CPU              | Not supported (can be fused in some scenarios)                                                      | Shape supports up to [1,4] shape_dim configurations. <br/>Axis supports [-4,3]. No support for N dimensions. Default value is 0, follows Caffe rules. |
| Flatten          | CPU              | Not supported (can be fused in some scenarios)                                                      | Axis range [-4,3], default is 1, -4 and 0 have the same meaning. Only supports End_axis == -1.      |
| Crop             | CPU              | Not supported                                                                                     | None                                                                                     |
| Dropout          | BPU             | Unlimited                                                                                         | None                                                                                     |
| LSTM             | BPU             | Only supports batch=1                                                                              | --                                                                                        |
| Normalize        | CPU              | Not supported                                                                                     | Type constraint: only supports float type.                                     |
| PassThrough      | BPU             | Supports mode=DCR and mode=CRD. <br/>Only supports rearrangement in H and W directions with blocksize=2. | Type constraint: only supports float type.                                      |
| CReLU            | CPU              | Not supported                                                                                     | Type constraint: only supports float type.                                      |
| RReLU            | CPU              | Not supported                                                                                     | None                                                                                     |
| Permute          | CPU              | Not supported                                                                                     | - Supports nhwc2nchw, perm: [0, 3, 1, 2]. <br/> - Supports nchw2nhwc, perm: [0, 2, 3, 1]. <br/> - Supports specified perm dimension conversions, data types: float, int8, int32. |
| MatMul           | BPU             | Optimized for specific scenarios: <br/>- K vs KxN, K vs 1xKxN, K vs 1x1xKxN<br/>- MxK vs K, MxK vs KxN, MxK vs 1x1xKxN<br/>- 1xMxK vs K, 1xMxK vs 1xKxN<br/>- 1x1xMxK vs K, 1x1xMxK vs 1xKxN, 1x1xMxK vs 1x1xKxN<br/>- BxMxK vs KxN (B>=1)<br/>- 1xBxMxK vs KxN (B>=1)<br/>- AxBxMxK vs KxN (A>1, B>1)<br/>For the opposite scenario: <br/>- 1xBxMxK vs 1x1xKxN (B>1)<br/>Optimized for two featuremaps: <br/>- 1xBxMxK vs 1x1xKxN (B>=1) | Type constraint: only supports float type.                                          |
| Upsample         | BPU             | `Input featuremap must be 4D NCHW, resize only on H and W dimensions, factors must be 2^N.` <br/>`Supports different factors for H and W, but H_factor <= W_factor required.` | None                                                                                     |
| ROIPooling       | CPU              | Not supported                                                                                     | None                                                                                     |
| PSROIPooling      | CPU              | Not supported                                                                                     | None                                                                                     |

## RDK X3 List of supported ONNX operators

| **ONNX Operator Name** | **CPU Computing/BPU Acceleration** | **X3 BPU Constraints** | **CPU Constraints** |
| --------------------- | --------------------------------- | --------------------- | ------------------ |
| Abs                   | CPU Calculation                   | --                    | Type constraint: only supports float type. |
| Acos                  | CPU Calculation                   | --                    | Type constraint: only supports float type. |
| Acosh                 | CPU Calculation                   | --                    | Type constraint: only supports float type. |
| Add                   | BPU Acceleration                 | `M <= 2048, supported cases:`<br/>1. NCHW and NCHW shapes for both inputs.<br/>2. NCHW and NC11 shapes (both inputs need to be outputs of other ops).<br/>3. Integrated into the previous conv in ResNet's shortcut structure for acceleration. | - Supports same shape inputs calculation.<br/>- Supports scalar input 1 or input 2 calculation.<br/>- Supports broadcast calculation with a max dimension of 5. |
| And                   | CPU Calculation                   | --                    | - Supports same shape inputs calculation.<br/>- Supports scalar input 1 or input 2 calculation.<br/>- Supports broadcast calculation with a max dimension of 5. |
| ArgMax                | BPU Acceleration                 | 1. Four-dimensional input (NCHW).<br/>2. Only supports argmax along the C dimension (axis=1).<br/>3.` C <= 64 `| Type constraint: only supports float type. |
| ArgMin                | CPU Calculation                   | --                    | Type constraint: only supports float type. |
| Asin                  | CPU Calculation                   | --                    | Type constraint: only supports float type. |
| Asinh                 | CPU Calculation                   | --                    | Type constraint: only supports float type. |
| Atan                  | CPU Calculation                   | --                    | Type constraint: only supports float type. |
| Atanh                 | CPU Calculation                   | --                    | Type constraint: only supports float type. |
| AveragePool           | BPU Acceleration                 | Kernel HxW: [1, 7]x[1, 7], Stride ∈{1, 185} | auto_pad attribute not supported.<br/>Only supports four-dimensional tensors. |
| BatchNormalization    | BPU Acceleration                 | Optimized to fuse with previous conv | Type constraint: only supports float type.<br/>Supports channel-first data layout (dim=1). |
| BitShift              | CPU Calculation(*)               | --                    | --                  |
| Cast                  | CPU Calculation                   | --                    | from_type supports double, float, bool, int64, uint32, int32, uint16, int16, uint8, int8.<br/>to_type supports double, float, bool, int64, uint32, int32, uint16, int16, uint8, int8. |
| Ceil                  | CPU Calculation                   | --                    | Type constraint: only supports float type. |
| Clip                  | BPU Acceleration                 | Unlimited              | Type constraint: only supports float type.<br/>Default min parameter when two inputs are provided. |
| Compress              | CPU Calculation(*)               | --                    | --                  |
| Concat                | BPU Acceleration                 | `Input/Output Channel: C<=2048` | --                  |
| ConcatFromSequence   | CPU Calculation(*)               | --                    | --                  |
| Constant              | BPU Acceleration                 | Optimized through constant folding | No support for sparse_tensor attribute.<br/>Type constraint: only supports float type. |
| ConstantOfShape       | BPU Acceleration                 | Optimized through constant folding | Supported types: float, int32, int8. |
| Conv                  | BPU Acceleration                 | Kernel HxW: [1, 7]x[1, 7].<br/>`Input/output Channel (for one group): <= 2048 (relaxed to <=4096 for non-dilated, group, depthwise conv).`<br/>Stride: Unrestricted (except stride=1 for Conv followed by Add in ResNet shortcut-connecting).<br/>Dilation: Only powers of 2 allowed, divisible by stride.<br/>`h_dilated ≤ w_dilated.`<br/>`Total kernel size limit: HxWxC ≤ 32768` | Only supports 4D Convolution.<br/>auto_pad attribute not supported.<br/>Type constraint: float, int32, int8.<br/>Pads constraint: [Hstart, Wstart, Hend, Wend] (4 elements) with Hstart==Hend and Wstart==Wend. |
| ConvInteger           | CPU Calculation(*)               | --                    | --                  |
| ConvTranspose         | BPU Acceleration                 | Kernel HxW: [2, 14]x[2, 14].<br/>`Input/output Channel: C <= 2048.`<br/>Padding HxW: [0,(Kernel_H-1)/2]x[0,(Kernel_W-1)/2].<br/>Stride: {2, 4}.<br/>`stride_h ≤ stride_w`.<br/>Dilation: (1, 1) only | auto_pad attribute not supported.<br/>Type constraint: float, int32, int8. |
| Cos                   | BPU Acceleration                 | `Limited to CxHxW <= 8192 for 1CHW tensor` | Type constraint: only supports float type. |
| Cosh                  | CPU Calculation                   | --                    | Type constraint: only supports float type. |
| CumSum                | CPU Calculation                   | --                    | --                  |



| Operator: from_type: | Description: BPU Acceleration | Supported Modes | Input Shape Constraints | Output Type Constraints |
| --- | --- | --- | --- | --- |
| DepthToSpace | BPU acceleration | Supports DCR and CRD modes. | Only supports rearrangement along H and W dimensions with blockSize=2. | <br/>- from_type: only float types allowed.<br/>- 4D Tensor computation only.<br/>- to_type: only float types allowed.<br/>- 4D Tensor computation only. |
| DequantizeLinear | CPU computation | -- | -- | -- |
| Det | CPU computation※ | -- | -- | -- |
| Div | BPU acceleration | 1. Supports featuremap inputs only (no constant inputs).<br/>2. Input shape constraints refer to Mul operator. | - Same input shape supported.<br/>- Supports scalar input1 or input2.<br/>- Broadcast calculation up to 5 dimensions. | 
| Dropout | BPU acceleration | Not computed in inference, removed by optimization. | -- | 
| Einsum | CPU computation※ | -- | -- | -- |
| Elu | CPU computation | -- | Type constraint: only float types. | 
| Equal | CPU computation | -- | - Same input shape supported.<br/>- Supports scalar input1 or input2.<br/>- Broadcast calculation up to 5 dimensions. | 
| Erf | CPU computation | -- | Type constraint: supports float and double types. | 
| Exp | BPU acceleration | -- | Type constraint: only float types. | 
| Expand | CPU computation | -- | -- | 
| EyeLike | CPU computation | -- | -- | 
| Flatten | CPU computation | -- | -- | 
| Floor | CPU computation | -- | Type constraint: only float types. | 
| GRU | CPU computation | -- | - direction attribute supports forward only.<br/>- Type constraint: only float types.<br/>- Input count must be 3, 4, or 6.<br/>- Output count is 2. | 
| Gather | CPU computation | -- | from_type:<br/>- input: types supported: float, int64, int32, int8, uint64, uint32, uint8.<br/>- indices: type supported: int32, int64.<br/>- to_type: types supported: float, int64, int32, int8, uint64, uint32, uint8. | 
| GatherElements | CPU computation | -- | -- | 
| GatherND | CPU computation | -- | from_type:<br/>- input: types supported: float, int32, int8.<br/>- indices: tensor(int64).<br/>- to_type: types supported: float, int32, int8. | 
| Gemm | BPU acceleration | Converted to Conv implementation. | <br/>- `HW <= 7 for both H and W if both are <= 7.`<br/>- `C <= 16384 if H/W = 1; otherwise, C <= 2048.`<br/>- `Low-precision int8 output if followed by BPU-supported node: H x W/8 x C/4 <= 1024.`<br/>- `High-precision int32 output if followed by non-BPU-supported node: H x W/8 x C/4 < 2048.`<br/>- Type constraint: only float types. | 
| GlobalAveragePool | BPU acceleration | `Input HxW must be <= 8192 for NCHW shape.` | -- | 
| GlobalLpPool | CPU computation | -- | Type constraint: supports float and double types.<br/>- 4D Tensor computation only. | 
| GlobalMaxPool | BPU acceleration | Input HxW range: [1, 1024]x[1, 1024] for NCHW shape. | Type constraint: only float types.<br/>- 4D Tensor only. | 
| Greater | CPU computation | -- | - Same input shape supported.<br/>- Supports scalar input1 or input2.<br/>- Broadcast calculation up to 5 dimensions. | 
| HardSigmoid | CPU computation | -- | Type constraint: only float types. | 
| Hardmax | CPU computation※ | -- | -- | 
| Identity | CPU computation | -- | -- | 
| If | CPU computation※ | -- | -- | 
| InstanceNormalization |  CPU Calculation   |                 |                                                              |                                                                                                                   |
| IsInf             | CPU Calculation |                                                             | Only supports float type.                                                                                         |
| IsNaN             | CPU Calculation |                                                             | Only supports float type.                                                                                         |
| LRN               | CPU Calculation | Only supports 4D Tensors and float type.                        |                                                                                                                   |
| LSTM              | BPU Accelerated | Supports batch_size=1 only.                                    | No attribute settings supported. Only supports inputs of 3, 4, or 8, and outputs of 2. Float type only.                  |
| LeakyRelu         | BPU Accelerated | N/A                                                          | N/A                                                                                                               |
| Less              | CPU Calculation | Supports same input shape, scalar input1 or input2, and broadcast | Supports up to 5-dimensional broadcast with same input shapes, and scalar inputs.                                   |
| LessOrEqual       | CPU Calculation | Same as 'Less'                                                 | Same as 'Less'.                                                                                                   |
| Log               | CPU Calculation | Only supports float type.                                      |                                                                                                                   |
| LogSoftmax        | CPU Calculation | Only supports float type.                                      |                                                                                                                   |
| Loop              | CPU Calculation |                                                             |                                                                                                                   |
| LpNormalization   | CPU Calculation | p-norm only supports 1 or 2, double or float type.            |                                                                                                                   |
| LpPool            | CPU Calculation | auto_pad not supported, double or float type, and 4D computation |                                                                                                                   |
| MatMulInteger     | CPU Calculation |                                                             |                                                                                                                   |
| MatMul            | BPU Accelerated | For scenarios where the two inputs are featuremap and weight, which involve element-wise multiplication between a featuremap and a constant, the following can be optimized for execution on a BPU:<br/>- K vs KxN, K vs 1xKxN, K vs 1x1xKxN<br/>- MxK vs K, MxK vs KxN, MxK vs 1x1xKxN<br/>- 1xMxK vs K, 1xMxK vs 1xKxN<br/>- 1x1xMxK vs K, 1x1xMxK vs 1xKxN, 1x1xMxK vs 1x1xKxN<br/>- BxMxK vs KxN (where B >= 1)<br/>- 1xBxMxK vs KxN (where B >= 1)<br/>- AxBxMxK vs KxN (where A > 1 and B > 1)<br/>For situations where both inputs are featuremaps (i.e., element-wise multiplication of featuremaps), the following can be optimized for the BPU:<br/>- 1xBxMxK vs 1x1xKxN (where B >= 1) | Only supports float type. Optimizations apply to specific input shapes: see details below.                           |
| Max               | CPU Calculation | Supports multiple inputs, same shape, scalar inputs, and broadcast | Up to 5-dimensional broadcast, supports scalar inputs.                                                               |
| MaxPool           | BPU Accelerated | Kernel size [1-64]x[1-64], stride [1-185], padding >= 0, no dilation | dilation only supports 1x1, data row-major storage, no auto_pad or storage_order support, 4D Tensors only. |
| MaxRoiPool        | CPU Calculation |                                                              |                                                                                                                   |
| Mean              | CPU Calculation |                                                             |                                                                                                                   |
| Min               | CPU Calculation | Same as 'Max'                                                 |                                                                                                                   |
| Mod               | CPU Calculation |                                                             |                                                                                                                   |
| Mul               | BPU Accelerated | `4D inputs with C <= 2048, specific shape rules apply`<br/>1. (1xCxHxW vs 1xCxHxW)。<br/>2. (1xCxHxW vs 1xCx1x1)。<br/>3. (1xCxHxW vs 1x1x1x1) 。| Same broadcast constraints as 'Mul'. Input values must not be 0.                                             |
| Multinomial       | CPU Calculation |                                                             |                                                                                                                   |
| Neg               | CPU Calculation |                                                             |                                                                                                                   |
| NonZero           | CPU Calculation | Supports float, int32, or int8 types, 1D or 4D computations      |                                                                                                                   |
| Not               | CPU Calculation |                                                             |                  |
| OneHot             | CPU            | --            | --     |
| Or                  | CPU            | --                                                                                      | Supports same input shape calculation. <br/>Supports scalar inputs. <br/>Broadcasting up to 5 dimensions. |
| PRelu              | BPU            | - Type constraints: Only supports float type.<br/>- from_type: X and slope.<br/>- to_type: Y.| - X's shape is data_shape, slope's is slope_shape.<br/>- data_shape == slope_shape.<br/>- slope_shape.ProdSize() == 1.<br/>- NCHW layout for 4D tensors with equal N and C dimensions.<br/>- HxW with 1x1 (slope_shape).<br/>- HxW with Hx1 (slope_shape).<br/>- HxW with 1xW (slope_shape).<br/>- Special case: 4D X and 3D slope, with data_shape[1] == slope_shape[0], slope_shape[1] == 1, slope_shape[2] == 1. |
| Pad                | BPU            | Supports mode=Constant. <br/>Only supports padding on H, W dimensions.               | Pad-10: <br/>- Type constraint: float only.<br/>- 4D NCHW tensors.<br/>- pads constraint: len(pads) == 8, pads[i] >= 0, pads[0] = pads[1] = pads[4] = pads[5] = 0.<br/>Pad-11: <br/>- from_type: data (float), pads (int64 tensor), optional constant_value (float).<br/>- 4D tensor, 2D or 3D padding only.<br/>- to_type: float only. |
| Pow                 | BPU            | Supports exponent as a single value.                                                  | - Type constraints: double, float, int64, int32.<br/>- Supports same shape, scalar inputs, and broadcasting up to 5 dimensions.<br/>- X and Y must be of the same type. |
| QLinearConv        | CPU※            | --                                                                                      | --                                                                       |
| QLinearMatMul      | CPU※            | --                                                                                      | --                                                                       |
| QuantizeLinear     | CPU            | --                                                                                      | --                                                                       |
| RNN                 | CPU            | --                                                                                      | - Type constraint: float only.<br/>- direction attribute: forward only.<br/>- Input constraints: X, W, R required, B, sequence_lens, initial_h unsupported.<br/>- Output constraint: Y_h output, shape [num_directions, batch_size, hidden_size]. |
| RandomNormal       | CPU※            | --                                                                                      | --                                                                       |
| RandomNormalLike   | CPU※            | --                                                                                      | --                                                                       |
| RandomUniform      | CPU            | --                                                                                      | --                                                                       |
| RandomUniformLike  | CPU            | --                                                                                      | --                                                                       |
| Range               | CPU            | Type constraints: float, int64, int32, int16.                                          | --                                                                       |
| Reciprocal          | BPU            | --                                                                                      | --                                                                       |
| ReduceL1            | CPU            | --                                                                                      | --                                                                       |
| ReduceL2            | CPU            | --                                                                                      | --                                                                       |
| ReduceLogSum        | CPU            | --                                                                                      | Only supports float, double data types.                                   |
| ReduceLogSumExp     | CPU            | --                                                                                      | Type constraints: float, double.                                         |
| ReduceMax           | CPU            | --                                                                                      | Axes support: 0, 1, or equal to input dimensions.                          |
| ReduceMean          | BPU            | Input featuremap must be 4D, axes=[2, 3].                                            | Axes support: 0, 1, or equal to input dimensions.                          |
| ReduceMin           | CPU            | --                                                                                      | --                                                                       |
| ReduceProd          | CPU            | --                                                                                      | --                                                                       |
| ReduceSum           | CPU            | --                                                                                      | Axes support: 0, 1, or equal to input dimensions.                          |
| ReduceSumSquare    | CPU            | --                                                                                      | Axes support: 0, 1, or equal to input dimensions.                          |
| Relu                | BPU        | --                                                                                      | --                                                                       |
| Reshape           | CPU          | --                | --                                                    |
| Resize            | BPU          | 1\. Input must be NCHW 4D and only resize in H and W dimensions. ROI input supported in ONNX opset=11 (manual modification required for PyTorch models to add ROI input, which only accepts constant inputs and works with tf_crop_and_resize mode).<br/>2\. Mode supports nearest and linear.<br/>3\. Supports scaling up and down.<br/>4\. For nearest mode, scaling factors should be powers of 2 (e.g., 2, 4, 8, 16, 32) and H_factor must be less than or equal to W_factor.<br/>5\. coordinate_transformation_mode supports half_pixel, pytorch_half_pixel, asymmetric, align_corners, and tf_crop_and_resize. When using tf_crop_and_resize, ensure ROI input coordinates are integers.<br/>resize-10<br/>- Use opset10 when input is 2.<br/>- Input is a 4D Tensor.<br/>resize-11<br/>- Use opset11 when input is greater than 2.<br/>- Input is a 4D Tensor.<br/>- coordinate_transformation_mode supports half_pixel, asymmetric, align_corners, and pytorch_half_pixel for nearest and linear modes, and half_pixel only for cubic mode.<br/>- extrapolation_value not supported. |
| ReverseSequence   | CPU          | --                | --                                                    |
| RoiAlign          | CPU          | --                | --                                                    |
| Round             | CPU          | --                | --                                                    |
| Scan              | CPU※          | --                | --                                                    |
| Scatter (deprecated)| CPU※         | --                | --                                                    |
| ScatterElements   | CPU          | --                | from_type: float, int32, int8<br/>indices: int32 only<br/>updates: float, int32, int8<br/>to_type: float, int32, int8 |
| ScatterND         | CPU          | --                | from_type: float, int32, int8<br/>updates: float, int32, int8<br/>to_type: float, int32, int8 |
| Selu              | CPU          | --                | Only supports float types.                           |
| SequenceAt        | CPU※          | --                | --                                                    |
| SequenceConstruct | CPU※          | --                | --                                                    |
| SequenceEmpty     | CPU※          | --                | --                                                    |
| SequenceErase     | CPU※          | --                | --                                                    |
| SequenceInsert    | CPU※          | --                | --                                                    |
| SequenceLength    | CPU※          | --                | --                                                    |
| Shape             | BPU          | Optimized to numerical storage via constant folding. | --                                                    |
| Shrink            | CPU※          | --                | --                                                    |
| Sigmoid           | BPU          | `Limited to 1CHW tensors where CxHxW <= 8192.`<br/>8W4C: pad W to multiples of 8 and C to multiples of 4.<br/>32C: pad C to multiples of 32.<br/>`Choose the smallest aligned shape between the two and ensure <= 8192.`| Only supports float types. |
| Sign              | CPU          | --                | None                                                  |
| Sin               | BPU          | `Limited to 1CHW tensors where CxHxW <= 8192.`       | Only supports float types.                            |
| Sinh              | CPU          | --                | Only supports float types.                            |
| Size              | BPU          | Optimized to numerical storage via constant folding. | --                                                    |
| Slice             | BPU          | Unlimited          | None                                                  |
| Softmax           | BPU          | Runs on CPU by default. Can be set to BPU for 4D inputs with axis=1 and as model output, using run_on_bpu. | Only supports float types. |
| Softplus               | BPU acceleration | `Supports CxHxW <= 8192 for a tensor of input dimension 1CHW.` | Only supports float type.                 |
| Softsign               | CPU computation | --                                                           | Only supports float type.                   |
| SpaceToDepth           | BPU acceleration | Supports DCR and CRD modes. <br/>Restrictions: H and W permutation, blocksize=2 only. | Only supports float type.                 |
| Split                  | BPU acceleration | Restrictions: NCHW input, divisible lengths, axis=1,2,3.     | Only supports float type.                 |
| SplitToSequence        | CPU computation(*) | --                                                           | --                                          |
| Sqrt                   | BPU acceleration | `Supports CxHxW <= 8192 for a tensor of input dimension 1CHW.` | Only supports float type.                 |
| Squeeze                | CPU computation | Removed by constant folding optimization if in constant substructure. | --                                          |
| StringNormalizer       | CPU computation(*) | --                                                           | --                                          |
| Sub                    | CPU computation | --                                                           | Supports same shape, scalar inputs, broadcast up to 5 dimensions. |
| Sum                     | BPU acceleration | Same restrictions as Add.                                       | Only supports float type.                 |
| Tan                     | CPU computation | --                                                           | Only supports float type.                 |
| Tanh                    | BPU acceleration | `Supports CxHxW <= 8192 for a tensor of input dimension 1CHW.` | Only supports float type.                 |
| TfIdfVectorizer       | CPU computation(*) | --                                                           | --                                          |
| ThresholdedRelu        | CPU computation | --                                                           | Only supports float type.                 |
| Tile                    | CPU computation | --                                                           | Supports float, int64, int32, uint64, uint32 types. |
| TopK                    | CPU computation | --                                                           | Only supports float type, opset-10.         |
| Transpose              | CPU computation | Supports nhwc2nchw, perm=[0, 3, 1, 2], nchw2nhwc, perm=[0, 2, 3, 1]. | Supports float, int8, int32 types.         |
| Unique                  | CPU computation(*) | --                                                           | --                                          |
| Unsqueeze               | CPU computation | Removed by constant folding optimization if in constant substructure. | --                                          |
| Upsample (replace resize)  | BPU acceleration | --                                                           | Upsample-10 for input=2, 4D Tensor.<br/>Upsample-11 for input>2, 4D Tensor. |
| Where                   | CPU computation | --                                                           | Supports float and int64 types. <br/>Shape constraints detailed in the description. |
| Xor                     | CPU computation(*) | --                                                           | --                                          |
| Function                | CPU computation(*) | --                                                           | --                                          |
| Celu                    | CPU computation(*) | --                                                           | --                                          |
| DynamicQuantizeLinear  | CPU computation(*) | --                                                           | --                                          |
| GreaterOrEqual         | CPU computation | --                                                           | Supports same shape, scalar inputs, broadcast up to 5 dimensions. |
| MeanVarianceNormalization | CPU computation(*) | --                                                           | --                                          |
| GridSample (PyTorch)    | CPU computation(*) | --                                                           | --                                          |



## RDK Ultra Supported Caffe Operators List

| **Caffe Operator Name**         | **CPU Computation/BPU Acceleration** | **RDK Ultra BPU Constraints** | **CPU Constraints** |
| -------------------------------- | --------------------------------- | --------------------------- | ------------------ |
| Convolution                     | BPU Accelerated                    | - `Kernel width and height: <= 32`<br/> - `Input/output channels (for one group): <= 8192 (or <= 65536 if last in quantized graph)`<br/> - `Stride: Unrestricted, stride for Conv followed by Add (ResNet shortcut-connection) should be {1, 2}`<br/> - `Dilation: <= 16`<br/> - `Only supports dilation=1 when dilation != 1`<br/> - `Axis default: 1`<br/> | - 4D Convolution only<br/> - auto_pad attribute not supported<br/> - Type constraints: float, int32, int8<br/> - Pads attribute constraint: [Hstart, Wstart, Hend, Wend] (4 elements) with Hstart==Hend and Wstart==Wend. |
| Deconvolution                   | BPU Accelerated                    | - `kernel >= stride`<br/> - `Input/output featuremaps <= 2048`<br/> - `pad <= kernel` / stride<br/> - out_pad < 2<br/> - `stride: 14 >= stride >= 1`, but stride_h and stride_w cannot both be 1<br/> - Axis configuration not supported | - Shape constraint: 4D Tensor computation only<br/> - Type constraint: float only<br/> - Attribute constraints: dilations, group, output_padding, pads, strides attributes<br/> - Pads attribute constraint: [hstart, wstart, hend, wend] must satisfy (hstart==hend and wstart==wend). |
| MaxUnpool                        | CPU Computation                    | ---                         | - from_type constraints: X - float, I - Tensor(int64)<br/> - to_type constraints: float only                                      |
| Pooling                          | BPU Accelerated                    | - Four types: MaxPooling, AveragePooling, GlobalMaxPooling, GlobalAveragePooling<br/> - `Constraints: MaxPooling - int16 input/output, kernel <= 256, stride <= 256, padding <= 256`<br/> - AveragePooling - same as MaxPooling<br/> - GlobalAveragePooling - unlimited<br/> - GlobalMaxPooling - H, W ∈ [1, 256] | None |
| SPP                              | CPU Computation                    | Not supported               | - `Supports pyramid_height with 2^n pooling, n < 7`<br/> - `pooling kernel <= 255`<br/> - `pool option, configurable values: {0, 1}` |
| InnerProduct                     | BPU Accelerated                    | Converted to Conv with Conv constraints<br/> - Axis configuration not supported | None |
| LRN                              | CPU Computation                    | Not supported               | - local_size supported<br/> - alpha, beta, norm_region supported (configurable values: ACROSS_CHANNELS, WITHIN_CHANNEL)<br/> - k supported |
| MVN                              | CPU Computation                    | Not supported               | - normalize_variance: configurable values {0, 1}<br/> - across_channels: configurable values {0, 1}<br/> - Float32 computation only |
| BatchNorm                        | BPU Accelerated                    | Unlimited                   | None |
| ELU                              | BPU Accelerated                    | - int16 input/output support<br/> - Input/output dimensions up to 10D, max dimension [1, 4096], others [1, 65536] | None |
| BNLL                             | CPU Computation                    | Not supported               | None |
| PReLU                            | CPU Computation                    | - type constraint: float only<br/> - from_type: X and slope<br/> - to_type: Y<br/> - Shape constraints: X = data_shape, slope = slope_shape<br/>   - data_shape == slope_shape<br/>   - slope_shape.ProdSize() == 1<br/>   - 4D NCHW layout for X and slope, N, C dimensions must be equal<br/>     - HxW or 1x1 for slope_shape<br/>     - Hx1 or 1xH for slope_shape<br/>     - 1xW or Wx1 for slope_shape<br/>   - Special case: 4D X and 3D slope with data_shape[1] = slope_shape[0] and slope_shape[1] = 1, slope_shape[2] = 1 | None |
| ReLU/LeakyReLU                    | BPU Accelerated                    | - int16 input/output support<br/> - Input/output dimensions up to 10D, max dimension [1, 4096], others [1, 65536] | None |
| Sigmoid                           | BPU Accelerated                    | - int16 input/output support<br/> - Input/output dimensions up to 10D, max dimension [1, 4096], others [1, 65536] | None |
| TanH                              | BPU Accelerated                    | - int16 input/output support<br/> - Input/output dimensions up to 10D, max dimension [1, 4096], others [1, 65536] | None |
| Eltwise                           | BPU Accelerated                    | Supports Add, Sub, Mul operations<br/> - int16 input/output support<br/> - Feature map and constant inputs, at most one constant<br/> - Broadcasting except first dimension<br/> - 2D, 3D, 4D, and 5D dimensions supported, with general limitations (see notes)<br/> - Different input dimensions supported, 5D inputs must meet: merge adjacent dimensions to 4D (e.g., NHWD1 and N1WDC), broadcast dimensions cannot be adjacent (e.g., NHWD1 and N11DC due to broadcast on H, W, and C) | None |
| Bias                              | BPU Accelerated                    | Refer to Eltwise (Add) constraints | None |
| Scale                             | BPU Accelerated                    | Refer to Eltwise (Mul) constraints | None |
| AbsVal                            | BPU Accelerated                    | - int16 input/output support<br/> - Input/output dimensions up to 10D, max dimension [1, 4096], others [1, 65536] | None |
| Exp                               | BPU Accelerated                    | - int16 input/output support<br/> - Input/output dimensions up to 10D, max dimension [1, 4096], others [1, 65536] | None |
| Log                               | BPU Accelerated                    | - int16 input/output support<br/> - Input/output dimensions up to 10D, max dimension [1, 4096], others [1, 65536] | None |
| Power    | BPU Op   | 1. Supports int16 input and output.<br/>2. Input and output support up to 10 dimensions, with max dimension ∈ [1, 4096], others ∈ [1, 65536].<br/>3. Second input only supports scalar. | -                                                                         |
| Threshold | CPU Computation | Not supported                                                                                   | -                                                                                               |
| Reduction | CPU Computation | Not supported. Operation supports SUM, ASUM, SUMSQ, MEAN, Max, LogSum, Min, Prod; Axis supports; Only supports Float32 computation. | -                                                                            |
| Softmax  | BPU Op   | 1. Supports int16 input and output.<br/>2. Defaults to CPU execution. Can run on BPU for 4D inputs with axis=1,2,3 if specified by run_on_bpu. | -                                                                          |
| ArgMax   | BPU Op   | `1. Only supports axis=1, c<=64.`<br/>`2. Does not support top_k ≠ 1.`<br/>`3. Supports int16 input and output.` | -                                                                          |
| Concat   | BPU Op   | 1. Supports int16 input and output.<br/>2. Does not support N-dimensional concat.                  | -                                                                          |
| Split    | BPU Op   | 1. Supports int16 input and output.<br/>2. Length of the original input must be a multiple of each split tensor length.<br/>3. Supports any dimension except N.<br/>4. Split count should be divisible.<br/>5. Supports non-four-dimensional input and output. | -                                   |
| Slice    | BPU Op   | 1. Supports int16 input and output.<br/>2. Unlimited, supports non-four-dimensional input and output. | -                                                                          |
| Reshape  | BPU Op   | 1. Supports int16 input and output.<br/>2. Supports up to 10-dimensional input and output.          | Shape supports [1,4] shape_dim configurations; Axis supports [-4,3], does not support N dimensions, default 0 follows Caffe rules; num_axes supports [-1,3], default -1 means all axes from axis start. |
| Flatten  | CPU Computation | Not supported (can be fused in some scenarios) | Axis range [-4,3], default is 1, with -4 and 0 having the same meaning. Only supports End_axis == -1. |
| Crop     | CPU Computation | Not supported                                                                               | -                                                                                               |
| Dropout  | BPU Op   | Unlimited                                                                                     | -                                                                                               |
| LSTM     | BPU Op   | Only supports batch=1                                                                             | -                                                                                               |
| Normalize | CPU Computation | Not supported                                                                               | Type constraint: only supports float types.                                      |
| PassThrough | BPU Op | Supports mode=DCR and mode=CRD. Only supports reordering along H and W directions with blocksize=2, e.g., NxCxHxW -> Nx(4C)x(H/2)x(W/2). | Type constraint: only supports float types. |
| CReLU    | CPU Computation | Not supported                                                                                 | Type constraint: only supports float types.                                      |
| RReLU    | CPU Computation | Not supported                                                                                 | None                                                                                           |
| Permute  | BPU Op   | 1. Supports arbitrary input dimensions.<br/>2. Supports conversion of any other dimension except batch dimension (first dimension). | - Supports nhwc2nchw, perm: [0, 3, 1, 2].<br/>- Supports nchw2nhwc, perm: [0, 2, 3, 1].<br/>- Supports permutation of specified dimensions, data types supported: float, int8, int32. |
| MatMul   | BPU Op   | C = MatMul(A, B), with dimension constraints for A and B:<br/>- Both A and B can have non-four-dimensional inputs but must meet these conditions:<br/>  - Dimensions of A and B must be the same.<br/>  - The lowest two dimensions M, K ∈ [1, 8192], higher dimensions ∈ [1, 4096].<br/>  Note: HDMK vs HDKN, MK/KN refers to the lowest two dimensions.<br/>- Broadcasting is supported under these conditions:<br/>  - All other dimensions than the lowest two of A and B are either 1 or do not require broadcasting.<br/>    - Supported example: HDMK vs H1KN<br/>    - Unsupported example: H1MK vs 1DKN<br/>  - A cannot have both broadcasting and non-broadcasting values in dimensions beyond its lowest two.<br/>    - Supported example: 11MK vs HDKN<br/>    - Unsupported example: H1MK vs HDKN<br/>  - If B has both broadcasting and non-broadcasting values in higher dimensions, non-broadcasting values must be contiguous.<br/>    - Supported example: BHDMK vs B11KN<br/>    - Unsupported example: BHDMK vs B1DKN<br/>- Broadcasting rules:<br/>- If A and B have unequal values in a given dimension, the 1 is considered the broadcasting value, and the non-1 is not.<br/>- If A and B have equal values in a given dimension, both are considered non-broadcasting values (e.g., HDMK vs H1KN, 1 is the broadcasting value, H is not). | Type constraint: only supports float types. |
| Upsample | BPU Op   | Requires four-dimensional NCHW input, resize only supported on H and W dimensions; factor cannot be less than 2. | -                                                                         |
| ROIPooling | CPU Computation | Not supported                                                                                 | -                                                                                               |
| PSROIPooling | CPU Computation | Not supported                                                                                 | -                                                                                               |



## RDK Ultra-supported ONNX Operators List

| **ONNX Operator Name** | **CPU/CPU Acceleration** | **RDK Ultra BPU Constraints** | **CPU Constraints** |
| ----------------- | --------------------- | --------------------------- | ------------------ |
| Abs                | BPU Accelerated         | 1. Supports int16 input/output.<br/>2. Input/output dimensions up to 10D, with max dimensions in [1, 4096] and others in [1, 65536]. | Type constraint: only supports float types. |
| Acos               | CPU Computation         | --                          | Type constraint: only supports float types. |
| Acosh              | CPU Computation         | --                         | Type constraint: only supports float types. |
| Add                | BPU Accelerated         | 1. Supports int16 input/output.<br/>2. Input can be featuremaps or constants, with at most one constant input.<br/>3. Supports broadcast except for the first dimension, including NHWC and N1WC broadcasting.<br/>4. Dimensions supported: 2D, 3D, 4D, and 5D, with general restrictions (see notes).<br/>5. In ResNet's shortcut connection, Add is fused into the preceding conv for acceleration. | - Supports computation with same input shape.<br/>- Supports scalar inputs as either input 1 or 2.<br/>- Supports broadcast up to 5D. |
| And                | CPU Computation         | --                         | - Supports same input shape calculation.<br/>- Supports scalar inputs as either input 1 or 2.<br/>- Supports broadcast up to 5D. |
| ArgMax             | BPU Accelerated         | 1. 4D input format NCHW.<br/>`2. Only supports argmax along the C axis (axis=1).<br/>3. C <= 64.`<br/>4. Supports int16 input/output. | Type constraint: only supports float types. |
| ArgMin             | BPU Accelerated         | Similar to ArgMax constraints | Type constraint: only supports float types. |
| Asin               | CPU Computation         | --                         | Type constraint: only supports float types. |
| Asinh              | CPU Computation         | --                         | Type constraint: only supports float types. |
| Atan               | BPU Accelerated         | 1. Supports int16 input/output.<br/>2. Input/output dimensions up to 10D, with max dimensions in [1, 4096] and others in [1, 65536]. | Type constraint: only supports float types. |
| Atanh              | CPU Computation         | --                         | Type constraint: only supports float types. |
| AveragePool        | BPU Accelerated         | `Kernel <= 256.`<br/>`Stride <= 256.<br/>Padding <= 256. `| No support for auto_pad attribute.<br/>Only supports 4D Tensors. |
| BatchNormalization | BPU Accelerated         | No limitations.             | Type constraint: only supports float types.<br/>Supports channel-first data layout (dimension 1 is channel). |
| BitShift           | CPU Computation※       | --                         | --                  |
| Cast               | CPU Computation         | --                         | from_type supports: double, float, bool, int64, uint32, int32, uint16, int16, uint8, int8.<br/>to_type supports: double, float, bool, int64, uint32, int32, uint16, int16, uint8, int8. |
| Ceil               | BPU Accelerated         | 1. Supports int16 input/output.<br/>2. Input/output dimensions up to 10D, with max dimensions in [1, 4096] and others in [1, 65536]. | Type constraint: only supports float types. |
| Clip               | BPU Accelerated         | 1. Supports int16 input/output.<br/>2. Input/output dimensions up to 10D, with max dimensions in [1, 4096] and others in [1, 65536].<br/>Opset 6: min, max as attributes, dtype only supports float.<br/>Opset 11: min, max as inputs, second input is min when there are two; dtype supports float, double. | 
| Compress            | CPU Computation※       | --                         | --                  |
| Concat             | BPU Accelerated         | 1. Supports int16 input/output.<br/>2. Does not support N-dimensional concatenation. | --                  |
| ConcatFromSequence | CPU Computation※       | --                         | --                  |
| Constant            | BPU Accelerated         | Optimized via constant folding | No support for sparse_tensor attribute. |
| ConstantOfShape    | BPU Accelerated         | Optimized via constant folding | Supported types: float, int32, int8. |
| Conv                | BPU Accelerated         | Supports 4D (conv2d) and 5D (conv3d) inputs.<br/>4D conv2d: Kernel size range: N,C ∈ [1, 8192]; H,W ∈ [1, 31].<br/>C*H*W ≤ 65535.<br/>Channel limits: 1 group, C ≤ 8192 (or 65536 if last operator in quantized graph).<br/>Stride: H,W ∈ [1, 256] (except for shortcut-connected conv, stride=1,2); dilation: H,W ∈ [1, 16], with H and W factors dividing input Tensor dimensions.<br/>Padding: H,W ∈ [0, 256].<br/>5D conv3d: NCDHW limits: N ∈ [1, 128]; H,W,D,C ∈ [1, 65536].<br/>Kernel size: N,C ∈ [1, 65536]; H,W ∈ [1, 31], D ∈ [1, 8191].<br/>Padding: DHW: H,W ∈ [0, 256], D ∈ [0, kernel_d/2].<br/>Stride: H, W must be 1 or 2.<br/>Group and dilation not supported.<br/>Size limit: 1GB; D*C ≤ 4096 for D*H*alignCeil(W, 256)*D*C < 1GB.<br/>Weight limit: D*C ≤ 8192. | Only supports 4D convolutions.<br/>No support for auto_pad attribute.<br/>Supported types: float, int32, int8.<br/>Pads constraint: [Hstart, Wstart, Hend, Wend] (4 elements) with Hstart==Hend and Wstart==Wend. |
| ConvInteger        | CPU Computation※       | --                         | --                  |
| ConvTranspose      | BPU Accelerated         | Input/output featuremap limits: N ∈ [1, 128], H,W ∈ [1, 65536], C ∈ [1, 2048].<br/>Size limit: 1GB.<br/>Weight size limits: N,C ∈ [1, 2048], H,W ∈ [1, 14], HW ≠ 1.<br/>Size: [1, 65535].<br/>Padding: For odd strides, H,W ∈ [0, kernel / stride); even strides, H,W ∈ [0, kernel / stride].<br/>Out_pad: H,W ∈ {0,1}.<br/>Stride: 1-14, not both stride_h and stride_w equal to 1. n ∈ {(1, 1)}. | Shape Constraint: Only supports 4D Tensors for computation.<br/>Type Constraint: Only supports float types.<br/>Attribute Constraints:<br/>- Supports only dilations, group, output_padding, pads, and strides attributes.<br/>- The pads attribute constraint is that [hstart, wstart, hend, wend] must satisfy (hstart==hend and wstart==wend).|
| Cos                       | BPU Acceleration | 1. This operator supports int16 input and output.<br/>2. Input and output support dimensions up to 10, with the highest dimension ∈ [1, 4096], and other dimensions ∈ [1, 65536].<br/>Type Constraint: Only supports float types.|
| Cosh                      | CPU Computation  | --                                                            |
| CumSum                    | CPU Computation  | --                                                            | Axis: Type Constraint is only for int32 types.|
| DepthToSpace             | BPU Acceleration | Supports modes DCR and CRD.<br/>Only rearrangement of H and W directions is supported, and blocksize=2 rearrangement only.<br/>Example: NxCxHxW -> Nx(C/4)x(2H)x(2W), where the number of channels must be a multiple of 4.| From_Type Constraints:<br/>- Type Constraint: Only supports float types.<br/>- Limited to 4D Tensor computation.<br/>To_Type Constraints:<br/>- Type Constraint: Only supports float types.<br/>- Limited to 4D Tensor computation.|
| DequantizeLinear         | CPU Computation | --                                                            |
| Det                       | CPU Computation※ | --                                                            |
| Div                       | BPU Acceleration | 1. Only supports featuremap inputs (not constant inputs);<br/>2. Input shape constraints refer to the Mul operator.<br/>- Supports same-input-shape computation.<br/>- Supports computation when input 1 is a scalar or input 2 is a scalar.<br/>- Supports broadcast computation with a maximum dimension of 5.|
| Dropout                   | BPU Acceleration | Does not participate in inference computations and will be removed during optimization. |
| Einsum                    | CPU Computation※ | --                                                            |
| Elu                       | BPU Acceleration | 1. This operator supports int16 input and output.<br/>2. Input and output support dimensions up to 10, with the highest dimension ∈ [1, 4096], and other dimensions ∈ [1, 65536].<br/>Type Constraint: Only supports float types.|
| Equal                     | BPU Acceleration | 1. Supports int16 input.<br/>2. Input and output dimensions support 2-5 dimensions.<br/>3. Supports broadcast across all dimensions, broadcast for fin0 or fin1 input allowed, but not mutual broadcasting. 5D broadcast has the following restrictions:<br/>- Must merge adjacent dimensions to reduce to 4D (including dimension N), e.g., NHWDC and NH1D1 can merge the NH dimension.<br/>- Broadcasted dimensions cannot merge with adjacent ones, e.g., NHWDC and N1W1C are unsupported due to inability to merge adjacent dimensions.<br/>4. Runs on CPU by default; can be specified to run on BPU with run_on_bpu.|
| Erf                       | CPU Computation | --                                                            | Type Constraint: Supports float and double data types.|
| Exp                       | BPU Acceleration | 1. Supports int16 input and output.<br/>2. Input and output support dimensions up to 10, with the highest dimension ∈ [1, 4096], and other dimensions ∈ [1, 65536].<br/>Type Constraint: Only supports float types.|
| Expand                    | BPU Acceleration | 1. Supports int16 input and output.<br/>2. Input and output support dimensions up to 10, with one differing dimension between input and output.<br/>3. Only allows one differing dimension between input and output.|
| EyeLike                   | CPU Computation | --                                                            |
| Flatten                   | BPU Acceleration | Constraints similar to Reshape. |
| Floor                     | BPU Acceleration | 1. Supports int16 input and output.<br/>2. Input and output support dimensions up to 10, with the highest dimension ∈ [1, 4096], and other dimensions ∈ [1, 65536].<br/>Type Constraint: Only supports float types.|
| GRU                       | CPU Computation | --                                                            | Direction Attribute: Only supports forward type.<br/>Type Constraint: Only supports float types.|
| Gather                    | BPU Acceleration | 1. All ranks of input/output/indices must be less than or equal to 4.<br/>2. Indices support:<br/>   - When indices are feature (other op outputs), type constraint is only for int32.<br/>   - When indices are weight (model constants), type constraint supports int32 and int64.<br/>From_Type Constraints:<br/>- input: Type constraint supports float, int64, int32, int8, uint64, uint32, uint8.<br/>- indices: Type constraint supports int32, int64.<br/>To_Type Constraints:<br/>- Type constraint supports float, int64, int32, int8, uint64, uint32, uint8.|
| GatherElements           | BPU Acceleration | 1. Supports int16 input and output.<br/>2. Input/indices/output dimensions support up to 10 dimensions.<br/>3. Indices type constraint supports int16/int32/int64.|
| GatherND                  | CPU Computation | --                          | From_Type Constraints:<br/>- input: Type constraint supports float, int32, int8.<br/>- indices: tensor(int64).<br/>To_Type Constraints: Type constraint supports float, int32, int8.|
| Gemm                      | BPU Acceleration | Gemm will be converted to Conv implementation, with boundary constraints referring to Conv. | Type Constraint: Only supports float types.|
| GlobalAveragePool        | BPU Acceleration | No limitations. | - Type Constraint: Only supports float types.<br/>- Limited to 4D Tensors.|
| GlobalLpPool              | CPU Computation | --                                                            | - Type Constraint: Supports float and double types.<br/>- Limited to 4D Tensor computation.|
| GlobalMaxPool             | BPU Acceleration | H, W ∈ [1, 256]. | - Type Constraint: Only supports float types.<br/>- Limited to 4D Tensors.|
| Greater                   | BPU Acceleration | 1. Supports int16 input.<br/>2. Input and output dimensions support 2-5 dimensions.<br/>3. Same as Equal operator constraints.<br/>4. Runs on CPU by default; can be specified to run on BPU with run_on_bpu.|
| HardSigmoid               | BPU Acceleration | 1. Supports int16 input and output.<br/>2. Input and output support dimensions up to 10, with the highest dimension ∈ [1, 4096], and other dimensions ∈ [1, 65536].<br/>Type Constraint: Only supports float types.|
| Hardmax                   | CPU Computation※ | --                                                            |
| Identity                  | CPU Computation | --                                                       |
| If | CPU Computation※ | -- | -- |
| InstanceNormalization | CPU Computation | -- | - Type constraint only supports float types.<br/>- Supports data layout with the first dimension as channels. |
| IsInf | CPU Computation※ | -- | -- |
| IsNaN | CPU Computation※ | -- | -- |
| LRN | CPU Computation | -- | - Type constraint only supports float types.<br/>- Only supports four-dimensional Tensors. |
| LSTM | BPU Acceleration | Supports batch_size=1 only. If using multiple batches, ensure LSTM's batch is 1 during ONNX export and configure the parameter input_batch=1 in the YAML. | - Type constraint only supports float types.<br/>- Attribute constraint: direction attribute only supports forward.<br/>- Input constraints:<br/>   - Supports X, W, R inputs;<br/>   - Supports X, W, R, B inputs (sequence_lens is empty or default);<br/>   - Supports X, W, R, B, sequence_lens, initial_h, initial_c, P inputs (sequence_lens is empty or default). |
| LeakyRelu | BPU Acceleration | 1. Supports int16 input and output.<br/>2. Input and output dimensions support 1-10 dimensions, with the highest dimension ∈ [1, 4096], others ∈ [1, 65536]. | Type constraint: only supports float types. |
| Less | BPU Acceleration | 1. Supports int16 input.<br/>2. Input/output dimensions support 2-5 dimensions.<br/>3. Runs on CPU by default; can be specified to run on BPU using run_on_bpu. | - Supports same shape inputs calculation.<br/>- Supports scalar input1 or scalar input2 calculation.<br/>- Supports broadcast calculation with a max dimension of 5. |
| LessOrEqual | BPU Acceleration | In opset11, single LessOrEqual not supported; Greater + Not operator is used instead, with the same limitations as Greater. | - Supports same shape inputs calculation.<br/>- Supports scalar input1 or scalar input2 calculation.<br/>- Supports broadcast calculation with a max dimension of 5. |
| Log | BPU Acceleration | 1. Supports int16 input and output.<br/>2. Input and output dimensions support 1-10 dimensions, with the highest dimension ∈ [1, 4096], others ∈ [1, 65536]. | Type constraint: only supports float types. |
| LogSoftmax | CPU Computation | -- | Type constraint: only supports float types. |
| Loop | CPU Computation※ | -- | -- |
| LpNormalization | CPU Computation | -- | - p-norm only supports 1 or 2.<br/>- Type constraint supports double and float types. |
| LpPool | CPU Computation | -- | - auto_pad attribute not supported.<br/>- Type constraint supports double and float types.<br/>- Limited to 4-dimensional computation. |
| MatMulInteger | CPU Computation※ | -- | -- |
| MatMul | BPU Acceleration | C = MatMul(A, B), with input A and B dimension restrictions:<br/>- Non-quadruple dimensional inputs allowed but must meet these constraints:<br/>  - A and B must have identical dimensions.<br/>  - The lowest two dimensions M, K ∈ [1, 8192], higher dimensions ∈ [1, 4096].<br/>  Note: HDMK vs HDKN, MK/KN refers to the lowest two dimensions.<br/>- Broadcast is supported under these conditions:<br/>  - For A and B, all dimensions except the lowest two must be either 1 or non-broadcastable values.<br/>    - Examples: HDMK vs H1KN<br/>    - Counterexample: H1MK vs 1DKN<br/>  - A's higher dimensions cannot contain both broadcastable and non-broadcastable values.<br/>    - Examples: 11MK vs HDKN<br/>    - Counterexample: H1MK vs HDKN<br/>  - If B's higher dimensions contain both broadcastable and non-broadcastable values, non-broadcastable ones must be consecutive high dimensions.<br/>    - Examples: BHDMK vs B11KN<br/>    - Counterexample: BHDMK vs B1DKN<br/>- Type constraint: only supports float types. |
| Max | BPU Acceleration | 1. Supports int16 input and output.<br/>2. Input/output dimensions support 2-5 dimensions.<br/>3. Supports broadcast across all dimensions, broadcast for fin0 or fin1 individually, not mutual broadcast. Restrictions for 5D broadcast:<br/>- Can merge adjacent dimensions to 4D (including dimension N), e.g., NHWDC and NH1D1 can merge NH.<br/>- Broadcast dimensions cannot merge with adjacent ones, e.g., NHWDC and N1W1C unsupported due to no adjacent dimension merge.<br/>- Other details in the documentation. | - Supports 1-∞ inputs.<br/>- Supports same shape inputs calculation.<br/>- Supports scalar input1 or scalar input2 calculation.<br/>- Supports broadcast calculation with a max dimension of 5. |
| MaxPool | BPU Acceleration | Supports int16 input and output.<br/>Kernel size ≤ 256.<br/>Stride ≤ 256.<br/>Padding ≤ 256.<br/>MaxPool does not support dilation. | 1. Dilation only supports 1x1.<br/>2. Data row-major storage only.<br/>3. auto_pad attribute not supported.<br/>4. storage_order attribute not supported.<br/>5. Limited to four-dimensional Tensor computation. |
| MaxRoiPool | CPU Computation | -- | No specific constraints. |
| Mean | CPU Computation※ | -- | -- |
| Min | BPU Acceleration | 1. Supports int16 input and output.<br/>2. Input/output dimensions support 2-5 dimensions.<br/>3. Similar to Max, but with different broadcast and dimension merge rules.<br/>4. Runs on CPU by default; can be moved to BPU using run_on_bpu. | - Similar to Max, but with different input constraints. |
| Mod | CPU Computation※ | -- | -- |
| Mul | BPU Acceleration | 1. Supports int16 input and output.<br/>2. Input types support feature maps and constants, with at most one constant input.<br/>3. Supports broadcast except the first dimension, mutual broadcast between inputs, like NH1C and N1WC.<br/>4. Dimensions up to 5D, with general restrictions (see notes). Supports different input dimensions, with specific restrictions for 5D input.<br/>(1) Merge adjacent dimensions to 4D, e.g., NHWD1 and N1WDC can merge W and D.<br/>(2) Cannot merge broadcast dimensions with adjacent ones, e.g., NHWD1 and N11DC unsupported due to H, W, and C being broadcast dimensions. | - Supports same shape inputs calculation.<br/>- Supports scalar input1 or scalar input2 calculation.<br/>- Supports broadcast calculation with a max dimension of 5. |
| Multinomial | CPU Computation※ | -- | -- |



| Operation | Implementation | Notes | Limitations |
| --- | --- | --- | --- |
| Neg | CPU computation |  |  |
| Not | CPU computation |  |  |
| OneHot | CPU computation |  |  |
| Or | CPU computation | - Supports same-input-shape computation.<br/>- Supports when Input 1 is a scalar or Input 2 is a scalar.<br/>- Supports broadcast calculation with a maximum dimension of 5. |
| PRelu | CPU computation | - Type constraint: only supports float types.<br/>- from_type: X and slope.<br/>- to_type: Y.<br/>- Constraints for X's shape (data_shape):<br/>  - data_shape == slope_shape.<br/>  - slope_shape.ProdSize() == 1.<br/>  - N, C dimensions must be equal in 4D NCHW layout.<br/>  - HxW with 1x1 (slope_shape), Hx1 (slope_shape), or 1xW (slope_shape).<br/>- Special case: 4D X and 3D slope with data_shape[1] == slope_shape[0] == 1 and slope_shape[2] == 1. |
| Pad | BPU acceleration | 1. Supports int16 input and output.<br/>2. Supports mode: Constant.<br/>3. Supports padding in all dimensions. | <br/>Pad-10:<br/>  - Type constraint: float only.<br/>  - 4D NCHW tensors only.<br/>  - Constraint on pads attribute:<br/>    - len(pads) == 8<br/>    - pads[i] >= 0<br/>    - pads[0] == pads[1] == pads[4] == pads[5] == 0.<br/>Pad-11:<br/>  - from_type: data - float only.<br/>  - pads: tensor(int64)<br/>  - constant_value (optional) - float only.<br/>  - to_type: float only.<br/>  - 4D Tensor only.<br/>  - Supports 2D or 3D padding only. |
| Pow | BPU acceleration | 1. Supports int16 input and output.<br/>2. Input/output support 1-10 dimensions, max dim ∈ [1, 4096], others ∈ [1, 65536].<br/>3. Second input must be a scalar. | - Type constraints: double, float, int64, int32.<br/>- Supports same-input-shape calculation.<br/>- Supports scalar inputs for either Input 1 or Input 2.<br/>- Supports broadcast calculation with a maximum dimension of 5.<br/>- Requires X and Y to have the same type. |
| QLinearConv | CPU computation※ |  |  |
| QLinearMatMul | CPU computation※ |  |  |
| QuantizeLinear | CPU computation |  |  |
| RNN | CPU computation |  - Type constraint: float only.<br/>- Attribute constraint: direction attribute supports forward only.<br/>- Input constraint: X, W, R inputs only, no optional inputs like B, sequence_lens, initial_h allowed.<br/>- Output constraint: Only Y_h output supported, shape [num_directions, batch_size, hidden_size]. |
| RandomNormal | CPU computation※ |  |  |
| RandomNormalLike | CPU computation※ |  |  |
| RandomUniform | CPU computation |  |  |
| RandomUniformLike | CPU computation |  |  |
| Range | CPU computation | Type constraints: float, int64, int32, int16. |  |
| Reciprocal | BPU acceleration | 1. Supports int16 input and output.<br/>2. Input/output support 1-10 dimensions, max dim ∈ [1, 4096], others ∈ [1, 65536]. |  |
| ReduceL1 | CPU computation |  |  |
| ReduceL2 | CPU computation |  |  |
| ReduceLogSum | CPU computation |  |  |
| ReduceLogSumExp | CPU computation | Type constraints: float, double. |  |
| ReduceMax | BPU acceleration | 1. Supports int16 input and output.<br/>2. Input supports 2-5 dimensions, requires axes attribute with 1 axis, no reduction across more than 1 dimension.<br/>3. Reduced dimension size ∈ [1, 8192].<br/>4. keepdims == 1 only.<br/>| Axes supported: 0, 1, or equal to input data dimensions. |
| ReduceMean | BPU acceleration | 1. Supports int16 input and output.<br/>2. Input supports 2-5 dimensions, requires axes attribute with 1 axis, no reduction across more than 1 dimension.<br/>3. Special case: Supports HW reduction when reduce_dim = 2.<br/>4. keepdims == 1 only.<br/>| Axes supported: 0, 1, or equal to input data dimensions. |
| ReduceMin | CPU computation |  |  |
| ReduceProd | CPU computation |  |  |
| ReduceSum | BPU acceleration | 1. Supports int16 input and output.<br/>2. Input supports 2-5 dimensions, requires axes attribute with 1 axis, no reduction across more than 1 dimension.| Axes supported: 0, 1, or equal to input data dimensions. |
| ReduceSumSquare | CPU computation |  |   |
| Relu                    | BPU acceleration      | Unlimited                                                                                               | Only supports float type.                                                                              |
| Reshape                 | BPU acceleration      | 1. Supports int16 inputs and outputs.<br/>2. Supports 1-10 dimensional inputs and outputs.                      | None.                                                                                                  |
| Resize                  | BPU acceleration      | 1. NCHW input featuremaps, resize only on H and W dimensions. onnx opset=11 supports ROI input (PyTorch models need manual modification to add ROI input, which only accepts constant inputs).<br/>2. Mode supports nearest and linear.<br/>3. Supports scaling up or down.<br/>`4. For nearest mode, scale factors must be powers of 2 (e.g., 2, 4, 8, 16, 32) with H_factor <= W_factor.`<br/>5. onnx opset=11 supports half_pixel, pytorch_half_pixel, asymmetric, align_corners, and tf_crop_and_resize. ROI input is only effective in tf_crop_and_resize mode, requiring integer boundary coordinates after conversion.<br/>6. extrapolation_value not supported. |
| ReverseSequence         | CPU computation      | --                                                                                                      | --                                                                                                     |
| RoiAlign                | CPU computation      | --                                                                                                      | --                                                                                                     |
| Round                   | CPU computation      | --                                                                                                      | --                                                                                                     |
| Scan                    | CPU computation*     | --                                                                                                      | --                                                                                                     |
| Scatter (deprecated)    | CPU computation*     | --                                                                                                      | --                                                                                                     |
| ScatterElements         | CPU computation      | --                                                                                                      | from_type: supports float, int32, int8.<br/>indices: only supports int32 type.<br/>updates: supports float, int32, int8.<br/>to_type: supports float, int32, int8. |
| ScatterND               | CPU computation      | --                                                                                                      | from_type: supports float, int32, int8.<br/>updates: supports float, int32, int8.<br/>to_type: supports float, int32, int8. |
| Selu                    | CPU computation      | --                                                                                                      | Only supports float type.                                                                             |
| SequenceAt              | CPU computation*     | --                                                                                                      | --                                                                                                     |
| SequenceConstruct       | CPU computation*     | --                                                                                                      | --                                                                                                     |
| SequenceEmpty           | CPU computation*     | --                                                                                                      | --                                                                                                     |
| SequenceErase           | CPU computation*     | --                                                                                                      | --                                                                                                     |
| SequenceInsert          | CPU computation*     | --                                                                                                      | --                                                                                                     |
| SequenceLength          | CPU computation*     | --                                                                                                      | --                                                                                                     |
| Shape                   | BPU acceleration      | Optimized through constant folding into numerical storage.                                          | --                                                                                                     |
| Shrink                  | CPU computation*     | --                                                                                                      | --                                                                                                     |
| Sigmoid                 | BPU acceleration      | 1. Supports int16 inputs and outputs.<br/>2. Supports 1-10 dimensional inputs, max dimension [1, 4096], others [1, 65536]. | Only supports float type.                                                                             |
| Sign                    | CPU computation      | Only supports float type.                                                                               | --                                                                                                     |
| Sin                     | BPU acceleration      | 1. Supports int16 inputs and outputs.<br/>2. Supports 1-10 dimensional inputs, max dimension [1, 4096], others [1, 65536]. | Only supports float type.                                                                             |
| Sinh                    | CPU computation      | Only supports float type.                                                                               | --                                                                                                     |
| Size                    | BPU acceleration      | Optimized through constant folding into numerical storage.                                          | --                                                                                                     |
| Slice                   | BPU acceleration      | 1. Supports int16 inputs and outputs.<br/>2. Unlimited, supports non-four-dimensional inputs and outputs. | No constraints.                                                                                        |
| Softmax                 | BPU acceleration      | - Supports int16 inputs and outputs.<br/>- Runs on CPU by default, with differences between onnx::softmax and pytorch::softmax:<br/>1. For onnx::softmax, can run on BPU if input is 4D and axis=3. Specify run_on_bpu.<br/>2. For pytorch::softmax, can run on BPU for 4D inputs and axis=1, 2, 3. Specify run_on_bpu.<br/>| Only supports float type.                                                                             |
| Softplus                | BPU acceleration      | 1. Supports int16 inputs and outputs.<br/>2. Supports 1-10 dimensional inputs, max dimension [1, 4096], others [1, 65536]. | Only supports float type.                                                                             |
| Softsign                | CPU computation      | --                                                                                                      | --                                                                                                     |



| Operator      | Acceleration   | Support modes and constraints                                                                                           | Type constraints |
|---------------|----------------|--------------------------------------------------------------------------------------------------------------|-----------------|
| SpaceToDepth  | BPU accelerated | Supports DCR and CRD modes. Only reordering along H and W dimensions is allowed, with blocksize=2.             | float only      |
| Split         | BPU accelerated | 1. Supports int16 inputs and outputs.<br/>2. Input length must be a multiple of each split tensor's length.<br/>3. Supports arbitrary dimensions except N.<br/>4. Split count must be divisible.<br/>5. Non-four-dimensional inputs and outputs supported. | float only      |
| SplitToSequence| CPU computation(*) | --                                                                                                             | --              |
| Sqrt          | BPU accelerated | 1. Supports int16 inputs and outputs.<br/>2. Input/output supports 1-10 dimensions, with max dimension in [1, 4096] and others in [1, 65536]. | float only      |
| Squeeze       | BPU accelerated | Converted to Reshape op. BPU constraints apply.                                                                          | --              |
| StringNormalizer| CPU computation(*) | --                                                                                                             | --              |
| Sub           | BPU accelerated | 1. Supports int16 inputs and outputs.<br/>2. Feature map and constant inputs supported, up to one constant.<br/>3. Broadcasting except first dimension, supports input broadcasting between NH1C and N1WC.<br/>4. 2D-5D dimensions supported, with general restrictions (see notes). Supports different input dimensions; for 5D inputs, see restrictions below.<br/>(1) Merge adjacent dimensions to 4D, e.g., NHWD1 and N1WDC.<br/>(2) Cannot merge broadcasted dimensions with adjacent ones, e.g., NHWD1 and N11DC not supported due to H, W, and C being broadcasted dimensions. | Same shape input support<br/>Scalar input support<br/>Broadcasting up to 5 dimensions. |
| Sum           | BPU accelerated | Constraints same as Add                                                                                             | float only      |
| Tan           | CPU computation | --                                                                                                             | float only      |
| Tanh          | BPU accelerated | 1. Supports int16 inputs and outputs.<br/>2. Input/output supports 1-10 dimensions, with max dimension in [1, 4096] and others in [1, 65536]. | float only      |
| TfIdfVectorizer| CPU computation(*) | --                                                                                                             | --              |
| ThresholdedRelu| CPU computation | --                                                                                                             | float only      |
| Tile          | BPU accelerated | 1. Supports int16 inputs and outputs.<br/>2. Only one dimension may have differing values between input and output. | float, int64, etc. |
| TopK          | BPU accelerated | 1. Supports int16 inputs and outputs.<br/>2. Input/indices/output dimensions: 1-10.<br/>3. Indices type: int16/int32/int64.<br/>4. Sorted parameter supports true only. | float only      |
| Transpose     | BPU accelerated | 1. Supports int16 inputs and outputs.<br/>2. Arbitrary input dimensions.                                        | nhwc2nchw, perm: [0, 3, 1, 2]<br/>nchw2nhwc, perm: [0, 2, 3, 1]<br/>Custom perm dimensions for float, int8, int32. |
| Unique        | CPU computation(*) | --                                                                                                             | --              |
| Unsqueeze     | BPU accelerated | Converted to Reshape op. BPU constraints apply.                                                                          | --              |
| Upsample (resize replacement)| BPU accelerated | --                                                                                                             | Upsample-10<br/>Input: 4D Tensor, opset10 when = 2<br/>Upsample-11<br/>Input: 4D Tensor, opset11 when > 2<br/>Coordinate transformation modes: nearest, linear (half_pixel, asymmetric, align_corners, pytorch_half_pixel), cubic (half_pixel only)<br/>Extrapolation_value unsupported. |
| Where          | CPU computation | --                                                                                                             | float, int64    |
| Xor           | CPU computation(*) | --                                                                                                             | --              |
| Function      | CPU computation(*) | --                                                                                                             | --              |
| Celu          | CPU computation(*) | --                                                                                                             | --              |
| DynamicQuantizeLinear | CPU computation(*) | --                                                                                                             | --              |
| GreaterOrEqual | BPU accelerated | Opset11 doesn't support standalone GreaterOrEqual; Less + Not on BPU for split conditions, with similar restrictions to Less. | Same shape, scalar, broadcast up to 5D. |
| MeanVarianceNormalization | CPU computation(*) | --                                                                                                             | --              |
| GridSample (PyTorch)| BPU accelerated | 1. Input dimensions: 4D, N ∈ [1, 4096], C ∈ [1, 65536], H, W ∈ [1, 1024], H*W ≤ 720*1024.<br/>2. Mode: bilinear, nearest.<br/>3. Padding_mode: zeros, border.<br/>4. Opset16 ONNX operator, exported via horizon_nn.torch.export_onnx (not opset11 native). See example below.| -- |


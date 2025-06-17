---
sidebar_position: 6
---
# Appendix

## Eager Mode

Similar to PyTorch official recommendation, we suggest users to use fx quantization mode as the first choice. horizon_plugin_pytorch currently supports quantization with eager mode.
The overall process of eager mode follows the quantization interface and concept from PyTorch officially, therefore, it is recommended to first read the relevant part about eager mode in [**PyTorch official documentation**](https://pytorch.org/docs/stable/quantization.html#quantization).

### Difference with fx mode

When using eager mode in horizon_plugin_pytorch, the main differences compared with fx mode are:

- Eager mode only supports module-based operators. You need to manually replace the functional operators in the floating-point model with Module-based operators in PyTorch or proprietary operators defined in horizon_plugin_pytorch, including but not limited to:

| Floating-point operators | Replaced operators |
|--------------------------|--------------------|
| torch.nn.functional.relu | torch.nn.ReLU() |
| a + b <br/> torch.add | horizon.nn.quantized.FloatFunctional().add |
| Tensor.exp | horizon.nn.Exp() |
| torch.nn.functional.interpolate | horizon.nn.Interpolate() |

- You need to manually define the operators to be fused and explicitly call the fusion function, and specify to use `fuser_func` provided in horizon_plugin_pytorch. The example is shown below:

```python
import torch
from torch import nn
import horizon_plugin_pytorch as horizon


class ConvBNReLU(nn.Sequential):
    def __init__(self, in_channels, out_channels, kernel_size):
        super(ConvBNReLU, self).__init__(
            nn.Conv2d(
            in_channels=in_channels,
            out_channels=out_channels,
            kernel_size=kernel_size
            ),
            nn.BatchNorm2d(num_features=out_channels),
            nn.ReLU()
        )

    # Specify the operators that can be fused
    def fuse_model(self):
        torch.quantization.fuse_modules(
            self,
            ['0', '1', '2'],
            inplace=True,
    # Specify the fuse function provided by horizon_plugin_pytorch in the horizon_plugin_pytorch package
    fuser_func=horizon.quantization.fuse_known_modules,
    )

    float_model = ConvBNReLU(1, 1, 1)
    # Need to explicitly call the fuse function
    float_model.fuse_model()

    print(float_model)
    # ConvBNReLU(
    #   (0): ConvReLU2d(
    #     (0): Conv2d(1, 1, kernel_size=(1, 1), stride=(1, 1))
    #     (1): ReLU()
    #   )
    #   (1): Identity()
    #   (2): Identity()
    # )
```

### Usage Flow

The overall flow of quantization-aware training in Eager mode is shown in the following figure:

![qat](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/04_toolchain_development/expert/qat.svg)

#### Build Float Model

When building a float model in Eager mode, there are a few things to note:

1. Insert quantization and dequantization nodes in the network. Generally, a quantization node should be inserted at the beginning of the float model, and a dequantization node should be inserted at the end. When the float model is converted to a QAT model for quantization-aware training, the inserted quantization node will quantize the input;

2. Replace some float-type function-form operators with operators inherited from Module in PyTorch or some proprietary operators provided by the Plugin;

3. Define the fusion function for float operators to fuse eligible operators.

```python
import torch
import torch.optim as optim
import horizon_plugin_pytorch as horizon
import os
from torch import nn
from torchvision import datasets, transforms
from torch.quantization import DeQuantStub
from horizon_plugin_pytorch.quantization import QuantStub

class ConvBNReLU(nn.Sequential):
    def __init__(self, in_channels, out_channels, kernel_size):
        super(ConvBNReLU, self).__init__(
            nn.Conv2d(
            in_channels=in_channels,
            in_channels = in_channels,
            out_channels = out_channels,
            kernel_size = kernel_size
            ),
            nn.BatchNorm2d(num_features=out_channels),
            nn.ReLU()
        )

    # Specify the floating point operators that can be fused
    def fuse_model(self):
        torch.quantization.fuse_modules(
            self,
            ['0', '1', '2'],
            inplace=True,
            fuser_func=horizon.quantization.fuse_known_modules,
        )

class ClassiFier(nn.Module):
    def __init__(self, in_channels, out_channels):
        super(ClassiFier, self).__init__()
        self.conv = nn.Conv2d(in_channels, out_channels, 1)

    def forward(self, data):
        return self.conv(data)

# Build the floating point model
class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        self.conv0 = ConvBNReLU(1, 10, 5)
        self.max_pool = nn.MaxPool2d(kernel_size=2)
        self.conv1 = ConvBNReLU(10, 20, 5)
        self.avg_pool = nn.AvgPool2d(kernel_size=8)
        self.classifier = ClassiFier(20, 10)
        # To adapt to the BPU, when getting input from the camera, the scale of the QuantStub must be set to 1/128 explicitly.
        self.quant = QuantStub(scale=1/128)
        self.dequant = DeQuantStub()

    def forward(self, x):
        # Insert quantization node to quantize the input
        x = self.quant(x)
        x = self.conv0(x)
        x = self.max_pool(x)
        x = self.conv1(x)
        x = self.avg_pool(x)
        x = self.classifier(x)
        # Insert dequantization node to dequantize the output
        x = self.dequant(x)
        return x

    # Define the fusion function```python
def fuse_model(self):
    from horizon_plugin_pytorch import quantization

    for m in self.modules():
        if type(m) == ConvBNReLU:
            m.fuse_model()
```

#### Float Model Pretraining {#float-model-pretrain}

```python
train_batch_size = 16
test_batch_size = 16
epoch_num = 1
neval_batches = 1
model_file = 'model.pt'

class AverageMeter(object):
    """Computes and stores the average and current value"""

    def __init__(self, name, fmt=":f"):
        self.name = name
        self.fmt = fmt
        self.reset()

    def reset(self):
        self.val = 0
        self.avg = 0
        self.sum = 0
        self.count = 0

    def update(self, val, n=1):
        self.val = val
        self.sum += val * n
        self.count += n
        self.avg = self.sum / self.count

    def __str__(self):
        fmtstr = "{name} {val" + self.fmt + "} ({avg" + self.fmt + "})"
        return fmtstr.format(**self.__dict__)

criterion = nn.CrossEntropyLoss()

def accuracy(output, target, topk=(1,)):
    """Computes the accuracy over the k top predictions for the specified
    values of k
    """
    with torch.no_grad():
        maxk = max(topk)
        batch_size = target.size(0)

        _, pred = output.topk(maxk, 1, True, True)
        pred = pred.t()
        correct = pred.eq(target.view(1, -1).expand_as(pred))

        res = []
        for k in topk:
            correct_k = correct[:k].reshape(-1).float().sum(0, keepdim=True)
            res.append(correct_k.mul_(100.0 / batch_size))
        return res


def get_train_data_loader():
    train_loader = torch.utils.data.DataLoader(
        datasets.MNIST(
            'mnist_data',
            train=True,
            download=True,
            transform=transforms.Compose(
                [transforms.ToTensor(),
                 transforms.Normalize((0.5,), (0.5,))]
            )
        ),
        batch_size=train_batch_size,
        shuffle=True,
    )
    return train_loader

def get_test_data_loader():
    train_loader = torch.utils.data.DataLoader(
        datasets.MNIST(
            'mnist_data',
            train=False,
            download=True,
            transform=transforms.Compose(
                [transforms.ToTensor(),
                 transforms.Normalize((0.5,), (0.5,))]
            )
        ),
        batch_size=test_batch_size,
        shuffle=True,
    )
    return train_loader

data_loader = get_train_data_loader()
test_loader = get_test_data_loader()

def train(model, device, optimizer, epoch):
    global min_loss
    model.train()
    for batch_idx, (data, target) in enumerate(data_loader):
        data = data.to(device)
        target = target.to(device)
        output = model(data)
        output = output.view(-1, 10)
        loss = criterion(output, target)
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        if batch_idx %  100 == 0:
            print ('Train Epoch: {} batch {} \t Loss: {:.6f}'.
                format(epoch, batch_idx, loss.item()))

def evaluate(model, device, neval_batches):
    model.eval()
    top1 = AverageMeter("Acc@1", ":6.2f")
    top5 = AverageMeter("Acc@5", ":6.2f")
    tested_batches = 0
    with torch.no_grad():
        for batch_idx, (data, target) in enumerate(test_loader):
            tested_batches += 1
            data = data.to(device)
            target = target.to(device)
            output = model(data)
            output = output.view(-1, 10)
            loss = criterion(output, target)
            acc1, acc5 = accuracy(output, target, topk=(1, 5))
            top1.update(acc1[0], data.size(0))
            top5.update(acc5[0], data.size(0))
            if tested_batches >= neval_batches:
                return top1, top5

    return top1, top5


def train_float_model(device):
    model = Net().to(device)
    optimizer = optim.SGD(model.parameters(), lr=0.001, momentum=0.1)
    for nepoch in range(epoch_num):
        train(model, device, optimizer, nepoch)
        top1, top5 = evaluate(model, device, neval_batches)
        print(
            "float training Epoch %d :float evaluation accuracy on %d images, \
            %2.2f" % (nepoch, neval_batches * test_batch_size, top1.avg)
        )
    torch.save(model.state_dict(), model_file)

train_float_model(torch.device('cuda'))
```



If you want to perform quantization-aware training on an existing floating-point model, you can first load the float model and then proceed with the steps for(fusion operators) and quantization training. If you're directly quantizing after float training without any intermediate step, there's no need to explicitly load the model. You can proceed directly.

```python
def load_model():
    model = Net()
    state_dict = torch.load(model_file)
    model.load_state_dict(state_dict)
    model.to('cpu')
    return model

# Load the float model for quantization-aware training
qat_model = load_model()
```

#### Set BPU architecture {#set-bpu}

```python
# Set march to BERNOULLI2 for **RDK X3** and BAYES for **RDK Ultra**.
horizon.march.set_march(horizon.march.March.BAYES)
```

#### Operator fusion {#op-fuse}

```python
qat_model.fuse_model()
```

#### Convert floating-point model to quantized model {#float-to-quantized}

```python
def load_and_prepare_qat_model(device):
    # Load pre-trained floating-point model
    global qat_model
    qat_model = qat_model.to(device)
    top1, top5 = evaluate(qat_model, device, neval_batches)
    print(
        "float evaluation accuracy on %d images, \
        %2.2f" % (neval_batches * test_batch_size, top1.avg)
    )
    # Set the quantization parameters for quantizing the weights and outputs of operators
    qat_model.qconfig = horizon.quantization.get_default_qat_qconfig()
    # Turn off quantization for the output layer to improve accuracy
    qat_model.classifier.qconfig = \
        horizon.quantization.get_default_qat_out_qconfig()
    # Convert the floating-point model to quantized model
    horizon.quantization.prepare_qat(qat_model, inplace=True)
    print(
        "After preparation for QAT, note fake-quantization modules \n",
        qat_model.conv0,
    )
    qat_model = qat_model.to(device)
    load_and_prepare_qat_model(torch.device('cuda'))
```

#### Quantization Tnjkraining

```python
def quantization_training(device):
    # Quantization training for the quantized model
    optimizer = optim.SGD(qat_model.parameters(), lr=0.0001)
    for nepoch in range(1):
        train(qat_model, device, optimizer, nepoch)
        # Evaluate the quantized model for one epoch
        top1, top5 = evaluate(qat_model, device, neval_batches)
        print(
            "QAT Epoch %d :float evaluation accuracy on %d images, %2.2f"
            % (nepoch, neval_batches * test_batch_size, top1.avg)
        )

quantization_training(torch.device('cuda'))
```

#### Convert Quantized Model to Fixed-point Model

```python
quantized_model = horizon.quantization.convert(
    qat_model.eval(), inplace=False
)
```

#### Check and Compile the Fixed-point Prediction Model

```python
def compile_quantized_model(device):
    example_input = torch.ones(size=(neval_batches, 1, 28, 28), device=device)
    traced_model = torch.jit.trace(quantized_model, example_input)
    top1, top5 = evaluate(traced_model, device, neval_batches)
    print(
        "Traced : int evaluation accuracy on %d images, %2.2f"
        % (neval_batches * test_batch_size, top1.avg)
    )

    # Check if the model can be compiled using hbdk. hbdk is a tool for compiling fixed-point models.
    horizon.quantization.check_model(quantized_model, example_input, advice=1)
    hbdk_dir = "hbdk_model"
    if not os.path.exists(hbdk_dir):
        os.mkdir(hbdk_dir)

    # Compile the model, and the model.hbm in the hbdk_model directory is the compiled on-board model.
    horizon.quantization.compile_model(traced_model, [example_input], opt=2, hbm=hbdk_dir + "/model.hbm"
)
# Static performance analysis of the model
horizon.quantization.perf_model(
    traced_model,
    [example_input],
    opt=2,
    input_source=["pyramid"],
    layer_details=True,
    out_dir=hbdk_dir,
)
horizon.quantization.visualize_model(
    traced_model,
    [example_input],
    save_path=hbdk_dir + "/model.svg",
    show=False,
)

compile_quantized_model(torch.device('cuda'))
```



## Supported General Operators

### Overall Explanation

1. Unless otherwise specified, the inputs and outputs of Bernoulli2 architecture-constrained operators are all 4-dimensional.
2. In eager mode, some operators need to be manually replaced, while fx mode does not need to replace operators manually.
3. By default, the supported operators do not perform operator fusion. For operators that can be fused (such as (conv, bn), relu), refer to the [**Operator Fusion**](./advanced_content.md#op_fusion) section.
4. In the inference phase, transparent operators (such as Identity, Dropout) will be optimized out during deployment.

### torch function class


| Operator   | Eager mode equivalent operator | Bernoulli2 | Input | Output | Bayes | Output | Other constraints |
|------------|-------------------------------|-----------------|------------|-----------|-------------|----------|-----------------|
|            |                               |   Input         |	 Output |	Other constraints|   Input     |	Output  | Other constraints |
|torch.abs   |                               | Not supported  |        |          |qint8, qint16 | Same as input |                 |
|torch.acos  |horizon.nn.Acos	             | Not supported  |        |          |qint8, qint16	|qint8, qint16|	Implementation using a lookup table, with accuracy risks|
|torch.acosh |	horizon.nn.Acosh             |	Not supported  |       |           |Refer to torch.acos|           |                  |
|torch.add    | torch.nn.quantized.FloatFunctional or horizon.nn.quantized.FloatFunctional  |	qint8, qint16| qint8, qint16| in_channel`<=`2048, not supported for operands as constants |qint8, qint16|qint8, qint16| Supports broadcasting except for N dimensions, only one input can be broadcasted, call add_scalar if one of the operands is a scalar|
|torch.argmax|		            |Refer to torch.max     |	    |   		|Refer to torch.max	|           |	|
|torch.argmin|		            |Refer to torch.max		|	    |           |Refer to torch.max	|           |	|
|torch.asin	| horizon.nn.Asin	|   Not supported          |       |           | Refer to torch.acos|  | |
|torch.asinh	|horizon.nn.Asinh	|Not supported| | |Refer to torch.acos| | |
|torch.atan|	horizon.nn.Atan|	Not supported| | |Refer to torch.acos| | |
|torch.atanh|	horizon.nn.Atanh|	Not supported| | |Refer to torch.acos| | |
| torch.cat | torch.nn.quantized.FloatFunctional or horizon.nn.quantized.FloatFunctional | qint8, qint16 | qint8, qint16 |  |qint8, qint16 | qint8, qint16 | input shape: [N, C, H, W], N`<=`4096, HWC`<=`65536, 2`<=`input number`<=`1024 |
| torch.ceil | horizon.nn.Ceil | Not supported |  | | qint8, qint16 | Same as input |Do not exceed the level of 1e6 for int8 input and the level of 1e8 for int16 input. |
| torch.clamp | Not supported | No | qint8, qint16 | Same as input | Supports min and max inputs as Tensor/Constant Tensor/Scalar/None. For Constant Tensor, the input data range should be consistent with input to avoid precision issues. |
| torch.clip | Not supported | No | - | Refer to torch.clamp | - |
| torch.cos | horizon.nn.Cos | Not supported | - | Refer to torch.acos | - |
| torch.cosh | horizon.nn.Cosh | Not supported | - | Refer to torch.acos | - |
| torch.div | horizon.nn.Div | Not supported | qint16 | qint16 | - |
| torch.eq | Not supported | No | qint8, qint16 | qbool | - |
| torch.erf | horizon.nn.Erf | Not supported | - | Refer to torch.acos | - |
| torch.exp | horizon.nn.Exp | qint8 | qint8 | Uses table lookup, has precision risk | Refer to torch.acos | - |
| torch.floor | horizon.nn.Floor | Not supported | qint8, qint16 | Same as input | Int8 inputs should not exceed 1e6 in magnitude, int16 inputs should not exceed 1e8. |
| torch.gather | Not supported | No | qint8, qint16, qint32 | Same as input | - |
| torch.ge | Not supported | No | - | Refer to torch.eq | - |
| torch.greater | Not supported | No | - | Refer to torch.eq | - |
| torch.greater_equal | Not supported | No | - | Refer to torch.eq | - |
| torch.gt | Not supported | No | - | Refer to torch.eq | - |
| torch.le | Not supported | No | - | Refer to torch.eq | - |
| torch.less | Not supported | No | - | Refer to torch.eq | - |
| torch.less_equal | Not supported | No | - | Refer to torch.eq | - |
| torch.log | horizon.nn.HardLog | Not supported | - | Refer to torch.acos | - |
| torch.lt | Not supported | No | - | Refer to torch.eq | - |
| torch.matmul | horizon.nn.quantized.FloatFunctional | qint8 | qint8, qint32 | - | `Input shape: [N, C, H, W], input size < 1 GB, N <= 4096, C, H, W <= 8192.` |
| torch.max | | qint8 | Same as input | Only for model output. Output format differs from torch: Compiler supports a Tensor with max_value in one channel and max_value_index in another. | qint8, qint16 output; int32 index | `Index can only be used as model output. Input shape: [N, C, H, W], 1 <= N <= 4096, 1 <= H, W, C <= 65535. Supports min and max inputs as Tensor/Constant Tensor/Scalar/None. Consistency in input data range with min and max is required for precision.` |
| torch.maximum | horizon.nn.quantized.FloatFunctional | Not supported | - | - | input: qint8, qint16<br/> other: qint8, qint16 | qint8, qint16 | - |
| torch.mean | horizon.nn.quantized.FloatFunctional | qint8, qint16 | qint8, qint16 | Supports channel-wise mean only. QAT has training parameters, don't use standalone in inference. | qint8, qint16 | qint8, qint16 | Supports mean in CHW. QAT has quantization parameters. |
| torch.min | Not supported | No | - | Refer to torch.max | - |
| torch.minimum | horizon.nn.quantized.FloatFunctional | Not supported | - | Refer to torch.maximum | - |
| torch.mul | torch.nn.quantized.FloatFunctional or horizon.nn.quantized.FloatFunctional | Refer to torch.add | - | Refer to torch.add | - |
| torch.pow | horizon.nn.Pow | Not supported | - | Refer to torch.acos | - |
| torch.reciprocal | horizon.nn.Reciprocal | Not supported | - | Refer to torch.acos | - |
| torch.selu | horizon.nn.Selu | Not supported | - | Refer to torch.acos | - |
| torch.sin | horizon.nn.Sin | Not supported | - | Refer to torch.acos | - |
| torch.sinh | horizon.nn.Sinh | Not supported | - | Refer to torch.acos | - |
| torch.split | | qint8, qint16 | Same as input | - | qint8, qint16 | Same as input | - |
| torch.sqrt | horizon.nn.Sqrt | Not supported | - | Refer to torch.acos | - |
| torch.sub | horizon.nn.quantized.FloatFunctional | qint8, qint16 | qint8, qint16 | `in_channel <= 2048` | qint8, qint16 | qint8, qint16 | Supports broadcasting except N dimensions. Only one input can broadcast. |
| torch.sum | horizon.nn.quantized.FloatFunctional | qint8 | qint8, qint32 | Supports batch and channel-wise sum. | qint8, qint16 | qint8, qint16 | Supports sum in HWC only. |
| torch.tan | horizon.nn.Tan | Not supported | - | Refer to torch.acos | - |
| torch.topk | Not supported | No | qint8, qint16, qint32 | Same as input | - |

### torch.nn.functional function class

| Operator        | Eager Mode Replacement Operator | Bernoulli2   |      |        | Bayes    |        |                |
|-----------------|---------------------------------|-----------------|-------|---------|-------------|----------|-----------------|
|            |                    |   input         |	output |	 other limits |	  input     |	output   |	other limits      |
| torch.nn.functional.grid_sample | N/A                             | Not supported | Not supported | Not supported | Input: qint8 <br/> Grid: qint8, qint16 | Output: qint8 | Input shape: [N, C, H, W], 1≤H, W≤1024 and H*W≤720*1024. Supports bilinear and nearest interpolation with padding modes only zeros and border. |
| torch.nn.functional.interpolate | qint8                            | qint8           | Supports nearest and bilinear interpolation. 1/256 ≤ scale ≤ 256 | qint8 | qint8 | Supports nearest and bilinear interpolation. Input shape: [N, C, H, W], 1≤C, H, W≤8192. align_corners supports False and None. Requires recompute_scale_factors to be True when scale=[]. |
| torch.nn.functional.pad | N/A                             | Not supported | Not supported | N/A | qint8, qint16 | Same as input | Reflect mode not supported. |
| torch.nn.functional.relu | torch.nn.ReLU                     | qint8           | qint8 | | qint8 | Same as input | Fused Conv2d+BN+ReLU operations will be automatically applied. |
| torch.nn.functional.relu6(fused) | torch.nn.ReLU6                   | N/A             | N/A | | qint8 | Same as input | N/A |



### torch.nn Module Class

| Operator | Eager Mode Replacement | Bernoulli2 | Input | Output | Other Constraints | Input (Bayes) | Output (Bayes) | Other Constraints (Bayes) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| torch.nn.AdaptiveAvgPool2d | Not supported | Not supported | Not supported | Not supported | qint8 | Same as input | Converted with AvgPool2d, accuracy issue |  |
| torch.nn.AvgPool2d | qint8 | Same as input | `1<=kernel<=7, 1<=stride<=185` | | | | `1<=kernel, stride, padding<=256`; |
| torch.nn.BatchNorm2d | | | | BN2d absorbed in QAT, not present in prediction models. Limited by compiler, uses BpuConvolution for standalone usage | qint8 | qint8 | BN2d absorbed in QAT, not shown in model. See Conv2d constraints for standalone use | 
| torch.nn.BatchNorm3d | | | | BN3d absorbed in QAT, not present in prediction models. Limited by compiler, uses BpuConvolution for standalone usage | qint8 | qint8 | BN3d absorbed in QAT, not shown in model. See Conv2d constraints for standalone use | 
| torch.nn.ChannelShuffle | qint8 | Same as input | | qint8, qint16 | Same as input | shuffle_index values must be unique |
| torch.nn.ConstantPad2d | | Refer to torch.nn.ZeroPad2d | Refer to torch.nn.ZeroPad2d | | Refer to torch.nn.ZeroPad2d | Refer to torch.nn.ZeroPad2d | Refer to torch.nn.ZeroPad2d | 
| torch.nn.Conv2d | qint8 | qint8, qint32 | | input: qint8, qint16; weight: qint8; bias: qint32 | qint8, qint16, qint32 | `out_channel<=8192, max out_channel for model output: 16384. Input channel<=8192, kernel<32, dilation<=16, stride=1 when dilation!=1. Supports sumin, sumin conv only supports stride (1, 1) or (2, 2). Weight shape: [N, C, H, W], N, C<=8192, H, W<=31. For model output, C<=16384, weight_size < 65535. Padding<=256. qint16 input overflow limits apply.` |
| torch.nn.Conv3d | Not supported | Not supported | Not supported | Not supported | input: qint8, weight: qint8, bias: qint32 | qint8 | `input: [N, C, D, H, W] int8, N<=128; H, W, D, C<=65536; weight: [C_o, C_i, D, H, W] int8, N, C<=65536, D, H<=9, W<=8191; bias: int32; output: [N, C, D, H, W] int8, int16, int32; stride: [D, H, W], D, H, W=1 or 2, same for all; padding: [D, H, W], D<=kernel_d/2, H<=kernel_h/2, W<=kernel_w/2 (kernel_w is the W dimension of weight); group, dilation unsupported` |
| torch.nn.ConvTranspose2d | qint8 | qint8 | `2<=kernel<=14, channel<=2048. Padding H*W=[0, (kernel_h-1)/2] * [0, (kernel_w-1)/2]. 2<=stride<=4, dilation=(1, 1)` | qint8 | qint8 | `Input shape: [N, C, H, W], 1<=N<=128, 1<=channel<=2048; Weight shape: [N, C, H, W], 1<=N, C<=2048, 2<=H, W<=14, weight_size<=65535; kernel>=stride, 1<=stride<=14, 1<=out_channel<=2048, in_channel<=2048, pad<=kernel/stride, 0<=out_pad<=1; Bias int32 type. Supports sumin, sumin input int8 type; 0<=output_padding<=1; Supports group, requires weight_n and input channel divisible by group; dilation=1` |
| torch.nn.Dropout | qint8, qint16, qint32 | Same as input | | qint8, qint16, qint32 | Same as input | |
| torch.nn.Dropout2d | qint8, qint16, qint32 | Same as input | | qint8, qint16, qint32 | Same as input | |
| torch.nn.ELU | Not supported | Not supported | Not supported | Not supported | Refer to torch.acos | Refer to torch.acos | |
| torch.nn.GELU | Refer to torch.exp | Refer to torch.exp | Refer to torch.exp | Refer to torch.acos | Refer to torch.acos | Refer to torch.acos | 
| torch.nn.GLU | Not supported | Not supported | | Refer to torch.acos | Refer to torch.acos | 
| torch.nn.HardSigmoid | Not supported | Not supported | Not supported | Not supported | Refer to torch.acos | Refer to torch.acos | 
| torch.nn.Identity | qint8, qint16, qint32 | Same as input | | qint8, qint16, qint32 | Same as input | |
| torch.nn.LayerNorm | Not supported | Not supported | Not supported | | qint8 | qint8, qint16 | Lower-level implementation uses multiple lookups, higher risk of precision loss. Use rsqrt_kwargs to control internal rsqrt lookup parameters. `H * W <= 16384`, normalized_shape `H * W < 16384` |
| torch.nn.LeakyReLU | Not supported | Not supported | Not supported | Not supported | Refer to torch.acos | Refer to torch.acos | 
| torch.nn.Linear | Not supported | Not supported | Not supported | Not supported | input: qint8; weight: qint8; bias: qint32 | qint8 | `in_features <= 8192, out_features <= 8192.` |
| torch.nn.LSTMCell | Not supported | Not supported | Not supported | Not supported | qint8, qint16 | qint8, qint16 | Input is 2-dimensional |
| torch.nn.MaxPool2d | qint8 | Same as input | `1<=kernel<=64, 1<=stride<=256, padding>=0` | qint8 | Same as input | `Input_shape: [N, C, H, W], 1<=H, W, C<=8192; 1<=kernel, stride<=256; 0<=padding<=255;` |
| torch.nn.MultiheadAttention | Not supported | Not supported | Not supported | Not supported | qint8, qint16 | qint8, qint16 | Unsupported: add_bias_kv, add_zero_attn, qkv embed_dim inconsistencies. Supports int8/int16 inputs, potential precision risks from table lookups and masking | 
| torch.nn.PixelShuffle | qint8, qint16 | Same as input | | qint8, qint16 | Same as input | |
| torch.nn.PixelUnshuffle | qint8, qint16 | Same as input | | qint8, qint16 | Same as input | |
| torch.nn.PReLU | Not supported | Not supported | Not supported | Not supported | Refer to torch.acos | Refer to torch.acos | 
| torch.nn.ReLU | qint8 | Same as input | | qint8, qint16 | Same as input | |
| torch.nn.ReLU6 | qint8 | Same as input | | qint8, qint16 | Same as input | |
| torch.nn.ReplicationPad2d | Refer to torch.nn.ZeroPad2d | Refer to torch.nn.ZeroPad2d | Refer to torch.nn.ZeroPad2d | Refer to torch.nn.ZeroPad2d | Refer to torch.nn.ZeroPad2d | Refer to torch.nn.ZeroPad2d | 
| torch.nn.Sigmoid | Refer to torch.exp | Refer to torch.exp | Refer to torch.exp | Refer to torch.acos | Refer to torch.acos | Refer to torch.acos | 
| torch.nn.SiLU | Refer to torch.exp | Refer to torch.exp | Refer to torch.exp | Refer to torch.acos | Refer to torch.acos | Refer to torch.acos | 
| torch.nn.Softmax | Not supported | Not supported | Not supported | qint8 | qint8, qint16 | Multiple lookups and summations involved, high precision risk | 
| torch.nn.Softplus | Not supported | Not supported | Not supported | Refer to torch.acos | Refer to torch.acos | 
| torch.nn.SyncBatchNorm | qint8 | qint8 | Uses torch.nn.Conv2d composition | qint8 | qint



| Operator        | Eager Mode Replacement Operator | Bernoulli2 |      | Constraints | Bayes        |          |                  |
|-----------------|--------------------------------|------------|-------|-------------|---------------|----------|------------------|
| torch.quantization.DeQuantStub | | qint8, qint16, qint32 | float32 | Common Use: Segmented network models, dequantizing data from BPU to CPU for CPU processing convenience. | qint8, qint16, qint32 | float32 | Same as above |
| torch.quantization.QuantStub | horizon.quantization.QuantStub | float32 | qint8, qint16 | Common Use: Model inputs, or before data is quantized from CPU to BPU in segmented models. Scale parameter setup: Set based on input data, aiming for high precision quantization of float data to int8. For example, if input float range is (-1, 1), use scale = 1 / 128. Pre-trained float models: In pre-trained models, use a special conv layer to handle scale settings, as the model may not follow this method. Requires uniform input distribution for QuantStub. | float32 | qint8, qint16 | Same as above with additional note about pre-trained models. |

### torch.Tensor method Class

| Operator | Eager Mode Replacement | Bernoulli2 | | | Bayes | | |
| --- | --- | --- | --- | --- | --- | --- | --- |
| torch.Tensor.__getitem__ | | qint8, qint16, qint32 | Same as input | | | | |
| torch.Tensor.transpose | | Not supported | Not supported | Not supported | qint8, qint16, qint32 | Tensor.dtype | Not supported for N-dimensional transposes |
| torch.Tensor.argmax | | Refer to torch.max | Refer to torch.max | Refer to torch.max | Refer to torch.max | Refer to torch.max | Refer to torch.max |
| torch.Tensor.argmin | | Refer to torch.max | Refer to torch.max | Refer to torch.max | Refer to torch.max | Refer to torch.max | Refer to torch.max |
| torch.Tensor.clamp | | Not supported | Not supported | Not supported | qint8, qint16 | Tensor.dtype | `dim <= 10, 1 <= each_dim_size < 65536` |
| torch.Tensor.clip | | Not supported | Not supported | Not supported | Refer to torch.Tensor.clip | Refer to torch.Tensor.clip | Refer to torch.Tensor.clip |
| torch.Tensor.eq | | Not supported | Not supported | Not supported | Refer to torch.eq | Refer to torch.eq | Refer to torch.eq |
| torch.Tensor.expand | | Not supported | Not supported | Not supported | qint8, qint16 | Tensor.dtype | |
| torch.Tensor.ge | | Not supported | Not supported | Not supported | Refer to torch.eq | Refer to torch.eq | Refer to torch.eq |
| torch.Tensor.greater | | Not supported | Not supported | Not supported | Refer to torch.eq | Refer to torch.eq | Refer to torch.eq |
| torch.Tensor.greater_equal | | Not supported | Not supported | Not supported | Refer to torch.eq | Refer to torch.eq | Refer to torch.eq |
| torch.Tensor.gt | | Not supported | Not supported | Not supported | Refer to torch.eq | Refer to torch.eq | Refer to torch.eq |
| torch.Tensor.le | | Not supported | Not supported | Not supported | Refer to torch.eq | Refer to torch.eq | Refer to torch.eq |
| torch.Tensor.less | | Not supported | Not supported | Not supported | Refer to torch.eq | Refer to torch.eq | Refer to torch.eq |
| torch.Tensor.less_equal | | Not supported | Not supported | Not supported | Refer to torch.eq | Refer to torch.eq | Refer to torch.eq |
| torch.Tensor.max | | Not supported | Not supported | Not supported | Refer to torch.max | Refer to torch.max | Refer to torch.max |
| torch.Tensor.min | | Not supported | Not supported | Not supported | Refer to torch.max | | |
| torch.Tensor.repeat | | Not supported | Not supported | Not supported | qint8, qint16 | Tensor.dtype | |
| torch.Tensor.reshape | | Not supported | Not supported | Not supported | | Tensor.dtype | |
| torch.Tensor.tile | | Not supported | Not supported | Not supported | qint8, qint16 | Tensor.dtype | |
| torch.Tensor.abs | | Not supported | Not supported | Not supported | qint8, qint16 | Tensor.dtype | |



### torchvision Operations

| Operator | Eager Mode Replacement | Bernoulli2 | Notes | Input | Bayesian | Output | Additional Constraints |
| --- | --- | --- | --- | --- | --- | --- | --- |
| torchvision.models.detection.rpn.AnchorGenerator | horizon.nn.AnchorGenerator | qint8, qint16, qint32, float32 | Supports Tensor.shape determinable offline | qint8, qint16, qint32, float32 | float32 | float32 | Input: int8/int16/int32/float32, Output: float32 |
| torchvision.ops.MultiScaleRoIAlign | horizon.nn.MultiScaleRoIAlign | Refer to torchvision.ops.RoIAlign | Refer to torchvision.ops.RoIAlign | Refer to torchvision.ops.RoIAlign | Refer to torchvision.ops.RoIAlign | Refer to torchvision.ops.RoIAlign | Refer to torchvision.ops.RoIAlign |
| torchvision.ops.RoIAlign | | qint8 | qint8 | | qint8 | qint8 | `1 <= feature number <= 5; Bboxes only support List[Tensor] format with shape [1, box_num, 4], where the last dimension represents [left, top, right, bottom].` |


---
sidebar_position: 2
---
# Quick Start {#quick_start}

The D-Robotics Plugin Pytorch (referred to as Plugin) refers to the PyTorch official quantization interface and ideas. The Plugin adopts the Quantization Aware Training (QAT) scheme. Therefore, it is recommended that users first read the relevant parts of the [**PyTorch official documentation**](https://pytorch.org/docs/stable/quantization.html#quantization) and be familiar with the usage of quantization training and deployment tools provided by PyTorch.

## Basic Process

The basic process of using the quantization training tool is as follows:

![quick_start](./image/expert/quick_start.svg)

Below, we take the `MobileNetV2` model in `torchvision` as an example to introduce the specific operations of each stage in the process.

For the sake of execution speed in the process demonstration, we use the `cifar-10` dataset instead of the ImageNet-1K dataset.


```python

    import os
    import copy
    import numpy as np
    import torch
    import torch.nn as nn
    import torchvision.transforms as transforms
    from torch import Tensor
    from torch.quantization import DeQuantStub
    from torchvision.datasets import CIFAR10
    from torchvision.models.mobilenetv2 import MobileNetV2
    from torch.utils import data
    from typing import Optional, Callable, List, Tuple

    from horizon_plugin_pytorch.functional import rgb2centered_yuv

    import torch.quantization
    from horizon_plugin_pytorch.march import March, set_march
    from horizon_plugin_pytorch.quantization import (
        QuantStub,
        convert_fx,
        prepare_qat_fx,
        set_fake_quantize,
        FakeQuantState,
        check_model,
        compile_model,
        perf_model,
        visualize_model,
    )
    from horizon_plugin_pytorch.quantization.qconfig import (
        default_calib_8bit_fake_quant_qconfig,
        default_qat_8bit_fake_quant_qconfig,
        default_qat_8bit_weight_32bit_out_fake_quant_qconfig,
        default_calib_8bit_weight_32bit_out_fake_quant_qconfig,
    )

    import logging
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
```

```shell

    2023-06-29 14:46:09,502] WARNING: `fx_force_duplicate_shared_convbn` will be set False by default after plugin 1.9.0. If you are not loading old checkpoint, please set `fx_force_duplicate_shared_convbn` False to train your new model.
    2023-06-29 14:46:09,575] WARNING: due to bug of torch.quantization.fuse_modules, it is replaced by horizon.quantization.fuse_modules

```

## Get Float Model

First, make necessary modifications to the float model to support quantization-related operations. The necessary operations for modifying the model are:

- Insert a ``QuantStub`` before the model input.

- Insert a ``DequantStub`` after the model output.

When modifying the model, please note:

- The inserted ``QuantStub`` and ``DequantStub`` must be registered as sub-modules of the model, otherwise their quantization states will not be handled correctly.

- Multiple inputs can share the same ``QuantStub`` only when their scales are the same. Otherwise, please define a separate ``QuantStub`` for each input.

- If you need to specify the data source of the input data when on board as "pyramid", please manually set the ``scale`` parameter of the corresponding ``QuantStub`` to 1/128.

- You can also use ``torch.quantization.QuantStub``, but only ``horizon_plugin_pytorch.quantization.QuantStub`` supports manually fixing the scale through parameters.

The modified model can seamlessly load the parameters of the original model, so if there is an already trained float model, you can load it directly. Otherwise, you need to train it normally in float.

:::caution Note

  The input image data when the model is on board is generally in the format of centered_yuv444. Therefore, when training the model, the image needs to be converted to the centered_yuv444 format (pay attention to the use of ``rgb2centered_yuv`` in the code below).
  If it is not possible to convert to centered_yuv444 format for model training, please refer to the **RGB888 Data Deployment** section for instructions on how to modify the model accordingly. (Note that this method may lead to a decrease in model accuracy)
  In this example, the number of epochs for float and QAT training is small, just to illustrate the usage of the training tool, and the accuracy does not represent the best performance of the model.
:::

```python

    ######################################################################
    # Users can modify the following parameters as needed
    # 1. Paths for saving model checkpoints and compilation artifacts
    model_path = "model/mobilenetv2"# 2. Data set download and save path
    data_path = "data"
    # 3. Batch size used during training
    train_batch_size = 256
    # 4. Batch size used during prediction
    eval_batch_size = 256
    # 5. Number of epochs for training
    epoch_num = 30
    # 6. Device used for model saving and computation
    device = (
        torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
    )
    ######################################################################


    # Prepare data loaders, please note the use of rgb2centered_yuv in collate_fn
    def prepare_data_loaders(
        data_path: str, train_batch_size: int, eval_batch_size: int
    ) -> Tuple[data.DataLoader, data.DataLoader]:
        normalize = transforms.Normalize(mean=0.0, std=128.0)

        def collate_fn(batch):
            batched_img = torch.stack(
                [
                    torch.from_numpy(np.array(example[0], np.uint8, copy=True))
                    for example in batch
                ]
            ).permute(0, 3, 1, 2)
            batched_target = torch.tensor([example[1] for example in batch])

            batched_img = rgb2centered_yuv(batched_img)
            batched_img = normalize(batched_img.float())

            return batched_img, batched_target

        train_dataset = CIFAR10(
            data_path,
            True,
            transforms.Compose(
                [
                    transforms.RandomHorizontalFlip(),
                    transforms.RandAugment(),
                ]
            ),
            download=True,
        )

        eval_dataset = CIFAR10(
            data_path,
            False,        download=True,
    )

    train_data_loader = data.DataLoader(
        train_dataset,
        batch_size=train_batch_size,
        sampler=data.RandomSampler(train_dataset),
        num_workers=8,
        collate_fn=collate_fn,
        pin_memory=True,
    )

    eval_data_loader = data.DataLoader(
        eval_dataset,
        batch_size=eval_batch_size,
        sampler=data.SequentialSampler(eval_dataset),
        num_workers=8,
        collate_fn=collate_fn,
        pin_memory=True,
    )

    return train_data_loader, eval_data_loader


# Make necessary modifications to the floating-point model
class FxQATReadyMobileNetV2(MobileNetV2):
    def __init__(
        self,
        num_classes: int = 10,
        width_mult: float = 1.0,
        inverted_residual_setting: Optional[List[List[int]]] = None,
        round_nearest: int = 8,
    ):
        super().__init__(
            num_classes, width_mult, inverted_residual_setting, round_nearest
        )
        self.quant = QuantStub(scale=1 / 128)
        self.dequant = DeQuantStub()

    def forward(self, x: Tensor) -> Tensor:
        x = self.quant(x)
        x = super().forward(x)
        x = self.dequant(x)

        return x


if not os.path.exists(model_path):
    os.makedirs(model_path, exist_ok=True)# Float model initialization
    float_model = FxQATReadyMobileNetV2()

    # Prepare dataset
    train_data_loader, eval_data_loader = prepare_data_loaders(
        data_path, train_batch_size, eval_batch_size
    )

    # Since the last layer of the model is inconsistent with the pretrained model, float finetuning is needed
    optimizer = torch.optim.Adam(
        float_model.parameters(), lr=0.001, weight_decay=1e-3
    )
    best_acc = 0

    for nepoch in range(epoch_num):
        float_model.train()
        train_one_epoch(
            float_model,
            nn.CrossEntropyLoss(),
            optimizer,
            None,
            train_data_loader,
            device,
        )

        # Float precision testing
        float_model.eval()
        top1, top5 = evaluate(float_model, eval_data_loader, device)

        print(
            "Float Epoch {}: evaluation Acc@1 {:.3f} Acc@5 {:.3f}".format(
                nepoch, top1.avg, top5.avg
            )
        )

        if top1.avg > best_acc:
            best_acc = top1.avg
            # Save the best parameters of the float model
            torch.save(
                float_model.state_dict(),
                os.path.join(model_path, "float-checkpoint.ckpt"),
            )

```

```shell

    Files already downloaded and verified
    Files already downloaded and verified
    ....................................................................................................................................................................................................Full cifar-10 train set: Loss 2.116 Acc@1 20.744 Acc@5 70.668
    ........................................
    Float Epoch 0: evaluation Acc@1 34.140 Acc@5 87.330
    ....................................................................................................................................................................................................
    Full cifar-10 train set: Loss 1.815 Acc@1 32.464 Acc@5 84.110
    ........................................
    Float Epoch 1: evaluation Acc@1 42.770 Acc@5 90.560
    ....................................................................................................................................................................................................
    Full cifar-10 train set: Loss 1.682 Acc@1 38.276 Acc@5 87.374
    ........................................
    Float Epoch 2: evaluation Acc@1 45.810 Acc@5 91.240
    ....................................................................................................................................................................................................
    Full cifar-10 train set: Loss 1.581 Acc@1 42.676 Acc@5 89.224
    ........................................
    Float Epoch 3: evaluation Acc@1 50.070 Acc@5 92.620
    ....................................................................................................................................................................................................
    Full cifar-10 train set: Loss 1.495 Acc@1 45.882 Acc@5 90.668
    ........................................
    Float Epoch 4: evaluation Acc@1 53.860 Acc@5 93.690
    ....................................................................................................................................................................................................
    Full cifar-10 train set: Loss 1.413 Acc@1 49.274 Acc@5 91.892
    ........................................
    Float Epoch 5: evaluation Acc@1 51.230 Acc@5 94.370
    ....................................................................................................................................................................................................
    Full cifar-10 train set: Loss 1.339 Acc@1 52.488 Acc@5 92.760
    ........................................
    Float Epoch 6: evaluation Acc@1 58.460 Acc@5 95.450
    ....................................................................................................................................................................................................
    Full cifar-10 train set: Loss 1.269 Acc@1 54.710 Acc@5 93.702
    ........................................
    Float Epoch 7: evaluation Acc@1 59.870 Acc@5 95.260
    ....................................................................................................................................................................................................
    Full cifar-10 train set: Loss 1.208 Acc@1 57.170 Acc@5 94.258
    ........................................
    Float Epoch 8: evaluation Acc@1 60.040 Acc@5 95.870
    ....................................................................................................................................................................................................
    Full cifar-10 train set: Loss 1.147 Acc@1 59.420 Acc@5 95.150
    ........................................
    Float Epoch 9: evaluation Acc@1 61.370 Acc@5 96.830
    ....................................................................................................................................................................................................
    Full cifar-10 train set: Loss 1.098 Acc@1 61.652 Acc@5 95.292
    ........................................
    Float Epoch 10: evaluation Acc@1 66.410 Acc@5 96.910
    ....................................................................................................................................................................................................
    Full cifar-10 train set: Loss 1.060 Acc@1 62.902 Acc@5 95.758
    ........................................
    Float Epoch 11: evaluation Acc@1 67.900 Acc@5 96.660
    ....................................................................................................................................................................................................
    Full cifar-10 train set: Loss 1.013 Acc@1 64.606 Acc@5 96.250
    ........................................Float Epoch 12: evaluation Acc@1 69.120 Acc@5 97.180
    ....................................................................................................................................................................................................
    Full cifar-10 train set: Loss 0.980 Acc@1 65.954 Acc@5 96.486
    ........................................
    Float Epoch 13: evaluation Acc@1 70.410 Acc@5 97.420
    ....................................................................................................................................................................................................
    Full cifar-10 train set: Loss 0.944 Acc@1 67.002 Acc@5 96.792
    ........................................
    Float Epoch 14: evaluation Acc@1 71.200 Acc@5 97.410
    ....................................................................................................................................................................................................
    Full cifar-10 train set: Loss 0.915 Acc@1 68.024 Acc@5 96.896
    ........................................
    Float Epoch 15: evaluation Acc@1 72.570 Acc@5 97.780
    ....................................................................................................................................................................................................
    Full cifar-10 train set: Loss 0.892 Acc@1 69.072 Acc@5 97.062
    ........................................
    Float Epoch 16: evaluation Acc@1 72.950 Acc@5 98.020
    ....................................................................................................................................................................................................
    Full cifar-10 train set: Loss 0.868 Acc@1 70.072 Acc@5 97.234
    ........................................
    Float Epoch 17: evaluation Acc@1 75.020 Acc@5 98.230
    ....................................................................................................................................................................................................
    Full cifar-10 train set: Loss 0.850 Acc@1 70.544 Acc@5 97.384
    ........................................
    Float Epoch 18: evaluation Acc@1 74.870 Acc@5 98.140
    ....................................................................................................................................................................................................
    Full cifar-10 train set: Loss 0.826 Acc@1 71.334 Acc@5 97.476
    ........................................
    Float Epoch 19: evaluation Acc@1 74.700 Acc@5 98.090
    ....................................................................................................................................................................................................
    Full cifar-10 train set: Loss 0.817 Acc@1 71.988 Acc@5 97.548
    ........................................
    Float Epoch 20: evaluation Acc@1 75.690 Acc@5 98.140
    ....................................................................................................................................................................................................
    Full cifar-10 train set: Loss 0.796 Acc@1 72.530 Acc@5 97.734
    ........................................
    Float Epoch 21: evaluation Acc@1 76.500 Acc@5 98.470
..............................................................
    Full cifar-10 train set: Loss 0.786 Acc@1 72.754 Acc@5 97.770
........................................
Float Epoch 22: evaluation Acc@1 76.200 Acc@5 98.290
    ....................................................................................................................................................................................................
    Full cifar-10 train set: Loss 0.772 Acc@1 73.392 Acc@5 97.802
........................................
Float Epoch 23: evaluation Acc@1 74.800 Acc@5 98.640
    ....................................................................................................................................................................................................
    Full cifar-10 train set: Loss 0.753 Acc@1 73.982 Acc@5 97.914
........................................
Float Epoch 24: evaluation Acc@1 77.150 Acc@5 98.490
    ....................................................................................................................................................................................................Full cifar-10 train set: Loss 0.741 Acc@1 74.278 Acc@5 98.038
........................................
Float Epoch 25: evaluation Acc@1 77.270 Acc@5 98.690
....................................................................................................................................................................................................
Full cifar-10 train set: Loss 0.737 Acc@1 74.582 Acc@5 97.916
........................................
Float Epoch 26: evaluation Acc@1 77.050 Acc@5 98.580
....................................................................................................................................................................................................
Full cifar-10 train set: Loss 0.725 Acc@1 75.254 Acc@5 98.038
........................................
Float Epoch 27: evaluation Acc@1 79.120 Acc@5 98.620
....................................................................................................................................................................................................
Full cifar-10 train set: Loss 0.714 Acc@1 75.290 Acc@5 98.230
........................................
Float Epoch 28: evaluation Acc@1 78.060 Acc@5 98.550
....................................................................................................................................................................................................
Full cifar-10 train set: Loss 0.711 Acc@1 75.662 Acc@5 98.218
........................................
Float Epoch 29: evaluation Acc@1 77.580 Acc@5 98.610
```

## Calibration{#Calibration}

After the model transformation is complete and the floating-point training is completed, calibration can be performed. In this process, the distribution of data is calculated by inserting observers into the model during the forward process, in order to calculate the appropriate quantization parameters:

- For some models, only calibration is necessary to achieve the required accuracy, and there is no need to perform time-consuming quantization-aware training.

- Even if the calibrated model does not meet the precision requirements, this process can reduce the difficulty of subsequent quantization-aware training, shorten the training time, and improve the final training accuracy.

```python

    ######################################################################
    # Users can modify the following parameters as needed
    # 1. Batch size used during calibration
    calib_batch_size = 256
    # 2. Batch size used during validation
    eval_batch_size = 256
    # 3. Number of examples used during calibration, set to inf to use all data
    num_examples = float("inf")
    # 4. The code for the target hardware platform
    march = March.BAYES
    ######################################################################

    # Set the hardware platform that the model will run on before the model transformation
    set_march(march)


    # Convert the model to calibration state to calculate the numerical distribution characteristics of data at each location
    calib_model = prepare_qat_fx(# The output model will share the attributes of the input model to avoid affecting the subsequent use of float_model. A 'deepcopy' is performed here.

    copy.deepcopy(float_model),
    {
        "": default_calib_8bit_fake_quant_qconfig,
        "module_name": {
            # When the output layer of the model is Conv or Linear, 'out_qconfig' can be used to configure high-precision output.
            "classifier": default_calib_8bit_weight_32bit_out_fake_quant_qconfig,
        },
    },
    ).to(
        device
    )  # The 'prepare_qat_fx' interface does not guarantee that the output model's device is completely consistent with the input model.

    # Prepare the dataset
    calib_data_loader, eval_data_loader = prepare_data_loaders(
        data_path, calib_batch_size, eval_batch_size
    )

    # Perform the Calibration process (no need for backward)
    # Note the control over the model's state, the model needs to be in 'eval' state to ensure that the behavior of BatchNorm matches the requirements.
    calib_model.eval()
    set_fake_quantize(calib_model, FakeQuantState.CALIBRATION)
    with torch.no_grad():
        cnt = 0
        for image, target in calib_data_loader:
            image, target = image.to(device), target.to(device)
            calib_model(image)
            print(".", end="", flush=True)
            cnt += image.size(0)
            if cnt >= num_examples:
                break
        print()

    # Test the precision of fake quantization
    # Note the control over the model's state
    calib_model.eval()
    set_fake_quantize(calib_model, FakeQuantState.VALIDATION)

    top1, top5 = evaluate(
        calib_model,
        eval_data_loader,
        device,
    )
    print(
        "Calibration: evaluation Acc@1 {:.3f} Acc@5 {:.3f}".format(
            top1.avg, top5.avg
        )
    )# Save Calibration Model Parameters
    torch.save(
        calib_model.state_dict(),
        os.path.join(model_path, "calib-checkpoint.ckpt"),
    )

```

```shell
    Files already downloaded and verified
    Files already downloaded and verified

    /home/users/yushu.gao/horizon/qat_logger/horizon_plugin_pytorch/quantization/observer_v2.py:405: UserWarning: _aminmax is deprecated as of PyTorch 1.11 and will be removed in a future release. Use aminmax instead. This warning will only appear once per process. (Triggered internally at ../aten/src/ATen/native/TensorCompare.cpp:568.)
    min_val_cur, max_val_cur = torch._aminmax(
    /home/users/yushu.gao/horizon/qat_logger/horizon_plugin_pytorch/quantization/observer_v2.py:672: UserWarning: _aminmax is deprecated as of PyTorch 1.11 and will be removed in a future release. Use aminmax instead. This warning will only appear once per process. (Triggered internally at ../aten/src/ATen/native/ReduceAllOps.cpp:45.)
    min_val_cur, max_val_cur = torch._aminmax(x)

    ....................................................................................................................................................................................................
    ........................................
    Calibration: evaluation Acc@1 77.890 Acc@5 98.640

```

If the quantization accuracy of the model after calibration already meets the requirements, you can directly proceed to the next step of **converting the model to fixed point**, otherwise, you need to perform **quantization-aware training** to further improve the accuracy.


## Quantization-Aware Training

Quantization-aware training inserts fake quantization nodes into the model to make the model aware of the impact of quantization during the training process. In this case, the model parameters are fine-tuned to improve the accuracy after quantization.

```python

    ######################################################################
    # The following parameters can be modified according to your needs
    # 1. Batch size used during training
    train_batch_size = 256
    # 2. Batch size used during validation
    eval_batch_size = 256
    # 3. Number of epochs for training
    epoch_num = 3
    ######################################################################

    # Prepare the data loaders
    train_data_loader, eval_data_loader = prepare_data_loaders(
        data_path, train_batch_size, eval_batch_size
    )

    # Convert the model to QAT state
    qat_model = prepare_qat_fx(
    copy.deepcopy(float_model),
    {
        "": default_qat_8bit_fake_quant_qconfig,
        "module_name": {
            "classifier": default_qat_8bit_weight_32bit_out_fake_quant_qconfig,
        },
    },
    ).to(device)

    # Load quantization parameters from calibration model
    qat_model.load_state_dict(calib_model.state_dict())

    # Perform Quantization-Aware Training (QAT)
    # As a fine-tuning process, QAT generally requires a smaller learning rate
    optimizer = torch.optim.Adam(
        qat_model.parameters(), lr=1e-3, weight_decay=1e-4
    )

    best_acc = 0

    for nepoch in range(epoch_num):
        # Control QAT model's training mode
        qat_model.train()
        set_fake_quantize(qat_model, FakeQuantState.QAT)

        train_one_epoch(
            qat_model,
            nn.CrossEntropyLoss(),
            optimizer,
            None,
            train_data_loader,
            device,
        )

        # Control QAT model's evaluation mode
        qat_model.eval()
        set_fake_quantize(qat_model, FakeQuantState.VALIDATION)

        top1, top5 = evaluate(
            qat_model,
            eval_data_loader,
            device,
        )
        print(
            "QAT Epoch {}: evaluation Acc@1 {:.3f} Acc@5 {:.3f}".format(
                nepoch, top1.avg, top5.avg
            )
        )

        if top1.avg > best_acc:best_acc = top1.avg

    torch.save(
        qat_model.state_dict(),
        os.path.join(model_path, "qat-checkpoint.ckpt"),
    )

```

```shell

    Files already downloaded and verified
    Files already downloaded and verified
    ...............................................................
    Full cifar-10 train set: Loss 0.759 Acc@1 73.692 Acc@5 97.940
    ........................................
    QAT Epoch 0: evaluation Acc@1 79.170 Acc@5 98.490
    ...............................................................
    Full cifar-10 train set: Loss 0.718 Acc@1 75.414 Acc@5 97.998
    ........................................
    QAT Epoch 1: evaluation Acc@1 78.540 Acc@5 98.580
    ...............................................................
    Full cifar-10 train set: Loss 0.719 Acc@1 75.180 Acc@5 98.126
    ........................................
    QAT Epoch 2: evaluation Acc@1 78.200 Acc@5 98.540

```

## Convert to Fixed-Point Model

After the accuracy of the quantization model meets the requirement, the model can be converted to a fixed-point model. It is generally believed that the results of the fixed-point model and the compiled model are exactly the same.

:::caution Note

Fixed-point models and quantized models cannot achieve exact numerical consistency, so please refer to the accuracy of the fixed-point model. If the fixed-point accuracy does not meet the requirements, further quantization training is needed.
:::

```python

    ######################################################################
    # Users can modify the following parameters as needed
    # 1. Use which model as the input of the process, you can choose calib_model or qat_model
    base_model = qat_model
    ######################################################################

    # Convert the model to fixed-point mode
    quantized_model = convert_fx(base_model).to(device)

    # Test the accuracy of the fixed-point model
    top1, top5 = evaluate(
    quantized_model, 
    eval_data_loader, 
    device, 
    ) 
    print(
        "Quantized model: evaluation Acc@1 {:.3f} Acc@5 {:.3f}".format(
            top1.avg, top5.avg
        )
    )
```



```shell
[2023-06-29 14:55:05,825] WARNING: AdaptiveAvgPool2d has not collected any statistics of activations and its scale is 1, please check whether this is intended!

........................................
Quantized model: evaluation Accuracy@1 78.390, Accuracy@5 98.640
```
## Model Deployment

After verifying the quantized model's accuracy and ensuring it meets requirements, proceed with deployment steps, including model inspection, compilation, performance testing, and visualization.

:::caution Note

- If desired, you can skip the actual calibration and quantization-aware training processes and directly proceed with model inspection to ensure there are no issues preventing successful compilation.
- Since the compiler only supports CPUs, both the model and data must be on the CPU.
:::

```python
######################################################################
# Users can modify these parameters as needed
# 1. Optimization level for compilation, can be 0-3; higher levels result in
#    faster execution on the board but slower compilation times.
compile_opt = "O1"
######################################################################

# `example_input` can also be randomly generated data, but using real data is recommended
# for more accurate performance testing
example_input = next(iter(eval_data_loader))[0]

# Trace the model, serialize it, and generate the computation graph. Ensure the model and data are on CPU
script_model = torch.jit.trace(quantized_model.cpu(), example_input)
torch.jit.save(script_model, os.path.join(model_path, "int_model.pt"))

# Model inspection
check_model(script_model, [example_input])
```

```shell
/home/users/yushu.gao/horizon/qat_logger/horizon_plugin_pytorch/qtensor.py:992: TracerWarning: Converting a tensor to a Python boolean might cause the trace to be incorrect. We can't record the data flow of Python values, so this value will be treated as a constant in the future. This means that the trace might not generalize to other inputs!
if scale is not None and scale.numel() > 1:
/home/users/yushu.gao/horizon/qat_logger/horizon_plugin_pytorch/nn/quantized/conv2d.py:290: TracerWarning: Converting a tensor to a Python boolean might cause the trace to be incorrect. We can't record the data flow of Python values, so this value will be treated as a constant in the future. This means that the trace might not generalize to other inputs!
per_channel_axis=-1 if self.out_scale.numel() == 1 else 1,
/home/users/yushu.gao/horizon/qat_logger/horizon_plugin_pytorch/nn/quantized/adaptive_avg_pool2d.py:30: TracerWarning: Converting a tensor to a Python boolean might cause the trace to be incorrect. We can't record the data flow of Python values, so this value will be treated as a constant in the future. This means that the trace might not generalize to other inputs!
if (
/home/users/yushu.gao/horizon/qat_logger/horizon_plugin_pytorch/utils/script_quantized_fn.py:224: UserWarning: operator() profile_node %59 : int[] = prim::profile_ivalue(%57)
does not have profile information (Triggered internally at ../torch/csrc/jit/codegen/cuda/graph_fuser.cpp:105.)
return compiled_fn(*args, **kwargs)
```

```shell
This model is supported!
HBDK model check PASS
```

```python

# Compile the model, the generated hbm file is the deployable model
compile_model(
    script_model,
    [example_input],
    hbm=os.path.join(model_path, "model.hbm"),
    input_source="pyramid",
    opt=compile_opt,
)
```

```shell

INFO: launch 16 threads for optimization
[==================================================] 100%
WARNING: arg0 can not be assigned to NCHW_NATIVE layout because it's input source is pyramid/resizer.
consumed time 10.4302
HBDK model compilation SUCCESS
```

```python

# Model performance testing
perf_model(
    script_model,
[example_input],
out_dir=os.path.join(model_path, "perf_out"),
input_source="pyramid",
opt=compile_opt,
layer_details=True,
)

```

```shell

INFO: launch 16 threads for optimization
[==================================================] 100%
WARNING: arg0 can not be assigned to NCHW_NATIVE layout because it's input source is pyramid/resizer.
```

```shell

consumed time 10.3666
HBDK model compilation SUCCESS
FPS=5722.98, latency = 44731.9 us   (see model/mobilenetv2/perf_out/FxQATReadyMobileNetV2.html)
HBDK model compilation SUCCESS
HBDK performance estimation SUCCESS

{'summary': {'BPU OPs per frame (effective)': 12249856,
    'BPU OPs per run (effective)': 3135963136,
    'BPU PE number': 1,
    'BPU core number': 1,
    'BPU march': 'BAYES',
    'DDR bytes per frame': 1403592.0,
    'DDR bytes per run': 359319552,
    'DDR bytes per second': 8032734694,
    'DDR megabytes per frame': 1.339,
    'DDR megabytes per run': 342.674,
    'DDR megabytes per second': 7660.6,
    'FPS': 5722.98,
    'HBDK version': '3.46.4',
    'compiling options': '--march bayes -m /tmp/hbdktmp_ocro1_9_.hbir -f hbir --O1 -o /tmp/hbdktmp_ocro1_9_.hbm --jobs 16 -n FxQATReadyMobileNetV2 -i pyramid --input-name arg0 --output-layout NCHW --progressbar --debug',
    'frame per run': 256,
    'frame per second': 5722.98,
    'input features': [['input name', 'input size'], ['arg0', '256x32x32x3']],
    'interval computing unit utilization': [0.081,
    0.113,
    0.021,
    0.001,
    0.063,
    0.004,
    0.092,
    0.019,
    0.001,
    0.065,
    0.078,
    0.11,
    0.108,
    0.235,
    0.078,
    0.179,
    0.246,
    0.219,
    0.154,
    0.046,
    0.16,
    0.108,
    0.064,
    0.099,
    0.113,
    0.153,
    0.046,
    0.052,
    0.075,
    0.041,
    0.077,
    0.081,
    0.081,
    0.06,
    0.1,
    0.304,
    0.603,
    0.521,
    0.104,
    0.11],
    'interval computing units utilization': [0.081,
    0.113,
    0.021,
    0.001,
    0.063,
    0.004,
    0.092,
    0.019,
    0.001,
    0.016,
    0.053,
    0.021,
    0.001,
    0.0930.065,
    0.078,
    0.11,
    0.108,
    0.235,
    0.078,
    0.179,
    0.246,
    0.219,
    0.154,
    0.046,
    0.16,
    0.108,
    0.064,
    0.099,
    0.113,
    0.153,
    0.046,
    0.052,
    0.075,
    0.041,
    0.077,
    0.081,
    0.081,
    0.06,
    0.1,
    0.304,
    0.603,
    0.521,
    0.104,
    0.11],
    'interval loading bandwidth (megabytes/s)': [798,
    2190,
    3291,
    3001,
    4527,
    6356,
    5985,
    4096,
    3098,
    5315,
    5907,
    3763,
    2887,
    4891,
    6121,
    4107,
    2900,
    1686,
    3146,
    3855,4372,
    2714,
    2180,
    2074,
    2516,
    3674,
    4533,
    3849,
    4317,
    3738,
    3378,
    3901,
    3068,
    4697,
    6180,
    3583,
    3760,
    6467,
    3897,
    3404,
    5554,
    4941,
    2143,
    0,
    2572,
    5019],
    'interval number': 45,
    'interval storing bandwidth (megabytes/s)': [4000,
    3368,
    4334,
    6936,
    5824,
    3695,
    2524,
    3720,
    6066,
    4863,
    2938,
    3924,
    6061,
    4752,
    2250,
    2238,
    3000,
    4500,
    3000,
    1500,
    3000,
    3000,
    3000,
    3041,
    4458,
    3617,
    3295,
    3841,
    3495,
    4500,
    3927,
    4839,
    5822,
    3302,
    3749,
    6609,
    3749,
    3177,
    5876,
    4570,
    2255,
    2187,
    3430,
    2812,
    942],
    'interval time (ms)': 1.0,
    'latency (ms)': 44.73,
    'latency (ms) by segments': [44.732],
    'latency (us)': 44731.9,
    'layer details': [['layer',
        'ops',
        'computing cost (no DDR)',
        'load/store cost'],
    ['_features_0_0_hz_conv2d',
        '113,246,208',
        '29 us (0% of model)',
        '267 us (0.5% of model)'],
    ['_features_1_conv_0_0_hz_conv2d',
        '37,748,736',
        '42 us (0% of model)',
        '1156 us (2.5% of model)'],
    ['_features_1_conv_1_hz_conv2d',
        '67,108,864',
        '20 us (0% of model)',
        '1 us (0% of model)'],
    ['_features_2_conv_0_0_hz_conv2d',
        '201,326,592',
        '44 us (0% of model)',
        '3132 us (7.0% of model)'],
    ['_features_2_conv_1_0_hz_conv2d',
        '28,311,552',
        '54 us (0.1% of model)',
        '3132 us (7.0% of model)'],['_features_2_conv_2_hz_conv2d',
    '75,497,472',
    '23 us (0% of model)',
    '1 us (0% of model)'],
    ['_features_3_conv_0_0_hz_conv2d',
    '113,246,208',
    '24 us (0% of model)',
    '638 us (1.4% of model)'],
    ['_features_3_conv_1_0_hz_conv2d',
    '42,467,328',
    '33 us (0% of model)',
    '3592 us (8.0% of model)'],
    ['_features_3_generated_add_0_hz_conv2d',
    '113,246,208',
    '14 us (0% of model)',
    '637 us (1.4% of model)'],
    ['_features_4_conv_0_0_hz_conv2d',
    '113,246,208',
    '45 us (0.1% of model)',
    '2433 us (5.4% of model)'],
    ['_features_4_conv_1_0_hz_conv2d',
    '10,616,832',
    '63 us (0.1% of model)',
    '2432 us (5.4% of model)'],
    ['_features_4_conv_2_hz_conv2d',
    '37,748,736',
    '21 us (0% of model)',
    '1 us (0% of model)'],
    ['_features_5_conv_0_0_hz_conv2d',
    '50,331,648',
    '40 us (0% of model)',
    '3 us (0% of model)'],
    ['_features_5_conv_1_0_hz_conv2d',
    '14,155,776',
    '45 us (0% of model)',
    '463 us (1.0% of model)'],
    ['_features_5_generated_add_0_hz_conv2d',
    '50,331,648',
    '23 us (0% of model)',
    '462 us (1.0% of model)'],
    ['_features_6_conv_0_0_hz_conv2d',
    '50,331,648',
    '40 us (0% of model)',
    '3 us (0% of model)'],
    ['_features_6_conv_1_0_hz_conv2d',
    '14,155,776',
    '23 us (0% of model)',
    '463 us (1.0% of model)'],
    ['_features_6_generated_add_0_hz_conv2d',
    '50,331,648',['23 us (0% of model)',
    '462 us (1.0% of model)'],
    ['_features_7_conv_0_0_hz_conv2d',
    '50,331,648',
    '61 us (0.1% of model)',
    '813 us (1.8% of model)'],
    ['_features_7_conv_1_0_hz_conv2d',
    '3,538,944',
    '76 us (0.1% of model)',
    '812 us (1.8% of model)'],
    ['_features_7_conv_2_hz_conv2d',
    '25,165,824',
    '47 us (0.1% of model)',
    '3 us (0% of model)'],
    ['_features_8_conv_0_0_hz_conv2d',
    '50,331,648',
    '76 us (0.1% of model)',
    '5 us (0% of model)'],
    ['_features_8_conv_1_0_hz_conv2d',
    '7,077,888',
    '75 us (0.1% of model)',
    '463 us (1.0% of model)'],
    ['_features_8_generated_add_0_hz_conv2d',
    '50,331,648',
    '67 us (0.1% of model)',
    '465 us (1.0% of model)'],
    ['_features_9_conv_0_0_hz_conv2d',
    '50,331,648',
    '76 us (0.1% of model)',
    '5 us (0% of model)'],
    ['_features_9_conv_1_0_hz_conv2d',
    '7,077,888',
    '75 us (0.1% of model)',
    '463 us (1.0% of model)'],
    ['_features_9_generated_add_0_hz_conv2d',
    '50,331,648',
    '67 us (0.1% of model)',
    '465 us (1.0% of model)'],
    ['_features_10_conv_0_0_hz_conv2d',
    '50,331,648',
    '76 us (0.1% of model)',
    '5 us (0% of model)'],
    ['_features_10_conv_1_0_hz_conv2d',
    '7,077,888',
    '51 us (0.1% of model)',
    '463 us (1.0% of model)'],
    ['_features_10_generated_add_0_hz_conv2d',
    '50,331,648',
    '67 us (0.1% of model)',
    '465 us (1.0% of model)']['_features_11_conv_0_0_hz_conv2d',
    '50,331,648',
    '76 us (0.1% of model)',
    '5 us (0% of model)'],

    ['_features_11_conv_1_0_hz_conv2d',
    '7,077,888',
    '75 us (0.1% of model)',
    '463 us (1.0% of model)'],

    ['_features_11_conv_2_hz_conv2d',
    '75,497,472',
    '110 us (0.2% of model)',
    '467 us (1.0% of model)'],

    ['_features_12_conv_0_0_hz_conv2d',
    '113,246,208',
    '43 us (0% of model)',
    '644 us (1.4% of model)'],

    ['_features_12_conv_1_0_hz_conv2d',
    '10,616,832',
    '46 us (0.1% of model)',
    '1274 us (2.8% of model)'],

    ['_features_12_generated_add_0_hz_conv2d',
    '113,246,208',
    '37 us (0% of model)',
    '643 us (1.4% of model)'],

    ['_features_13_conv_0_0_hz_conv2d',
    '113,246,208',
    '43 us (0% of model)',
    '644 us (1.4% of model)'],

    ['_features_13_conv_1_0_hz_conv2d',
    '10,616,832',
    '55 us (0.1% of model)',
    '1274 us (2.8% of model)'],

    ['_features_13_generated_add_0_hz_conv2d',
    '113,246,208',
    '31 us (0% of model)',
    '642 us (1.4% of model)'],

    ['_features_14_conv_0_0_hz_conv2d',
    '113,246,208',
    '67 us (0.1% of model)',
    '644 us (1.4% of model)'],

    ['_features_14_conv_1_0_hz_conv2d',
    '2,654,208',
    '72 us (0.1% of model)',
    '1274 us (2.8% of model)'],

    ['_features_14_conv_2_hz_conv2d',
    '47,185,920',
    '108 us (0.2% of model)',
    '647 us (1.4% of model)'],

    ['_features_15_conv_0_0_hz_conv2d',
    '78,643,200'    
    ['_features_15_conv_1_0_hz_conv2d',
        '4,423,680',
        '51 us (0.1% of model)',
        '1973 us (4.4% of model)'],
    ['_features_15_generated_add_0_hz_conv2d',
        '78,643,200',
        '48 us (0.1% of model)',
        '1004 us (2.2% of model)'],
    ['_features_16_conv_0_0_hz_conv2d',
        '78,643,200',
        '39 us (0% of model)',
        '1004 us (2.2% of model)'],
    ['_features_16_conv_1_0_hz_conv2d',
        '4,423,680',
        '56 us (0.1% of model)',
        '1973 us (4.4% of model)'],
    ['_features_16_generated_add_0_hz_conv2d',
        '78,643,200',
        '41 us (0% of model)',
        '1004 us (2.2% of model)'],
    ['_features_17_conv_0_0_hz_conv2d',
        '78,643,200',
        '80 us (0.1% of model)',
        '1004 us (2.2% of model)'],
    ['_features_17_conv_1_0_hz_conv2d',
        '4,423,680',
        '63 us (0.1% of model)',
        '1973 us (4.4% of model)'],
    ['_features_17_conv_2_hz_conv2d',
        '157,286,400',
        '99 us (0.2% of model)',
        '1021 us (2.2% of model)'],
    ['_features_18_0_hz_conv2d',
        '209,715,200',
        '878 us (1.9% of model)',
        '2601 us (5.8% of model)'],
    ['_features_18_0_hz_conv2d_torch_native', '0', '11 us (0% of model)', '0'],
    ['_classifier_1_hz_linear_torch_native',
        '6,553,600',
        '5 us (0% of model)',
        '7 us (0% of model)']],
    'loaded bytes per frame': 707232,
    'loaded bytes per run': 181051392,
    'model json CRC': '51b16a11',
    'model json file': '/tmp/hbdktmp_ocro1_9_.hbir',
    'model name': 'FxQATReadyMobileNetV2',
    'model param CRC': '00000000',
    'multicore sync time (ms)': 0.0,'run per second': 22.36,
'runtime version': '3.15.29.0',
'stored bytes per frame': 696360,
'stored bytes per run': 178268160,
'worst FPS': 5722.98

```

```python

    # Model visualization
    visualize_model(
        script_model,
        [example_input],
        save_path=os.path.join(model_path, "model.svg"),
        show=False,
    )

```

```shell

    INFO: launch 1 threads for optimization

    consumed time 1.6947
    HBDK model compilation SUCCESS

```
---
sidebar_position: 2
---

# 快速上手{#quick_start} 

Horizon Plugin Pytorch (下称 Plugin ) 参考了 PyTorch 官方的量化接口和思路，Plugin 采用的是 Quantization Aware Training(QAT) 方案，因此建议用户先阅读 [**PyTorch 官方文档**](https://pytorch.org/docs/stable/quantization.html#quantization)中和 QAT 相关部分，熟悉 PyTorch 提供的量化训练和部署工具的使用方法。

## 基本流程

量化训练工具的基本使用流程如下：

![quick_start](./image/expert/quick_start.svg)

下面以 ``torchvision`` 中的 ``MobileNetV2`` 模型为例，介绍流程中每个阶段的具体操作。

出于流程展示的执行速度考虑，我们使用了 ``cifar-10`` 数据集，而不是 ImageNet-1K 数据集。


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

## 获取浮点模型{#Float-Model}

首先，对浮点模型做必要的改造，以支持量化相关操作。模型改造必要的操作有：

- 在模型输入前插入 ``QuantStub``

- 在模型输出后插入 ``DequantStub``

改造模型时需要注意：

- 插入的 ``QuantStub`` 和 ``DequantStub`` 必须注册为模型的子模块，否则将无法正确处理它们的量化状态

- 多个输入仅在 ``scale`` 相同时可以共享 ``QuantStub``，否则请为每个输入定义单独的 ``QuantStub``

- 若需要将上板时输入的数据来源指定为 ``"pyramid"``，请手动设置对应 ``QuantStub`` 的 ``scale`` 参数为 1/128

- 也可以使用 ``torch.quantization.QuantStub``，但是仅有 ``horizon_plugin_pytorch.quantization.QuantStub`` 支持通过参数手动固定 scale

改造后的模型可以无缝加载改造前模型的参数，因此若已有训练好的浮点模型，直接加载即可，否则需要正常进行浮点训练。

:::caution 注意

  模型上板时的输入图像数据一般为 centered_yuv444 格式，因此模型训练时需要把图像转换成 centered_yuv444 格式（注意下面代码中对 ``rgb2centered_yuv`` 的使用）。
  如果无法转换成 centered_yuv444 格式进行模型训练，请参考 [**RGB888 数据部署**](./advanced_content#rgb888-数据部署) 章节中的介绍，对模型做相应的改造。（注意，该方法可能导致模型精度降低）
  本示例中浮点和 QAT 训练的 epoch 较少，仅为说明训练工具使用流程，精度不代表模型最好水平。
:::

```python

    ######################################################################
    # 用户可根据需要修改以下参数
    # 1. 模型 ckpt 和编译产出物的保存路径
    model_path = "model/mobilenetv2"
    # 2. 数据集下载和保存的路径
    data_path = "data"
    # 3. 训练时使用的 batch_size
    train_batch_size = 256
    # 4. 预测时使用的 batch_size
    eval_batch_size = 256
    # 5. 训练的 epoch 数
    epoch_num = 30
    # 6. 模型保存和执行计算使用的 device
    device = (
        torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
    )
    ######################################################################


    # 准备数据集，请注意 collate_fn 中对 rgb2centered_yuv 的使用
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
            False,
            download=True,
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


    # 对浮点模型做必要的改造
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
        os.makedirs(model_path, exist_ok=True)

    # 浮点模型初始化
    float_model = FxQATReadyMobileNetV2()

    # 准备数据集
    train_data_loader, eval_data_loader = prepare_data_loaders(
        data_path, train_batch_size, eval_batch_size
    )

    # 由于模型的最后一层和预训练模型不一致，需要进行浮点 finetune
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

        # 浮点精度测试
        float_model.eval()
        top1, top5 = evaluate(float_model, eval_data_loader, device)

        print(
            "Float Epoch {}: evaluation Acc@1 {:.3f} Acc@5 {:.3f}".format(
                nepoch, top1.avg, top5.avg
            )
        )

        if top1.avg > best_acc:
            best_acc = top1.avg
            # 保存最佳浮点模型参数
            torch.save(
                float_model.state_dict(),
                os.path.join(model_path, "float-checkpoint.ckpt"),
            )

```

```shell

    Files already downloaded and verified
    Files already downloaded and verified
    ....................................................................................................................................................................................................
    Full cifar-10 train set: Loss 2.116 Acc@1 20.744 Acc@5 70.668
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
    ........................................
    Float Epoch 12: evaluation Acc@1 69.120 Acc@5 97.180
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
    ....................................................................................................................................................................................................
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
    ....................................................................................................................................................................................................
    Full cifar-10 train set: Loss 0.741 Acc@1 74.278 Acc@5 98.038
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

模型改造完成并完成浮点训练后，便可进行 Calibration。此过程通过在模型中插入 Observer 的方式，在 forward 过程中统计各处的数据分布情况，从而计算出合理的量化参数：

- 对于部分模型，仅通过 Calibration 便可使精度达到要求，不必进行比较耗时的量化感知训练。

- 即使模型经过量化校准后无法满足精度要求，此过程也可降低后续量化感知训练的难度，缩短训练时间，提升最终的训练精度。

```python

    ######################################################################
    # 用户可根据需要修改以下参数
    # 1. Calibration 时使用的 batch_size
    calib_batch_size = 256
    # 2. Validation 时使用的 batch_size
    eval_batch_size = 256
    # 3. Calibration 使用的数据量，配置为 inf 以使用全部数据
    num_examples = float("inf")
    # 4. 目标硬件平台的代号
    march = March.BAYES
    ######################################################################

    # 在进行模型转化前，必须设置好模型将要执行的硬件平台
    set_march(march)


    # 将模型转化为 Calibration 状态，以统计各处数据的数值分布特征
    calib_model = prepare_qat_fx(
        # 输出模型会共享输入模型的 attributes，为不影响 float_model 的后续使用,
        # 此处进行了 deepcopy
        copy.deepcopy(float_model),
        {
            "": default_calib_8bit_fake_quant_qconfig,
            "module_name": {
                # 在模型的输出层为 Conv 或 Linear 时，可以使用 out_qconfig
                # 配置为高精度输出
                "classifier": default_calib_8bit_weight_32bit_out_fake_quant_qconfig,
            },
        },
    ).to(
        device
    )  # prepare_qat_fx 接口不保证输出模型的 device 和输入模型完全一致

    # 准备数据集
    calib_data_loader, eval_data_loader = prepare_data_loaders(
        data_path, calib_batch_size, eval_batch_size
    )

    # 执行 Calibration 过程（不需要 backward）
    # 注意此处对模型状态的控制，模型需要处于 eval 状态以使 Bn 的行为符合要求
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

    # 测试伪量化精度
    # 注意此处对模型状态的控制
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
    )

    # 保存 Calibration 模型参数
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

模型经过 Calibration 后的量化精度若已满足要求，便可直接进行 **转定点模型** 的步骤，否则需要进行 **量化训练** 进一步提升精度。


## 量化训练

量化训练通过在模型中插入伪量化节点的方式，在训练过程中使模型感知到量化带来的影响，在这种情况下对模型参数进行微调，以提升量化后的精度。

```python

    ######################################################################
    # 用户可根据需要修改以下参数
    # 1. 训练时使用的 batch_size
    train_batch_size = 256
    # 2. Validation 时使用的 batch_size
    eval_batch_size = 256
    # 3. 训练的 epoch 数
    epoch_num = 3
    ######################################################################

    # 准备数据集
    train_data_loader, eval_data_loader = prepare_data_loaders(
        data_path, train_batch_size, eval_batch_size
    )

    # 将模型转为 QAT 状态
    qat_model = prepare_qat_fx(
        copy.deepcopy(float_model),
        {
            "": default_qat_8bit_fake_quant_qconfig,
            "module_name": {
                "classifier": default_qat_8bit_weight_32bit_out_fake_quant_qconfig,
            },
        },
    ).to(device)

    # 加载 Calibration 模型中的量化参数
    qat_model.load_state_dict(calib_model.state_dict())

    # 进行量化感知训练
    # 作为一个 filetune 过程，量化感知训练一般需要设定较小的学习率
    optimizer = torch.optim.Adam(
        qat_model.parameters(), lr=1e-3, weight_decay=1e-4
    )

    best_acc = 0

    for nepoch in range(epoch_num):
        # 注意此处对 QAT 模型 training 状态的控制方法
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

        # 注意此处对 QAT 模型 eval 状态的控制方法
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

        if top1.avg > best_acc:
            best_acc = top1.avg

            torch.save(
                qat_model.state_dict(),
                os.path.join(model_path, "qat-checkpoint.ckpt"),
            )

```

```shell

    Files already downloaded and verified
    Files already downloaded and verified
    ....................................................................................................................................................................................................
    Full cifar-10 train set: Loss 0.759 Acc@1 73.692 Acc@5 97.940
    ........................................
    QAT Epoch 0: evaluation Acc@1 79.170 Acc@5 98.490
    ....................................................................................................................................................................................................
    Full cifar-10 train set: Loss 0.718 Acc@1 75.414 Acc@5 97.998
    ........................................
    QAT Epoch 1: evaluation Acc@1 78.540 Acc@5 98.580
    ....................................................................................................................................................................................................
    Full cifar-10 train set: Loss 0.719 Acc@1 75.180 Acc@5 98.126
    ........................................
    QAT Epoch 2: evaluation Acc@1 78.200 Acc@5 98.540

```

## 转定点模型

伪量化精度达标后，便可将模型转为定点模型。一般认为定点模型的结果和编译后模型的结果是完全一致的。

:::caution 注意

  定点模型和伪量化模型之间无法做到完全数值一致，所以请以定点模型的精度为准。若定点精度不达标，需要继续进行量化训练。
:::

```python

    ######################################################################
    # 用户可根据需要修改以下参数
    # 1. 使用哪个模型作为流程的输入，可以选择 calib_model 或 qat_model
    base_model = qat_model
    ######################################################################

    # 将模型转为定点状态
    quantized_model = convert_fx(base_model).to(device)

    # 测试定点模型精度
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

    2023-06-29 14:55:05,825] WARNING: AdaptiveAvgPool2d has not collected any statistics of activations and its scale is 1, please check whether this is intended!

    ........................................
    Quantized model: evaluation Acc@1 78.390 Acc@5 98.640
```

## 模型部署

测试定点模型精度并确认符合要求后，便可以进行模型部署的相关流程，包括模型检查、编译、性能测试和可视化。

:::caution 注意

  - 也可以跳过 Calibration 和量化感知训练中的实际校准和训练过程，先直接进行模型检查，以保证模型中不存在无法编译的操作。
  - 由于编译器只支持 CPU，因此模型和数据必须放在 CPU 上。
:::

```python

    ######################################################################
    # 用户可根据需要修改以下参数
    # 1. 编译时启用的优化等级，可选 0~3，等级越高编译出的模型上板执行速度越快，
    #    但编译过程会慢
    compile_opt = "O1"
    ######################################################################

    # 这里的 example_input 也可以是随机生成的数据，但是推荐使用真实数据，以提高
    # 性能测试的准确性
    example_input = next(iter(eval_data_loader))[0]

    # 通过 trace 将模型序列化并生成计算图，注意模型和数据要放在 CPU 上
    script_model = torch.jit.trace(quantized_model.cpu(), example_input)
    torch.jit.save(script_model, os.path.join(model_path, "int_model.pt"))

    # 模型检查
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

    # 模型编译，生成的 hbm 文件即为可部署的模型
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

    # 模型性能测试
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
    0.016,
    0.053,
    0.021,
    0.001,
    0.093,
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
    0.093,
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
    4372,
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
        '3132 us (7.0% of model)'],
    ['_features_2_conv_2_hz_conv2d',
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
        '50,331,648',
        '23 us (0% of model)',
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
        '465 us (1.0% of model)'],
    ['_features_11_conv_0_0_hz_conv2d',
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
        '78,643,200',
        '47 us (0.1% of model)',
        '1004 us (2.2% of model)'],
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
    'multicore sync time (ms)': 0.0,
    'run per second': 22.36,
    'runtime version': '3.15.29.0',
    'stored bytes per frame': 696360,
    'stored bytes per run': 178268160,
    'worst FPS': 5722.98}}

```

```python

    # 模型可视化
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
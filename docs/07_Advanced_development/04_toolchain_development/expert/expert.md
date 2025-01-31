# 7.4.4 高阶指南

量化是指以低于浮点精度的比特宽度执行计算和存储张量的技术。量化模型使用整数而不是浮点值对张量执行部分或全部操作。与典型的 FP32 模型相比，horizon_plugin_pytorch 支持 INT8 量化，从而使模型大小减少 4 倍，内存带宽需求减少 4 倍。对 INT8 计算的硬件支持通常比 FP32 计算快 2 到 4 倍。量化主要是一种加速推理的技术，量化运算只支持前向计算。

horizon_plugin_pytorc 提供了适配 BPU 的量化操作，支持量化感知训练，该训练使用伪量化模块对前向计算和反向传播中的量化误差进行建模。请注意，量化训练的整个计算过程是使用浮点运算执行的。在量化感知训练结束时，horizon_plugin_pytorch 提供转换函数，将训练后的模型转换为定点模型，在 BPU 上使用更紧凑的模型表示和高性能矢量化操作。

本章内容为您详细介绍D-Robotics 基于 PyTorch 开发的 horizon_plugin_pytorch 的量化训练工具。
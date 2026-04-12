---
sidebar_position: 6
---

# SYS（模块绑定）API

`SYS` API提供了以下的接口：

| 函数 | 功能 |
| ---- | ----- |
| sp_module_bind | **绑定数据源、目标模块** |
| sp_module_unbind | **解除模块间的绑定** |

### sp_module_bind  

**【函数原型】**  

`int32_t sp_module_bind(void *src, int32_t src_type, void *dst, int32_t dst_type)`

**【功能描述】**  

本接口可以把 `VIO`，`ENCODER`，`DECODER`，`DISPLAY`, 这四个模块的输出与输入进行内部绑定，绑定后的两个模块的数据会在内部自动流转，无需用户操作。比如绑定 `VIO` 和 `DISPLAY` 后，打开的mipi摄像头的数据会直接显示到显示屏上，不需要调用`VIO`的`sp_vio_get_frame`接口获取数据，之后再调用`DISPLAY`的`sp_display_set_image`接口进行显示。

支持绑定的模块关系如下：

| 源数据模块 | 目标数据模块 |
| ---- | ----- |
| VIO | ENCODER |
| VIO | DISPLAY |
| DECODER | ENCODER |
| DECODER | DISPLAY |

**【参数】**

- `src`： 数据源模块的对象指针（调用各模块初始化接口得到）
- `src_type`：源数据模块类型，支持 `SP_MTYPE_VIO` 和 `SP_MTYPE_DECODER`
- `dst`： 目标模块的对象指针（调用各模块初始化接口得到）
- `dst_type`：目标数据模块类型，支持 `SP_MTYPE_ENCODER` 和 `SP_MTYPE_DISPLAY`

**【返回类型】**  

成功返回 0，失败返回其他值。

### sp_module_unbind  

**【函数原型】**  

`int32_t sp_module_unbind(void *src, int32_t src_type, void *dst, int32_t dst_type)`

**【功能描述】**  

本接口完成已经绑定的两个模块的解绑，模块退出前需要先完成解绑。

**【参数】**

- `src`： 数据源模块的对象指针（调用各模块初始化接口得到）
- `src_type`：源数据模块类型，支持 `SP_MTYPE_VIO` 和 `SP_MTYPE_DECODER`
- `dst`： 目标模块的对象指针（调用各模块初始化接口得到）
- `dst_type`：目标数据模块类型，支持 `SP_MTYPE_ENCODER` 和 `SP_MTYPE_DISPLAY`

**【返回类型】**  

成功返回 0，失败返回其他值。

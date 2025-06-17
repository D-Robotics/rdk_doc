---
sidebar_position: 6
---
# SYS (Module Binding) API

The `SYS` API provides the following interfaces:

| Function | Description |
| -------- | ----------- |
| sp_module_bind | **Bind data source and destination module** |
| sp_module_unbind | **Unbind modules** |

### sp_module_bind  

**[Function Prototype]**  

`int32_t sp_module_bind(void *src, int32_t src_type, void *dst, int32_t dst_type)`

**[Description]**  

This interface can internally bind the output and input of the `VIO`, `ENCODER`, `DECODER`, and `DISPLAY` modules. After binding, the data between the two modules will automatically flow internally without user intervention. For example, after binding `VIO` and `DISPLAY`, the data from the opened MIPI camera will be directly displayed on the screen without calling the `sp_vio_get_frame` interface to retrieve the data and then calling the `sp_display_set_image` interface of `DISPLAY` for display.

The supported module binding relationships are as follows:

| Source Module | Destination Module |
| ------------- | ----------------- |
| VIO | ENCODER |
| VIO | DISPLAY |
| DECODER | ENCODER |
| DECODER | DISPLAY |

**[Parameters]**

- `src`: Object pointer of the data source module (obtained by calling various module initialization interfaces)
- `src_type`: Type of the source data module, supports `SP_MTYPE_VIO` and `SP_MTYPE_DECODER`
- `dst`: Object pointer of the destination module (obtained by calling various module initialization interfaces)
- `dst_type`: Type of the destination data module, supports `SP_MTYPE_ENCODER` and `SP_MTYPE_DISPLAY`

**[Return Type]**  

Returns 0 on success, and other values on failure.

### sp_module_unbind  

**[Function Prototype]**  

`int32_t sp_module_unbind(void *src, int32_t src_type, void *dst, int32_t dst_type)`

**[Description]**

This interface completes the unbinding of two modules that have already been bound. The unbinding needs to be completed before the module exits.

**【Parameters】**

- `src`: Object pointer of the data source module (obtained by calling various module initialization interfaces)
- `src_type`: Type of the source data module, supports `SP_MTYPE_VIO` and `SP_MTYPE_DECODER`
- `dst`: Object pointer of the target module (obtained by calling various module initialization interfaces)
- `dst_type`: Type of the target data module, supports `SP_MTYPE_ENCODER` and `SP_MTYPE_DISPLAY`

**【Return】**  

Returns 0 on success, returns other values on failure.
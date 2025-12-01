---
sidebar_position: 6
---

# SYS (Module Binding) API

The `SYS` API provides the following interfaces:

| Function | Description |
| ---- | ----- |
| sp_module_bind | **Bind source and destination modules** |
| sp_module_unbind | **Unbind modules** |

### sp_module_bind  

**[Function Prototype]**  

`int32_t sp_module_bind(void *src, int32_t src_type, void *dst, int32_t dst_type)`

**[Description]**  

This API enables internal binding between the inputs and outputs of the following four modules: `VIO`, `ENCODER`, `DECODER`, and `DISPLAY`. Once bound, data flows automatically between the two modules internally without requiring user intervention. For example, after binding `VIO` and `DISPLAY`, data from an opened MIPI camera will be directly displayed on the screen, eliminating the need to call `sp_vio_get_frame` to retrieve data from `VIO` and then call `sp_display_set_image` to display it via `DISPLAY`.

The supported module binding relationships are as follows:

| Source Module | Destination Module |
| ---- | ----- |
| VIO | ENCODER |
| VIO | DISPLAY |
| DECODER | ENCODER |
| DECODER | DISPLAY |

**[Parameters]**

- `src`: Pointer to the source module object (obtained by calling the respective module's initialization function)
- `src_type`: Type of the source module; supports `SP_MTYPE_VIO` and `SP_MTYPE_DECODER`
- `dst`: Pointer to the destination module object (obtained by calling the respective module's initialization function)
- `dst_type`: Type of the destination module; supports `SP_MTYPE_ENCODER` and `SP_MTYPE_DISPLAY`

**[Return Value]**  

Returns 0 on success; returns a non-zero value on failure.

### sp_module_unbind  

**[Function Prototype]**  

`int32_t sp_module_unbind(void *src, int32_t src_type, void *dst, int32_t dst_type)`

**[Description]**  

This API unbinds two previously bound modules. Modules must be unbound before they are deinitialized or exited.

**[Parameters]**

- `src`: Pointer to the source module object (obtained by calling the respective module's initialization function)
- `src_type`: Type of the source module; supports `SP_MTYPE_VIO` and `SP_MTYPE_DECODER`
- `dst`: Pointer to the destination module object (obtained by calling the respective module's initialization function)
- `dst_type`: Type of the destination module; supports `SP_MTYPE_ENCODER` and `SP_MTYPE_DISPLAY`

**[Return Value]**  

Returns 0 on success; returns a non-zero value on failure.
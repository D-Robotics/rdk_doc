---
sidebar_position: 6
---
# SYS (Module Binding) API

The `SYS` API provides interfaces for binding and unbinding different modules such as `VIO`, `ENCODER`, `DECODER`, and `DISPLAY`. This allows seamless data transfer between the modules without requiring manual data fetching and setting.

## Functions

| Function                      | Description                                                       |
|-------------------------------|-------------------------------------------------------------------|
| sp_module_bind                 | **Bind source and destination modules for automatic data flow**  |
| sp_module_unbind               | **Unbind previously bound modules**                              |

## sp_module_bind

**Function Declaration**  
`int32_t sp_module_bind(void *src, int32_t src_type, void *dst, int32_t dst_type)`

**Description**  
This function binds the output of one module to the input of another. Once bound, data flows automatically between the modules without user intervention. For example, if `VIO` and `DISPLAY` are bound, the data from a connected MIPI camera will be automatically displayed on the screen without needing to manually fetch frames from `VIO` and pass them to `DISPLAY`.

The following module bindings are supported:

| Source Module | Destination Module |
|---------------|---------------------|
| VIO           | ENCODER             |
| VIO           | DISPLAY             |
| DECODER       | ENCODER             |
| DECODER       | DISPLAY             |

**Parameters**  
- `src`: The source module object pointer (obtained by calling the initialization interface for each module).
- `src_type`: The type of the source module. Supported types are `SP_MTYPE_VIO` and `SP_MTYPE_DECODER`.
- `dst`: The destination module object pointer (obtained by calling the initialization interface for each module).
- `dst_type`: The type of the destination module. Supported types are `SP_MTYPE_ENCODER` and `SP_MTYPE_DISPLAY`.

**Return Type**  
- Returns `0` on success.
- Returns a non-zero value on failure.

## sp_module_unbind

**Function Declaration**  
`int32_t sp_module_unbind(void *src, int32_t src_type, void *dst, int32_t dst_type)`

**Description**  
This function unbinds previously bound modules, stopping the automatic data flow between them. It is necessary to unbind modules before exiting or shutting down the system.

**Parameters**  
- `src`: The source module object pointer (obtained by calling the initialization interface for each module).
- `src_type`: The type of the source module. Supported types are `SP_MTYPE_VIO` and `SP_MTYPE_DECODER`.
- `dst`: The destination module object pointer (obtained by calling the initialization interface for each module).
- `dst_type`: The type of the destination module. Supported types are `SP_MTYPE_ENCODER` and `SP_MTYPE_DISPLAY`.

**Return Type**  
- Returns `0` on success.
- Returns a non-zero value on failure.

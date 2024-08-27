---
sidebar_position: 1
---

# hrut_boardid

**hrut_boardid** 命令是用来获取当前开发板的编号  (不同开发板的编号不同）。

> ⚠️ boardid 会影响到启动时硬件的初始化，请谨慎设置。

## 语法说明

```
Usage:  hrut_boardid [OPTIONS] <Values>
Example:
       hrut_boardid g
Options:
       g   get board id(veeprom)
       s   set board id(veeprom)
       G   get board id(bootinfo)
       S   set board id(bootinfo)
       c   clear board id(veeprom)
       C   clear board id(bootinfo)
       h   display this help text

```

- **g：**从`veeprom`获取开发板编号。
- **s：**从`veeprom`设置开发板编号。
- **G：**从`bootinfo`获取开发板编号。
- **S：**从`bootinfo`设置开发板编号。
- **c：**清除`veeprom`中的开发板编号配置。
- **C：**清除`bootinfo`中的开发板编号配置。
- **h：**获取帮助信息。

------

## boardid编号定义

|                     | 含义                | 长度             | 取值范围                                                     |
| :------------------ | :------------------ | :--------------- | :----------------------------------------------------------- |
| **auto detect**     | DDR 自动探测功能    | 1bit<br/>[31]    | 0x0：auto detection<br/>0x1：不使用LPDDR4 auto detection功能 |
| **model**           | DDR厂商信息         | 3bit<br/>[30:28] | 0x0： auto detection<br/>0x1： hynix，海力士<br/>0x2： micron，镁光<br/>0x3： samsung，三星 |
| **ddr_type**        | DDR类型             | 4bit<br/>[27:24] | 0x0： auto detection<br/>0x1： LPDDR4<br/>0x2： LPDDR4X<br/>0x3： DDR4<br/>0x4： DDR3L |
| **frequency**       | DDR频率             | 4bit<br/>[23:20] | 0x0： auto detection<br/>0x1： 667<br/>0x2： 1600<br/>0x3： 2133<br/>0x4： 2666<br/>0x5： 3200<br/>0x6： 3733<br/>0x7： 4266<br/>0x8： 1866<br/>0x9： 2400<br/>0xa： 100<br/>0xb： 3600 |
| **capacity**        | DDR容量             | 4bit<br/>[19:16] | 0x0： auto detection<br/>0x1： 1GB<br/>0x2： 2GB<br/>0x4： 4GB |
| **ecc**             |                     | 4bit<br/>[15:12] | 0x0： default ECC config<br/>0x1： inline ECC all<br/>0x2： inline ecc option1<br/>0x3： inline ecc option2 |
| **som_type**        | SOM类型             | 4bit<br/>[11:8]  | 0x0： auto detection<br/>0x3：sdb v3<br/>0x4：sdb v4<br/>0x5：RDK X3 v1<br/>0x6：RDK X3 v1.2<br/>0x8：RDK X3 v2<br/>0xb：RDK Module<br/>0xF： X3E |
| **DFS EN**          | 调频使能位          | 1bit<br/>[7]     | 1：使能调频功能<br/>0：不使能调频功能                        |
| **alternative**     | alternaive paramter | 3bit<br/>[6:4]   | 0x0： default configure<br/>0x1： config1                    |
| **base_board_type** | 底板类型            | 4bit<br/>[3:0]   | 0x0： auto detection<br/>0x1： X3 DVB<br/>0x4： X3 SDB<br/>0x5： customer board |

**各字段定义如下：**

- **model:** hynix 和 micron, samsung
- **ddr_type：**LPDDR4、LPDDR4X、DDR4、DDR3L
- **frequency：** 667、1600、2133、2666、3200、3733、4266
- **capacity：** 1G、2G、4G
- **som_type：** sdb v3、sdb v4、RDK X3 v1、RDK X3 v1.2、RDK X3 v2、RDK Module、X3E
- **base_board_type：** x3dvb、X3 SDB、customer_board

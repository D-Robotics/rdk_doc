---
sidebar_position: 1
---
# hrut_boardid

The **hrut_boardid** command is used to retrieve the board ID of the current development board (different development boards have different IDs).

> ⚠️ The board ID will affect the hardware initialization during startup, so please set it carefully.

## Syntax

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

- **g:** Get the board ID from `veeprom`.
- **s:** Set the board ID in `veeprom`.
- **G:** Get the board ID from `bootinfo`.
- **S:** Set the board ID in `bootinfo`.
- **c:** Clear the board ID configuration in `veeprom`.
- **C:** Clear the board ID configuration in `bootinfo`.
- **h:** Display help information.

------

## Board ID definitions

|                     | Meaning                             | Length      | Value Range                                                                                    |
| :------------------ | :---------------------------------- | :---------- | :-------------------------------------------------------------------------------------------- |
| **auto detect**     | DDR self-detection feature           | 1bit<br/>[31] | 0x0: auto detection<br/>0x1: do not use LPDDR4 auto detection feature                          |
| **model**           | DDR manufacturer information         | 3bit<br/>[30:28] | 0x0: auto detection<br/>0x1: hynix<br/>0x2: micron<br/>0x3: samsung                              |
| **ddr_type**        | DDR type                            | 4bit<br/>[27:24] | 0x0: auto detection<br/>0x1: LPDDR4<br/>0x2: LPDDR4X<br/>0x3: DDR4<br/>0x4: DDR3L                  |
| **frequency**       | DDR frequency                       | 4bit<br/>[23:20] | 0x0: auto detection<br/>0x1: 667<br/>0x2: 1600<br/>0x3: 2133<br/>0x4: 2666<br/>0x5: 3200<br/>0x6: 3733<br/>0x7: 4266<br/>0x8: 1866<br/>0x9: 2400<br/>0xa: 100<br/>0xb: 3600 |
| **capacity**        | DDR capacity                        | 4bit<br/>[19:16] | 0x0: auto detection<br/>0x1: 1GB<br/>0x2: 2GB<br/>0x4: 4GB                                      |
| **ecc**             |                                     | 4bit<br/>[15:12] | 0x0: default ECC config<br/>0x1: inline ECC all<br/>0x2: inline ECC option1<br/>0x3: inline ECC option2 |
| **som_type**        | SOM type                            | 4bit<br/>[11:8]  | 0x0: auto detection<br/>0x3: sdb v3<br/>0x4: sdb v4<br/>0x5: RDK X3 v1<br/>0x6: RDK X3 v1.2<br/>0x8: RDK X3 v2<br/>0xb: RDK Module<br/>0xF: X3E |
| **DFS EN**          | Dynamic Frequency Scaling (DFS) enable bit | 1bit<br/>[7]    | 1: enable DFS feature<br/>0: disable DFS feature                                               |
| **alternative**     | Alternative parameter                | 3bit<br/>[6:4]  | 0x0: default configuration<br/>0x1: configuration 1                                           |
| **base_board_type**  | Board Type           | 4bit<br/>[3:0]   | 0x0: auto detection<br/>0x1: X3 DVB<br/>0x4: X3 SDB<br/>0x5: customer board |

**The definitions of each field are as follows:**

- **model:** hynix and micron, samsung
- **ddr_type:** LPDDR4, LPDDR4X, DDR4, DDR3L
- **frequency:** 667, 1600, 2133, 2666, 3200, 3733, 4266
- **capacity:** 1G, 2G, 4G
- **som_type:** sdb v3, sdb v4, RDK X3 v1, RDK X3 v1.2, RDK X3 v2, RDK Module, X3E
- **base_board_type:** x3dvb, X3 SDB, customer_board
---
sidebar_position: 1
---
# Port Usage Guide
## Basic Overview
The Port subsystem is a subsystem on the MCU used to configure the functions and attributes of PINs.

## List of PIN Names Corresponding to PIN Numbers in the Port_Func Module {#pin_list}
The meanings of each column in the table below are as follows:
    - PIN Number: **PIN number used in the Port submodule**;
    - PIN Name: Name of the PIN;
    - GPIO Number: Number used to obtain the GPIO controller address offset and GPIO controller register offset;

| PIN Number | PIN Name      | GPIO Number       |
|------------|---------------|-------------------|
| 0          | FUSA_ERR0     | GPIO_MCU[0]       |
| 1          | FUSA_ERR1     | GPIO_MCU[1]       |
| 2          | PPS_INOUT     | GPIO_MCU[2]       |
| 3          | LIN1_TXD      | GPIO_MCU[3]       |
| 4          | LIN2_TXD      | GPIO_MCU[4]       |
| 5          | CAN0_TX       | GPIO_MCU[5]       |
| 6          | CAN1_TX       | GPIO_MCU[6]       |
| 7          | CAN2_TX       | GPIO_MCU[7]       |
| 8          | CAN3_TX       | GPIO_MCU[8]       |
| 9          | CAN4_TX       | GPIO_MCU[9]       |
| 10         | CAN5_TX       | GPIO_MCU[10]      |
| 11         | CAN6_TX       | GPIO_MCU[11]      |
| 12         | CAN7_TX       | GPIO_MCU[12]      |
| 13         | CAN8_TX       | GPIO_MCU[13]      |
| 14         | CAN9_TX       | GPIO_MCU[14]      |
| 15         | SPI2_CSN1     | GPIO_MCU[15]      |
| 16         | SPI2_CSN0     | GPIO_MCU[16]      |
| 17         | SPI2_MOSI     | GPIO_MCU[17]      |
| 18         | SPI2_MISO     | GPIO_MCU[18]      |
| 19         | SPI2_SCLK     | GPIO_MCU[19]      |
| 20         | SPI3_CSN0     | GPIO_MCU[20]      |
| 21         | SPI3_CSN1     | GPIO_MCU[21]      |
| 22         | SPI3_MOSI     | GPIO_MCU[22]      |
| 23         | SPI3_MISO     | GPIO_MCU[23]      |
| 24         | SPI3_SCLK     | GPIO_MCU[24]      |
| 25         | SPI4_CSN0     | GPIO_MCU[25]      |
| 26         | SPI4_CSN1     | GPIO_MCU[26]      |
| 27         | SPI4_MOSI     | GPIO_MCU[27]      |
| 28         | SPI4_MISO     | GPIO_MCU[28]      |
| 29         | SPI4_SCLK     | GPIO_MCU[29]      |
| 30         | SPI5_CSN0     | GPIO_MCU[30]      |
| 31         | SPI5_CSN1     | GPIO_MCU[31]      |
| 32         | SPI5_MOSI     | GPIO_MCU[32]      |
| 33         | SPI5_MISO     | GPIO_MCU[33]      |
| 34         | SPI5_SCLK     | GPIO_MCU[34]      |
| 35         | SPI6_CSN0     | GPIO_MCU[35]      |
| 36         | SPI6_CSN1     | GPIO_MCU[36]      |
| 37         | SPI6_MOSI     | GPIO_MCU[37]      |
| 38         | SPI6_MISO     | GPIO_MCU[38]      |
| 39         | SPI6_SCLK     | GPIO_MCU[39]      |
| 40         | XSPI_MOSI_IO0 | GPIO_MCU[40]      |
| 41         | XSPI_MISO_IO1 | GPIO_MCU[41]      |
| 42         | XSPI_WP_IO2   | GPIO_MCU[42]      |
| 43         | XSPI_HOLD_IO3 | GPIO_MCU[43]      |
| 44         | XSPI_OCT_IO4  | GPIO_MCU[44]      |
| 45         | XSPI_OCT_IO5  | GPIO_MCU[45]      |
| 46         | XSPI_OCT_IO6  | GPIO_MCU[46]      |
| 47         | XSPI_OCT_IO7  | GPIO_MCU[47]      |
| 48         | XSPI_SCLK     | GPIO_MCU[48]      |
| 49         | XSPI_SCLK_INV | GPIO_MCU[49]      |
| 50         | XSPI_DQS      | GPIO_MCU[50]      |
| 51         | EMAC_TX_CLK   | GPIO_MCU[51]      |
| 52         | EMAC_TX_EN    | GPIO_MCU[52]      |
| 53         | EMAC_TX_D3    | GPIO_MCU[53]      |
| 54         | EMAC_TX_D2    | GPIO_MCU[54]      |
| 55         | EMAC_TX_D1    | GPIO_MCU[55]      |
| 56         | EMAC_TX_D0    | GPIO_MCU[56]      |
| 57         | EMAC_RX_CLK   | GPIO_MCU[57]      |
| 58         | EMAC_RX_DV    | GPIO_MCU[58]      |
| 59         | EMAC_RX_D3    | GPIO_MCU[59]      |
| 60         | EMAC_RX_D2    | GPIO_MCU[60]      |
| 61         | EMAC_RX_D1    | GPIO_MCU[61]      |
| 62         | EMAC_RX_D0    | GPIO_MCU[62]      |
| 63         | XSPI_CSN      | GPIO_MCU[63]      |
| 64         | XSPI_RST_N    | GPIO_MCU[64]      |
| 65         | XSPI_ECC_FAIL | GPIO_MCU[65]      |
| 66         | XSPI_HYP_INT  | GPIO_MCU[66]      |
| 67         | BIFSPI_CSN    | GPIO_MCU[67]      |
| 68         | BIFSPI_SCLK   | GPIO_MCU[68]      |
| 69         | BIFSPI_MOSI   | GPIO_MCU[69]      |
| 70         | BIFSPI_MISO   | GPIO_MCU[70]      |
| 71         | PMIC_ERR0     | GPIO_MCU[71]      |
| 72         | JTG_TCK       | GPIO_MCU[72]      |
| 73         | JTG_TRSTN     | GPIO_MCU[73]      |
| 74         | JTG_TMS       | GPIO_MCU[74]      |
| 75         | JTG_TDI       | GPIO_MCU[75]      |
| 76         | JTG_TDO       | GPIO_MCU[76]      |
| 77         | EMAC_MDC      | GPIO_MCU[77]      |
| 78         | EMAC_MDIO     | GPIO_MCU[78]      |
| 79         | Reserved      | N/A               |
| 80         | I2C6_SCL      | GPIO_MCU[79]      |
| 81         | I2C6_SDA      | GPIO_MCU[80]      |
| 82         | I2C7_SCL      | GPIO_MCU[81]      |
| 83         | I2C7_SDA      | GPIO_MCU[82]      |
| 84         | I2C8_SCL      | GPIO_MCU[83]      |
| 85         | I2C8_SDA      | GPIO_MCU[84]      |
| 86         | PWM0_IO       | GPIO_MCU[85]      |
| 87         | PWM1_IO       | GPIO_MCU[86]      |
| 88         | CAN0_RX       | GPIO_AON[0]       |
| 89         | CAN1_RX       | GPIO_AON[1]       |
| 90         | CAN2_RX       | GPIO_AON[2]       |
| 91         | CAN3_RX       | GPIO_AON[3]       |
| 92         | CAN4_RX       | GPIO_AON[4]       |
| 93         | CAN5_RX       | GPIO_AON[5]       |
| 94         | CAN6_RX       | GPIO_AON[6]       |
| 95         | CAN7_RX       | GPIO_AON[7]       |
| 96         | CAN8_RX       | GPIO_AON[8]       |
| 97         | CAN9_RX       | GPIO_AON[9]       |
| 98         | LIN1_RXD      | GPIO_AON[10]      |
| 99         | LIN2_RXD      | GPIO_AON[11]      |
| 100        | Reserved      | N/A               |
| 101        | Reserved      | N/A               |
| 102        | Reserved      | N/A               |
| 103        | Reserved      | N/A               |
| 104        | Reserved      | N/A               |
| 105        | WAKEUP_IO     | GPIO_AON[12]      |

## Port_Func Module
The Port_Func module is provided by D-Robotics to initialize and configure all PINs belonging to a specific functional module, as well as to operate GPIOs for that module.

### Example: Configuring PIN Functions Using the Port_Func Module
#### Code Example
Refer to `samples/Spi/SPI_sample/Spi_sample.c` for usage examples. The basic usage logic is as follows:
```C
...
#include <Port_Func.h>

...

	/* Configure Pin for SPI5 */
	Port_SetFunctionPins(PORT_FUNC_SPI5);

...
```

#### Peripheral Configurations Provided by the Port_Func Module
The default peripheral PIN configurations provided by D-Robotics are defined in the file `McalCdd/Port/inc/Port_Func.h` using an enum type:
```C
...

typedef enum PinFunctions {
    PORT_FUNC_UART4,
    PORT_FUNC_UART5,
    PORT_FUNC_UART6,
    PORT_FUNC_SPI2,
    PORT_FUNC_SPI3,
    PORT_FUNC_SPI4,
    PORT_FUNC_SPI5,
    PORT_FUNC_SPI6,
    PORT_FUNC_SPI7,
    PORT_FUNC_CAN0,
    PORT_FUNC_CAN1,
    PORT_FUNC_CAN2,
    PORT_FUNC_CAN3,
    PORT_FUNC_CAN4,
    PORT_FUNC_CAN5,
    PORT_FUNC_CAN6,
    PORT_FUNC_CAN7,
    PORT_FUNC_CAN8,
    PORT_FUNC_CAN9,
    PORT_FUNC_I2C6,
    PORT_FUNC_I2C7,
    PORT_FUNC_I2C8,
    PORT_FUNC_I2C9,
    PORT_FUNC_PWM0,
    PORT_FUNC_PWM1,
    PORT_FUNC_PWM2,
    PORT_FUNC_PWM3,
    PORT_FUNC_PWM4,
    PORT_FUNC_PWM5,
    PORT_FUNC_PWM6,
    PORT_FUNC_PWM7,
    PORT_FUNC_PWM8,
    PORT_FUNC_PWM9,
    PORT_FUNC_PWM10,
    PORT_FUNC_PWM11,
    PORT_FUNC_PPS_IN0,
    PORT_FUNC_PPS_IN1,
    PORT_FUNC_PPS_IN2,
    PORT_FUNC_PPS_OUT,
    PORT_FUNC_EMAC,
    PORT_FUNC_MAX,
} PinFunc_e;

...

```
### Example: Operating GPIOs Using the Port_Func Module
The GPIO interfaces provided by Port_Func use PinIdx values listed in the [PIN Name List Corresponding to PIN Numbers in the Port_Func Module](#pin_list).

#### Code Example
Refer to `samples/Gpio/src/Gpio_sample.c` for usage examples. The basic usage logic is as follows:
```C
    /* Configure the "PinIdx" PIN as a GPIO function */
    RetVal = Port_SetGpioByIndex(PinIdx);
    /* Configure the direction of "PinIdx" PIN as OUTPUT and set the output level to Level */
    RetVal = Port_GpioDirectionOutput(PinIdx, Level);

    /* Configure the direction of "PinIdx" PIN as INPUT */
    RetVal = Port_GpioDirectionInput(PinIdx);

    /* Read the value of "PinIdx" PIN */
    RetVal = Port_GpioGetValue(PinIdx);

...
```  
```
:::info Note

    - The value returned by the "Port_GpioGetValue" function may vary when the external PIN is floating, as it is affected by the PinCtrl configuration;
    - The GPIO interfaces provided by the Port_Func module perform checks on the GPIO being operated. Certain pins are not allowed to be accessed. For details, refer to the `Gpio_Blacklist` array in the file `McalCdd/Port/src/Port_Func.c`:
        ```C
        const uint8_t Gpio_Blacklist[] = {
            0,  /* S100 Power related pins */
            5,  /* S100 debug uart tx */
            38, /* S100 Power related pins */
            15, /* S100 Power related pins */
            68, /* S100 Power related pins */
            69, /* S100 Power related pins */
            71, /* S100 Power related pins */
            80, /* S100 Power related pins */
            81, /* S100 Power related pins */
            82, /* S100 Power related pins */
            83, /* S100 Power related pins */
            AON_PIN_NUM(0),  /* S100 debug uart rx */
            AON_PIN_NUM(12), /* S100 Power related pins */
        };
       
:::
```
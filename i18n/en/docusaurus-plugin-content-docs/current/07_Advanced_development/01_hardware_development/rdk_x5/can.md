---
sidebar_position: 3
---
# CAN Usage

## Protocol Overview
**CAN**
CAN stands for Controller Area Network, which is a serial communication protocol standardized by ISO. It is widely used in real-time applications, utilizing twisted pair cables for signal transmission. It is one of the most widely used fieldbus technologies worldwide.

Advantages of CAN bus:
- **High reliability and strong anti-interference capabilities**  
  The physical layer chip uses differential signals and twisted pair wiring for transmission, effectively mitigating electromagnetic interference. The hardware-based data link layer automatically resolves synchronization issues between multiple nodes.

- **Strong error detection capabilities**  
  The hardware-based data link layer includes features like CRC and bit detection, ensuring near 100% detection of communication anomalies.

- **Comprehensive error management**  
  If a message arbitration fails or is corrupted during transmission, it will be automatically retransmitted. Nodes with severe errors will automatically disconnect from the bus, without affecting normal operation of the network.

- **Real-time performance**  
  CAN bus supports high data transmission speeds and can handle large amounts of data in real time, meeting the needs of automotive electronic control systems.

- **Low cost**  
  The hardware cost for CAN bus systems is relatively low, reducing the overall cost of automotive electronic control systems.

- **Long communication distance and high message transmission speed**  
  Direct communication can cover up to 10 km (at rates below 4 kbps); the transmission speed can reach up to 1 Mbps, with a maximum distance of 40 meters at this speed.

- **Multi-master communication support, lossless arbitration**  
  In the event that two nodes send messages simultaneously, the higher priority message will be transmitted without interference.

- **Eliminates the concept of "address"**  
  A key feature of CAN is the elimination of traditional address encoding, replaced by message-based communication.

- **High flexibility and ease of network expansion**  
  The number of nodes on a CAN network is not limited. Adding new nodes does not affect the existing hardware and software of the network.

**CAN FD**
The need for higher performance in CAN networks led to the development of CAN FD (Flexible Data-rate). CAN FD maintains the advantages of standard CAN but addresses its shortcomings with the following features:
- **Extended data field**  
  CAN FD supports a data field of up to 64 bytes per frame, compared to the 8 bytes in standard CAN. This greatly increases data transfer capacity.

- **Dual bit-rate mode**  
  CAN FD supports dual bit-rate operation, using a nominal bit-rate (up to 1 Mbps) for the arbitration phase and a higher bit-rate (up to 5 Mbps) for the data phase, enhancing data transmission speed while maintaining compatibility.

- **Improved CRC and padding bit counter**  
  CAN FD introduces improvements in CRC and padding bit counters to enhance error detection and data integrity.

- **Remote frame support removed**  
  CAN FD simplifies the frame structure by removing support for remote frames, making communication fully data-frame based.

## Interface Description
![img-20241009-1](../../../../../../../static/img/07_Advanced_development/01_hardware_development/rdk_x5/img-20241009-1.png)
- The RDK X5 provides a CAN communication interface, equipped with a 120-ohm terminal resistor switch. To enable the terminal resistor, simply close the switch.
- The terminal interface is of type SH1.0 1X3P.

## Module Overview
The RDK X5 integrates the **TCAN4550** chip.  
The **TCAN4550** is a CAN FD controller with an integrated transceiver supporting data rates of up to 8 Mbps. It complies with the ISO11898-1:2015 high-speed CAN data link layer specification and the ISO11898–2:2016 high-speed CAN physical layer requirements. The TCAN4550 interfaces between the CAN bus and the system processor via the Serial Peripheral Interface (SPI), supporting both classic CAN and CAN FD, and provides port expansion or CAN support for processors that do not support CAN FD. The TCAN4550 features differential transmission and reception capabilities for CAN FD transceivers.
- Supports CAN at 1 Mbps and CAN FD at 2 Mbps.

## Driver Guide
**dts**



  ```bash
  &spi5 {
    status = "okay";
    pinctrl-names = "default";
    pinctrl-0 = <&pinctrl_spi5 &lsio_gpio0_7 &lsio_gpio0_12>;

    tcan4x5x: tcan4x5x@0 {
      compatible = "ti,tcan4x5x";
      reg = <0>;
      #address-cells = <1>;
      #size-cells = <1>;
      spi-max-frequency = <10000000>;
      bosch,mram-cfg = <0x0 0 0 16 0 0 1 1>;
      interrupt-parent = <&ls_gpio0_porta>;
      interrupts = <12 IRQ_TYPE_EDGE_FALLING>;
      reset-gpios = <&ls_gpio0_porta 7 GPIO_ACTIVE_HIGH>;
    };
  };
  ```

## Driver Code

  ```bash
kernel\drivers\net\can\m_can\tcan4x5x-core.c
  ```

## can-utils Introduction

can-utils is an open-source toolset for the Linux operating system, specifically designed for tasks related to the CAN (Controller Area Network) bus. CAN bus is widely used in automotive and industrial automation for communication between devices. 

This toolset provides various command-line utilities for sending, receiving, and processing data on the CAN network. Some common tools include:
- **cansend**: Sends a single CAN frame.
- **candump**: Captures and displays data passing through the CAN interface.
- **canplayer**: Replays data recorded by `candump`.
- **cansniffer**: Displays changes in CAN data.

can-utils also includes advanced tools for tasks like configuring CAN hardware filters or debugging CAN devices and networks. These tools are accessible via the command line and can be easily integrated into scripts and automation systems.

### Basic Usage
**1. candump**
Displays, filters, and records CAN data

Basic usage:

  ```bash
candump can0
  ```
- Display all CAN data through the can0 interface.

Filter by specific ID:

  ```bash
candump can0,123:7FF
  ```
- Display CAN frames with ID 123.

Record data to a file:
  ```bash
candump -l can0
  ```
- This will record data passing through can0 to a file, with the default filename format as `candump-YYYY-MM-DD.log`.


**2. canplayer**
**canplayer** is used to replay CAN data logs recorded by `candump`.


Basic usage:
  ```bash
canplayer -I candump.log
  ```
- Replay CAN data from the file `candump.log`.


**3. cansend**
 used to send a specified CAN frame.

Basic usage:
  ```bash
cansend can0 123#1122334455667788
  ```
- Send a CAN frame with ID 123 and data `1122334455667788` to the can0 interface.

**4. cangen**
Generate random or specific-patterned CAN traffic for testing or simulation purposes.

Basic usage:
  ```bash
cangen can0 -I 1A -L 8 -D i -g 10 -n 100
  ```
- Generate 100 CAN frames with ID 1A, 8-byte length, and incrementing payloads on `can0`, with a 10ms interval between each frame.

**5. cansequence**  
Send a sequence of CAN frames with incrementing payloads and check for any frame loss.

Basic usage:
  ```bash
cansequence can0
  ```
- Send and monitor a sequence of CAN frames with incrementing payloads on `can0`.

**6. cansniffer**  
Used to display changes in CAN data, which is very useful for debugging and understanding the data flow.

Basic usage:
  ```bash
cansniffer can0
  ```
- Monitor and display any changes in CAN data on the `can0` interface.

## Test Guide

### Loopback Test
Configure the CAN bus bitrate and enable loopback mode.

  ```bash
ip link set down can0
ip link set can0 type can bitrate 125000
ip link set can0 type can loopback on
ip link set up can0
  ```

 View CAN0 Configuration Information
  ```bash
ip -details link show can0
  ```
![img-20241009-2](../../../../../../../static/img/07_Advanced_development/01_hardware_development/rdk_x5/img-20241009-2.png)

To check the configuration details of the `can0` interface, you can use the following command:

  ```bash
candump can0 -L &
  ```
To send a test message and verify that data is immediately received, use the following command:

  ```bash
cansend can0 123#1122334455667788
  ```
Test Results
![img-20241009-3](../../../../../../../static/img/07_Advanced_development/01_hardware_development/rdk_x5/img-20241009-3.png)

### CANFD Loopback Test

The arbitration segment bitrate is set to 500K, and the data segment bitrate is set to 2M.


  ```bash
ip link set can0 down
ip link set can0 type can bitrate 500000 dbitrate 2000000  fd on
ip link set can0 type can loopback on
ip link set can0 up
  ```
Sending and Receiving CAN FD Data
  ```bash
candump can0 -L &
cansend can0 123##300112233445566778899aabbccddeeff
  ```

### Dual Device Communication Test

**Hardware Connection**  
![img-20241009-4](../../../../../../../static/img/07_Advanced_development/01_hardware_development/rdk_x5/img-20241009-4.png)  
- GND to GND, L to L, H to H.

**Test Instructions**  
Configure both devices with the same CAN bus bit rate.

  ```bash
ip link set down can0
ip link set can0 type can bitrate 125000
ip link set up can0
  ```
One device should be configured to **receive**.
  ```bash
candump can0 -L
  ```
One device should be configured to **send**.
  ```bash
cansend can0 123#1122334455667788
  ```

## Application Guide

Linux provides a SocketCAN interface, which makes CAN bus communication similar to Ethernet communication. The application programming interface is more universal and flexible. Using SocketCAN is like using TCP/IP.

Here is a simple example of sending and receiving CAN data:

 Configure CAN Bus Bitrate and Loopback Mode

  ```bash
ip link set down can0
ip link set can0 type can bitrate 125000
ip link set can0 type can loopback on
ip link set up can0
  ```

write code
  ```bash
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <fcntl.h>

int main() {
    int sock;
    struct sockaddr_can addr;
    struct can_frame frame;

    // 创建 Socket
    sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) {
        perror("Socket");
        return 1;
    }

    // 获取 can0 接口
    struct ifreq ifr;
    strcpy(ifr.ifr_name, "can0");
    ioctl(sock, SIOCGIFINDEX, &ifr);

    // 绑定 Socket
    addr.can_family = PF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(sock, (struct sockaddr *)&addr, sizeof(addr));

    // 发送消息
    frame.can_id = 0x123;
    frame.can_dlc = 4;
    memcpy(frame.data, "\xde\xad\xbe\xef", 4);
    write(sock, &frame, sizeof(struct can_frame));

    // 接收消息
    while (1) {
        int nbytes = read(sock, &frame, sizeof(struct can_frame));
        if (nbytes > 0) {
            printf("Received: ID: 0x%X Data: ", frame.can_id);
            for (int i = 0; i < frame.can_dlc; i++) {
                printf("%02X ", frame.data[i]);
            }
            printf("\n");
        }
    }

    close(sock);
    return 0;
}
  ```

Compile and Run

Save the code to a file named `can_loopback.c`.

compile the program:

  ```bash
gcc -o can_loopback can_loopback.c
  ```
Run the Program

  ```bash
sudo ./can_loopback
  ```
The code will send a CAN message and continuously receive and print the messages it receives.

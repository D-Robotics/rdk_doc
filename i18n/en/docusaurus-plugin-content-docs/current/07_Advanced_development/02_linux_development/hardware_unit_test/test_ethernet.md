---
sidebar_position: 6
---
# Network Performance Testing

## Test Description

Using the `iperf3` tool for testing (the SDK source code package is equipped with this tool).
`iperf3` is a tool for measuring network bandwidth for `TCP`, `UDP`, and `SCTP`. It is used to measure the maximum bandwidth that can be achieved on an IP network.

## Test Method

First, make sure that the development board and the PC can ping each other before proceeding to the next test.

### PC Side

The PC side acts as the server, and execute `iperf3 -s -f m`.

### Development Board Side

The development board acts as the client, and execute `iperf3 -c 192.168.1.1 -f m -i 1 -t 60` to perform the network testing.

### Common Parameters for iperf3

For the server, the common configuration parameters for `iperf3` are as follows:

```bash
-s       indicates the server side;
-p port  defines the port number;
-i sec   sets the time interval between each report in seconds. If set to a non-zero value, the test report will be output according to this time interval. The default value is zero.
```

For the client, the common configuration parameters for `iperf3` are as follows:

```shell
-c ip   indicates the IP address of the server;
-p port indicates the port number of the server;
-t sec  specifies the duration of the transmission test. Iperf will repeatedly send data packets of the specified length within the specified time. The default is 10 seconds.
-i sec  sets the time interval between each report in seconds. If set to a non-zero value, the test report will be output according to this time interval. The default value is zero;
-w size sets the socket buffer to the specified size. For TCP, this setting is the TCP window size. For UDP, this setting is the buffer size for receiving UDP packets, limiting the maximum value of the packets that can be received.
--logfile    saves the output test results to a file.
-J  outputs test results in JSON format.
-R  performs reverse transmission. By default, iperf3 uses upload mode: the client sends data, and the server receives it; if you need to test download speed, use the -R parameter on the client side.
```

## Test Criteria

Received Bandwidth: **870Mbits/sec**Sending Bandwidth: **940Mbits/sec**
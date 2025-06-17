---
sidebar_position: 1
---

# ip

The **ip** command is similar to the `ifconfig` command, but more powerful. Its main function is to display or configure network devices.

The **ip** command is an enhanced version of the network configuration tool in Linux, which replaces the ifconfig command.

## Syntax

```
ip [ OPTIONS ] OBJECT { COMMAND | help }
ip [ -force ] -batch filename
```

- **OBJECT**:

  ```shell
  OBJECT := { link | address | addrlabel | route | rule | neigh | ntable |
         tunnel | tuntap | maddress | mroute | mrule | monitor | xfrm |
         netns | l2tp | macsec | tcp_metrics | token }

  -V: Display version information of the command;
  -s: Output more detailed information;
  -f: Force the usage of the specified protocol family;
  -4: Specify that the network layer protocol used is IPv4;
  -6: Specify that the network layer protocol used is IPv6;
  -0: Output each record in a single line, even if the content is long;
  -r: When displaying hosts, use domain names instead of IP addresses.
  ```

- **OPTIONS**:

  ```shell
  OPTIONS := { -V[ersion] | -s[tatistics] | -d[etails] | -r[esolve] |
          -h[uman-readable] | -iec |
          -f[amily] { inet | inet6 | ipx | dnet | bridge | link } |
          -4 | -6 | -I | -D | -B | -0 |
          -l[oops] { maximum-addr-flush-attempts } |
          -o[neline] | -t[imestamp] | -ts[hort] | -b[atch] [filename] |
          -rc[vbuf] [size] | -n[etns] name | -a[ll] }

  Network object: Specify the network object to manage;
  Specific operation: Perform specific operation on the specified network object;
  help: Display help information on the supported operation commands of the network object.
  ```

------

## Common Commands

```shell
ip link show                     # Show network interface information
ip link set eth0 up             # Enable network card
ip link set eth0 down            # Disable network card
ip link set eth0 promisc on      # Enable promiscuous mode for network card
ip link set eth0 promisc off     # Disable promiscuous mode for network card
ip link set eth0 txqueuelen 1200 # Set the queue length for the network card
ip link set eth0 mtu 1400        # Set the maximum transmission unit for the network card

ip addr show     # Show IP information for the network card
ip addr add 192.168.0.1/24 dev eth0 # Assign IP address 192.168.0.1 to eth0 network card
ip addr del 192.168.0.1/24 dev eth0 # Delete IP address of eth0 network card

ip route show # Show system routes
ip route add default via 192.168.1.254   # Set the default route for the system
ip route list                 # View route information
ip route add 192.168.1.0/24  via  192.168.0.254 dev eth0 # Set the gateway for 192.168.4.0 network segment to 192.168.0.254, using eth0 interface
ip route add default via  192.168.0.254  dev eth0        # Set the default gateway to 192.168.0.254
ip route del 192.168.1.0/24   # Delete the gateway for 192.168.4.0 network segment
ip route del default          # Delete the default route
ip route delete 192.168.1.0/24 dev eth0 # Delete route

```

Get all network interfaces of the host

```shell
ip link | grep -E '^[0-9]' | awk -F: '{print $2}'
```
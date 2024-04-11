# Linux SLIP TUN Interface

## Introduction

This repository provides a Linux SLIP (Serial Line Internet Protocol) TUN (Network Tunneling) interface.
It allows communication between a Linux system and a device connected via a serial port using SLIP protocol.

Special thanks to [marcinbor85](https://github.com/marcinbor85/slip.git) for his custom implementation of SLIP with CRC.

⚠️ **Warning:** This is not a standard SLIP protocol. It includes a CRC checksum for packet verification.


## Clone

```bash
git clone --recurse-submodules git@github.com:evil-lrs/slip-netif.git
```

## Build

```bash
make
make install
```

## Usage

```bash
slip-netif -s <serial_name> -b <baud_rate> [-i <interface_name>] [-v]
```

## Configure Interface

To configure the interface, use the following command:

```bash
ifconfig tun0 10.1.1.1/24 up mtu 1500
```

## Prevent Broadcast/Multicast Packet Flooding (Optional)

To prevent broadcast/multicast packet flooding, you can use iptables:

```bash
sudo iptables -A OUTPUT -o tun0 -m pkttype --pkt-type multicast -j DROP
sudo iptables -A OUTPUT -o tun0 -m pkttype --pkt-type broadcast -j DROP
```
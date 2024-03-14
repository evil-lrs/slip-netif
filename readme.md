# Linux SLIP TUN Interface

## Introduction

This repository provides a Linux SLIP (Serial Line Internet Protocol) TUN (Network Tunneling) interface.
It allows communication between a Linux system and a device connected via a serial port using SLIP protocol.

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
slip-netif -s [serial speed] -l [serial device] [-d]
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
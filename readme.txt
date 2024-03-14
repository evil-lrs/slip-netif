
Clone



Build

    make
    make install

Usage

    slip-netif


Configure interface
    sudo ifconfig tun0 10.1.1.1 up mtu 500 -multicast

Block broadcast/multicast flooding

    sudo iptables -A OUTPUT -o eth0 -m pkttype --pkt-type multicast -j DROP
    sudo iptables -A OUTPUT -o eth0 -m pkttype --pkt-type broadcast -j DROP

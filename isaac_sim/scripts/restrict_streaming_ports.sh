#!/usr/bin/env bash
# Run on the VM only after approving IP-restricted streaming exposure.
# This changes only TCP 49100/8210 and UDP 47998; never SSH or default policy.
set -euo pipefail
client_ip="${1:?Usage: restrict_streaming_ports.sh CLIENT_IPV4}"
python3 -c 'import ipaddress,sys; ipaddress.IPv4Address(sys.argv[1])' "${client_ip}"
sudo iptables -N VGM-STREAM
sudo iptables -A VGM-STREAM -i lo -j ACCEPT
sudo iptables -A VGM-STREAM -s "${client_ip}/32" -j ACCEPT
sudo iptables -A VGM-STREAM -j DROP
sudo iptables -I INPUT 1 -p tcp -m multiport --dports 49100,8210 -j VGM-STREAM
sudo iptables -I INPUT 1 -p udp --dport 47998 -j VGM-STREAM
# No IPv6 streaming clients are authorized.
sudo ip6tables -I INPUT 1 -p tcp -m multiport --dports 49100,8210 -j DROP
sudo ip6tables -I INPUT 1 -p udp --dport 47998 -j DROP
sudo iptables -S VGM-STREAM
sudo iptables -S INPUT
sudo ip6tables -S INPUT

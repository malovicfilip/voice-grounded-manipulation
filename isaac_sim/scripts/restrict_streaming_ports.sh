#!/usr/bin/env bash
# Run on the VM only after approving IP-restricted streaming exposure.
# This changes only TCP 49100/8210 and UDP 47998; never SSH or default policy.
set -euo pipefail
client_ip="${1:?Usage: restrict_streaming_ports.sh CLIENT_IPV4}"
python3 -c 'import ipaddress,sys; ipaddress.IPv4Address(sys.argv[1])' "${client_ip}"
if sudo nft list table inet vgm_stream >/dev/null 2>&1; then
  echo 'Existing vgm_stream table: inspect it; refusing to overwrite.' >&2
  exit 2
fi
# One atomic nftables transaction, compatible with Brev's native nft rules.
# The inet table covers IPv4 and IPv6. Only the named IPv4 client is allowed.
sudo nft -f - <<RULES
table inet vgm_stream {
  chain allow_client {
    iifname "lo" accept
    ip saddr ${client_ip}/32 accept
    counter drop
  }
  chain input {
    type filter hook input priority -10; policy accept;
    tcp dport { 49100, 8210 } jump allow_client
    udp dport 47998 jump allow_client
  }
}
RULES
sudo nft list table inet vgm_stream

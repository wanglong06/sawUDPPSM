# allow port forwarding
sudo iptables -A FORWARD -i eth2 -j ACCEPT
sudo iptables -A FORWARD -i eth3 -j ACCEPT
sudo echo '1' > /proc/sys/net/ipv4/conf/eth0/forwarding
sudo echo '1' > /proc/sys/net/ipv4/conf/eth2/forwarding

# cleanup all existing rules
sudo iptables -F
sudo iptables -X
sudo iptables -t nat -F
sudo iptables -t nat -X
sudo iptables -t mangle -F
sudo iptables -t mangle -X
sudo iptables -P INPUT ACCEPT
sudo iptables -P FORWARD ACCEPT
sudo iptables -P OUTPUT ACCEPT

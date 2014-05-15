# allow port forwarding
sudo echo '0' > /proc/sys/net/ipv4/conf/eth2/forwarding
sudo echo '0' > /proc/sys/net/ipv4/conf/eth3/forwarding

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

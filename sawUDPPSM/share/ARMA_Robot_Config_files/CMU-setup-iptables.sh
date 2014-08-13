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

# to CMU 
sudo iptables -t nat -A PREROUTING -p udp -s 192.168.1.167 --dport 10020 --j DNAT --to 128.2.178.3:10020
# from CMU
#sudo iptables -t nat -A PREROUTING -p udp -s 128.2.178.3 --dport 10020 --j DNAT --to 192.168.1.167:10020
sudo iptables -t nat -A POSTROUTING -j MASQUERADE

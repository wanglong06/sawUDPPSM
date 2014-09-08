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

# from JHU Master Right
sudo iptables -t nat -A PREROUTING -p udp -s 128.220.72.28 --dport 10005 --j DNAT --to 192.168.1.201:10005
# to JHU Slave Right
sudo iptables -t nat -A PREROUTING -p udp -s 192.168.1.201 --dport 10005 --j DNAT --to 128.220.72.28:10005
# from JHU Master Left
sudo iptables -t nat -A PREROUTING -p udp -s 128.220.72.28 --dport 10004 --j DNAT --to 192.168.1.201:10004
# to JHU Slave Left
sudo iptables -t nat -A PREROUTING -p udp -s 192.168.1.201 --dport 10004 --j DNAT --to 128.220.72.28:10004
# From JHU dragonfly camera, in this protocal, JHU will fetch the video source
sudo iptables -t nat -A PREROUTING -p tcp -s 128.220.72.28 --dport 10010 --j DNAT --to 192.168.1.105:10010
# to CMU 
sudo iptables -t nat -A PREROUTING -p udp -s 192.168.1.201 --dport 10020 --j DNAT --to 128.220.72.28:10020
# from CMU
sudo iptables -t nat -A PREROUTING -p udp -s 128.220.72.28 --dport 10020 --j DNAT --to 192.168.1.201:10020
sudo iptables -t nat -A POSTROUTING -j MASQUERADE

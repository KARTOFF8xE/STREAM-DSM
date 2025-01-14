#!/bin/zsh

sudo usermod -a -G softhsm $USERNAME
mkdir /workspaces/DiplArbeitContainer/ws/src/Fast-DDS
cd /workspaces/DiplArbeitContainer/ws/src/Fast-DDS
p11-kit list-modules
openssl engine pkcs11 -t
wget https://raw.githubusercontent.com/eProsima/Fast-DDS/master/fastdds.repos /workspaces/DiplArbeitContainer/ws/fastdds.repos
mkdir src
vcs import src < fastdds.repos
# colcon build --packages-up-to fastdds --cmake-args -DFASTDDS_STATISTICS=ON --symlink-install
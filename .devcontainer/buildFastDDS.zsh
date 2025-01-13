#!/bin/zsh

sudo usermod -a -G softhsm $USERNAME
mkdir /workspaces/DiplArbeitContainer/Fast-DDS
cd Fast-DDS
p11-kit list-modules
openssl engine pkcs11 -t
wget https://raw.githubusercontent.com/eProsima/Fast-DDS/master/fastdds.repos
mkdir src
vcs import src < fastdds.repos
colcon build --packages-up-to fastdds --cmake-args -DFASTDDS_STATISTICS=ON --symlink-install
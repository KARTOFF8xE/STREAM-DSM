#!/bin/zsh

filename=babeltrace2-2.0.6.tar.bz2

mkdir ~/Downloads
cd ~/Downloads
wget https://www.efficios.com/files/babeltrace/${filename}
echo "\e[1munzipping ${filename} \e[33m(showing warnings and errors only)\e[0m"
tar -xvjf ${filename} > /dev/null
cd babeltrace2-2.0.6
echo "\e[1mconfiguring ${filename} \e[33m(showing warnings and errors only)\e[0m"
BABELTRACE_DEV_MODE=1 BABELTRACE_MINIMAL_LOG_LEVEL=TRACE ./configure --disable-debug-info > /dev/null
echo "\e[1mmake \e[33m(showing warnings and errors only)\e[0m"
make > /dev/null
echo "\e[1msudo make install \e[33m(showing warnings and errors only)\e[0m"
sudo make install > /dev/null
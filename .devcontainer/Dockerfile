FROM osrf/ros:rolling-desktop AS rollyrolling
WORKDIR /usr/local/app
ARG USERNAME=ubuntu

RUN set -x && \
  echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME && \
  chmod 0440 /etc/sudoers.d/$USERNAME

RUN apt update && apt upgrade -y
RUN apt install apt-utils -y

# zsh
RUN apt install wget zsh curl rustc cargo -y
RUN git clone https://github.com/ogham/exa.git && \
  cd exa && \
  cargo build --release && \
  cp target/release/exa /usr/local/bin/
RUN wget https://raw.githubusercontent.com/KARTOFF8xE/configs/main/zsh/.zshrc -O /home/$USERNAME/.zshrc
RUN wget "https://github.com/sharkdp/vivid/releases/download/v0.8.0/vivid_0.8.0_amd64.deb" -O /tmp/vivid && \
  dpkg -i /tmp/vivid
RUN mkdir -p /home/$USERNAME/.zsh_scripts && \
  git clone https://github.com/jimmijj/chromatic-zsh.git /home/$USERNAME/.zsh_scripts/chromatic-zsh
RUN mkdir -p /home/$USERNAME/.zsh_scripts/plugins && \
  git clone https://github.com/zsh-users/zsh-autosuggestions.git /home/$USERNAME/.zsh_scripts/plugins/zsh-autosuggestions && \
  wget https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/plugins/git/git.plugin.zsh -O /home/$USERNAME/.zsh_scripts/plugins/git.plugin.zsh && \
  zsh -c "source /home/$USERNAME/.zsh_scripts/plugins/git.plugin.zsh"
RUN mkdir -p /home/$USERNAME/.cache/zsh && \
  touch /home/$USERNAME/.cache/zsh/history
RUN SNIPPET="export HISTFILE=/home/$USERNAME/.cache/zsh/history \nsetopt APPEND_HISTORY \nsetopt INC_APPEND_HISTORY \nsetopt SHARE_HISTORY\n" \
  && echo "$SNIPPET" >> "/home/$USERNAME/.zshrc"
RUN chmod 777 /home/$USERNAME/.cache/zsh
RUN chsh $user -s $(which zsh)

# ros2-tracing
RUN apt update && apt upgrade -y
RUN apt install lttng-tools liblttng-ust-dev liblttng-ctl-dev babeltrace2 -y
RUN apt install python3-babeltrace python3-lttng python3-pytest-cov -y

# Babeltrace
RUN apt install wget -y
ARG filename=babeltrace2-2.0.6.tar.bz2
RUN wget https://www.efficios.com/files/babeltrace/${filename}
RUN tar -xvjf ${filename}
RUN cd babeltrace2-2.0.6 && \
  BABELTRACE_DEV_MODE=1 BABELTRACE_MINIMAL_LOG_LEVEL=TRACE ./configure --disable-debug-info && \
  make && \
  make install

# install requirements for Fast-DDS with statistics Module
RUN apt update && apt upgrade -y
RUN apt install cmake g++ python3-pip \
 libasio-dev libtinyxml2-dev libssl-dev libp11-dev libengine-pkcs11-openssl \
 softhsm2 pip -y
RUN pip3 install --break-system-packages -U colcon-common-extensions vcstool

# nlohmann/json.git
RUN apt update && apt upgrade -y
RUN git clone https://github.com/nlohmann/json.git /home/$USERNAME/json && \
  cd /home/$USERNAME/json && \
  mkdir -p build && \
  cd build && \
  cmake .. && \
  make install

# install process exporter
RUN apt-get update && apt-get install -y wget tar
RUN wget https://github.com/ncabatoff/process-exporter/releases/download/v0.7.7/process-exporter-0.7.7.linux-amd64.tar.gz -O /tmp/process-exporter.tar.gz \
 && tar -xzf /tmp/process-exporter.tar.gz -C /usr/local/bin --strip-components=1 process-exporter-0.7.7.linux-amd64/process-exporter \
 && rm /tmp/process-exporter.tar.gz
COPY configs/process-exporter.yml /etc/process-exporter.yml

# other stuff
RUN apt install nano -y
RUN apt install openssh-client -y
RUN apt install terminator -y
RUN apt install tmux -y
RUN apt install htop -y
RUN apt install gdb -y
RUN apt install python3-requests -y
RUN apt update && apt upgrade -y

USER $USERNAME
WORKDIR /home/$USERNAME
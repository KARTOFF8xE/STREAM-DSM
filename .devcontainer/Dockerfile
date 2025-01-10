FROM osrf/ros:humble-desktop AS humblihumble
WORKDIR /usr/local/app


# non-root user support
RUN apt install sudo -y
ARG USERNAME=Georg
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME
RUN useradd --uid $USER_UID --gid $USER_GID -m $USERNAME

RUN set -x && \
  echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME && \
  chmod 0440 /etc/sudoers.d/$USERNAME

RUN apt update && apt upgrade -y
RUN apt install apt-utils -y

# zsh
## install packages
RUN apt install wget zsh exa curl -y
## get .zshrc
RUN wget https://raw.githubusercontent.com/KARTOFF8xE/configs/main/zsh/.zshrc -O /home/$USERNAME/.zshrc
## get vivid-Repo
RUN wget "https://github.com/sharkdp/vivid/releases/download/v0.8.0/vivid_0.8.0_amd64.deb" -O /tmp/vivid && \
  dpkg -i /tmp/vivid
## clone chromatic-zsh
RUN mkdir -p /home/$USERNAME/.zsh_scripts && \
  git clone https://github.com/jimmijj/chromatic-zsh.git /home/$USERNAME/.zsh_scripts/chromatic-zsh
## adding plugins
RUN mkdir -p /home/$USERNAME/.zsh_scripts/plugins && \
  git clone https://github.com/zsh-users/zsh-autosuggestions.git /home/$USERNAME/.zsh_scripts/plugins/zsh-autosuggestions && \
  wget https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/plugins/git/git.plugin.zsh -O /home/$USERNAME/.zsh_scripts/plugins/git.plugin.zsh && \
  zsh -c "source /home/$USERNAME/.zsh_scripts/plugins/git.plugin.zsh"
## add history
RUN mkdir -p /home/$USERNAME/.cache/zsh && \
  touch /home/$USERNAME/.cache/zsh/history
## change login shell
RUN chsh $user -s $(which zsh)

# ros2/tracing
RUN apt update
RUN apt install lttng-tools liblttng-ust-dev -y
RUN apt install python3-babeltrace python3-lttng python3-pytest-cov -y

# other stuff
RUN apt install ament-cmake -y

# set user
USER $USERNAME
WORKDIR /home/$USERNAME
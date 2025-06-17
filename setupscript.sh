mkdir -p /opt/STREAMDSM

apt install lttng-tools liblttng-ust-dev liblttng-ctl-dev babeltrace2 -y
apt install python3-babeltrace python3-lttng python3-pytest-cov -y

apt install libglib2.0-dev -y
filename=babeltrace2-2.0.6.tar.bz2
  cd /opt/STREAMDSM && \
  apt install wget -y && \
  wget https://www.efficios.com/files/babeltrace/${filename} && \
  tar -xvjf ${filename}
cd /opt/STREAMDSM/babeltrace2-2.0.6 && \
  BABELTRACE_DEV_MODE=1 BABELTRACE_MINIMAL_LOG_LEVEL=TRACE ./configure --disable-debug-info && \
  make && \
  make install

apt update && apt upgrade -y
git clone https://github.com/nlohmann/json.git /opt/STREAMDSM/json && \
  cd /opt/STREAMDSM/json && \
  mkdir -p build && \
  cd build && \
  cmake .. && \
  make install

apt-get update && apt-get install wget tar -y
wget https://github.com/ncabatoff/process-exporter/releases/download/v0.7.7/process-exporter-0.7.7.linux-amd64.tar.gz -O /opt/STREAMDSM/process-exporter.tar.gz && \
  tar -xzf /opt/STREAMDSM/process-exporter.tar.gz -C /usr/local/bin --strip-components=1 process-exporter-0.7.7.linux-amd64/process-exporter && \
  rm /opt/STREAMDSM/process-exporter.tar.gz

apt install libcurl4-openssl-dev -y
apt install tmux -y

apt-get update && apt-get install wget tar -y
wget https://github.com/ncabatoff/process-exporter/releases/download/v0.7.7/process-exporter-0.7.7.linux-amd64.tar.gz -O /tmp/process-exporter.tar.gz \
 && tar -xzf /tmp/process-exporter.tar.gz -C /usr/local/bin --strip-components=1 process-exporter-0.7.7.linux-amd64/process-exporter \
 && rm /tmp/process-exporter.tar.gz
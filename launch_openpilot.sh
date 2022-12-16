#!/usr/bin/bash
set -e

# wait for weston to come up
sleep 5

# launch spinner
echo "Building" | ./selfdrive/ui/spinner &
spinner_pid=$!

# build
scons selfdrive/ui/_notouch selfdrive/ui/_ui -j8 --extras
kill -9 $spinner_pid

# launch ui and let user set up ssh
./selfdrive/ui/ui || true

echo "Installing dependencies" | ./selfdrive/ui/spinner &
spinner_pid=$!

# install deps
sudo apt update
sudo apt-get install -y --no-install-recommends gstreamer1.0-tools gstreamer1.0-libav gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad || true
kill -9 $spinner_pid

# convert videos to mpeg4
echo "Downloading videos" | ./selfdrive/ui/spinner &
spinner_pid=$!
./selfdrive/assets/videos/download-videos.sh
kill -9 $spinner_pid

# launch notouch ui
./selfdrive/ui/notouch

#!/bin/bash
set -e
cd "$(dirname "$0")"

AZ_BASEDIR="https://commadataci.blob.core.windows.net/cesdemo"
declare -a videos=("three-rotating.mp4" "website-home-video.mp4")

echo "downloading and converting videos to mpeg4"
mkdir -p out/

for f in "${videos[@]}"; do
  wget "$AZ_BASEDIR/$f" -O "$f"
  ffmpeg -y -i "$f" -vf scale=2160:1080 -c:v mpeg4 "out/$f"
done;

echo "Success!"

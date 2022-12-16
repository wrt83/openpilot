#!/bin/bash
set -e
cd "$(dirname "$0")"

AZ_BASEDIR="https://commadataci.blob.core.windows.net/cesdemo/out"
declare -a videos=("three-rotating.mp4" "website-home-video.mp4")

echo "downloading videos"
mkdir -p out/

for f in "${videos[@]}"; do
  wget "$AZ_BASEDIR/$f" -O "out/$f"
done;

echo "Success!"

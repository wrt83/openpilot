#!/bin/bash
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
cd $DIR

if [ ! -d rapidjson ]; then
 git clone https://github.com/Tencent/rapidjson.git
fi

cd rapidjson
git fetch
git checkout a98e99992bd633a2736cc41f96ec85ef0c50e44d

rm -rf $DIR/include
cp -r include/ $DIR/include/

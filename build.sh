#!/bin/bash

BASE_DIR=$(cd $(dirname $0);pwd)
echo "BASE_DIR: ${BASE_DIR}"

cd ${BASE_DIR}/

mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j
make install
rm -rf build
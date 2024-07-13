#!/bin/bash

BASE_DIR=$(cd $(dirname $0);pwd)
cd ${BASE_DIR}

mkdir release
cd release
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j8
make test
# make install
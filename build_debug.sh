#!/bin/bash

BASE_DIR=$(cd $(dirname $0);pwd)
cd ${BASE_DIR}

mkdir debug
cd debug
cmake -DCMAKE_BUILD_TYPE=Debug ..
make -j8
make test
# make install
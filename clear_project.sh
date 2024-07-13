#!/bin/bash

BASE_DIR=$(cd $(dirname $0);pwd)
cd ${BASE_DIR}

rm -r build/
rm -r release/
rm -r debug/
rm -r install/
rm -r unit_test/test_data/*.txt
rm -r unit_test/test_data/*.bin
rm -r unit_test/test_data/*.json
#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

pushd $(dirname $SCRIPT_DIR)

find . -iname *.h -o -iname *.cpp | xargs clang-format -i
python3 -m black .
python3 -m cmakelang.format -i CMakeLists.txt cmake/spark_dsgConfig.cmake.in python/CMakeLists.txt

popd

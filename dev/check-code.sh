#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# partially based on: https://stackoverflow.com/questions/4632028/how-to-create-a-temporary-directory
WORK_DIR=$(mktemp -d)

if [[ ! "$WORK_DIR" || ! -d "$WORK_DIR" ]]; then
  echo "Could not create temp dir"
  exit 1
fi

function cleanup {
  rm -rf "$WORK_DIR"
  echo "Deleted temp working directory $WORK_DIR"
}

trap cleanup EXIT


pushd $(dirname $SCRIPT_DIR)

find ./python -iname '*.py' | xargs python3 -m flake8

pushd $WORK_DIR
cmake $(dirname $SCRIPT_DIR) -DSPARK_DSG_BUILD_TESTS=OFF -DCMAKE_EXPORT_COMPILE_COMMANDS=ON 2>&1 > /dev/null
popd

find src -iname *.cpp | xargs clang-tidy -p $WORK_DIR/compile_commands.json

popd

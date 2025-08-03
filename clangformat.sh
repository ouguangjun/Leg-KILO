#!/usr/bin/env bash

for dir in ./legkilo/include ./legkilo/src; do
  find "$dir" -type f \( -iname "*.cc" -o -iname "*.hpp" -o -iname "*.h" \) \
    -exec clang-format -i -style=file {} +
done

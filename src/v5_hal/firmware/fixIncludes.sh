#!/bin/bash

grep -irl '#include' $1 | xargs perl -pi \
    -e 's/^#include "(?!main)(?!pros)(?!stddef)(?!stdlib)(?!string)(?!stdint)(?!math)(?!ros_lib)/#include "ros_lib\//g' \

#!/usr/bin/env bash -eu

PX4_FUZZ=1 make apn_danfe-v1_default
cp build/apn_danfe-v1_default/bin/px4 $OUT/px4

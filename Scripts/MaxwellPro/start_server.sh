#!/bin/sh
killstdiserver
stdiserver -t 2 -b 524288 -m 1500 -B 1536 -i 8 -h p1p2 -P 7021 -l p1p1

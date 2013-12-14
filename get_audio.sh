#!/bin/sh
arecord -f S16_LE -c2 -d 10 -D hw:1,0 -t raw | lame -r - /tmp/robot.mp3
#arecord -f S16_LE -c2 -d 10 -D hw:1,0 /tmp/robot.wav

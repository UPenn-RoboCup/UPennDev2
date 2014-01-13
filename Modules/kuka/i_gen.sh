#!/bin/sh
for name in `ls /home/youbot/youbot_driver/youbot/*.hpp`
do
	echo '#include "youbot/$name"' >> kuka.i
done

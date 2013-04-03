#!/bin/sh
for name in `ls /usr/local/include/openrave-0.9/openrave`
do
	echo "#include <openrave/$name>" >> openrave.i
done
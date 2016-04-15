#!/bin/sh
#c++ -g -Wall kdtree.c system_single_integrator.cpp rrts_main.cpp -o rrts

echo "Compiling..."

c++ -O2 -Wall kdtree.cpp system_single_integrator.cpp Transform.cpp rrts_humanoid.cpp -o /tmp/test_rrts

# Check the exit status code and immediately run if all OK
if [ "$?" -ne "0" ]; then
  echo "Sorry, cannot compile"
  exit 1
fi

echo "Done!"
echo ""

exec /tmp/test_rrts

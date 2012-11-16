THOR
====

Tactical Hazardous Operations Robot
-----------------------------------

This is the development repository for team THOR.

Dependencies
------------

For Ubuntu 12.04, install the necessary dependencies using:

    sudo apt-get install build-essential lua5.1 liblua5.1-0-dev luajit swig \
    libboost1.46-dev mesa-common-dev gnuplot libpopt-dev libncurses5-dev

    sudo ln -s /usr/bin/luajit* /usr/bin/luajit

Build Instructions
------------------

To build the codebase, run 'make' with one of the following options:

    make ash        # for operating the physical ASH robot
    make webots_ash # for running ASH simulations in webots
    make teststand  # for operating the linear actuator teststand
    make tools      # for run-time monitoring using Matlab

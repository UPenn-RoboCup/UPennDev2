-- Script to change the ID of servo

require('Dynamixel');
require('unix');

twait = 0.010;

Dynamixel.open();

current_id = 1;
new_id = 4;

Dynamixel.set_id(current_id, new_id);
unix.usleep(100000);
Dynamixel.ping_probe(twait);



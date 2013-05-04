-- Script to move dynamixel servos to desired angle
require ('Dynamixel');

twait = 0.010;

Dynamixel.open();
Dynamixel.ping_probe(twait);

curr_id = 6; -- Find out the id of the servo to be tested using test_dynamixel.lua script or using the line above

curr_posn = Dynamixel.get_position(curr_id); -- Get the current postion of the servo

Dynamixel.set_command(curr_id, curr_posn+250); -- Add or subtract within limits. This script does NOT pop out a warning / error message
                                               -- if the servo is being driven outside its limits. It will just execute clean and the 
                                               -- fact the servo didn't move INDICATES out of bounds. Adjust accordingly.



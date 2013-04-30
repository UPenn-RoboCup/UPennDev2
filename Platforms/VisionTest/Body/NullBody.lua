module(..., package.seeall);

function set_actuator_command(a, index)
end

function set_actuator_velocity(a, index)
end

function set_actuator_hardness(a, index)
end

function get_sensor_position(index)
  return vector.zeros(20);
end

function get_sensor_imuAngle(index)
  return {0,0,0};
end

function get_sensor_button(index)
  return {0,0};
end

function get_head_position()
  return vector.zeros(2);
end
function get_larm_position()
  return vector.zeros(3);
end
function get_rarm_position()
  return vector.zeros(3);
end
function get_lleg_position()
  return vector.zeros(6);
end
function get_rleg_position()
  return vector.zeros(6);
end

function set_body_hardness(val)
end
function set_head_hardness(val)
end
function set_larm_hardness(val)
end
function set_rarm_hardness(val)
end
function set_lleg_hardness(val)
end
function set_rleg_hardness(val)
end
function set_head_command(val)
end
function set_lleg_command(val)
end
function set_rleg_command(val)
end
function set_larm_command(val)
end
function set_rarm_command(val)
end

function update()
end

-- Extra for compatibility
function set_syncread_enable(val)
end

function set_actuator_eyeled( val )
end

function set_waist_hardness( val )
end

function set_waist_command( val )
end

function get_sensor_imuGyr0()
  return vector.zeros(3)
end

function get_sensor_imuGyr( )
  return vector.zeros(3)
end

function get_sensor_imuGyrRPY( )
  return vector.zeros(3)
end

--Acceleration in X,Y,Z axis in g unit
function get_sensor_imuAcc( )
  return vector.zeros(3)
end

function set_actuator_eyeled(color)
end

function set_actuator_headled(color)
end

-- Set API compliance functions
function set_indicator_state(color)
end

function set_indicator_team(teamColor)
end

function set_indicator_kickoff(kickoff)
end

function set_indicator_batteryLevel(level)
end

function set_indicator_role(role)
end

function set_indicator_ball(color)
end

function set_indicator_goal(color)
end

function get_battery_level()
  return 10;
end

function get_change_state()
  return 0;
end

function get_change_enable()
  return 0;
end

function get_change_team()
  return 0;
end

function get_change_role()
  return 0;
end

function get_change_kickoff()
  return 0;
end

-- OP does not have the UltraSound device
function set_actuator_us()
end

function get_sensor_usLeft()
  return vector.zeros(10);
end

function get_sensor_usRight()
  return vector.zeros(10);
end

function set_lleg_slope(val)
end
function set_rleg_slope(val)
end

function set_lleg_slope(val)
end
function set_rleg_slope(val)
end

-- Gripper method needed
function set_gripper_hardness(val)
end
function set_gripper_command(val)
end


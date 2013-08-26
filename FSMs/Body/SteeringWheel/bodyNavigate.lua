module(..., package.seeall);

require('unix')
require('hcm')
require('scm')

Body = require(Config.Body);

waypoint={};

dist_threshold = 0.02;
angle_threshold = 1*math.pi/180;
turn_threshold = 5*math.pi/180;
RAD = math.pi/180;

function entry()
  print(_NAME..' Entry' ) 
  Body.set_lwheel_velocity(0);
  Body.set_rwheel_velocity(0);
  pose = scm:get_pose();
  waypoint_count = 1;

  target_pose = waypoints[waypoint_count];
  target_pose[3]=0;
  relPoseTarget = util.pose_relative(target_pose,pose);
  aTurn = util.mod_angle(math.atan2(relPoseTarget[2],relPoseTarget[1]));
  
  print("Current pose:",pose[1],pose[2],pose[3]*180/math.pi)
  print("Current target:",target_pose[1],target_pose[2] );
  print("Rel pose:",relPoseTarget[1],relPoseTarget[2] );
  print("target angle:",aTurn/math.pi*180);
    
  print("\n")

  
end

function set_waypoint(wp)
  print("Waypoint SET")

  waypoints={}
  for i=1,#wp/2 do
    waypoints[i]={wp[i], wp[i+#wp/2]};
    print(string.format("WP #%d: %.2f %.2f",i,waypoints[i][1],waypoints[i][2]));
     
  end
  
  
end

function update()
--  print(_NAME..' Update' ) 
  if waypoint_count > #waypoints then
    return "done"
  end

  pose = scm:get_pose();
  target_pose = waypoints[waypoint_count];
  target_pose[3]=0;

  relPoseTarget = util.pose_relative(target_pose,pose);
  aTurn = util.mod_angle(math.atan2(relPoseTarget[2],relPoseTarget[1]));

  dist_error = math.sqrt(
		(pose[1]-target_pose[1])^2 + 
		(pose[2]-target_pose[2])^2 ); 

  run_reverse = 1;
  if math.abs(aTurn)>math.pi/180 * 90  then
    aTurnDir = util.mod_angle(aTurn+math.pi);
    run_reverse = -1;
  else
    aTurnDir = aTurn;
  end


  if dist_error>dist_threshold then
    forwardVel = math.min(2000, dist_error/0.20 * 2000) + 1000;  
    turnVel = math.min(1000, math.abs(aTurnDir)/(10*RAD)*1000) + 500;           
    if aTurnDir<0 then turnVel = -turnVel; end
    aTurnMag = math.min(1, math.abs(aTurnDir)/turn_threshold);
    lVel = run_reverse*forwardVel * (1-aTurnMag) - turnVel * aTurnMag;
    rVel = run_reverse*forwardVel * (1-aTurnMag) + turnVel * aTurnMag;
    Body.set_wheel_velocity_direct({lVel,rVel});
  else
    --waypoint reached, increment counter
    print(string.format("Waypoint %d reached",waypoint_count ))
    waypoint_count = waypoint_count + 1;
    forwardVel = 0;
  end
end

function exit()
  Body.set_lwheel_velocity(0);
  Body.set_rwheel_velocity(0);
  print(_NAME..' Exit' ) 
end

module(..., package.seeall);

require('Config');
require('vector');
require('unix');
require('util');
require('wcm');

require('SoundComm');
require('Body');

-- robot pose
odomPose = {x=0, y=0, a=0};

-- last detection count
lastDet = SoundComm.get_detection();

-- TODO: disparity threshold should come from config
-- max allowed disparity
disparityThres = 10;

-- TODO: disparity conversion should come from config
-- convertion factor for signal disparity to angle (radians)
radPerDisparity = 10 * math.pi/180;

-- TODO: sound histogram filter size should be set in the config
radPerDiv = 30 * math.pi/180;
-- histogram filter
ndiv = math.floor(2*math.pi/radPerDiv);
detFilter = vector.zeros(ndiv);
zeroInd = math.floor(ndiv/2);


maxDetCount = 100;
updateRate = 1;
decayRate = 0.1;

count = 0;


function entry()
end


function update()
   -- increment iteration counter
   count = count + 1;

   -- check for new detection
   local det = SoundComm.get_detection();
   if (det.count ~= lastDet.count) then
      -- save a copy of the detection
      lastDet = det;

      -- new detection, update histogram accordingly
      --    only accept detections with a reasonable disparity
      if (math.abs(det.lIndex - det.rIndex) <= disparityThres) then

        -- get the current head angles
        local headAngles = Body.get_head_position();

        print(string.format('pan: %f, lindex: %d, rindex: %d, dindex: %d', headAngles[1], det.lIndex, det.rIndex, det.lIndex - det.rIndex));

        -- full orientation of the robot, body + head
        --    only head pan affects the audio detection
        local a = odomPose.a + headAngles[1];

        -- estimate signal direction based on left/right the disparity
        local dir = -1 * util.sign(det.lIndex - det.rIndex);
        local a_wrtHead = dir * radPerDisparity * math.abs(det.lIndex - det.rIndex);

        -- world location of sound
        local afront = util.mod_angle(a + a_wrtHead);

        -- account for cone of confusion
        --    each correlation can correspond to one of two directions
        --    (from the front or from behind)
        local aback = util.mod_angle(a + (math.pi - a_wrtHead));

        -- update histogram
        local ifront = math.max(1, math.min(math.floor(zeroInd + afront / radPerDiv) + 1, ndiv));
        local iback = math.max(1, math.min(math.floor(zeroInd + aback / radPerDiv) + 1, ndiv));

        print(string.format('afront: %f, aback: %f, ifront: %d, iback: %d', afront, aback, ifront, iback));

        detFilter[ifront] = detFilter[ifront] + updateRate;
        detFilter[iback] = detFilter[iback] + updateRate;

        -- cap count
        detFilter[ifront] = math.min(detFilter[ifront], maxDetCount);
        detFilter[iback] = math.min(detFilter[iback], maxDetCount);

      end
   end

   -- TODO: decay histogram
   if (count % 10 == 0) then
      for i = 1,#detFilter do
         detFilter[i] = detFilter[i] - decayRate;
         detFilter[i] = math.max(0, detFilter[i]);
      end
   end 

   update_shm();
end


function odometry(dx, dy, da)
   -- update pose based on walking odometry
   ca = math.cos(odomPose.a);
   sa = math.sin(odomPose.a);
   odomPose.x = odomPose.x + dx*ca - dy*sa;
   odomPose.y = odomPose.y + dx*sa + dy*ca;
   odomPose.a = odomPose.a + da;
end


function resolve_goal_detection(gtype, vgoal)
   -- given a goal detection
   --    determine if it is the attacking or defending goal
   --
   -- gtype:   goal detection type (0 - unkown, 1 - left post, 2 - right post, 3 - both posts)
   --             for types (0,1,2) only the first pose of vgoal is set
   -- vgoal:   goal post poses, {(x,y,a), (x,y,a)} relative to the robot
   -- return:  0 - unknown
   --         +1 - attacking
   --         -1 - defending 
   
   -- find the direction of the goalie
   -- TODO: if we are not confident in the goalie location return unkown
   local mv, mind = util.max(detFilter);
   if (mv < 10) then
      print('max detection < 10');
      return 0;
   end
   -- transform the goalie direction to the robot body frame
   local agoalie_wrtWorld = (mind - zeroInd) * radPerDiv;
   local agoalie_wrtRobot = util.mod_angle(agoalie_wrtWorld - odomPose.a);

   -- angle of goal from the robot
   local post1 = vgoal[1];
   local post2 = vgoal[2];
   local agoal = math.atan2(post1[2], post1[1]);
   if (gtype == 3) then
      -- if both posts were detected then use the angle between the two
      agoal = util.mod_angle(post1[3] + 0.5 * util.mod_angle(math.atan2(post2[2], post2[1]) - post2[3]));
   end
   agoal = util.mod_angle(agoal);

   print(string.format('agoal: %f, agoaliew: %f, agoalier: %f', agoal * 180/math.pi, agoalie_wrtWorld * 180/math.pi, agoalie_wrtRobot * 180/math.pi));

   -- compare the location of the detected goal with the location of the goalie
   if (math.abs(agoalie_wrtRobot - agoal) < 45*math.pi/180) then
      -- the goal is ours (defending)
      return -1;
   elseif (math.abs(agoalie_wrtRobot + math.pi - agoal) < 45*math.pi/180) then
      -- the goal is theirs (attacking)
      return 1;
   else
      -- the detected goal posts and goalie do not correlate well
      return 0;
   end

   -- if we make it this far then we do not know which goal it is
   return 0;
end


function reset()
   -- zero out histogram
   detFilter = 0 * detFilter;

   -- reset odometry?
   odomPose.x = 0;
   odomPose.y = 0;
   odomPose.a = 0;
end


function update_shm()
   wcm.set_sound_odomPose({odomPose.x, odomPose.y, odomPose.a});
   wcm.set_sound_detFilter(detFilter);

   -- store last detection information
   wcm.set_sound_detCount(lastDet.count);
   wcm.set_sound_detTime(lastDet.time);
   wcm.set_sound_detLIndex(lastDet.lIndex);
   wcm.set_sound_detRIndex(lastDet.rIndex);
end


function exit()
end


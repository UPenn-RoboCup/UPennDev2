------------------------------
-- Fix the head angle during approaching
------------------------------

module(..., package.seeall);

require('Body')
require('wcm')
require('mcm')

t0 = 0;

--TODO: implement headkick for nao head SMs

timeout = Config.fsm.headKick.timeout;
tLost = Config.fsm.headKick.tLost;
pitch0 = Config.fsm.headKick.pitch0;
xMax = Config.fsm.headKick.xMax;
yMax = Config.fsm.headKick.yMax;

function entry()
  print("Head SM:".._NAME.." entry");
  t0 = Body.get_time();
  kick_dir=wcm.get_kick_dir();
end

function update()
  local t = Body.get_time();
  local ball = wcm.get_ball();

  if ball.x<xMax and math.abs(ball.y)<yMax then
     Body.set_head_command({0, pitch0});
  else
   local yaw, pitch = HeadTransform.ikineCam(ball.x, ball.y, 0.03);
   local currentYaw = Body.get_head_position()[1];
   local currentPitch = Body.get_head_position()[2];
   local p = 0.3;
   yaw = currentYaw + p*(yaw - currentYaw);
   pitch = currentPitch + p*(pitch - currentPitch);
   Body.set_head_command({yaw, pitch});
  end

  if (t - ball.t > tLost) then
    return "ballLost";
  end
  if (t - t0 > timeout) then
    return "timeout";
  end
end

function exit()
end

--forikinecam
camOffsetZ = Config.head.camOffsetZ;
pitchMin = Config.head.pitchMin;
pitchMax = Config.head.pitchMax;
yawMin = Config.head.yawMin;
yawMax = Config.head.yawMax;
cameraPos = Config.head.cameraPos;
cameraAngle = Config.head.cameraAngle;

function ikineCam(x, y, z)
  --Bottom camera by default (cameras are 0 indexed so add 1)
  select = 2;
  --Look at ground by default
  z = z or 0;

  z = z-camOffsetZ;
  local norm = math.sqrt(x^2 + y^2 + z^2);
  local yaw = math.atan2(y, x);
  local pitch = math.asin(-z/(norm + 1E-10));

  pitch = pitch - cameraAngle[select][2];

  yaw = math.min(math.max(yaw, yawMin), yawMax);
  pitch = math.min(math.max(pitch, pitchMin), pitchMax);

  return yaw, pitch;
end

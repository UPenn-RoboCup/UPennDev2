require 'torch'

function imuCorrent(imu, Bias)
  local ac = torch.DoubleTensor(3, 1):copy(-Bias)

  ac[1] = ac[1] - imu.ax
  ac[2] = ac[2] + imu.ay
  ac[3] = ac[3] - imu.az
  local gyr = torch.DoubleTensor(3, 1):fill(0)
  gyr[1] = -imu.wr
  gyr[2] = imu.wp
  gyr[3] = -imu.wy

  return ac, gyr
end


function accTiltCompensate(acc)
  -- need -180 ~ 180
  local roll = math.atan2(acc[2][1], acc[3][1])
  -- need -90 ~ 90
  local tanPitch = -acc[1][1] / (acc[2][1]*math.sin(roll)+acc[3][1]*math.cos(roll))
  local pitch = math.atan(tanPitch)

  local Ry = rotY(pitch)
  local Rx = rotX(roll)
  local _acc = Ry * Rx * acc
  return _acc
end


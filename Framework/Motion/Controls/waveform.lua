waveform = {}

----------------------------------------------------------------------
-- waveform : utilities for pattern generators
----------------------------------------------------------------------

function waveform.step_sin(theta, alpha)
  -- generate a smoothed sinusoidal waveform with extended zero regions 
  -- alpha in [0, 1] determines the percentage of deadband in a cycle

  alpha = math.min(math.max(alpha, 0), 1)
  local PI = math.pi
  local theta = theta % (2*PI)
  local offset = PI/2*alpha
  if ((theta > 0)
  and (theta < offset)) then
    return 0
  elseif ((theta >= offset)
  and     (theta <= PI - offset)) then
    return (alpha == 1) and 1 or math.sin((theta - offset)/(1 - alpha))^2
  elseif ((theta > PI - offset)
  and     (theta < PI + offset)) then
    return 0
  elseif ((theta >= PI + offset)
  and     (theta <= 2*PI - offset)) then
    return (alpha == 1) and -1 or -math.sin((theta - PI - offset)/(1 - alpha))^2
  else
    return 0
  end
end

function waveform.step_cos(theta, alpha)
  return waveform.step_sin(theta + math.pi/2, alpha) 
end

function waveform.stride_sin(theta, alpha)
  -- generate a sinusoidal waveform with extended peak regions
  -- alpha in [0, 1] determines the percentage of deadband in a cycle

  alpha = math.min(math.max(alpha, 0), 1)
  local PI = math.pi
  local theta = theta % (2*PI)
  local offset = PI/2*alpha
  if ((theta > 0)
  and (theta < PI/2 - offset)) then
    return math.sin((theta)/(1 - alpha))
  elseif ((theta >= PI/2 - offset)
  and     (theta <= PI/2 + offset)) then
    return 1
  elseif ((theta > PI/2 + offset)
  and     (theta < 3*PI/2 - offset)) then
    return (alpha == 1) and -1 or -math.sin((theta - PI)/(1 - alpha))
  elseif ((theta >= 3*PI/2 - offset)
  and     (theta <= 3*PI/2 + offset)) then
    return -1
  else
    return math.sin((theta - 2*PI)/(1 - alpha))
  end
end

function waveform.stride_cos(theta, alpha)
  return waveform.stride_sin(theta + math.pi/2, alpha) 
end

return waveform

local vector = require'vector'

local util = {}

function util.tablesize(table)
  local count = 0
  for _ in pairs(table) do count = count + 1 end
  return count 
end

function util.ptable(t)
  -- print a table key, value pairs
  for k,v in pairs(t) do print(k,v) end
end

function util.ptorch(data, W, Precision)
  local w = W or 5
  local precision = Precision or 10
  local torch = require'torch'
  local tp = type(data)
  if tp == 'userdata' then
    tp = torch.typename(data) or ''
    local dim = data:dim()
    local row = data:size(1)
    local col = 1
    if dim == 1 then
      for i = 1, row do print(data[i]) end
      print('\n'..tp..' - size: '..row..'\n')
    elseif dim == 2 then 
      col = data:size(2) 
      for r = 1, row do
        for c = 1, col do
          io.write(string.format("%"..w.."."..precision.."f",data[r][c])..' ')
        end
        io.write('\n')
      end
      print('\n'..tp..' - size: '..row..'x'..col..'\n')
    else
      print'Printing torch objects with more than 2 dimensions is not support'
    end
  else
    print(data)
  end
  io.flush()
end

function util.mod_angle(a)
	-- Reduce angle to [-pi, pi)
	local b = a % (2*math.pi)
	if b >= math.pi then return (b - 2*math.pi) end
	return b
end

function util.sign(x)
  -- return sign of the number (-1, 0, 1)
  if (x > 0) then return 1;
  elseif (x < 0) then return -1;
  else return 0;
  end
end

function util.min(t)
  -- find the minimum element in the array table
  -- returns the min value and its index
  local imin = 0;
  local tmin = math.huge;
  for i = 1,#t do
    if (t[i] < tmin) then
      tmin = t[i];
      imin = i;
    end
  end
  return tmin, imin;
end

function util.max(t)
  -- find the maximum element in the array table
  -- returns the min value and its index
  local imax = 0;
  local tmax = -math.huge;
  for i = 1,#t do
    if (t[i] > tmax) then
      tmax = t[i];
      imax = i;
    end
  end
  return tmax, imax;
end

function util.se2_interpolate(t, u1, u2)
  -- helps smooth out the motions using a weighted average
  return vector.new{u1[1]+t*(u2[1]-u1[1]),
                    u1[2]+t*(u2[2]-u1[2]),
                    u1[3]+t*util.mod_angle(u2[3]-u1[3])};
end

function util.se3_interpolate(t, u1, u2, u3)
  --Interpolation between 3 xya values
  if t<0.5 then
    tt=t*2;
    return vector.new{u1[1]+tt*(u2[1]-u1[1]),
                    u1[2]+tt*(u2[2]-u1[2]),
                    u1[3]+tt*util.mod_angle(u2[3]-u1[3])};
  else
    tt=t*2-1;
    return vector.new{u2[1]+tt*(u3[1]-u2[1]),
                    u2[2]+tt*(u3[2]-u2[2]),
                    u2[3]+tt*util.mod_angle(u3[3]-u2[3])};
  end
end



function util.procFunc(a,deadband,maxvalue)
  --Piecewise linear function for IMU feedback
  if a>0 then
        b=math.min( math.max(0,math.abs(a)-deadband), maxvalue);
  else
        b=-math.min( math.max(0,math.abs(a)-deadband), maxvalue);
  end
  return b;
end

function util.pose_global(pRelative, pose)
  local ca = math.cos(pose[3]);
  local sa = math.sin(pose[3]);
  return vector.new{pose[1] + ca*pRelative[1] - sa*pRelative[2],
                    pose[2] + sa*pRelative[1] + ca*pRelative[2],
                    pose[3] + pRelative[3]};
end

function util.pose_relative(pGlobal, pose)
  local ca = math.cos(pose[3]);
  local sa = math.sin(pose[3]);
  local px = pGlobal[1]-pose[1];
  local py = pGlobal[2]-pose[2];
  local pa = pGlobal[3]-pose[3];
  return vector.new{ca*px + sa*py, -sa*px + ca*py, util.mod_angle(pa)};
end

---table of uniform distributed random numbers
--@param n length of table to return
--@return table of n uniformly distributed random numbers
function util.randu(n)
  local t = {};
  for i = 1,n do
    t[i] = math.random();
  end
  return t;
end

---Table of normal distributed random numbers.
--@param n length of table to return
--@return table of n normally distributed random numbers
function util.randn(n)
  local t = {};
  for i = 1,n do
    --Inefficient implementation:
    t[i] = math.sqrt(-2.0*math.log(1.0-math.random())) *
                      math.cos(math.pi*math.random());
  end
  return t;
end

-- For HZD
--[[
% Plot a left knee angle from the above coefficients
s = linspace(0, 1, 50) ;
figure ; plot(s*100, polyval_bz(alpha_L(ind_LKneePitch, :), s)*180/pi) ;
grid on ; xlabel('% gait') ; ylabel('deg') ; title('Left stance Knee') ;
--]]

  -- wikipedia
function util.factorial(n)
  if n == 0 then
  return 1
  else
return n * factorial(n - 1)
  end
  end

  --[[
  % Function to evaluate bezier polynomials
% Inputs: Alpha - Bezeir coefficients (alpha_0 ... alpha_M)
  %         s - s parameter. Range [0 1]
  % Outputs: b = sum(k=0 to m)[ alpha_k * M!/(k!(M-k)!) s^k (1-s)^(M-k)]
  --]]
function util.polyval_bz(alpha, s)
  b = 0;
  M = #alpha-1 ;  -- length(alpha) = M+1
  for k =0,M do
  b = b + alpha[k+1] * factorial(M)/(factorial(k)*factorial(M-k)) * s^k * (1-s)^(M-k) ;
  end
  return b;
  end

function util.bezier( alpha, s )
--  [n, m] = size(alpha);
  n = #alpha;
  m = #alpha[1];
  value=vector.zeros(n);
  M = m-1;
  if M==3 then
  k={1,3,3,1};
  elseif M==4 then
  k={1,4,6,4,1};
  elseif M==5 then
  k={1,5,10,10,5,1};
  elseif M==6 then
  k={1,6,15,20,15,6,1};
  else
  return;
  end

  x = vector.ones(M+1);
  y = vector.ones(M+1);
  for i=1,M do
  x[i+1]=s*x[i];
  y[i+1]=(1-s)*y[i];
  end
  for i=1,n do
  value[i] = 0;
  for j=1,M+1 do
  value[i] = value[i] + alpha[i][j]*k[j]*x[j]*y[M+2-j];
  end
  end

  return value;
  end

function util.get_wireless_ip()
  ifconfig = io.popen('/sbin/ifconfig wlan0 | grep "inet " | cut -d" " -f10-11');
  ip = ifconfig:read();
  return ip;
end

local loadconfig = function(configName,Config)
  local local_config=require(configName)
  for k,v in pairs(local_config) do
    Config[k]=local_config[k]
  end
	return Config
end

function util.LoadConfig(params, platform, Config)
  local file_header = "Config_"..platform.name;
  for k, v in pairs(params.name) do
    local file_name = params[v] or "";
    local overload_platform = params[v..'_Platform'] or "";
    if string.len(overload_platform) ~= 0 then 
      file_header = "Config_"..overload_platform;
    else
      file_header = "Config_"..platform.name;
    end
    if string.len(file_name) ~= 0 then file_name = '_'..file_name; end
    file_name = v..'/'..file_header..'_'..v..file_name
    Config = loadconfig(file_name,Config)
  end
	return Config
end

return util

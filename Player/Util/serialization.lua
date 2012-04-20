module(..., package.seeall);

require('cutil')


function serialize(o)
  local str = "";
  if type(o) == "number" then
    str = tostring(o);
  elseif type(o) == "string" then
    str = string.format("%q",o);
  elseif type(o) == "table" then
    str = "{";
    for k,v in pairs(o) do
      str = str..string.format("[%s]=%s,",serialize(k),serialize(v));
    end
    str = str.."}";
  else	
    str = "nil";
  end
  return str;
end

--[[
--New serialization code omiting integer indexes for tables
function serialize(o)
  local str = "";
  if type(o) == "number" then
    if o%1==0 then --quickest check for integer
      str=tostring(o);
    else
      str = string.format("%.2f",o);--2-digit precision should be good enough
    end
  elseif type(o) == "string" then
    str = string.format("%q",o);
  elseif type(o) == "table" then
    str = "{";
    local is_num=true;
    for k,v in pairs(o) do
      if type(k)=="string" then 
        str = str..string.format("[%s]=%s,",serialize(k),serialize(v));
      else
        str = str..string.format("%s,",serialize(v));
      end
    end
    str = str.."}";
  else	
    str = "nil";
  end
  return str;
end
--]]

function serialize_array(ud, width, height, dtype, arrName, arrID)
  -- function to serialize an userdata array
  -- returns an array of lua arr tables
  -- Max size of a UDP packet
  local maxSize = 2^16 - 2^12;

  local dsize = cutil.sizeof(dtype);
  local arrSize = width*height*dsize;

  -- determine break size account for byte->ascii
  local rowSize = 2*width*dsize;
  local nrows = math.floor(maxSize/rowSize);
  local npackets = math.ceil(height/nrows);

  local ret = {};
  local cptr = ud;
  local rowsRemaining = height;
  for p = 1,npackets do
    local crows = math.min(nrows, rowsRemaining);
    rowsRemaining = rowsRemaining - crows;
    local name = string.format('%s.%d.%d.%d', arrName, arrID, p, npackets); 
    ret[p] = cutil.array2string(cptr, width, crows, dtype, name);
    cptr = cutil.ptr_add(cptr, width*crows, dtype);
  end

  return ret;
end

--For sending yuyv image
--We don't care even rows in yuyv
--So just skip every other line and save 1/2 bandwidth

function serialize_array2(ud, width, height, dtype, arrName, arrID)
  -- function to serialize an userdata array
  -- returns an array of lua arr tables
  -- Max size of a UDP packet
  local maxSize = 2^16 - 2^12;

  local dsize = cutil.sizeof(dtype);
  local arrSize = width*height*dsize/2; --skip every other line

  -- determine break size account for byte->ascii
  local rowSize = 2*width*dsize;
  local nrows = math.floor(maxSize/rowSize)*2; --skip every other line
  local npackets = math.ceil(height/nrows); 

  local ret = {};
  local cptr = ud;
  local rowsRemaining = height;
  for p = 1,npackets do
    local crows = math.min(nrows, rowsRemaining);
    rowsRemaining = rowsRemaining - crows;
    local name = string.format('%s.%d.%d.%d', arrName, arrID, p, npackets); 
    ret[p] = cutil.array2string2(cptr, width, crows, dtype, name);
	--skip every other line
    cptr = cutil.ptr_add(cptr, width*crows, dtype);
  end
  return ret;
end



function deserialize(s)
  --local x = assert(loadstring("return "..s))();
  if not s then
    return '';
  end
  -- protected loadstring call
  ok, ret = pcall(loadstring('return '..s));
  --local x = loadstring("return "..s)();
  if not ok then 
    --print(string.format("Warning: Could not deserialize message:\n%s",s));
    return '';
  else
    return ret;
  end
end

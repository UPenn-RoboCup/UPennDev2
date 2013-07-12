require 'include'
require 'torch'

require 'GeographicLib'

function nmea2degree(lat, latD, lnt, lntD)
--  print(lat, latD, lnt, lntD)
  -- NMEA Latitude DDDMM.MMM to DDD.DDD
  local nmea2deg = function(value, dir)
--    print(value, dir)
    local degree = math.floor(value/100)
    local minute = value - degree * 100
--    print(degree, minute)
    deg = degree + minute / 60
    if dir == 'S' or dir == 'W' then deg = -deg end
    return deg
  end

  Lat = nmea2deg(lat, latD)
  Lnt = nmea2deg(lnt, lntD)
  return Lat, Lnt
end

function findDateFromGPS(gps)
  local date = ""
  for i = 1, #gps do
    if gps[i].datastamp ~= nil and gps[i].datastamp ~= "" then
      date = gps[i].datastamp..gps[i].utctime
      break;
    end
  end
  return date
end

function global2metric(gps)
  local lat, lnt = nmea2degree(gps.latitude, gps.northsouth, gps.longtitude, gps.eastwest)
  local gpspos = GeographicLib.Forward(lat, lnt, gps.height)
  local pos = torch.DoubleTensor({gpspos.x, gpspos.y, gpspos.z})
  return pos
end

function gpsChecksum(line)
  star = line:find('*')
  if star then
    local trueline = line:sub(2, line:find('*')-1)
    local checksum = line:sub(line:find('*')+1, #line)
    local sum = 0
    for i = 1, #trueline do
      sum = bit.bxor(sum, trueline:byte(i,i))
    end
    sum = bit.tohex(sum)
    sumstr = string.format('%s', sum)
    sumstr = string.upper(sumstr:sub(#sumstr-1, #sumstr))
    return sumstr:sub(1,2) == checksum:sub(1,2)
  else
 --   print('bad gps string')
    return false;
  end
end

function readGPSLine(str, len, startptr)
  local gps = {}
  gps.type = 'gps'
  gps.timestamp = tonumber(string.sub(str, 1, 16))
  local startpt = startptr or 17
--  if str[17] ~= '$' then  
--    startpt = 19
--  end
  local line = string.sub(str, startpt)
  local stype = string.sub(str, startpt, startpt+5)
  gps.line = line
  if stype == '$GPGGA' then
--    print('GPGGA'..line)
    value = split(line)
    gps.id = 'GGA'
    gps.utctime = value[1]
    gps.latitude = value[2]
    gps.northsouth = value[3]
    gps.longtitude = value[4]
    gps.eastwest = value[5]
    gps.quality = value[6]
    gps.satellites = value[7]
    gps.HDOP = value[8]
    gps.height = value[9]
    gps.wgs84height = value[11]

  elseif stype == '$GPGLL' then 
--  print('GPGLL') 
    value = split(line)
    gps.id = 'GLL'
    gps.utctime = value[5]
    gps.latitude = value[1]
    gps.northsouth = value[2]
    gps.longtitude = value[3]
    gps.eastwest = value[4]
    gps.status = value[6]
    gps.posMode = value[7]:sub(1, #value[7]-3)

  elseif stype == '$GPGSA' then 
--  print('GPGSA') 
    value = split(line)
    gps.id = 'GSA'
    gps.opMode = value[1]
    gps.navMode = value[2]
    gps.PDOP = value[15]
    gps.HDOP = value[16]
    gps.VDOP = value[17]:sub(1, #value[17]-4)

--  elseif stype == '$GPGSV' then
----  print('GPGSV') 
--    value = split(line)
--    gps.utctime = ''
--    gps.id = 'GSV'
  elseif stype == '$GPRMC' then 
--  print('GPRMC') 
    value = split(line)
    gps.id = 'RMC'
    gps.utctime = value[1]
    gps.status = value[2]
    gps.latitude = value[3]
    gps.northsouth = value[4]
    gps.longtitude = value[5]
    gps.eastwest = value[6]
    gps.nspeed = value[7]
    gps.truecourse = value[8]
    gps.datastamp = value[9]
    gps.magneticvar = value[10]
    gps.magneticvard = value[11]
    gps.posMode = value[12]:sub(1, #value[12]-3)
  elseif stype == '$GPVTG' then 
--    print('GPVTG') 
    value = split(line)
    gps.id = 'VTG'
    gps.truecourse = value[1]
    gps.magneticcourse = value[3]
    gps.nspeed = value[5]
    gps.kspeed = value[7]
    gps.posMode = value[9]:sub(1, #value[9]-3)
  else
--    print('broken', line)
  end

  return gps;
end

function gpsDataCheck(gpsContent)
  local datavalid = true
  if gpsContent.id == 'GLL' or gpsContent.id == 'RMC' then 
    if gpsContent.status == 'V' then
      print('fail check ', gpsContent.id, gpsContent.status) 
      datavalid = false
    end
  end
  if gpsContent.id == 'GGA' then 
    if gpsContent.quality ~= '1' and gpsContent.quality ~= '2' then
      datavalid = false
      print('fail check ', gpsContent.id, gpsContent.quality) 
    end
  end
  if gpsContent.id == 'GSA' then 
    if gpsContent.navMode == '1' or gpsContent.navMode == '2' then
      datavalid = false
      print('fail check ', gpsContent.id, gpsContent.navMode) 
    end
  end
  if gpsContent.id == 'GLL'  or gpsContent.id == 'RMC' or gpsContent.id == 'VTG' then
    if string.find(gpsContent.posMode, 'A') == nil 
                and string.find(gpsContent.posMode, 'D') == nil then
      datavalid = false
      print( 'fail check', gpsContent.id, gpsContent.posMode) 
    end
  end
  if gpsContent.id == nil then datavalid = false end
  return datavalid
end

--pos = geo.Forward(27.99, 86.93, 8820)

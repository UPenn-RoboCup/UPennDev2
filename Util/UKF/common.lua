
dofile('include.lua')

function checkLen(value, len)
  if len == value then
    return true
  else
    return false
  end
end

function loadRawData(path, stamp, datatype)
  local data = {}
  data.Path = path
  data.Stamp = stamp
  data.Type = datatype
  data.Name = data.Path..data.Type..data.Stamp..'*'
  data.FileList = assert(io.popen('/bin/ls '..data.Name, 'r'))
  data.FileNum = 0
  for lines in data.FileList:lines() do data.FileNum = data.FileNum + 1 end
  return data
end

function saveData(dataset, dtype, path)
  local serialization = require 'serialization'
  local Path = path or './'
  local filecnt = 0
  local filetime = os.date('%m.%d.%Y.%H.%M.%S')
  local filename = string.format(dtype.."-%s-%d", filetime, filecnt)
  
  local file = io.open(Path..filename, "w")
  
  print(#dataset)
  for i = 1, #dataset do
    io.write('\rline #'..i)
    savedata = serialization.serialize(dataset[i])
    file:write(savedata)
    file:write('\n')
  --  print(savedata)
  end
  io.write('\n')
  file:close()
  print(filename)
end

function saveCSV(dataset, dtype, path, Debug)
  local Path = path or './'
  local filecnt = 0
  local filetime = os.date('%m.%d.%Y.%H.%M.%S')
  local filename = string.format(dtype.."-%s-%d.csv", filetime, filecnt)
  local debug = Debug or 0
  local file = io.open(Path..filename, "w")

  local data1 = dataset[1]
  local titles = {}
  for k, v in pairs(data1) do
    titles[#titles+1] = k
  end

  local headstr = ''
  for i = 1, #titles do
    headstr = headstr..titles[i]..','
  end
  headstr = headstr:sub(1, #headstr-1)
  file:write(headstr..'\n')

  for i = 1, #dataset do
    local line = ''
    for j = 1, #titles do
      if debug > 0 then print(titles[j], dataset[i][titles[j]]) end
      line = line..(dataset[i][titles[j]] or '')..','
    end
    line = line:sub(1, #line-1)
    file:write(line..'\n')
  end
  file:close()
  print(filename)
end

function saveDataMP(dataset, dtype, path)
  local msgpack = require 'msgpack'
  local Path = path or './'
  local filecnt = 0
  local filetime = os.date('%m.%d.%Y.%H.%M.%S')
  local filename = string.format(dtype.."-%s-%d", filetime, filecnt)
  local file = io.open(Path..filename, "wb")
  
  print(Path..filename)
  for i = 1, #dataset do
    io.write('\rline #'..i)
    savedata = msgpack.pack(dataset[i])
    file:write(savedata)
  end
  io.write('\n')
  file:close()
  print(filename)
end

function getFileName(path, dtype)
  local file = assert(io.popen('/bin/ls '..path..dtype..'*', 'r'))
  local filename = file:read();
  return filename
end

function loadData(path, dtype, maxlines, Debug)
  local serialization = require 'serialization'
  local filename = getFileName(path, dtype)
  local file = assert(io.open(filename, 'r'))
  local line = file:read()
  local datacounter = 0
  local data = {}
  local debug = Debug or 0
  while line ~= nil do
--    print(line)
    datacounter = datacounter + 1
    if debug == 1 then
      io.write('\r', dtype..' '..datacounter)
    end
    dataPoint = serialization.deserialize(line)
    data[datacounter] = dataPoint
--    util.ptable(dataPoint)
    line = file:read();
    if maxlines and datacounter >= maxlines then break end
  end
  if debug == 1 then
    io.write('\n')
    print(filename..' '..datacounter)
  end
  return data
end

function loadDataMP(path, dtype, maxlines, Debug)
  local msgpack = require 'msgpack'
  local filename = getFileName(path, dtype)
  local file = assert(io.open(filename, 'r'))
  local content = file:read('*a')
  local datacounter = 0
  local data = {}
  local debug = Debug or 0

  local current = file:seek()
  local size = file:seek('end')
  file:seek('set', current)

  t = msgpack.unpacker( content )
  local val = t:unpack()
  while val ~= nil do
    data[#data+1] = val
    val = t:unpack()
  end
  file:close()
  return data
end


function pruneTUC(set) -- select the first tuc imu
  local lastTuc = 0
  local Pruned = {}
  local Prunedcount = 0
  for i = 1, #set do
    if set[i].tuc ~= lastTuc then
      lastTuc = set[i].tuc
      Prunedcount = Prunedcount + 1
      Pruned[Prunedcount] = set[i]
    end
  end
  return Pruned
end

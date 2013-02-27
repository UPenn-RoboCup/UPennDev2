require 'common'

local datapath = '/home/yida/drcsim/ros/atlas_description/gazebo/atlas/meshes/' 
local filename = 'head.stl' 

local stlfile = assert(io.open(datapath..filename, 'rb'))
local header = stlfile:read(80)
if header:sub(1, 5) == 'solid' then
  error('ASCII STL')
end
local ptr = 80

--printbyte(header)
stlfile:seek('set', ptr)
local numOfTri = stlfile:read(4)
local numOfTriangle = parseUINT32(numOfTri)
print('num of triangles:'..numOfTriangle)
ptr = ptr + 4
--for i = 1, numOfTriangle do
for i = 1, 1 do
  local tri = {}
  stlfile:seek('set', ptr)
  local data = stlfile:read(50) 
  tri.normVec = {}
  for i = 1, 3 do 
    local j = 1 + (i - 1) * 4
 --   print(j, j+3)
    tri.normVec[i] = parseSINGLE(data:sub(j, j + 3))
  end
  tri.ver1 = {}
  for i = 1, 3 do
    local j = 1 + (i - 1) * 4 + 12
  --  print(j, j+3)
    tri.ver1[i] = parseSINGLE(data:sub(j, j + 3))
  end
  tri.ver2 = {}
  for i = 1, 3 do
    local j = 1 + (i - 1) * 4 + 24
  --  print(j, j+3)
    tri.ver2[i] = parseSINGLE(data:sub(j, j + 3))
  end
  tri.ver3 = {}
  for i = 1, 3 do
    local j = 1 + (i - 1) * 4 + 36 
  --  print(j, j+3)
    tri.ver3[i] = parseSINGLE(data:sub(j, j + 3))
  end
  local attr = parseUINT16(data:sub(49, 50))
  ptableR(tri)

  ptr = ptr + 50
end


require 'common'
require 'include'
require 'vrml'

function parseBinarySTL(filename)
  local stlfile = assert(io.open(filename, 'rb'))
  print(fsize(stlfile))
  local header = stlfile:read(80)
  if header:sub(1, 5) == 'solid' then
    error('ASCII STL')
  end
  local ptr = 80
  
  stlfile:seek('set', ptr)
  local numOfTriStr = stlfile:read(4)
  local numOfTriangle = parseUINT32(numOfTriStr)
  print('num of triangles:'..numOfTriangle)
  Tri = {}
  ptr = ptr + 4
  for i = 1, numOfTriangle do
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
 
    ptr = ptr + 50
    Tri[i] = tri;
  end
  stlfile:close()  
  return Tri
end

function facets2vertexes(facets)
  -- hashtable to find unique vertexes
  local ver = {}
  local verNum = 0
  for i = 1, #facets do
    for j = 1, 3 do
      verTag = facets[i]['ver'..j][1]..','..facets[i]['ver'..j][2]..','..facets[i]['ver'..j][3]
      if ver[verTag] == nil then 
        verNum = verNum + 1
        ver[verTag] = {facets[i]['ver'..j][1], facets[i]['ver'..j][2], facets[i]['ver'..j][3]} 
        ver[verNum] = {facets[i]['ver'..j][1], facets[i]['ver'..j][2], facets[i]['ver'..j][3]} 
        ver[verTag].__index = verNum
      end
    end
  end
  return ver
end

function generatePoint(ver)
  local point = {}
  for k, v in ipairs(ver) do
    verTag = v[1]..','..v[2]..','..v[3]
    assert(ver[verTag].__index == k)
    for i = 1, 3 do
      point[#point + 1] = v[i]
    end
  end
  return point
end

function generateCoordIndex(facets, vertexes)
  local coord = {}
  coordCount = 0 
  for i = 1, #facets do
    for j = 1, 3 do
      local verTag = facets[i]['ver'..j][1]..','..facets[i]['ver'..j][2]..','..facets[i]['ver'..j][3]
      coord[coordCount + j] = vertexes[verTag].__index - 1
    end
    coord[coordCount + 4] = -1
    coordCount = coordCount + 4
  end
  return coord
end

local datapath = ''
local filename = 'head.stl' 

facets = parseBinarySTL(datapath..filename)
--print(#facets)
--for i = 1, #facets do
--  print('facets:'..i)
--  print('vertex1',facets[i].ver1[1], facets[i].ver1[2], facets[i].ver1[3])
--  print('vertex2',facets[i].ver2[1], facets[i].ver2[2], facets[i].ver2[3])
--  print('vertex3',facets[i].ver3[1], facets[i].ver3[2], facets[i].ver3[3])
--end

vertexes = facets2vertexes(facets)

local header = '#VRML V2.0 utf8'
local model = createVRML(header)

local geometry = createNode('geometry', 'IndexedFaceSet')
local point = generatePoint(vertexes)
local points = createMultiField('point', point)
geometry[1] = createNode('coord', 'Coordinate')
geometry[1][1] = points
geometry[1][1].__delimiter = ','
geometry[1][1].__delimiterFreq = 3
coordIndex = generateCoordIndex(facets, vertexes)
geometry[2] = createMultiField('coordIndex', coordIndex)

local Shape = createNode(_, 'Shape')
Shape[1] = geometry
local children = createMultiField('children')
children[1] = Shape

local Transform = createNode(_, 'Transform')
local scale = {1,1,1}
Transform[1] = createField('scale', scale)
local translation = {0,0,0}
Transform[2] = createField('translation', translation)
Transform[3] = children

local NavigationInfo = createNode(_, 'NavigationInfo')
local navitype = {'EXAMINE', 'ANY'}
NavigationInfo[1] = createMultiField('type', navitype)

model[1] = Transform

saveVRML(model, 'robot.wrl')

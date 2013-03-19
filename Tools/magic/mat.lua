require 'include'
require 'torch-load'

local ffi = require 'ffi'
--local Z = require 'Z'
local util = require 'util'
require 'zlib'

local debug = false

local sizeOfDataType = 8
local headerSize = 128
local tagSize = 8

local miINT8       = 1
local miUINT8      = 2
local miINT16      = 3
local miUINT16     = 4
local miINT32      = 5
local miUINT32     = 6
local miSINGLE     = 7
local miDOUBLE     = 9
local miINT64      = 12
local miUINT64     = 13
local miMATRIX     = 14
local miCOMPRESSED = 15
local miUTF8       = 16
local miUTF16      = 17
local miUTF32      = 18

local miCELL_CLASS = 1
local miSTRUCT_CLASS = 2
local miOBJECT_CLASS = 3
local miCHAR_CLASS = 4
local miSPARSE_CLASS = 5
local miDOUBLE_CLASS = 6
local miSINGLE_CLASS = 7
local miINT8_CLASS = 8
local miUINT8_CLASS = 9
local miINT16_CLASS = 10
local miUINT16_CLASS = 11
local miINT32_CLASS = 12
local miUINT32_CLASS = 13


function matType(nType)
  local Type = ''
  if nType == 1 then
    Type = 'miINT8'
  elseif nType == 2 then
    Type = 'miUINT8'  
  elseif nType == 3 then
    Type = 'miINT16'
  elseif nType == 4 then
    Type = 'miUINT16'
  elseif nType == 5 then
    Type = 'miINT32'
  elseif nType == 6 then
    Type = 'miUINT32'
  elseif nType == 7 then
    Type = 'miSINGLE'
  elseif nType == 9 then
    Type = 'miDOUBLE'
  elseif nType == 12 then
    Type = 'miINT64'
  elseif nType == 13 then
    Type = 'miUINT64'
  elseif nType == 14 then
    Type = 'miMATRIX'
  elseif nType == 15 then
    Type = 'miCOMPRESSED'
  elseif nType == 16 then
    Type = 'miUTF8'
  elseif nType == 17 then
    Type = 'miUTF16'
  elseif nType == 18 then
    Type = 'miUTF32'
  end
  return Type
end

function typeByteSize(Type)
  if Type == 1 then
    return 1
  elseif Type == 2 then
    return 1
  elseif Type == 3 then
    return 2
  elseif Type == 4 then
    return 2
  elseif Type == 5 then
    return 4
  elseif Type == 6 then
    return 4
  elseif Type == 7 then
    return 4
  elseif Type == 9 then
    return 8
  elseif Type == 12 then
    return 8
  elseif Type == 13 then
    return 8
  elseif Type == 16 then
    return 1
  elseif Type == 17 then 
    return 2
  elseif Type == 18 then
    return 4
  else
    return 0
  end
end

function matArrayType(nType)
  local Type = ''
  if nType == 1 then 
    Type = 'mxCELL_CLASS'
  elseif nType == 2 then
    Type = 'mxSTRUCT_CLASS'
  elseif nType == 3 then
    Type = 'mxOBJECT_CLASS'
  elseif nType == 4 then
    Type = 'mxCHAR_CLASS'
  elseif nType == 5 then
    Type = 'mxSPARSE_CLASS'
  elseif nType == 6 then
    Type = 'mxDOUBLE_CLASS'
  elseif nType == 7 then
    Type = 'mxSINGLE_CLASS'
  elseif nType == 8 then
    Type = 'mxINT8_CLASS'
  elseif nType == 9 then
    Type= 'mxUINT8_CLASS'
  elseif nType == 10 then
    Type = 'mxINT16_CLASS'
  elseif nType == 11 then
    Type = 'mxUINT16_CLASS'
  elseif nType == 12 then
    Type = 'mxINT32_CLASS'
  elseif nType == 13 then
    Type = 'mxUINT32_CLASS'
  end
  return Type
end

function parseTag(tag)
  local dataT = parsemiUINT16(tag:sub(1,2))
  local dataSize = parsemiUINT16(tag:sub(3,4))
  local realSize = 4
  if dataSize == 0 then
    realSize = 8
    dataSize = parsemiUINT32(tag:sub(5, 8))
  end
  return dataT, dataSize, realSize
end

function parseHeader(header)
  local textSize = 116 -- Descriptive Text 
  local destext = header:sub(1, textSize)
  print(destext)
  local version = tonumber(ffi.new('int16_t', 
                  bit.bor(bit.lshift(header:byte(126), 8), header:byte(125))))
  if debug then print('Version '..bit.tohex(version)) end
  -- TODO byte-swapping
  endian = header:sub(127, 128)
  if debug then print('Endian '..endian) end
  return #header
end

function fsize(file)
  local current = file:seek()
  local size = file:seek('end')
  file:seek('set', current)
  return size
end

function printbyte(str)
  print(str:byte(1, #str))
end

function parsemiUINT64(data, num)
  if num == nil then num = 1 end
  local n = tonumber(ffi.new("uint64_t", bit.bor(
              bit.lshift(data:byte(8), 56), bit.lshift(data:byte(7), 48), 
              bit.lshift(data:byte(6), 40), bit.lshift(data:byte(5), 32), 
              bit.lshift(data:byte(4), 24), bit.lshift(data:byte(3), 16), 
              bit.lshift(data:byte(2), 8), data:byte(1))))
  return n
end

function parsemiINT64(data, num)
  if num == nil then num = 1 end
  local n = tonumber(ffi.new("int64_t", bit.bor(
              bit.lshift(data:byte(8), 56), bit.lshift(data:byte(7), 48), 
              bit.lshift(data:byte(6), 40), bit.lshift(data:byte(5), 32), 
              bit.lshift(data:byte(4), 24), bit.lshift(data:byte(3), 16), 
              bit.lshift(data:byte(2), 8), data:byte(1))))
  return n
end

function parsemiUINT16(data, num)
  if num == nil then num = 1 end
  assert(num * 2 <= #data)
  local n = {}
  for j = 1, num do
    local i = 1 + (j - 1) * 2
    n[j] = tonumber(ffi.new("uint16_t", bit.bor(
              bit.lshift(data:byte(i + 1), 8), data:byte(i))))
  end
  if num == 1 then return n[1] else return n end
end

function parsemiINT16(data, num)
  if num == nil then num = 1 end
  assert(num * 2 <= #data)
  local n = {}
  for j = 1, num do
    local i = 1 + (j - 1) * 2
    n[j] = tonumber(ffi.new("int16_t", bit.bor(
              bit.lshift(data:byte(i + 1), 8), data:byte(i))))
  end
  if num == 1 then return n[1] else return n end
end

function parsemiUINT8(data, num)
  if num == nil then num = 1 end
  assert(num * 1 == #data)
  local n = {}
  for j = 1, num do
    n[j] = tonumber(ffi.new("uint8_t", data:byte(j)))
  end
  if num == 1 then return n[1] else return n end
end

function parsemiINT8(data, num, flag)
  if num == nil then num = 1 end
  assert(num * 1 == #data)
  local n = {}
  for j = 1, num do
    n[j] = tonumber(ffi.new("int8_t", data:byte(j)))
  end

  if flag == 'string' then
    local str = ''
    for i = 1, num do
      str = str..string.format('%c', n[i])
    end
    return str
  end
  if num == 1 then return n[1] else return n end
end

function parsemiUTF8(data, num)
  if num == nil then num = 1 end
  local str = ''
  for i = 1, num do
    str = str..string.format('%c', data:byte(i))
  end
  return str
end

function parsemiUINT32(data, num)
  if num == nil then num = 1 end
  assert(num * 4 <= #data)
  local n = {}
  for j = 1, num do
    local i = 1 + (j - 1) * 4
    n[j] = tonumber(ffi.new("uint32_t", 
              bit.bor(bit.lshift(data:byte(i + 3), 24), bit.lshift(data:byte(i + 2), 16), 
              bit.lshift(data:byte(i + 1), 8), data:byte(i))))
  end
  if num == 1 then return n[1] else return n end
end

function parsemiINT32(data, num)
  if num == nil then num = 1 end
  assert(num * 4 <= #data)
  local n = {}
  for j = 1, num do
    local i = 1 + (j - 1) * 4
    n[j] = tonumber(ffi.new("int32_t", 
              bit.bor(bit.lshift(data:byte(i + 3), 24), bit.lshift(data:byte(i + 2), 16), 
              bit.lshift(data:byte(i + 1), 8), data:byte(i))))
  end
  if num == 1 then return n[1] else return n end
end

function parsemiDOUBLE(data, num)
  if debug then print('Parsing DOUBLE data') end
  if num == nil then num = 1 end
  assert(num * 8 <= #data)
  local n = {}
  for j = 1, num do
    local i = 1 + (j - 1) * 8
    local L = bit.bor(
              bit.lshift(data:byte(i + 3), 24), bit.lshift(data:byte(i + 2), 16), 
              bit.lshift(data:byte(i + 1), 8), data:byte(i))
    local H = bit.bor(
              bit.lshift(data:byte(i + 7), 24), bit.lshift(data:byte(i + 6), 16), 
              bit.lshift(data:byte(i + 5), 8), data:byte(i + 4))
    
    local value = 0
    local order = 52
    for k = 1, 32 do
      local c = bit.band(1, bit.rshift(L, k - 1))
      value = value + c * 2^(-order)
      order = order - 1
    end
    for k = 1, 20 do
      local c = bit.band(1, bit.rshift(H, k - 1))
      value = value + c * 2^(-order)
      order = order - 1
    end
    local e = 0
    for k = 21, 31 do
      local c = bit.band(1, bit.rshift(H, k - 1))
      e = e + 2^(k - 21) * c
    end
    value = value + 1
    local sign = bit.band(1, bit.rshift(H, 31))
    value = value * 2^(e-1023)
    value = value * (-1)^sign
    n[j] = value
  end
  if num == 1 then return n[1] else return n end
end

function parsemiSINGLE(data, num)
  if debug then print('Parsing SINGLE data') end
  if num == nil then num = 1 end
  assert(num * 4 <= #data)
  local n = {}
  for j = 1, num do
    local i = 1 + (j - 1) * 4
    local L = bit.bor(
              bit.lshift(data:byte(i + 1), 8), data:byte(i))
    local H = bit.bor(
              bit.lshift(data:byte(i + 3), 8), data:byte(i + 2))
    
    local value = 0
    local order = 23 
    for k = 1, 16 do
      local c = bit.band(1, bit.rshift(L, k - 1))
      value = value + c * 2^(-order)
      order = order - 1
    end
    for k = 1, 7 do
      local c = bit.band(1, bit.rshift(H, k - 1))
      value = value + c * 2^(-order)
      order = order - 1
    end
    local e = 0
    for k = 8, 15 do
      local c = bit.band(1, bit.rshift(H, k - 1))
      e = e + 2^(k - 8) * c
    end
    value = value + 1
    local sign = bit.band(1, bit.rshift(H, 15))
    value = value * 2^(e-127)
    value = value * (-1)^sign
    n[j] = value
  end
  if num == 1 then return n[1] else return n end
end

function parsemxNUMERIC_CLASS(data)
  -- generic parser
  -- Dimension array
  local ptr = 0
  local DimTag = data:sub( 1, tagSize) 
  local DimType, DimSize, realTagSize = parseTag(DimTag)
  ptr = ptr + realTagSize
  local DimByteSize = typeByteSize(DimType)
  local numberOfDims = DimSize / DimByteSize
  if numberOfDims % 2 == 1 then numberOfDims = numberOfDims + 1 end
  local DimBody = data:sub(ptr + 1, 
                  ptr + numberOfDims * DimByteSize)
  ptr = ptr + numberOfDims * DimByteSize
  local Dim = _G['parse'..matType(DimType)](DimBody, numberOfDims)
  -- array name
  local ANTag = data:sub(ptr + 1, ptr + tagSize)
  local ANDataT, ANDataSize, realTagSize = parseTag(ANTag)
  ptr = ptr + realTagSize
  local padding = 0
  if (ptr + ANDataSize * typeByteSize(ANDataT)) % 8 ~= 0 then
    -- padding
    local roundint = math.ceil((ptr + ANDataSize * typeByteSize(ANDataT)) / 8)
    padding = roundint * 8 - ptr - ANDataSize * typeByteSize(ANDataT)
  end
  local ANBody = data:sub(ptr + 1, ptr + ANDataSize * typeByteSize(ANDataT))
  ptr = ptr + ANDataSize * typeByteSize(ANDataT) + padding
  local AN = _G['parse'..matType(ANDataT)](ANBody, ANDataSize, 'string')
  -- pr
  local PrTag = data:sub(ptr + 1, ptr + tagSize)
  local PrDataT, PrDataSize, realTagSize = parseTag(PrTag)
  ptr = ptr + realTagSize
  local padding = 0
  if (ptr + PrDataSize * typeByteSize(PrDataT)) % 8 ~= 0 then
    -- padding
    local roundint = math.ceil((ptr + PrDataSize * typeByteSize(PrDataT)) / 8)
    padding = roundint * 8 - ptr - PrDataSize * typeByteSize(PrDataT)
  end
  local PrBody = data:sub(ptr + 1, ptr + PrDataSize * typeByteSize(PrDataT))
  if debug then  print('Pr data type:'..matType(PrDataT), 'data size:'..PrDataSize, 'data name:'..AN) end
  local nums = _G['parse'..matType(PrDataT)](PrBody, PrDataSize / typeByteSize(PrDataT)) 
  -- reorganize to torch 
  -- TODO general dimensions

  if type(nums) == 'number' then
    return AN, nums
  else
    local content = torch.Tensor(#nums, 1)
    for i = 1, #nums do
      content[i][1] = nums[i]
    end
    if #Dim == 2 then
      content:resize(Dim[2], Dim[1])
    elseif #Dim == 3 then
      content:resize(Dim[3], Dim[2], Dim[1])
    end
    content = content:t()
    return AN, content
  end

end

function parsemxSTRUCT_CLASS(data)
  local ptr = 0
  local DimTag = data:sub( 1, tagSize) 
  local DimType, DimSize, realTagSize = parseTag(DimTag)
  ptr = ptr + realTagSize
  local DimByteSize = typeByteSize(DimType)
  local numberOfDims = DimSize / DimByteSize
  if numberOfDims % 2 == 1 then numberOfDims = numberOfDims + 1 end
  local DimBody = data:sub(ptr + 1, 
                  ptr + numberOfDims * DimByteSize)
  ptr = ptr + numberOfDims * DimByteSize
  local Dim = _G['parse'..matType(DimType)](DimBody, numberOfDims)
  -- array name
  local ANTag = data:sub(ptr + 1, ptr + tagSize)
  local ANDataT, ANDataSize, realTagSize = parseTag(ANTag)
  ptr = ptr + realTagSize
  local padding = 0
  if (ptr + ANDataSize * typeByteSize(ANDataT)) % 8 ~= 0 then
    -- padding
    local roundint = math.ceil((ptr + ANDataSize * typeByteSize(ANDataT)) / 8)
    padding = roundint * 8 - ptr - ANDataSize * typeByteSize(ANDataT)
  end
  local ANBody = data:sub(ptr + 1, ptr + ANDataSize * typeByteSize(ANDataT))
  ptr = ptr + ANDataSize * typeByteSize(ANDataT) + padding
  local AN = _G['parse'..matType(ANDataT)](ANBody, ANDataSize, 'string')
  -- FieldNameLength 
  local FieldNameLengthTag = data:sub(ptr + 1, ptr + tagSize)
  local FieldNameLengthType, FieldNameLengthSize, realTagSize = parseTag(FieldNameLengthTag)
  if debug then print(matType(FieldNameLengthType), FieldNameLengthSize, realTagSize) end
  ptr = ptr + realTagSize
  local FieldNameLengthBody = data:sub(ptr + 1, ptr + FieldNameLengthSize)
  local FieldNameLength = _G['parse'..matType(FieldNameLengthType)](FieldNameLengthBody, 1)
  ptr = ptr + FieldNameLengthSize
  -- Field Name
  local FieldNamesTag = data:sub(ptr + 1, ptr + tagSize)
  local FieldNamesType, FieldNamesSize, realTagSize = parseTag(FieldNamesTag)
  if debug then print(matType(FieldNamesType), FieldNamesSize, realTagSize) end
  ptr = ptr + realTagSize

  if (ptr + FieldNamesSize * typeByteSize(FieldNamesType)) % 8 ~= 0 then
    -- padding
    local roundint = math.ceil((ptr + FieldNamesSize * typeByteSize(FieldNamesType)) / 8)
    padding = roundint * 8 - ptr - FieldNamesSize * typeByteSize(FieldNamesType)
  end
  local FieldNamesBody = data:sub(ptr + 1, 
                  ptr + FieldNamesSize * typeByteSize(FieldNamesType))
  local nElements = FieldNamesSize / FieldNameLength
  ptr = ptr + FieldNamesSize * typeByteSize(FieldNamesType) + padding
  local FieldNames = {}
  for i = 1, nElements do
    FieldNames[i] = _G['parse'..matType(FieldNamesType)](FieldNamesBody:sub(1 + 
            FieldNameLength * (i-1), FieldNameLength * i), FieldNameLength, 'string')
  end
  local padding = 0
  if (ptr + FieldNamesSize * typeByteSize(FieldNamesType)) % 8 ~= 0 then
    -- padding
    local roundint = math.ceil((ptr + FieldNamesSize * typeByteSize(FieldNamesType)) / 8)
    padding = roundint * 8 - ptr - FieldNamesSize * typeByteSize(FieldNamesType)
  end
--  ptr = ptr + FieldNamesSize * typeByteSize(FieldNamesType) + padding

  local content = {}
  for i = 1, nElements do
    if debug then print(FieldNames[i]) end
    local CellTag = data:sub(ptr + 1, ptr + tagSize) 
    local CellType, CellSize, realTagSize = parseTag(CellTag)
    content[FieldNames[i]] = _G['parse'..matType(CellType)](data:sub(ptr + 1, ptr + CellSize + realTagSize))
    ptr = ptr + realTagSize
    if debug then print(matType(CellType), CellSize, realTagSize) end
    ptr = ptr + CellSize
  end
 

  return AN, content
end

function parsemxCELL_CLASS(data)
end

function parsemxOBJECT_CLASS(data)
end

function parsemxCHAR_CLASS(data)
  local ptr = 0
  local DimTag = data:sub( 1, tagSize) 
  local DimType, DimSize, realTagSize = parseTag(DimTag)
  ptr = ptr + realTagSize
  local DimByteSize = typeByteSize(DimType)
  local numberOfDims = DimSize / DimByteSize
  if numberOfDims % 2 == 1 then numberOfDims = numberOfDims + 1 end
  local DimBody = data:sub(ptr + 1, 
                  ptr + numberOfDims * DimByteSize)
  ptr = ptr + numberOfDims * DimByteSize
  local Dim = _G['parse'..matType(DimType)](DimBody, numberOfDims)
  -- array name
  local ANTag = data:sub(ptr + 1, ptr + tagSize)
  local ANDataT, ANDataSize, realTagSize = parseTag(ANTag)
  ptr = ptr + realTagSize
  local padding = 0
  if (ptr + ANDataSize * typeByteSize(ANDataT)) % 8 ~= 0 then
    -- padding
    local roundint = math.ceil((ptr + ANDataSize * typeByteSize(ANDataT)) / 8)
    padding = roundint * 8 - ptr - ANDataSize * typeByteSize(ANDataT)
  end
  local ANBody = data:sub(ptr + 1, ptr + ANDataSize * typeByteSize(ANDataT))
  ptr = ptr + ANDataSize * typeByteSize(ANDataT) + padding
  local AN = _G['parse'..matType(ANDataT)](ANBody, ANDataSize, 'string')
  -- pr
  local PrTag = data:sub(ptr + 1, ptr + tagSize)
  local PrDataT, PrDataSize, realTagSize = parseTag(PrTag)
  ptr = ptr + realTagSize
  local padding = 0
  if (ptr + PrDataSize * typeByteSize(PrDataT)) % 8 ~= 0 then
    -- padding
    local roundint = math.ceil((ptr + PrDataSize * typeByteSize(PrDataT)) / 8)
    padding = roundint * 8 - ptr - PrDataSize * typeByteSize(PrDataT)
  end
  local PrBody = data:sub(ptr + 1, ptr + PrDataSize * typeByteSize(PrDataT))
  if debug then  print('Pr data type:'..matType(PrDataT), 'data size:'..PrDataSize, 'data name:'..AN) end
  local nums = _G['parse'..matType(PrDataT)](PrBody, PrDataSize / typeByteSize(PrDataT)) 
  -- reorganize to torch 
  -- TODO general dimensions
  if type(nums) == 'string' then
    print(nums)
    return AN, nums
  end

--  local content = torch.Tensor(#nums, 1)
--  for i = 1, #nums do
--    content[i][1] = nums[i]
--  end
--  if #Dim == 2 then
--    content:resize(Dim[2], Dim[1])
--  elseif #Dim == 3 then
--    content:resize(Dim[3], Dim[2], Dim[1])
--  end
--  content = content:t()

  return AN, content

end

function parsemxSPARSE_CLASS(data)
end

function parsemxDOUBLE_CLASS(data)
  return parsemxNUMERIC_CLASS(data)
end

function parsemxSINGLE_CLASS(data)
  return parsemxNUMERIC_CLASS(data)
end

function parsemxINT8_CLASS(data)
  return parsemxNUMERIC_CLASS(data)
end

function parsemxUINT8_CLASS(data)
  return parsemxNUMERIC_CLASS(data)
end

function parsemxINT16_CLASS(data)
  return parsemxNUMERIC_CLASS(data)
end

function parsemxUINT16_CLASS(data)
  return parsemxNUMERIC_CLASS(data)
end

function parsemxINT32_CLASS(data)
  return parsemxNUMERIC_CLASS(data)
end

function parsemxUINT32_CLASS(data)
  return parsemxNUMERIC_CLASS(data)
end

function parsemiMATRIX(data)
  local matrix = {}
  local tag = data:sub(1, tagSize)
  local dataT, dataSize, realTagSize = parseTag(tag)
  if debug then  print('Matrix data type:'..matType(dataT)..' data size:'..dataSize) end
  local ptr = realTagSize
  -- Array flags
  local AFSize = 2 * sizeOfDataType
  local AF = data:sub(ptr + 1, ptr+AFSize)
  ptr = ptr + AFSize
  local AFTag = AF:sub(1, tagSize)
  local AFDataT, AFDataSize, realTagSize = parseTag(AFTag)
  local AFBody = AF:sub(realTagSize + 1, realTagSize + AFDataSize)
--  print(AFBody:byte(1, #AFBody))
  local ArrayClass = AFBody:byte(1)
  if debug then  print('Matrix Class:'..matArrayType(ArrayClass)) end
  -- TODO parse flags for complex, global, logical
  name, content = _G['parse'..matArrayType(ArrayClass)](data:sub(ptr+1, #data))
  matrix[name] = content
  return matrix
end

function load(filename, Debug)
  debug = Debug or debug
  local content = {}
  file = assert(io.open(filename, 'rb'))
  local filesize = fsize(file)
  if debug then  print('filesize:'..filesize) end
  -- Read Header
  local header = file:read(headerSize)
  local ptr = parseHeader(header)

  -- iteration to read all data
  while ptr < filesize do
    file:seek('set', ptr)
    local tag = file:read(tagSize)
    local dataT, dataSize, realTagSize = parseTag(tag)
    ptr = ptr + realTagSize
    file:seek('set', ptr)
--    print(dataT, dataSize, realTagSize)
    if debug then print('data type:'..matType(dataT)..' data size:'..dataSize) end
    local data = file:read(dataSize)
    -- move ptr to next data
    ptr = ptr + dataSize

    -- process data
    -- decompress data if necessary
    if dataT == miCOMPRESSED then
      data = uncompress(data, 40 * #data)
      if debug then print('Decompress data, new data size:'..#data) end
      -- read tag and parse for real data type and size
      tag = data:sub(1, tagSize)
      dataT, dataSize = parseTag(tag)
      if debug then print('data type:'..matType(dataT)..' data size:'..dataSize) end
    end
    -- process data based on type
     content[#content + 1] = _G['parse'..matType(dataT)](data)
--     print(content[#content])
  end
  file:close()
  return content
end

return {load = load}
  
--if #arg > 0 then
--  filename = arg[1]
--end
--
--local content = load(filename)
--
--print(type(content))
--for i = 1, #content do
--  util.ptable(content[i])
--end

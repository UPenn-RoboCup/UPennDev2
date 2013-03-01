local ffi = require 'ffi'

function parseUINT16(data)
  local n = tonumber(ffi.new("uint16_t", bit.bor(
              bit.lshift(data:byte(2), 8), data:byte(1))))
  return n
end

function parseUINT32(data)
  local n = tonumber(ffi.new("uint32_t", bit.bor(
            bit.lshift(data:byte(4), 24), bit.lshift(data:byte(3), 16), 
            bit.lshift(data:byte(2), 8), data:byte(1))))
  return n
end

function parseSINGLE(data)
  local L = bit.bor(
            bit.lshift(data:byte(2), 8), data:byte(1))
  local H = bit.bor(
            bit.lshift(data:byte(4), 8), data:byte(3))
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

  return value
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


function ptableR(table)
  for k, v in pairs(table) do
    if type(v) == 'table' then ptableR(v) 
    else
      print(k, v)
    end
  end
end

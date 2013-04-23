package.cpath = '../Util/Unix/?.so;'..package.cpath

local unix = require 'unix'
local msgpack = require 'msgpack'

function TEST_NUM(num_in)
  local pack_str = msgpack.pack(num_in)
  local num_out = msgpack.unpack(pack_str)
  print("test case:", num_out == num_in, "input:", num_in, "output:", num_out, 'with pack length:', #pack_str)
end
print("\n================= Test msgpack on Num ==================")
TEST_NUM(3.1415926 + 10^200)
TEST_NUM(-3.1415926 + 10)
TEST_NUM(2^8 - 2)
TEST_NUM(2^16 - 2)
TEST_NUM(2^32 - 2)
TEST_NUM(2^64 - 2)
TEST_NUM(-2^8 + 2)
TEST_NUM(-2^16 + 2)
TEST_NUM(-2^32 + 2)
TEST_NUM(-2^64 + 2)


function TEST_TABLE(tbl_in)
  local print_table = function(t)
    for k, v in pairs(tbl_in) do
      print("key:", k, 'value:', v, 'key type:'..type(k), 'value type:'..type(v))
    end
  end

  local pack_str = msgpack.pack(tbl_in)
  print('input table')
  print_table(tbl_in)

  local tbl_out = msgpack.unpack(pack_str)
  print('output table')
  print_table(tbl_out)
end

print("\n================= Test msgpack on Table ==================")
a = {1,2,3,4,5,'hello', ['num'] = 3234}
TEST_TABLE(a)
--b = {['num'] = 3234, ['table'] = {1,2,3,4}}
--TEST_TABLE(b)

function TEST_STRING(str_in)
  local pack_str = msgpack.pack(str_in)
  local str_out = msgpack.unpack(pack_str)
  print("test case", str_in == str_out, "input "..str_in)
  print("output "..str_out)
  print('Pack length '..#pack_str)
end
print("\n================= Test msgpack on String ==================")
TEST_STRING("\"License\" shall mean the terms and conditions for use, reproduction, and distribution as defined by Sections 1 through 9 of this document.  ")
TEST_STRING('hello')
TEST_STRING('')

-- unpacker
--[[
print("\n================= Test unpacker ==================")
local file = io.open('testMP', 'r');
local file_str = file:read('*a')

local t0 = unix.time()
local unpacker = msgpack.unpacker(file_str)
local dataset = {}
local tbl = unpacker:unpack()
while tbl do
  dataset[#dataset+1] = tbl
  tbl = unpacker:unpack()
end
print("streaming unpack "..#dataset.." tables takes "..(unix.time() - t0).."with pack length "..#file_str)
file:close()

t0 = unix.time()
local len = 0
for i = 1, #dataset do
  local pack_str = msgpack.pack(dataset[i])
  len = len + #pack_str
end
print("pack "..#dataset.." tables takes "..(unix.time() - t0).."with pack length "..len)
--]]

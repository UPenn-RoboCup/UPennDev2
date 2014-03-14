package.cpath = '../unix/?.so;'..package.cpath
package.cpath = '../carray/?.so;'..package.cpath
package.path = '../../Util/?.lua;'..package.path

local unix = require 'unix'
local msgpack = require 'msgpack'
local util = require 'util'

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
b = {['num'] = 3234, ['table'] = {1,2,3,4}}
TEST_TABLE(b)
a = {['num'] = 3234, ['table'] = {1,2,3,4}}
b = {['table'] = {1,2,3,4}, ['tbl'] = {3.4,2,4,'helo'}}
b.a = a
TEST_TABLE(b)


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
----[[
print("\n================= Test unpacker ==================")
local file = io.open('dummy_mp_data.msg', 'r');
local dataset = {}
if file then
	--local file_str = file:read('*a')

	local t0 = unix.time()
	local unpacker = msgpack.unpacker(2048)
	local buf,nbuf = file:read(512),0
	while buf do
		nbuf = nbuf + #buf
		local res,left = unpacker:feed(buf)
		local tbl = unpacker:pull()
		--print(res,left,nbuf,type(tbl))
		while tbl do
			--util.ptable(tbl)
			dataset[#dataset+1] = tbl
			tbl = unpacker:pull()
		end
		buf = file:read(left)
	end
	print("streaming unpack ", #dataset, (unix.time() - t0))
	print("with pack length ",nbuf)
	file:close()
end

t0 = unix.time()
local len = 0
for i = 1, #dataset do
  local pack_str = msgpack.pack(dataset[i])
  len = len + #pack_str
end
print("pack "..#dataset.." tables takes "..(unix.time() - t0).."with pack length "..len)

--[[
print("\n================= Test Msgpack for lightuserdata ==================")
package.cpath = '../carray/?.so;'..package.cpath
local carray = require 'carray'

local ud = carray.float(1081)
for i = 1, 1081 do
  ud[i] = math.pi * i
end

-- lightuserdata, len = ud:pointer()
local pack_str = msgpack.pack(ud:pointer())
local dp = carray.float(msgpack.unpack(pack_str))
print("test case ", ud == dp)

--]]

--[[
function TEST_TORCH(aa)
  local pack_str = msgpack.pack(aa)
  local tbl = msgpack.unpack(pack_str)
  for k, v in pairs(tbl) do
    print('Key:', k, 'Value:', v)
  end
  print('pack size:',#pack_str)
end
print("\n================= Test Msgpack for Torch ==================")
local torch = require 'torch'
aa = torch.DoubleTensor({{3.0, 4.0, 5.24, 6.123, 7.90}, {1,2,3,4,5}})
TEST_TORCH(aa)
aa = torch.CharTensor({{-1,4,-6},{7,8,9},{-10, -30, -40}})
TEST_TORCH(aa)
aa = torch.IntTensor(3,3):fill(1)
TEST_TORCH(aa)
aa = torch.DoubleTensor(3,3,3):rand(3,3,3)
TEST_TORCH(aa)

package.path = '../../Util/?.lua;'..package.path
local torch = require 'torch'
local util = require 'util'

local aa = torch.DoubleTensor({{3.0, 4.0, 5.24, 6.123, 7.90}, {1,2,3,4,5}})
local pack_str = msgpack.pack(aa)
local tbl = msgpack.unpack(pack_str)
util.ptable(tbl)

local ttbl = msgpack.unpack(pack_str, 'torch')
print("\n Print Torch")
util.ptorch(ttbl)

local aa = 12
local pack_str = msgpack.pack(aa)
local na = msgpack.unpack(pack_str, 'torch')
print("\n Print Torch")
util.ptorch(na)

--]]

dofile('../../include.lua')
local dijkstra = require 'dijkstra'
local carray = require 'carray'
local util = require 'util'
local torch = require'torch'
torch.Tensor = torch.DoubleTensor

function gen_costs(N, M, Sparsity)
  local n = N or 100
  local m = M or n
  local sparsity = Sparsity or 0.1
  local c0 = torch.rand(m, n)
  for r = 1, m do
    for c = 1, n do
      if c0[r][c] > sparsity then c0[r][c] = 0 end
    end
  end
	c0:div(sparsity)
  local c1 = c0:mul(5):exp()
----[[
  local nconv = 10
  local hnconv = math.abs(nconv / 2)
  local kconv1 = torch.DoubleTensor():linspace(1, nconv, nconv):reshape(1, nconv)
  kconv1 = torch.exp(torch.abs(kconv1:div(nconv):add(-0.5)):mul(-2))
  local kconv = kconv1:t() * kconv1
  kconv:div(torch.sum(kconv))
  local c = torch.conv2(c1, kconv, 'F')
	c = c:narrow(1, hnconv + 1, m):narrow(2, hnconv + 1, n)
--]]
	-- Return the contiguous version :)
  --return c:sub(1):mul(100):clone()
	return c:clone()
	--return c1
end

local torch = require 'torch'
costs = gen_costs(100, 100, 0.01)
local goal = {80, 80}
t0 = unix.time()
local ctg = dijkstra.matrix(costs, goal[1], goal[2])
t1 = unix.time() - t0
print(t1)

ip1, jp1 = dijkstra.path(ctg, costs, 40, 40);
--ip2, jp2 = dijkstra.path2(ctg, costs, 1, 1);

--util.ptorch(ip1)
--util.ptorch(jp1)

-- Save the maps for viewing in MATLAB
print('Costs',costs,"contiguous",costs:isContiguous())
print('Costs to Go',ctg,"contiguous",ctg:isContiguous())
print('Path',ip1,jp1,"contiguous",ip1:isContiguous(),jp1:isContiguous())

local costs_ptr, n_costs = costs:storage():pointer(), #costs:storage()
local costs_arr = carray.double(costs_ptr, n_costs)
print('Costs | Writing',n_costs)
local f_costs = io.open('costs.raw', 'w')
f_costs:write( tostring(costs_arr) )
f_costs:close()

local ctg_ptr, n_ctg = ctg:storage():pointer(), #ctg:storage()
local ctg_arr = carray.double(ctg_ptr, n_ctg)
print('Costs to Go | Writing',n_ctg)
local f_ctg = io.open('ctg.raw', 'w')
f_ctg:write( tostring(ctg_arr) )
f_ctg:close()

-- Save the path
local ip1_ptr, n_ip1 = ip1:storage():pointer(), #ip1:storage()
local ip1_arr = carray.double(ip1_ptr, n_ip1)
print('Costs to Go | Writing',n_ip1)
local f_ip1 = io.open('ip1.raw', 'w')
f_ip1:write( tostring(ip1_arr) )
f_ip1:close()

local jp1_ptr, n_jp1 = jp1:storage():pointer(), #jp1:storage()
local jp1_arr = carray.double(jp1_ptr, n_jp1)
print('Costs to Go | Writing',n_jp1)
local f_jp1 = io.open('jp1.raw', 'w')
f_jp1:write( tostring(jp1_arr) )
f_jp1:close()

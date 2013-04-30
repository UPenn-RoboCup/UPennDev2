dofile('../../Run/include.lua')

local util = require 'util'
local dijkstra = require 'dijkstra'
local unix = require 'unix'

function gen_costs(N, M, Sparsity)
  local torch = require 'torch'
  local n = N or 100
  local m = M or n
  local sparsity = Sparsity or 0.1
  local c0 = torch.DoubleTensor(m, n):rand(m, n)
  for r = 1, m do
    for c = 1, n do
      if c0[r][c] > sparsity then c0[r][c] = 0 end
    end
  end
  c0:div(sparsity)
  c0:exp()

  local nconv = 5
  local hnconv = math.abs(nconv / 2)
  local kconv1 = torch.DoubleTensor():linspace(1, nconv, nconv):reshape(1, nconv)
  kconv1 = torch.exp(torch.abs(kconv1:div(nconv):add(-0.5)):mul(-2))
  local kconv = kconv1:t() * kconv1
  kconv:div(torch.sum(kconv))
  local c = torch.conv2(c0, kconv, 'F')
  return c:narrow(1, hnconv + 1, m):narrow(2, hnconv + 1, n)
end

local torch = require 'torch'
costs = gen_costs(100, 100, .05)
local goal = {60, 60}
t0 = unix.time()
local ctg = dijkstra.matrix(costs, goal[1], goal[2])
t1 = unix.time() - t0
print(t1)

--ip1, jp1 = dijkstra.path(ctg, costs, 1, 1);
ip2, jp2 = dijkstra.path2(ctg, costs, 1, 1);


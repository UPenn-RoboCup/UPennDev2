USE_IMG = true
ENABLE_PLAN = true
dofile('../../include.lua')
local dijkstra = require 'dijkstra'
local carray = require 'carray'
local cutil = require 'cutil'
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
  local nconv = 10
  local hnconv = math.abs(nconv / 2)
  local kconv1 = torch.DoubleTensor():linspace(1, nconv, nconv):reshape(1, nconv)
  kconv1 = torch.exp(torch.abs(kconv1:div(nconv):add(-0.5)):mul(-2))
  local kconv = kconv1:t() * kconv1
  kconv:div(torch.sum(kconv))
  local c = torch.conv2(c1, kconv, 'F')
	c = c:narrow(1, hnconv + 1, m):narrow(2, hnconv + 1, n)
	-- Return the contiguous version :)
	return c:clone()
end

-- Import costs from a PPM file (gmapping)
local img_t, cost_img, img_str
if USE_IMG then
	--local f_img = io.open('KUHill-1.ppm','r')
	local f_img = io.open('KUHill-1.pgm','r')
	local f_type = f_img:read('*line')
	print('NetPBM type',f_type)
	local is_comment = true
	local comments = {}
	local resolution = nil
	repeat
		comment = f_img:read('*line')
		is_comment = comment:sub(1,1)=='#'
		if is_comment then
			table.insert(comments,comment)
		else
			-- Is resolution
			resolution = comment:gmatch("%d+")
		end
	until not is_comment
	local max = tonumber( f_img:read('*line') )
	img_str = f_img:read('*all')
	f_img:close()
	local ncolumns = tonumber(resolution())
	local nrows = tonumber(resolution())
	print('Resolution',ncolumns,nrows)
	print('Max',max)
	print('Comments')
	print(table.concat(comments,'\n'))
	local n_el = #img_str
	print('Size of data:',#img_str)

	if f_type=='P5' then
		print('Grayscale')
		assert(n_el==ncolumns*nrows,'Bad resolution check!')
		img_t = torch.ByteTensor(nrows,ncolumns)
		-- Copy the img string to the tensor
		cutil.string2storage(img_str,img_t:storage())
		print(img_t[225][225])
		print(img_t[150][150])
		cost_img = torch.DoubleTensor(nrows,ncolumns)
		cost_img:copy(img_t):mul(-1):add(256)
	else
		print('RGB')
		assert(n_el==3*ncolumns*nrows,'Bad resolution check!')
		img_t = torch.ByteTensor(ncolumns,nrows,3)
		-- TODO: Select, or otherwise convert... to grayscale
	end

	-- Convolve on the map so that the robot size is taken into account


end

-- Generate random costs
local costs, goal, start
if USE_IMG then
	costs = cost_img
	-- Seems like {y,x} for the image imported...
	--start = {100, 160}
	--goal = {200, 225}
	start = {150, 70}
	goal = {175, 225}
else
	costs = gen_costs(100, 100, 0.01)
	start = {40,40}
	goal = {80, 80}
end

t0 = unix.time()
local ctg
if USE_IMG then
	ctg = dijkstra.matrix(cost_img, goal[1], goal[2])
else
	ctg = dijkstra.matrix(costs, goal[1], goal[2])
end

t1 = unix.time() - t0
print('Time to make cost map:',t1)

if ENABLE_PLAN then
	t0 = unix.time()
	ip1, jp1 = dijkstra.path(ctg, costs, start[1], start[2]);
	--ip2, jp2 = dijkstra.path2(ctg, costs, 1, 1);
	t1 = unix.time() - t0
	print('Time to find path:',t1)
	print(ip1:nElement(),'Path steps',ip1,jp1)
end

-- Save the maps for viewing in MATLAB
print()
print('Costs',costs,"contiguous",costs:isContiguous())
print('Costs to Go',ctg,"contiguous",ctg:isContiguous())

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
if ENABLE_PLAN then
	print('Path',ip1,jp1,"contiguous",ip1:isContiguous(),jp1:isContiguous())
	local ip1_ptr, n_ip1 = ip1:storage():pointer(), #ip1:storage()
	local ip1_arr = carray.int(ip1_ptr, n_ip1)
	print('Costs to Go | Writing',n_ip1)
	local f_ip1 = io.open('ip1.raw', 'w')
	f_ip1:write( tostring(ip1_arr) )
	f_ip1:close()

	local jp1_ptr, n_jp1 = jp1:storage():pointer(), #jp1:storage()
	local jp1_arr = carray.int(jp1_ptr, n_jp1)
	print('Costs to Go | Writing',n_jp1)
	local f_jp1 = io.open('jp1.raw', 'w')
	f_jp1:write( tostring(jp1_arr) )
	f_jp1:close()
end
-- Plot in MATLAB
--os.execute('matlab -nodesktop -nosplash -r view_dijkstra')

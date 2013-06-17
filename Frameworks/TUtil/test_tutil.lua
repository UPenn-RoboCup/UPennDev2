require 'torch'
torch.Tensor = torch.DoubleTensor
local tutil = require 'tutil'
require 'unix'

-- Perform band_mask test
print('Test Band Mask')
local points = torch.Tensor(1081):random(30)
local nPoints = points:size(1)
points[1] = 30
points[4] = .2
print('Start with',nPoints, 'points',points:storage():pointer())
for i=1,5 do
	io.write(points[i],'\t')
end
io.write('\n')
-- Perform the band pass
t0 = unix.time()
local nPoints2 = tutil.band_mask(points,.25,28)
t1 = unix.time()
nPoints = points:size(1)

print('Stop with',nPoints, 'points',points:storage():pointer())
for i=1,5 do
	io.write(points[i],'\t')
end
io.write('\n')
print(string.format('Band Mask took %f ms',(t1-t0)*1000) )


-- Perform band_mask_key test
print()
print('Test Band Mask Key')
points = torch.FloatTensor(1081):fill(10)
nPoints = points:size(1)
points[1] = 30
points[4] = .2
local angles = torch.Tensor(nPoints):fill(math.pi)
print('Starts with',nPoints, 'points',points:storage():pointer(), nPoints,
'angles',angles:storage():pointer())
for i=1,5 do
	angles[i] = i
	io.write(angles[i],'\t',points[i],'\n')
end
io.write('\n')

-- Perform the band pass
t0 = unix.time()
local nPoints2 = 
	tutil.band_mask_key( points,.25,28,angles,angles)
t1 = unix.time()
nPoints = points:size(1)

print('\nStops with',nPoints,'points',points:storage():pointer(), nPoints2,
'angles',angles:storage():pointer())
for i=1,5 do
	io.write(angles[i],'\t')
end
io.write('\n')
print(string.format('Band Mask Key took %f ms',(t1-t0)*1000) )

-- Perform some more tests
print('\n\n\nMEMCPY TEST\n')
local carray = require 'carray'
local scan = carray.float(1081)
print('Scan',#scan)
for i=1,#scan do
	scan[i] = (i-#scan/2)*.2
end
local tscan = torch.FloatTensor(#scan)
print('tscan 0001:',tscan[1])
print('tscan 0010:',tscan[10])
print('tscan 0100:',tscan[100])
print('tscan 1000:',tscan[1000])
print('Copying...')
t0 = unix.time()
tutil.memcpy( tscan:storage():pointer(), scan:pointer(), scan:bytesize() )
--tutil.memcpy( tscan:storage():pointer(), tostring(scan) ) -- a bit slower
t1 = unix.time()
print('tscan 0001:',tscan[1],scan[1])
print('tscan 0010:',tscan[10],scan[10])
print('tscan 0100:',tscan[100],scan[100])
print('tscan 1000:',tscan[1000],scan[1000])
print(string.format('Copy took %f ms',(t1-t0)*1000) )
local tscan2 = torch.FloatTensor(#scan)
t0 = unix.time()
for i=1,#scan do
	tscan2[i] = scan[i]
end
t1 = unix.time()
print(string.format('Slow Copy took %f ms',(t1-t0)*1000) )

-- Image copy to the 30th line
local tscan = torch.FloatTensor(500,900)
local row = tscan:select( 1, 30 )
local scan_str = tostring(scan)
t0 = unix.time()
tutil.memcpy_ranges_fov( row, scan_str, 100, 1000 )
t1 = unix.time()

print( 'FOV memcpy', (t1-t0)*1000, 'ms' )

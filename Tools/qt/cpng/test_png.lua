local pwd = os.getenv('PWD')
--package.cpath = pwd..'/../../../Player/Lib/?.so;'..package.cpath
--package.path = pwd..'/../../../Player/Util/ffi/?.lua;'..package.path

require 'unix'
require 'cpng'
require 'torch'

t0 = unix.time()
local p = cpng.new('small.png')
print(unix.time() - t0)

print('width  '..p:width())
print('height  '..p:height())
print('stride  '..p:stride())
print('color type  '..p:color_type())
print('bit depth  '..p:bit_depth())
print('data lenght  '..p:__len())

--for i = 1, p:__len(), 3 do
--  print(p[i], p[i+1], p[i+2])
--end
print(p[1920]) -- start from 1
print(p[1]) -- start from 1

--print(p:pointer())

t0 = unix.time()
p:write('iii.png')
--print(unix.time() - t0)
--print(p)

require 'carray'
w = 320
h = 240
ch = 3;
nbytes = w*h*ch;
img = carray.byte(w*h*ch)
for k=1,nbytes,ch do
  -- Blue up top
  img[k] = 0;
  img[k+1] = 0;
  img[k+2] = 255;
  -- Green halfway through
  if k>nbytes/2 then
    img[k] = 255;
    img[k+1] = 255;
    img[k+2] = 0;
  end
end

--t = torch.ByteTensor({0,4,5,6})
--print(t:storage():pointer())
--p:read((t:storage():pointer()))
t = torch.ByteTensor(640 * 480 * 3):fill(0)
cpng.load('small.png', t:storage():pointer())
for i = 1, 640 * 480 * 3, 3 do
  print(t[i], t[i+1], t[i+2])
end
--print(t[1], t[2], t[3])
--print(img[1])


-- libpng APIs
-- image_userdata = cpng.new(file_name)
-- image_userdata:write(file_name)
-- w = image_userdata:width()
-- h = image_userdata:height()
-- stride = image_userdata:stride()
-- bit_depth = image_userdata:bit_depth()
-- color_type = image_userdata:color_type()

-- t = torch.ByteTensor(640 * 480 * 3):fill(0)
-- cpng.load(file_name, lightuserdata)
-- cpng.load(file_name, t:storage():pointer())
-- t = carray.byte(640 * 480 * 3)
-- cpng.load(file_name, t:pointer())
--
-- cpng.save(file_name, lightuserdata, h, stride, color_type)
--   color_type = 0 - Gray
--   color_type = 2 - RGB 
--   color_type = 4 - Gray Alpha
--   color_type = 6 - RGB Alpha
--
-- file_str = cpng.compress(lightuserdata, h, stride, color_type)
-- image_userdata = cpng.uncompress( file_str )

local pwd = os.getenv('PWD')
package.cpath = pwd..'/../../../Player/Lib/?.so;'..package.cpath

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
--for i = 1, 640 * 480 * 3, 3 do
--  print(t[i], t[i+1], t[i+2])
--end
--print(t[1], t[2], t[3])
--print(img[1])

cpng.save('ddd.png', t:storage():pointer(), 480, 640*3, 2)
cpng.save('eee.png', img:pointer(), 240, 320*3, 2)


-- gray
ch = 1;
nbytes = w*h*ch;
print('Filling a '..w..' by '..h..' image',ch..' channels.')

require 'carray'
img = carray.byte(w*h*ch)

for k=1,nbytes,ch do
  if k>nbytes/2 then
    img[k] = 255;
  else
    img[k] = 0;
  end
end


cpng.load('small.png', t:storage():pointer())

t0 = unix.time()
file_str = cpng.compress(img:pointer(), 240, 320*1, 0)
f = io.open('img_gray.png','wb')
n = f:write( file_str )
f:close()
print(unix.time() - t0)
print(#file_str)

file = io.open('eee.png', 'rb')
str = file:read('*a')
print(#str)
file:close()
img_2 = cpng.uncompress( str )
print(img_2)
print('width  '..img_2:width())
print('height  '..img_2:height())
print('stride  '..img_2:stride())
print('color type  '..img_2:color_type())
print('bit depth  '..img_2:bit_depth())
print('data lenght  '..img_2:__len())

img_2:write('hhh.png')

--t0 = unix.time()
--cpng.save('fff.png', img:pointer(), 240, 320*1, 0)
--print(unix.time() - t0)

--img_jpeg = cjpeg.compress( img:pointer(), w, h, 1 )--gray

--print(type(img_jpeg),'Compression Ratio:', #img_jpeg, #img_jpeg/nbytes )

--f = io.open('img_gray.jpeg','w')
--n = f:write( img_jpeg )
--f:close()

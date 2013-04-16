os.execute('rm -f *.jpeg')
local cjpeg = require 'cjpeg'
w = 320
h = 240
ch = 3;
nbytes = w*h*ch;
if bgra then --webots uses this
  ch = 4;--bgra
end
print('Filling a '..w..' by '..h..' image',ch..' channels.')

if using_luajit then
  ffi = require 'ffi'
  img = ffi.new('uint8_t[?]', w*h*ch, 0)
else
  carray = require 'carray'
  img = carray.new('c',w*h*ch)
end

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

img_jpeg = cjpeg.compress( carray.pointer(img), w, h )
print(type(img_jpeg),'Compression Ratio:', #img_jpeg, #img_jpeg/nbytes )

f = io.open('img.jpeg','w')
n = f:write( img_jpeg )
f:close()


-- gray
ch = 1;
nbytes = w*h*ch;
print('Filling a '..w..' by '..h..' image',ch..' channels.')

if using_luajit then
  ffi = require 'ffi'
  img = ffi.new('uint8_t[?]', w*h*ch, 0)
else
  require 'carray'
  img = carray.new('c',w*h*ch)
end

for k=1,nbytes,ch do
  if k>nbytes/2 then
    img[k] = 255;
  else
    img[k] = 0;
  end
end

img_jpeg = cjpeg.compress( carray.pointer(img), w, h, 1 )--gray
print(type(img_jpeg),'Compression Ratio:', #img_jpeg, #img_jpeg/nbytes )

f = io.open('img_gray.jpeg','w')
n = f:write( img_jpeg )
f:close()

os.execute('rm -f *.jpeg')

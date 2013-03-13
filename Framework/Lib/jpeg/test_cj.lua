require 'cjpeg'
w = 320
h = 240
ch = 3;
nbytes = w*h*ch;
if webots then
  ch = 4;--bgra for webots
end
print('Filling a '..w..' by '..h..' image',ch..' channels.')

if using_luajit then
  ffi = require 'ffi'
  img = ffi.new('uint8_t[?]', w*h*ch, 0)
else
  require 'carray'
  img = carray.new('c',w*h*ch)
end

for k=1,nbytes,ch do
  -- Blue up top
  img[k] = 0;
  img[k+1] = 0;
  img[k+2] = 255;
  -- Green halfway through
  if k>3*w*h/2 then
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

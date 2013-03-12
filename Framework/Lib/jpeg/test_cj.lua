require 'cjpeg'
require 'carray'
local ffi = require 'ffi'
w = 320
h = 240
ch = 4;--rgba
img = carray.new('c',w*h*ch)
img_cd = ffi.cast('uint8_t*',carray.pointer(img))
print(img,w*h*ch)

for k=1,ch*w*h,ch do
  img[k] = 255;
  img[k+1] = 0;
  img[k+2] = 0;
  if k>3*w then
    img[k] = 0;
    img[k+1] = 255;
    img[k+2] = 0;
  end
  if k>3*w*h/2 then
    img[k] = 0;
    img[k+1] = 0;
    img[k+2] = 255;
  end
end

for hh=0,h-1 do
for ww=0,w-1 do
  img_cd[ch*ww+ch*w*hh] = 255*hh/(h-1)
  img_cd[1+ch*ww+ch*w*hh] = 255*hh/(h-1)
  img_cd[2+ch*ww+ch*w*hh] = 255*hh/(h-1)

  if ww>w/2 then
    img_cd[ch*ww+ch*w*hh] = 0
    img_cd[2+ch*ww+ch*w*hh] = 0
  end

end
end

img_str = ffi.string(img_cd,w*h*ch)
print('img sz:',#img_str)

cmp = cjpeg.compress( carray.pointer(img), w, h )
print('cmp sz:',#cmp)

f = io.open('img.jpeg','w')
n = f:write( cmp )
f:close()

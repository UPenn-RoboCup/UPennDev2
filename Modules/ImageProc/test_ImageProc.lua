local ImageProc = require('ImageProc');
local carray = require('carray');

cdt = carray.byte(262144);

width = 320;
height = 240;

rgb = carray.byte(3*width*height);

yuyv = ImageProc.rgb_to_yuyv(rgb:pointer(), width, height);
--print(#yuyv)

label = ImageProc.yuyv_to_label(yuyv, cdt:pointer(), width, height / 2);

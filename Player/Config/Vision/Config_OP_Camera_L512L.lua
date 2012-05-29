module(..., package.seeall);
require('vector')
--require('vcm')

-- Camera Parameters

camera = {};
camera.ncamera = 1;
camera.switchFreq = 0; --unused for OP
camera.width = 640;
camera.height = 480;
camera.x_center = 328;
camera.y_center = 248;

camera.focal_length = 533; -- in pixels
camera.focal_base = 640; -- image width used in focal length calculation

--[[
queryctrl: "White Balance Temperature" 0x98091a
queryctrl: "Sharpness" 0x98091b
queryctrl: "Backlight Compensation" 0x98091c
queryctrl: "Exposure, Auto" 0x9a0901
querymenu: Auto Mode
querymenu: Manual Mode
querymenu: Shutter Priority Mode
querymenu: Aperture Priority Mode
queryctrl: "Exposure (Absolute)" 0x9a0902
queryctrl: "Exposure, Auto Priority" 0x9a0903
queryctrl: "Pan (Absolute)" 0x9a0908
queryctrl: "Tilt (Absolute)" 0x9a0909
queryctrl: "Brightness" 0x980900
queryctrl: "Contrast" 0x980901
queryctrl: "Saturation" 0x980902
queryctrl: "White Balance Temperature, Auto" 0x98090c
queryctrl: "Gain" 0x980913
queryctrl: "Power Line Frequency" 0x980918
--]]

camera.auto_param = {};
camera.auto_param[1] = {key='white balance temperature, auto', val={0}};
camera.auto_param[2] = {key='power line frequency',   val={0}};
camera.auto_param[3] = {key='backlight compensation', val={0}};
camera.auto_param[4] = {key='exposure, auto',val={1}}; --1 for manual
camera.auto_param[5] = {key="exposure, auto priority",val={0}};
--camera.auto_param[6] = {key='autogain',               val={0}};

--[[
camera.param = {};
camera.param[1] = {key='brightness',    val={0}};
camera.param[2] = {key='contrast',      val={18}};
camera.param[3] = {key='saturation',    val={32}};
camera.param[4] = {key='gain',          val={150}};
camera.param[5] = {key='white balance temperature', val={1600}};
camera.param[6] = {key='sharpness',     val={0}};
camera.param[7] = {key='exposure (absolute)',      val={400}};
--]]

--[[ Levine 512 - 05/15/2012 - 10PM --]]
--[[camera.param = {};
camera.param[1] = {key='brightness',    val={128}};
camera.param[2] = {key='contrast',      val={27}};
camera.param[3] = {key='saturation',    val={28}};
camera.param[4] = {key='gain',          val={255}};
camera.param[5] = {key='white balance temperature', val={3220}};
camera.param[6] = {key='sharpness',     val={70}};
camera.param[7] = {key='exposure (absolute)',      val={1312}};
--]]

--[[ Levine 512 - 05/17/2012 - 8PM --]]
--camera.param = {};
--camera.param[1] = {key='brightness',    val={128}};
--camera.param[2] = {key='contrast',      val={29}};
--camera.param[3] = {key='saturation',    val={28}};
--camera.param[4] = {key='gain',          val={255}};
--camera.param[5] = {key='white balance temperature', val={1800}};
--camera.param[6] = {key='sharpness',     val={70}};
--camera.param[7] = {key='exposure (absolute)',      val={1367}};

--[[ Levine 512 - 05/22/2012 - 1730 --]]

camera.param = {};
camera.param[1] = {key='brightness',    val={141}};
camera.param[2] = {key='contrast',      val={36}};
camera.param[3] = {key='saturation',    val={28}};
camera.param[4] = {key='gain',          val={255}};
camera.param[5] = {key='white balance temperature', val={7000}};
camera.param[6] = {key='sharpness',     val={77}};
camera.param[7] = {key='exposure (absolute)',      val={711}};

--camera.lut_file = 'lut_low_contrast_pink_n_green.raw';
--camera.lut_file = 'lut_Grasp_Greenonly.raw';
--camera.lut_file = 'lut_OP_Grasp_GreenOnly.raw';
--camera.lut_file = 'lut_VT_aditya_05_06_time_23_30.raw';
--camera.lut_file = 'lut_VT_aditya_05_06_time_23_30.raw';
--camera.lut_file = 'lut_yida_vt_op_0507.raw';
camera.lut_file = 'lut_larry_L512_0515_10pm.raw';


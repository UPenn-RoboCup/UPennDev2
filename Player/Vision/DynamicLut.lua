module(..., package.seeall);

require('Config')
require('ImageProc')
require('vcm')

-- Enable Webots specific
if (string.find(Config.platform.name,'Webots')) then
  webots = 1;
end

enable_lut_for_obstacle = Config.vision.enable_lut_for_obstacle or 0;

function learn_lut_from_mask()
  -- Learn ball color from mask and rebuild colortable
  if enable_lut_for_obstacle == 1 then
    print("learn new colortable for random ball from mask");
    mask = ImageProc.label_to_mask(labelA.data_obs, labelA.m, labelA.n);
    if webots == 1 then
      print("learn in webots")
      lut_update = Camera.get_lut_update(mask, carray.pointer(camera.lut_obs));
    else
      print("learn in op")
      lut_update = ImageProc.yuyv_mask_to_lut(vcm.get_image_yuyv(), mask, camera.lut, 
                                              labelA.m, labelA.n);
    end
    print(type(mask),type(labelB.data))
  else
    print('Enable lut for obstacle in Vision to enable lut from mask');
  end
end

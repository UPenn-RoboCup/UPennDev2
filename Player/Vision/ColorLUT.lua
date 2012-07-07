module(..., package.seeall);

require('Config')
require('carray')
require('ImageProc')
require('vcm')

-- Enable Webots specific
if (string.find(Config.platform.name,'Webots')) then
  require('Camera')
  webots = 1;
end

enable_lut_for_obstacle = Config.vision.enable_lut_for_obstacle or 0;

function load_LUT(camera)

  print('loading lut: '..Config.camera.lut_file);
  camera.lut = carray.new('c', 262144);
  load_lutfile(Config.camera.lut_file, camera.lut);

  --ADDED to prevent crashing with old camera config
  if Config.camera.lut_file_obs == null then
    Config.camera.lut_file_obs = Config.camera.lut_file;
  end

  -- Load the obstacle LUT as well
  if enable_lut_for_obstacle == 1 then
    print('loading obs lut: '..Config.camera.lut_file_obs);
    camera.lut_obs = carray.new('c', 262144);
    load_lutfile(Config.camera.lut_file_obs, camera.lut_obs);
  end

  return camera;
end

function load_lutfile(fname, lut)
  if not string.find(fname,'.raw') then
    fname = fname..'.raw';
  end
  local cwd = unix.getcwd();
  if string.find(cwd, "WebotsController") then
    cwd = cwd.."/Player";
  end
  cwd = cwd.."/Data/";
  local f = io.open(cwd..fname, "r");
  assert(f, "Could not open lut file");
  local s = f:read("*a");
  for i = 1,string.len(s) do
    lut[i] = string.byte(s,i,i);
  end
end

function save_lutfile(fname, lut)
  if not string.find(fname, '.raw') then
    fname = fname..'.raw';
  end
  local cwd = unix.getcwd();
  if string.find(cwd, "WebotsController") then
    cwd = cwd.."/Player";
  end
  cwd = cwd.."/Data/";
  local f = io.open(cwd..fname, "w+");
  assert(f, "Could not open lut file");
  for i = 1, #lut do
    f:write(string.char(lut[i]));
  end
  f:close();
end

function save_rgb(rgb)
  saveCount = saveCount + 1;
  local filename = string.format("/tmp/rgb_%03d.raw", saveCount);
  local f = io.open(filename, "w+");
  assert(f, "Could not open save image file");
  for i = 1,3*camera.width*camera.height do
    local c = rgb[i];
    if (c < 0) then
      c = 256+c;
    end
    f:write(string.char(c));
  end
  f:close();
end

function learn_lut_from_mask()
  camera = {};
  -- Learn ball color from mask and rebuild colortable
  if enable_lut_for_obstacle == 1 then
    -- load colortable
    camera = load_LUT(camera);
    -- get yuyv image from shm
    yuyv = vcm.get_image_yuyv();
    image_width = vcm.get_image_width();
    image_height = vcm.get_image_height();
    -- get labelA
    if webots == 1 then
      labelA_mask = Camera.get_labelA_obs( carray.pointer(camera.lut) );
      labelA_m = Config.camera.width;
      labelA_n = Config.camera.height;
    else
      labelA_mask  = ImageProc.yuyv_to_label_obs(vcm.get_image_yuyv(),
                                    carray.pointer(camera.lut), image_width/2, image_height);
      labelA_m = Config.camera.width/2;
      labelA_n = Config.camera.height/2;
    end
    print("learn new colortable for random ball from mask");
    mask = ImageProc.label_to_mask(labelA_mask, labelA_m, labelA_n);

    if webots == 1 then
      print("learn in webots")
      lut_update = Camera.get_lut_update(mask, carray.pointer(camera.lut));
    else
      print("learn in op")
      lut_update = ImageProc.yuyv_mask_to_lut(vcm.get_image_yuyv(), mask, carray.pointer(camera.lut), 
                                              labelA_m, labelA_n);
    end
    save_lutfile(Config.camera.lut_file, 
                  carray.cast(lut_update, 'c', 262144));
  else
    print('Enable lut for obstacle in Vision to enable lut from mask');
  end
  vcm.set_camera_reload_LUT(1)
end

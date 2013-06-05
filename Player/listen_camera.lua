cwd = os.getenv('PWD')
require('init')
require('carray');
require('vector');
require('vcm');
require ('Config')

--Copy data to shm 1-1
Config.game.teamNumber = 1;
Config.game.playerID = 1;
Config.listen_monitor = 1;

require('Camera')

camera = {}

camera.auto_param = {};
camera.auto_param[1] = {key='white balance temperature, auto', val={0}};
camera.auto_param[2] = {key='power line frequency',   val={0}};
camera.auto_param[3] = {key='backlight compensation', val={0}};
camera.auto_param[4] = {key='exposure, auto',val={1}}; --1 for manual
camera.auto_param[5] = {key="exposure, auto priority",val={0}};

camera.param = {};
camera.param[1] = {key='brightness',    val={90}};
camera.param[2] = {key='contrast',      val={11}};
camera.param[3] = {key='saturation',    val={61}};
camera.param[4] = {key='gain',          val={0}};
camera.param[5] = {key='white balance temperature', val={2000}};
camera.param[6] = {key='sharpness',     val={255}};
camera.param[7] = {key='exposure (absolute)',      val={800}};

Config.camera = camera;


function camera_init()
  for c=1,1 do
    for i,auto_param in ipairs(Config.camera.auto_param) do
      print('Camera '..c..': setting '..auto_param.key..': '..auto_param.val[c]);
      Camera.set_param(auto_param.key, auto_param.val[c]);
      unix.usleep(100000);
      print('Camera '..c..': set to '..auto_param.key..': '..Camera.get_param(auto_param.key));
    end   
    for i,param in ipairs(Config.camera.param) do
      print('Camera '..c..': setting '..param.key..': '..param.val[c]);
      Camera.set_param(param.key, param.val[c]);
      unix.usleep(10000);
      print('Camera '..c..': set to '..param.key..': '..Camera.get_param(param.key));
    end
  end
end
  
camera = {};
camera_init();
lastImageCount={0,0}
count = 0;

function update()
  -- get image from camera
  camera.image = Camera.get_image();
  local status = Camera.get_camera_status();
  if status.count ~= lastImageCount[status.select+1] then
    lastImageCount[status.select+1] = status.count;
  else
    return false; 
  end
  vcm.set_image_yuyv(camera.image);
  count = count + 1;
  return true;
end


t0 = unix.time();
t_interval = 1.0;


--Just loop 
while 1 do
  update()
  unix.usleep(1E6*0.030)
  t=unix.time();
  if t-t0>t_interval then
    print(string.format("Listen camera: %.2f fps",count/(t-t0)));
    t0 = t;
    count=0;  
  end
end

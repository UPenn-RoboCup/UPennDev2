module(..., package.seeall);

CommWired=require('Comm');
-- Only send items from shared memory
require('vcm')
require('serialization');
require('Config');
require('Z');

-- Initiate Sending Address
--enable_online_learning = Config.vision.enable_online_colortable_learning or 0;
--if enable_online_learning == 1 then
--  command = 'last -w -d -i -1 | grep "darwin " | cut -d" " -f12-13'
--  loginIP  = io.popen(command)
--  IP = string.gsub(tostring(loginIP:read()),' ','')
----  IP = '192.168.1.201';
--  PORT = Config.dev.ip_wired_port;
--else
--  IP = Config.dev.ip_wired;
--  PORT = Config.dev.ip_wired_port;
--end
--CommWired.init(IP,PORT);
--print('Receiving LUT from port',PORT);
IP = '192.168.1.100'
CommWired.init(IP,Config.dev.ip_wired_port);
print('Receiving from port',Config.dev.ip_wired_port);

-- Add a little delay between packet sending
-- pktDelay = 500; -- time in us
-- Empirical value for keeping all packets intact

FIRST_LUT = true
lut_flag = {}
lut_all = {}
lut_t_full = unix.time()
fps_count=0;
fps_interval = 15;


function check_flag(flag)
  sum = 0;
  for i = 1 , #flag do
    sum = sum + flag[i];
  end
  return sum;
end

function parse_name(namestr)
  name = {}
  name.str = string.sub(namestr,1,string.find(namestr,"%p")-1);
  namestr = string.sub(namestr,string.find(namestr,"%p")+1);
  name.size = tonumber(string.sub(namestr,1,string.find(namestr,"%p")-1));
  namestr = string.sub(namestr,string.find(namestr,"%p")+1);
  name.partnum = tonumber(string.sub(namestr,1,string.find(namestr,"%p")-1));
  namestr = string.sub(namestr,string.find(namestr,"%p")+1);
  name.parts = tonumber(namestr);
  return name
end

function pack_lut(obj)
--print('receive lut parts');
  lut = cutil.test_array();
  name = parse_name(obj.arr.name);
  if (FIRST_LUT == true) then
    print("initiate lut flag");
    lut_flag = vector.zeros(name.parts);
    FIRST_LUT = false;
    lut_t_full = 0;
  end

  lut_flag[name.partnum] = 1;
  lut_all[name.partnum] = obj.arr.data;

  --Just push the image after all segments are filled at the first scan
  --Because the image will be broken anyway if packet loss occurs

  if (check_flag(lut_flag) == name.parts and name.partnum==name.parts ) then
    print("full lut\t"..1/(unix.time() - lut_t_full).." fps" );
    vcm.set_camera_broadcast(1);
    lut_t_full = unix.time();
    local lut_str = "";
    for i = 1 , name.parts do --fixed
      lut_str = lut_str .. lut_all[i];
    end

    height= 512;
    cutil.string2userdata(lut,lut_str,obj.arr.width,height);
    vcm.set_image_lut(lut);
    matcm.set_control_key(obj.ctrl_key);
  else
    vcm.set_camera_broadcast(3);
  end

end

function update()
  if CommWired.size() > 0 then
    msg = CommWired.receive();
    msg = Z.uncompress(msg, #msg);
    obj = serialization.deserialize(msg);
    if (obj.arr) then
      if (string.find(obj.arr.name,'lut')) then
        pack_lut(obj);  
      end
    end
  end
end

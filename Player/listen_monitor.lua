cwd = os.getenv('PWD')
require('init')

require ('Config')
--We always store data from robot to shm (1,1) 
Config.game.teamNumber = 1; 
Config.game.playerID = 1; 

require ('cutil')
require ('vector')
require ('serialization')
CommWired = require ('Comm')
require ('util')

require ('wcm')
require ('gcm')
require ('vcm')
require ('ocm')
require ('mcm')
require ('matcm')

require 'unix'
require 'Z'

yuyv_all = {}
yuyv_flag = {}
labelA_all = {}
labelA_flag = {}
yuyv2_all = {}
yuyv2_flag = {}
lut_all = {}
lut_flag = {}
FIRST_YUYV = true
FIRST_YUYV2 = true
FIRST_LABELA = true
FIRST_LUT = true
yuyv_t_full = unix.time();
yuyv2_t_full = unix.time();
yuyv3_t_full = unix.time();
labelA_t_full = unix.time();
labelB_t_full = unix.time();
data_t_full = unix.time();
lut_t_full = unix.time()
fps_count=0;
fps_interval = 15;
yuyv_type=0;
lut_updated = 0;

CommWired.init(Config.dev.ip_wired,Config.dev.ip_wired_port);
print('Sending to',Config.dev.ip_wired, 'Receving from ANY');

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


function push_yuyv(obj)
--print('receive yuyv parts');
  yuyv = cutil.test_array();
  name = parse_name(obj.name);
  if (FIRST_YUYV == true) then
    print("initiate yuyv flag");
    yuyv_flag = vector.zeros(name.parts);
    FIRST_YUYV = false;
  end

  yuyv_flag[name.partnum] = 1;
  yuyv_all[name.partnum] = obj.data;

  --Just push the image after all segments are filled at the first scan
  --Because the image will be broken anyway if packet loss occurs

  if (check_flag(yuyv_flag) == name.parts and name.partnum==name.parts ) then
    fps_count=fps_count+1;
    if fps_count%fps_interval ==0 then
      print("full yuyv\t"..1/(unix.time() - yuyv_t_full).." fps" );
    end
    yuyv_t_full = unix.time();
    local yuyv_str = "";
      for i = 1 , name.parts do --fixed
      yuyv_str = yuyv_str .. yuyv_all[i];
    end

    height= string.len(yuyv_str)/obj.width/4;
    cutil.string2userdata2(yuyv,yuyv_str,obj.width,height);
--  cutil.string2userdata(yuyv,yuyv_str,obj.width,height);
    vcm.set_image_yuyv(yuyv);
  end
end



yuyv2_part_last = 0;

function push_yuyv2(obj)
--	print('receive yuyv parts');
  yuyv2 = cutil.test_array();
  name = parse_name(obj.name);
  if (FIRST_YUYV2 == true) then
    print("initiate yuyv2 flag");
    yuyv2_flag = vector.zeros(name.parts);
    FIRST_YUYV2 = false;
  end

--[[
  if name.partnum==yuyv2_part_last then
    print("Duplicated packet");
  elseif name.partnum~=(yuyv2_part_last%name.parts)+1 then
    print("Missing packet");
  end
--]]

  yuyv2_part_last = name.partnum;
  yuyv2_flag[name.partnum] = 1;
  yuyv2_all[name.partnum] = obj.data;

  --Just push the image after all segments are filled at the first scan
  --Because the image will be broken anyway if packet loss occurs
  if (check_flag(yuyv2_flag) == name.parts and name.partnum==name.parts ) then
     fps_count=fps_count+1;
     if fps_count%fps_interval ==0 then
       print("yuyv2\t"..1/(unix.time() - yuyv2_t_full).." fps" );
     end

     yuyv2_t_full = unix.time();
     local yuyv2_str = "";
     for i = 1 , name.parts do --fixed
       yuyv2_str = yuyv2_str .. yuyv2_all[i];
     end
     height= string.len(yuyv2_str)/obj.width/4;
     cutil.string2userdata2(yuyv2,yuyv2_str,obj.width,height);
     vcm.set_image_yuyv2(yuyv2);
   end
end

function push_yuyv3(obj)
-- 1/4 size, we don't need to divide it 

  fps_count=fps_count+1;
  if fps_count%fps_interval ==0 then
     print("yuyv3\t"..1/(unix.time() - yuyv3_t_full).." fps" );
  end
  yuyv3_t_full = unix.time();
  yuyv3 = cutil.test_array();
  name = parse_name(obj.name);
  height= string.len(obj.data)/obj.width/4;
  cutil.string2userdata2(yuyv3,obj.data,obj.width,height);
  vcm.set_image_yuyv3(yuyv3);
end


--Function to OLD labelA packet
--[[
function push_labelA(obj)
--  print('receive labelA parts');
  local labelA = cutil.test_array();
  local name = parse_name(obj.name);
  if (FIRST_LABELA == true) then
    labelA_flag = vector.zeros(name.parts);
    FIRST_LABELA = false;
  end

  labelA_flag[name.partnum] = 1;
  labelA_all[name.partnum] = obj.data;
  if (check_flag(labelA_flag) == name.parts) then
--  print("full labelA\t",.1/(unix.time() - labelA_t_full).."fps" );
--  labelA_t_full = unix.time();
    labelA_flag = vector.zeros(name.parts);
    local labelA_str = "";
    for i = 1 , name.parts do
      labelA_str = labelA_str .. labelA_all[i];
    end

    cutil.string2userdata(labelA,labelA_str);
    vcm.set_image_labelA(labelA);
    labelA_all = {};
  end
end
--]]

--Function for new compactly encoded labelA
function push_labelA(obj)
  local name = parse_name(obj.name);
  local labelA = cutil.test_array();
--  cutil.string2label_double(labelA,obj.data);	
  cutil.string2label_rle(labelA,obj.data);	
  vcm.set_image_labelA(labelA);
end

function push_labelB(obj)
  local name = parse_name(obj.name);
  local labelB = cutil.test_array();
--cutil.string2userdata(labelB,obj.data);	
--cutil.string2label(labelB,obj.data);	
--  cutil.string2label_double(labelB,obj.data);	
  cutil.string2label_rle(labelB,obj.data);	
  vcm.set_image_labelB(labelB);
end

function push_occmap(obj)
  occmap = cutil.test_array();
  name = parse_name(obj.name);
  cutil.string2userdata2(occmap, obj.data, obj.width, obj.height);
  ocm.set_occ_map(occmap);
end

function push_data(obj)
--	print('receive data');
--  print("data\t",.1/(unix.time() - data_t_full).."fps");
--	data_t_full = unix.time();

  if type(obj)=='string' then print(obj); return end

  for shmkey,shmHandler in pairs(obj) do
    for sharedkey,sharedHandler in pairs(shmHandler) do
      for itemkey,itemHandler in pairs(sharedHandler) do
	local shmk = string.sub(shmkey,1,string.find(shmkey,'shm')-1);
        local shm = _G[shmk];
        shm['set_'..sharedkey..'_'..itemkey](itemHandler);
      end
    end
  end
end

lut_updated = 0;
function send_lut()
  pktDelay = 1E6 * 0.001; --For image and colortable
  -- send lut
  if matcm.get_control_lut_updated() ~= lut_updated  then
    lut_updated = matcm.get_control_lut_updated();
    sendlut = {}
    print("send lut, since it changed");
    lut = vcm.get_image_lut();
    width = 512;
    height = 512;
    count = vcm.get_image_count();

    array = serialization.serialize_array(lut, width,
                    height, 'uint8', 'lut', count);
    
    sendlut.updated = lut_updated;
    sendlut.arr = array;
    local tSerialize = 0;
    local tSend = 0;
    local totalSize = 0;
    for i = 1, #array do
      sendlut.arr = array[i];
      print(sendlut.arr.name, i)
      t0 = unix.time();
      senddata = serialization.serialize(sendlut);
      senddata = Z.compress(senddata, #senddata);
      t1 = unix.time();
      tSerialize = tSerialize + t1 - t0;
      CommWired.send(senddata, #senddata);
      t2 = unix.time();
      totalSize = totalSize + #senddata;
      tSend = tSend + t2 - t1

      unix.usleep(pktDelay);
    end
    print("LUT info array num:",#array,"Total size",totalSize);
    print("Total Serialize time:",#array,"Total",tSerialize);
    print("Total Send time:",tSend);
--    senddata = serialization.serialize(sendlut);
--    print(senddata);
--    CommWired.send(senddata);
  end
end

function push_lut(obj)
--print('receive lut parts');
  lut = cutil.test_array();
  name = parse_name(obj.name);
  if (FIRST_LUT == true) then
    print("initiate lut flag");
    lut_flag = vector.zeros(name.parts);
    FIRST_LUT = false;
  end

  lut_flag[name.partnum] = 1;
  lut_all[name.partnum] = obj.data;

  --Just push the image after all segments are filled at the first scan
  --Because the image will be broken anyway if packet loss occurs

  if (check_flag(lut_flag) == name.parts and name.partnum==name.parts ) then
    print("full lut\t"..1/(unix.time() - lut_t_full).." fps" );
    lut_t_full = unix.time();
    local lut_str = "";
    for i = 1 , name.parts do --fixed
      lut_str = lut_str .. lut_all[i];
    end

    height= 512;
    cutil.string2userdata(lut,lut_str,obj.width,height);
    vcm.set_image_lut(lut);
  end

end

while( true ) do
  msg = CommWired.receive();
  if( msg ) then
--    print(#msg);
    msg = Z.uncompress(msg, #msg);
--    print(msg);
--    print(#msg)
    local obj = serialization.deserialize(msg);
    if( obj.arr ) then
    	if ( string.find(obj.arr.name,'yuyv') ) then 
     	  push_yuyv(obj.arr);
    	  yuyv_type=1;
    	elseif ( string.find(obj.arr.name,'ysub2') ) then 
     	  push_yuyv2(obj.arr);
    	  yuyv_type=2;
    	elseif ( string.find(obj.arr.name,'ysub4') ) then 
     	  push_yuyv3(obj.arr);
    	  yuyv_type=3;
    	elseif ( string.find(obj.arr.name,'labelA') ) then 
    	  push_labelA(obj.arr);
    	elseif ( string.find(obj.arr.name,'labelB') ) then 
    	  push_labelB(obj.arr);
      elseif ( string.find(obj.arr.name,'occmap') ) then
        push_occmap(obj.arr);
      elseif ( string.find(obj.arr.name,'lut') ) then
        push_lut(obj.arr);
    	end
    else
    	push_data(obj);
    end
  end
  vcm.set_camera_yuyvType(yuyv_type);

  send_lut();
  unix.usleep(1E6*0.005);

end

module(... or '', package.seeall)

-- Get Platform for package path
cwd = '.';
local platform = os.getenv('PLATFORM') or '';
if (string.find(platform,'webots')) then cwd = cwd .. '/Player';
end

-- Get Computer for Lib suffix
local computer = os.getenv('COMPUTER') or '';
if (string.find(computer, 'Darwin')) then
  -- MacOS X uses .dylib:
  package.cpath = cwd .. '/Lib/?.dylib;' .. package.cpath;
else
  package.cpath = cwd .. '/Lib/?.so;' .. package.cpath;
end

package.path = cwd .. '/?.lua;' .. package.path;
package.path = cwd .. '/Util/?.lua;' .. package.path;
package.path = cwd .. '/Config/?.lua;' .. package.path;
package.path = cwd .. '/Lib/?.lua;' .. package.path;
package.path = cwd .. '/Dev/?.lua;' .. package.path;
package.path = cwd .. '/Motion/?.lua;' .. package.path;
package.path = cwd .. '/Motion/keyframes/?.lua;' .. package.path;
package.path = cwd .. '/Vision/?.lua;' .. package.path;
package.path = cwd .. '/World/?.lua;' .. package.path;

require ('Config')
require ('cutil')
require ('vector')
require ('serialization')
require ('MonitorComm')
require ('util')

require ('wcm')
require ('vcm')

yuyv_all = {}
labelA_all = {}
yuyv_flag = {}
labelA_flag = {}
FIRST_YUYV = true
FIRST_LABELA = true

MonitorComm.init(Config.dev.ip);
print('Receiving from',Config.dev.ip);

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
--	print('receive yuyv parts');
	yuyv = cutil.test_array();
	name = parse_name(obj.name);
	if (FIRST_YUYV == true) then
		print("initiate yuyv flag");
		yuyv_flag = vector.zeros(name.parts);
		FIRST_YUYV = false;
	end

	yuyv_flag[name.partnum] = 1;
	yuyv_all[name.partnum] = obj.data
--	print(check_flag(yuyv_flag));
	if (check_flag(yuyv_flag) == name.parts) then
		print("full yuyv");
--		print(obj.width,obj.height);
		yuyv_flag = vector.zeros(name.parts);
		local yuyv_str = "";
		for i = 1 , name.partnum do
			yuyv_str = yuyv_str .. yuyv_all[i];
		end
--		print(yuyv_str);
		cutil.string2userdata(yuyv,yuyv_str);
		vcm.set_image_yuyv(yuyv);
		yuyv_all = {}
	end
end

function push_labelA(obj)
--	print('receive labelA parts');
	local labelA = cutil.test_array();
	local name = parse_name(obj.name);
	if (FIRST_LABELA == true) then
		labelA_flag = vector.zeros(name.parts);
		FIRST_LABELA = false;
	end

	labelA_flag[name.partnum] = 1;
	labelA_all[name.partnum] = obj.data;
	if (check_flag(labelA_flag) == name.parts) then
		print("full labelA");
		labelA_flag = vector.zeros(name.parts);
		local labelA_str = "";
		for i = 1 , name.partnum do
			labelA_str = labelA_str .. labelA_all[i];
		end

		cutil.string2userdata(labelA,labelA_str);
		vcm.set_image_labelA(labelA);
		labelA_all = {};
	end
end

function push_labelB(obj)
--	print('receive labelB parts');
	local name = parse_name(obj.name);
	local labelB = cutil.test_array();
	cutil.string2userdata(labelB,obj.data);	
	vcm.set_image_labelB(labelB);
end

function push_data(obj)
--	print('receive data');
end

while( true ) do

  msg = MonitorComm.receive();
  if( msg ) then
    local obj = serialization.deserialize(msg);
    if( obj.arr ) then
			if ( string.find(obj.arr.name,'yuyv') ) then 
				push_yuyv(obj.arr);
			elseif ( string.find(obj.arr.name,'labelA') ) then 
				push_labelA(obj.arr);
			elseif ( string.find(obj.arr.name,'labelB') ) then 
				push_labelB(obj.arr);
			end
    else
			push_data(obj);
    end
  end

end

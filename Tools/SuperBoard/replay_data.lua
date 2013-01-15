module(... or '', package.seeall)

-- Add the required paths
cwd = '.';

uname  = io.popen('uname -s')
system = uname:read();

package.cpath = cwd.."/?.so;"..package.cpath;
package.cpath = cwd.."/../../Player/Lib/?.so;"..package.cpath;
package.path = cwd.."/../../Player/Util/?.lua;"..package.path;
package.path = cwd.."/../../Player/Config/?.lua;"..package.path;
package.path = cwd.."/../../Player/Vision/?.lua;"..package.path;

require('serialization');
require 'cutil'
require 'carray'
require('unix');
require('rcm');

-- Create files
imufilecnt = 0;
filetime = os.date('%m.%d.%Y.%H.%M');
filetime = '12.19.2012.21.21'
filename = string.format("data/imu%s-%d", filetime, imufilecnt);

print('Opening '..filename)

file = io.open(filename, "r");
linecount = 0;
maxlinecount = 5000;

cnti = 0;
t0 = unix.time();
while (1) do
  t1 = unix.time();
  --    print (1/(unix.time()-t0));
  t0 = unix.time(); -- timestamp
  local saveIMU = file:read()

  if not saveIMU then
    print('EOF')

    linecount = 0;
    file:close();
    imufilecnt = imufilecnt + 1;
    filename = string.format("data/imu%s-%d", filetime, imufilecnt);
    print('Opening ',filename)
    file = io.open(filename, "r");
    if not file then
      return
    end
    saveIMU = file:read()
  end

  local savedata=serialization.deserialize(saveIMU);
  local arr = savedata.arr;
  if not arr then
    print('NIL~')
  else
    local width = arr.width;
    local height = arr.height;

    local imudata = carray.new('c',width*height)
    local ptr = carray.pointer( imudata )
    cutil.string2userdata(ptr,arr.data)
    print('=============')
    local str = ' '
    for i=1,height*width do
      str = str..string.format(" %d",imudata[i] )
      if( i%3==0) then
        print(str)
        str = ' '
      end
    end
    print(str)
    print()
    --    savedata = Z.compress(savedata, #savedata);
  end

end

file:close();

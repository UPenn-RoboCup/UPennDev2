s = require 'serialization'
mc = require('OPMonitorComm')
require 'util'

while( true ) do

  str = mc.receive();
  if( str ) then
    local obj = s.deserialize(str);
    if( obj.arr ) then
      print( obj.arr.name )
    else
     util.ptable( obj );
    end
  end

end

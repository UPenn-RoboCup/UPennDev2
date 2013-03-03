-- mapShift(myMap,x,y)
-- myMap: Map table to be copied.
-- x: Amount (in meters) to shift map by in the x direction
-- y: Amount (in meters) to shift map by in the y direction
-- Steve McGill's Torch copy of Alex's mapResize function
	
function mapShift(myMap,x,y)

	-- If no shifting is necessary, then just exit!
if x==0 and y==0 then
  return;
end

-- Make a memory space for the new map
newMapData = myMap.data:clone():zero();

-- Convert meters to map indices
dxi = torch.ceil(torch.abs(x)*myMap.invRes);
dyi = torch.ceil(torch.abs(y)*myMap.invRes);

-- Find the indices of the map data that we wish to preserve
-- Find the new indicies to place this data into on the new map
if x>0 then
  xis  = torch.range( 1+dxi, myMap.sizex );
  nxis = torch.range( 1, myMap.sizex-dxi );
elseif x<0 then
  xis  = torch.range( 1, myMap.sizex-dxi );
  nxis = torch.range( 1+dxi, myMap.sizex );
else
  xis  = torch.range( 1, myMap.sizex );
  nxis = torch.range( 1, myMap.sizex);
end
if y>0 then
  yis  = torch.range( 1+dyi, myMap.sizey );
  nyis = torch.range( 1, myMap.sizey-dyi );
elseif y<0 then
  yis  = torch.range( 1, myMap.sizey-dyi );
  nyis = torch.range( 1+dyi, myMap.sizey );
else
  yis  = torch.range( 1, myMap.map.sizey );
  nyis = torch.range( 1, myMap.map.sizey );
end

-- View the just the map which we wish to preserve
keepMap = myMap.data:sub( xis[1],xis[-1],  yis[1],yis[-1] );
-- View the just the new map which we wish to place the preserved data
local newKeep = newMapData:sub( nxis[1],nxis[-1], nyis[1],nyis[-1] );
-- Perform the copy from old map to new map
newKeep:copy(keepMap)

-- Update the map information based on the shift
myMap.xmin = myMap.xmin + x;
myMap.ymin = myMap.ymin + y;
myMap.xmax = myMap.xmax + x;
myMap.ymax = myMap.ymax + y;

-- Discard the old map by reassigning new map
-- NOTE: Garbage collection should release this memory
myMap.data = newMapData;

print( string.format(
'Map (%s) shifted. New bounding box: x(%f %f) y(%f %f)\n',
myMap.name,myMap.xmin,myMap.xmax,myMap.ymin,myMap.ymax
));

end
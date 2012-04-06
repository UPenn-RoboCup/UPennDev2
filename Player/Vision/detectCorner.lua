module(..., package.seeall);

require('Config');	-- For Ball and Goal Size


function get_min_dist(line,i,j)
  xi1=line.endpoint[i][1];
  xi2=line.endpoint[i][2];
  yi1=line.endpoint[i][3];
  yi2=line.endpoint[i][4];

  xj1=line.endpoint[j][1];
  xj2=line.endpoint[j][2];
  yj1=line.endpoint[j][3];
  yj2=line.endpoint[j][4];

  --L shape detection 
  dist11 = (xi1-xj1)*(xi1-xj1) + (yi1-yj1)*(yi1-yj1);
  dist12 = (xi1-xj2)*(xi1-xj2) + (yi1-yj2)*(yi1-yj2);
  dist21 = (xi2-xj1)*(xi2-xj1) + (yi2-yj1)*(yi2-yj1);
  dist22 = (xi2-xj2)*(xi2-xj2) + (yi2-yj2)*(yi2-yj2);

  --TODO: T shape detection

  mindist = math.min (math.min(dist11,dist12),math.min(dist21,dist22));

  if mindist==dist11 then
    return mindist, 
	{(xi1+xj1)/2,(yi1+yj1)/2},	--corner position
	{xi2,yi2},			--other line endpoint 1
	{xj2,yj2};			--other line endpoint 2
  elseif mindist==dist12 then
    return mindist, 
	{(xi1+xj2)/2,(yi1+yj2)/2},
	{xi2,yi2},			--other line endpoint 1
	{xj1,yj1};			--other line endpoint 2
  elseif mindist==dist21 then
    return mindist,
	{(xi2+xj1)/2,(yi2+yj1)/2},
	{xi1,yi1},			--other line endpoint 1
	{xj2,yj2};			--other line endpoint 2
  else
    return mindist, 
	{(xi2+xj2)/2,(yi2+yj2)/2},
	{xi1,yi1},			--other line endpoint 1
	{xj1,yj1};			--other line endpoint 2
  end
end


function get_line_length(line,i)
  xi1=line.endpoint[i][1];
  xi2=line.endpoint[i][2];
  yi1=line.endpoint[i][3];
  yi2=line.endpoint[i][4];
  return math.sqrt((xi1-xi2)^2+(yi1-yi2)^2);
end

min_dist=5;
min_length=3;

function detect(line)
  --TODO: test line detection
  corner = {};
  corner.detect = 0;

  if line.detect==0 or line.nLines<2 then 
    return corner;
  end

  linepair={};
  linepaircount=0;
  linepairvc0={};
  linepairv10={};
  linepairv20={};
  linepairangle={}
  linepairdist={}

  -- Check perpendicular lines
  for i=1,line.nLines-1 do
    for j=i+1,line.nLines do
      ang=math.abs(util.mod_angle(line.angle[i]-line.angle[j]));
      if math.abs(ang-math.pi/2)<20*math.pi/180 then
	--Check endpoint distances in labelB
	mindist, vc0, v10, v20 = get_min_dist(line,i,j);
	if mindist<min_dist and
	get_line_length(line,i)>min_length and
	get_line_length(line,j)>min_length then 
  	  linepaircount=linepaircount+1;
  	  linepair[linepaircount]={i,j};
	  linepairvc0[linepaircount]=vc0;
	  linepairv10[linepaircount]=v10;
	  linepairv20[linepaircount]=v20;
	  linepairangle[linepaircount]=ang;
	  linepairdist[linepaircount]=mindist;
	end
       end
    end
  end

  if linepaircount==0 then 
    return corner;
  end

  vcm.add_debug_message(string.format("\nCorner: total %d lines\n",line.nLines))

  for i=1,linepaircount do
    vcm.add_debug_message(string.format("line %d-%d angle %d mindist %d\n",
	linepair[i][1],linepair[i][2],linepairangle[i]*180/math.pi,
	linepairdist[i] ));
  end

  best_corner=1;
  min_corner_dist = 100;
  --Pick the closest corner
  for i=1,linepaircount do
    vc0=linepairvc0[i];
    vc = HeadTransform.coordinatesB({vc0[1],vc0[2]});
    vc = HeadTransform.projectGround(vc,0);
    corner_dist=vc[1]*vc[1]+vc[2]*vc[2];
    if min_corner_dist>corner_dist then
      min_corner_dist = corner_dist;
      best_corner=i;
    end
  end

  corner.linepair=linepair[best_corner]

  vc0=linepairvc0[best_corner];
  v10=linepairv10[best_corner];
  v20=linepairv20[best_corner];

  vc = HeadTransform.coordinatesB({vc0[1],vc0[2]});
  vc = HeadTransform.projectGround(vc,0);

  v1 = HeadTransform.coordinatesB({v10[1],v10[2]});
  v1 = HeadTransform.projectGround(v1,0);

  v2 = HeadTransform.coordinatesB({v20[1],v20[2]});
  v2 = HeadTransform.projectGround(v2,0);

  --position in labelB
  corner.vc0=vc0;
  corner.v10=v10;
  corner.v20=v20;

  --position in xy
  corner.v = vc;
  corner.v1 = v1;
  corner.v2 = v2;

  --Center circle rejection
  pose=wcm.get_robot_pose();
  cornerpos = util.pose_global(corner.v,pose);
  center_dist = math.sqrt(cornerpos[1]^2+cornerpos[2]^2);
  if center_dist < 1.5 then     
    vcm.add_debug_message(string.format(
     "Corner: center circle check fail at %.2f\n",center_dist))
    return corner;
  end
 
  corner.type = 1;--1 for L, 2 for T

  corner.detect = 1;
  return corner;
end

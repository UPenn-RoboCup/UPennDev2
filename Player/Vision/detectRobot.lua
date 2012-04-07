module(..., package.seeall);

require('Config');      -- For Ball and Goal Size
require('ImageProc');
require('HeadTransform');       -- For Projection
require('Vision');
require('Body');
require('shm');
require('vcm');
require('Detection');
require('Debug');

-- Define Color
colorOrange = Config.color.orange;
colorYellow = Config.color.yellow;
colorCyan = Config.color.cyan;
colorField = Config.color.field;
colorWhite = Config.color.white;


map_div=10; --0.1m resolution
weight=vector.zeros(6*4*map_div*map_div);
updated=vector.zeros(6*4*map_div*map_div);

labelB_div=5; --labelB resolution
labelB_div=2; --labelB resolution

gamma=0.998;
min_fillrate = 0.2;

function discount_weight()
  for i=1,6*4*map_div*map_div do     
    weight[i]=weight[i]*gamma;
  end
end

function update_patch(v1,v2,w)
  xindex1=math.min(6*map_div,math.max(0,
		math.floor((v1[1]+3)*map_div+0.5) 
		));
  xindex2=math.min(6*map_div,math.max(0,
		math.floor((v2[1]+3)*map_div+0.5) 
		));
  yindex1=math.min(4*map_div,math.max(0,
		math.floor((v1[2]+3)*map_div+0.5) 
		));
  yindex2=math.min(4*map_div,math.max(0,
		math.floor((v2[2]+3)*map_div+0.5) 
		));
  for i=math.min(xindex1,xindex2) ,math.max(xindex1,xindex2) do
    for j=math.min(yindex1,yindex2) ,math.max(yindex1,yindex2) do
      index=(j-1)*(6*map_div) + i;
      updated[index]=1;
      weight[index]=w;
    end
  end
end     


function gen_patch(i,j,w)
  fieldBBox = {}; 
  fieldBBox[1] = labelB_div*(i-1)+1;
  fieldBBox[2] = labelB_div*i;
  fieldBBox[3] = labelB_div*(j-1)+1;
  fieldBBox[4] = labelB_div*j;

  v1=HeadTransform.coordinatesB({fieldBBox[1],fieldBBox[3],0,0});
  v1=HeadTransform.projectGround(v1,0);
  v2=HeadTransform.coordinatesB({fieldBBox[2],fieldBBox[4],0,0});
  v2=HeadTransform.projectGround(v2,0);
  pose=wcm.get_robot_pose();
  global_v1 = util.pose_global(v1,pose);
  global_v2 = util.pose_global(v2,pose);
  update_patch(global_v1,global_v2,w);
end


covered={};
blocked={};

function detect(color)
  local robot = {};
  robot.detect = 0;
  count=0;

  for i=1,Vision.labelB.m/labelB_div do
    covered[i]=vector.zeros(Vision.labelB.n/labelB_div);
    blocked[i]=vector.zeros(Vision.labelB.n/labelB_div);
    for j=1,Vision.labelB.n/labelB_div do 
       covered[i][j]=1;
       fieldBBox = {}; 
       fieldBBox[1] = labelB_div*(i-1)+1;
       fieldBBox[2] = labelB_div*i;
       fieldBBox[3] = labelB_div*(j-1)+1;
       fieldBBox[4] = labelB_div*j;
       fieldBBoxStats = ImageProc.color_stats(
  	   Vision.labelB.data, Vision.labelB.m, Vision.labelB.n, 
	   colorField, fieldBBox);
       fillrate=fieldBBoxStats.area/labelB_div/labelB_div;
       if fillrate<min_fillrate then
        blocked[i][j]=1;
       end
     end
   end
--[[
  for i=1,Vision.labelB.m/labelB_div do
    for j=1,Vision.labelB.n/labelB_div do 
       if covered[i][j]>0 then
	 gen_patch(i,j,-1); --set green first
       end
    end
  end
--]]

  for i=1,Vision.labelB.m/labelB_div do
    for j=1,Vision.labelB.n/labelB_div do 
       if blocked[i][j]>0 then
	 gen_patch(i,j,1); --set black over green
       end
    end
  end

  discount_weight();
  update_shm();
end

function update_shm()
  vcm.set_robot_map(weight);
end

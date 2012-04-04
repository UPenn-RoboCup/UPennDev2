module(..., package.seeall);

require('Config');	-- For Ball and Goal Size
require('ImageProc');
require('HeadTransform');	-- For Projection
require('Vision');

-- Dependency
require('Detection');

-- Define Color
colorOrange = 1;
colorYellow = 2;
colorCyan = 4;
colorField = 8;
colorWhite = 16;

--min_white_pixel = Config.vision.line.min_white_pixel or 200;
--min_green_pixel = Config.vision.line.min_green_pixel or 5000;

min_white_pixel = 200;
min_green_pixel =  5000;


function detect()
  --TODO: test line detection
  line = {};
  line.detect = 0;

  if (Vision.colorCount[colorWhite] < min_white_pixel) then 
    --print('under 200 white pixels');
    return line;
  end
  if (Vision.colorCount[colorField] < min_green_pixel) then 
    --print('under 5000 green pixels');
    return line; 
  end

  --max width 8
  linePropsB = ImageProc.field_lines(Vision.labelB.data, Vision.labelB.m, Vision.labelB.n, 8); 
  if #linePropsB==0 then 
    --print('linePropsB nil')
    return line; 
  end

  line.propsB=linePropsB;
  nLines=0;
  --Check the number of valid lines
  for i=1,#line.propsB do
    if line.propsB[i].count>15 then
      nLines=i;
    end
  end
  if (nLines==0) then
    return line; 
  end

  line.detect=1;
  line.nLines = nLines;
  line.v={};
  line.endpoint={};
  line.angle={};

  for i = 1,4 do
    line.endpoint[i] = vector.zeros(4);
    line.v[i]={};
    line.v[i][1]=vector.zeros(4);
    line.v[i][2]=vector.zeros(4);
    line.angle[i] = 0;
  end

  for i=1,nLines do
    local vendpoint = {};
    line.endpoint[i]= line.propsB[i].endpoint;
    vendpoint[1] = HeadTransform.coordinatesB(
		vector.new({line.propsB[i].endpoint[1], line.propsB[i].endpoint[3]}));
    vendpoint[2] = HeadTransform.coordinatesB(
		vector.new({line.propsB[i].endpoint[2], line.propsB[i].endpoint[4]}));
    vendpoint[1] = HeadTransform.projectGround(vendpoint[1],0);
    vendpoint[2] = HeadTransform.projectGround(vendpoint[2],0);

    line.v[i]={};
    line.v[i][1]=vendpoint[1];
    line.v[i][2]=vendpoint[2];
  end

  line.detect = 1;
  return line;
end

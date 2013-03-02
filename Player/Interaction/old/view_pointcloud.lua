-- Based on 3-D gear wheels, which is in the public domain.

-- Include the right directories
cwd = '.';
package.path = cwd.."/../Util/?.lua;"..package.path;
-- The ffi libraty is with luajit
local ffi  = require( "ffi" )
-- The ffi directory is in Util
local gl   = require( "ffi/OpenGL" )
local glfw = require( "ffi/glfw" )
require 'carray'
-- Convienence functions
local pi   = math.pi
local bor, sin, cos, sqrt, pi = bit.bor, math.sin, math.cos, math.sqrt, math.pi

--  Draw a gear wheel.  You'll probably want to call this function when
--  building a display list since we do a lot of trig here.
--
--  Input:  inner_radius - radius of hole at center
--          outer_radius - radius at center of teeth
--          width - width of gear teeth - number of teeth
--          tooth_depth - depth of tooth
local function gear( inner_radius, outer_radius, width, teeth, tooth_depth )
  local r0 = inner_radius;
  local r1 = outer_radius - tooth_depth / 2
  local r2 = outer_radius + tooth_depth / 2
  local da = 2 * pi / teeth / 4

  gl.glShadeModel( gl.GL_FLAT )
  gl.glNormal3d( 0, 0, 1 )

  gl.glBegin( gl.GL_QUAD_STRIP )
  for i = 0, teeth do
    local angle = i * 2 * pi / teeth
    gl.glVertex3d( r0*cos(angle), r0*sin(angle), width * 0.5 )
    gl.glVertex3d( r1*cos(angle), r1*sin(angle), width * 0.5 )
    if i < teeth then
      gl.glVertex3d( r0*cos(angle),        r0*sin(angle), width * 0.5 )
      gl.glVertex3d( r1*cos(angle + 3*da), r1*sin(angle + 3*da), width * 0.5)
    end
  end
  gl.glEnd()

  gl.glBegin( gl.GL_QUADS )
  local da = 2 * pi / teeth / 4
  for i = 0, teeth - 1 do
    local angle = i * 2 * pi / teeth
    gl.glVertex3d( r1*cos(angle),        r1*sin(angle),        width * 0.5 )
    gl.glVertex3d( r2*cos(angle + da),   r2*sin(angle + da),   width * 0.5 )
    gl.glVertex3d( r2*cos(angle + 2*da), r2*sin(angle + 2*da), width * 0.5 )
    gl.glVertex3d( r1*cos(angle + 3*da), r1*sin(angle + 3*da), width * 0.5 )
  end
  gl.glEnd()

  gl.glNormal3d( 0, 0, -1 )

  gl.glBegin( gl.GL_QUAD_STRIP )
  for i = 0, teeth do
    local angle = i * 2 * pi / teeth
    gl.glVertex3d( r1*cos(angle), r1*sin(angle), -width * 0.5 )
    gl.glVertex3d( r0*cos(angle), r0*sin(angle), -width * 0.5 )
    if i < teeth then
      gl.glVertex3d( r1*cos(angle+ 3*da), r1*sin(angle+ 3*da), -width * 0.5)
      gl.glVertex3d( r0*cos(angle),       r0*sin(angle),       -width * 0.5)
    end
  end
  gl.glEnd()

  gl.glBegin( gl.GL_QUADS )
  local da = 2 * pi / teeth / 4
  for i = 0, teeth - 1 do
    local angle = i * 2 * pi / teeth
    gl.glVertex3d( r1*cos(angle + 3*da), r1*sin(angle + 3*da), -width * 0.5 )
    gl.glVertex3d( r2*cos(angle + 2*da), r2*sin(angle + 2*da), -width * 0.5 )
    gl.glVertex3d( r2*cos(angle +   da), r2*sin(angle +   da), -width * 0.5 )
    gl.glVertex3d( r1*cos(angle),        r1*sin(angle),        -width * 0.5 ) 
  end
  gl.glEnd()

  gl.glBegin( gl.GL_QUAD_STRIP )
  for i = 0, teeth - 1 do
    local angle = i * 2 * pi / teeth
    gl.glVertex3d( r1*cos(angle), r1*sin(angle),  width * 0.5 )
    gl.glVertex3d( r1*cos(angle), r1*sin(angle), -width * 0.5 )
    local u = r2*cos( angle + da ) - r1*cos( angle )
    local v = r2*sin( angle + da ) - r1*sin( angle )
    local len = sqrt( u * u + v * v )
    u = u / len;
    v = v / len;
    gl.glNormal3d( v, -u, 0 )
    gl.glVertex3d( r2*cos(angle + da), r2*sin(angle + da),  width * 0.5 )
    gl.glVertex3d( r2*cos(angle + da), r2*sin(angle + da), -width * 0.5 )
    gl.glNormal3d( cos(angle), sin(angle), 0 )
    gl.glVertex3d( r2*cos(angle + 2*da), r2*sin(angle + 2*da),  width * 0.5 )
    gl.glVertex3d( r2*cos(angle + 2*da), r2*sin(angle + 2*da), -width * 0.5 )
    u = r1 * cos(angle + 3 * da) - r2*cos(angle + 2 * da)
    v = r1 * sin(angle + 3 * da) - r2*sin(angle + 2 * da)
    gl.glNormal3d( v, -u, 0 )
    gl.glVertex3d( r1*cos(angle + 3*da), r1*sin(angle + 3*da),  width * 0.5 )
    gl.glVertex3d( r1*cos(angle + 3*da), r1*sin(angle + 3*da), -width * 0.5 )
    gl.glNormal3d(    cos(angle),           sin(angle),         0 )
  end

  gl.glVertex3d( r1*cos(0), r1*sin(0),  width * 0.5 )
  gl.glVertex3d( r1*cos(0), r1*sin(0), -width * 0.5 )
  gl.glEnd();
  gl.glShadeModel( gl.GL_SMOOTH )

  gl.glBegin( gl.GL_QUAD_STRIP )
  for i = 0, teeth do
    local angle = i * 2 * pi / teeth
    gl.glNormal3d( -cos(angle), -sin(angle), 0 )
    gl.glVertex3d( r0*cos(angle), r0*sin(angle), -width * 0.5 )
    gl.glVertex3d( r0*cos(angle), r0*sin(angle),  width * 0.5 )
  end
  gl.glEnd()
end

-- draw(): Draw a particular view of the scene
--
-- Establish variables to control which are persistent
-- across calls, and can be modified in the main loop
--local view_rotx, view_roty, view_rotz = 20, 30, 0
local view_rotx, view_roty, view_rotz = 0, 0, 0
local gear1, gear2, gear3 = 0, 0, 0
local angle = 0
--
local function draw()
  gl.glClear( bor( gl.GL_COLOR_BUFFER_BIT, gl.GL_DEPTH_BUFFER_BIT ) )
  gl.glPushMatrix()
  do
    -- This rotates the GEARS not the CAMERAVIEW
    --[[
    gl.glRotated( view_rotx, 1, 0, 0 )
    gl.glRotated( view_roty, 0, 1, 0 )
    gl.glRotated( view_rotz, 0, 0, 1 )
    --]]

    -- Red
    gl.glPushMatrix()
    do
      -- Static transtion
      gl.glTranslatef( -3, -2, 0 )
      -- Dynamic rotation of the gears
      -- This is the based in time stuff
      gl.glRotated( angle, 0,  0, 1 )
      -- The list has had the trig precalculated
      -- using lua and stored as gl calls
      -- using the glBegin/glEnd sections
      -- we just care about the predefined glcalls
      -- and execute them here
      gl.glCallList( gear1 )
    end
    gl.glPopMatrix()

    -- Green
    gl.glPushMatrix();
    do
      gl.glTranslated( 3.1, -2, 0 )
      gl.glRotated( -2 * angle - 9, 0, 0, 1 )
      gl.glCallList( gear2 )
    end
    gl.glPopMatrix()

    -- Blue
    gl.glPushMatrix();
    do
      gl.glTranslated( -3, 5, 0 )
      gl.glRotated( -2 * angle - 25, 0, 0, 1 )
      gl.glCallList( gear3 )
    end
    gl.glPopMatrix()
  end
  gl.glPopMatrix()
end

-- animate(): Change the angle based on the elapsed time
-- This angle is what rotates the gears
local function animate()
  angle = 100 * glfw.glfwGetTime()
end

-- reshape(): This function reacts to a user
-- changing the window size
local function reshape( width, height )
  local h     = height / width
  local znear = 0.001
  local zfar  = 300
  local xmax  = znear * 0.5
  gl.glViewport( 0, 0, width, height )
  gl.glMatrixMode( gl.GL_PROJECTION )
  gl.glLoadIdentity()
  gl.glFrustum( -xmax, xmax, -xmax*h, xmax*h, znear, zfar )
  gl.glMatrixMode( gl.GL_MODELVIEW )
  gl.glLoadIdentity()
  -- the last number is how far away from the gears we are
  -- glTranslatef: float
  -- glTranslated: double
  gl.glTranslatef( 0, 0, -50 );
  -- You can also rotate the view
  -- Is this better than rotation of the gears?
  -- I think they only rotate the gears and not the view
  -- because they are doing rotation there anyway, so it saves
  -- on some computation potentially
  -- In our case, we will not be having rotation of 
  -- the gears always, so rotation of the view is probably safe
  --  gl.glRotated( view_rotx, 1, 0, 0 )
  --  gl.glRotated( view_roty, 0, 1, 0 )
  --  gl.glRotated( view_rotz, 0, 0, 1 )
end

-- Initialize the gears, lighting
local function init()
	
	-- White background
	gl.glClearColor(1.0, 1.0, 1.0, 0.0);
	
  local pos   = ffi.new( "float[4]", 5, 5, 10, 0 )
  local red   = ffi.new( "float[4]", 0.8, 0.1, 0, 1 )
  local green = ffi.new( "float[4]", 0, 0.8, 0.2, 1 )
  local blue  = ffi.new( "float[4]", 0.2, 0.2, 1, 1 )

  gl.glLightfv( gl.GL_LIGHT0, gl.GL_POSITION, pos);
  gl.glEnable(  gl.GL_CULL_FACE  )
	-- http://www.opengl.org/discussion_boards/showthread.php/149490-glColorPointer-PROBLEM!!
	gl.glColorMaterial(gl.GL_FRONT_AND_BACK, gl.GL_AMBIENT_AND_DIFFUSE);
	gl.glEnable(gl.GL_COLOR_MATERIAL);
  gl.glEnable(  gl.GL_LIGHTING   )
  gl.glEnable(  gl.GL_LIGHT0     )
  gl.glEnable(  gl.GL_DEPTH_TEST )

  gear1 = gl.glGenLists(1)
  gl.glNewList( gear1, gl.GL_COMPILE )
  gl.glMaterialfv( gl.GL_FRONT, gl.GL_AMBIENT_AND_DIFFUSE, red )
  gear( 1, 4, 1, 20, 0.7 )
  gl.glEndList()

  gear2 = gl.glGenLists(1)
  gl.glNewList( gear2, gl.GL_COMPILE )
  gl.glMaterialfv( gl.GL_FRONT, gl.GL_AMBIENT_AND_DIFFUSE, green )
  gear( 0.5, 2, 2, 10, 0.7 )
  gl.glEndList()

  gear3 = gl.glGenLists(1)
  gl.glNewList( gear3, gl.GL_COMPILE )
  gl.glMaterialfv( gl.GL_FRONT, gl.GL_AMBIENT_AND_DIFFUSE, blue )
  gear( 1.3, 2, 0.5, 10, 0.7 )
  gl.glEndList()

  gl.glEnable( gl.GL_NORMALIZE )
end

-- Helper function to translate key presses
local function pressed( key )
  return glfw.glfwGetKey( glfw["GLFW_KEY_" .. key:upper()] ) == glfw.GLFW_PRESS
end

--local buffer = ffi.new( "GLuint" )
--http://stackoverflow.com/questions/7561165/rendering-kinect-point-cloud-with-vertex-buffer-object-vbo
dd = carray.int(1)
dd_star = dd:pointer()
buffer = ffi.cast('GLuint',dd)
buffer_star = ffi.cast('GLuint*',dd_star)
local function init_pointcloud()
  gl.glGenBuffers(1, buffer_star); -- Needs pointer
  gl.glBindBuffer(gl.GL_ARRAY_BUFFER, buffer); -- Needs value
  gl.glBufferData(gl.GL_ARRAY_BUFFER, 
  (ffi.sizeof('GLfloat') * 640 * 480 * 3) --position
  + (ffi.sizeof('GLfloat') * 640 * 480 * 3), --color
  nil, gl.GL_STREAM_DRAW);
	
	-- Set point size
	gl.glEnable( gl.GL_POINT_SMOOTH );
	gl.glPointSize( .01 );
end

local point_cloud = {}
point_cloud.points_color = ffi.new('GLfloat[?]',640 * 480 * 3, 127)
point_cloud.points_position = ffi.new('GLfloat[?]',640 * 480 * 3, 0)

for j=1,480 do
 for i=1,640 do
	local coord = ( (j-1)*640 + (i-1) )*3;
	local x = (i - 320) / 10;
	local y = (j - 240) / 10;
	local z = 10/(x^2+y^2+1);
	point_cloud.points_position[coord] = z;
	point_cloud.points_position[coord+1] = y
	point_cloud.points_position[coord+2] = x;
	if x<0 then
		point_cloud.points_color[coord] = 0
		point_cloud.points_color[coord+1] = 1
		point_cloud.points_color[coord+2] = 0
	else
		point_cloud.points_color[coord] = 0
		point_cloud.points_color[coord+1] = 0
		point_cloud.points_color[coord+2] = 1
	end
 end
end

local function update_pointcloud()
  -- save the initial ModelView matrix before modifying ModelView matrix
  --gl.glPushMatrix();

  gl.glBindBuffer(gl.GL_ARRAY_BUFFER, buffer);
  gl.glBufferSubData(gl.GL_ARRAY_BUFFER, 0,  
	ffi.sizeof('GLfloat') * 640 * 480 * 3, 
	point_cloud.points_color);
  gl.glBufferSubData(gl.GL_ARRAY_BUFFER, 
	ffi.sizeof('GLfloat') * 640 * 480 * 3, --offset of the size of the colors
	(ffi.sizeof('GLfloat') * 640 * 480 * 3), 
	point_cloud.points_position);

  -- enable vertex arrays
  gl.glEnableClientState(gl.GL_VERTEX_ARRAY);
  gl.glEnableClientState(gl.GL_COLOR_ARRAY);

  gl.glColorPointer(3, gl.GL_FLOAT, 0, ffi.cast('void*',0) );
	gl.glVertexPointer(3, gl.GL_FLOAT, 0, ffi.cast('void*',ffi.sizeof('GLfloat')*640*480*3) );

	-- Draw the points
	gl.glDrawArrays(gl.GL_POINTS, 0, 640*480);

-- disable vertex arrays
  gl.glDisableClientState(gl.GL_VERTEX_ARRAY);  
  gl.glDisableClientState(gl.GL_COLOR_ARRAY);

  gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0);

  --gl.glPopMatrix();
end

local t0 = 0;
local disp_t = 0; -- Last display update
local dtScaling = 50;
local function main()
  assert( glfw.glfwInit() )
  glfw.glfwOpenWindowHint( glfw.GLFW_DEPTH_BITS, 8 );
  local window = glfw.glfwOpenWindow(1024,768, -- width, height
  0,0,0, -- pixel bits for RGB
  0,8,0, --alpha, depth, stencil,
  glfw.GLFW_WINDOW)
  glfw.glfwSetWindowTitle("Team THOR 3D Point Cloud Visualizer")
  init()
  init_pointcloud()
  local ffi_w, ffi_h = ffi.new( "int[1]" ), ffi.new( "int[1]" )
  local width, height

  while glfw.glfwGetKey( glfw.GLFW_KEY_ESC ) ~= glfw.GLFW_PRESS do
    -- Resize 
    glfw.glfwGetWindowSize( ffi_w, ffi_h)
    if width ~= ffi_w[0] or height ~= ffi_h[0] then
      width, height = ffi_w[0], ffi_h[0]
      reshape( width, height )
    end

    --[[
    gl.glRotated( view_rotx, 1, 0, 0 )
    gl.glRotated( view_roty, 0, 1, 0 )
    gl.glRotated( view_rotz, 0, 0, 1 )
    --]]
		
    draw();
    animate();
update_pointcloud();
    glfw.glfwSwapBuffers();
    glfw.glfwPollEvents();

    -- Timing and display
    local dt = glfw.glfwGetTime() - t0;
    t0 = glfw.glfwGetTime()
    if math.floor(t0)>disp_t then
      disp_t = math.floor(t0)
      glfw.glfwSetWindowTitle(
      string.format("Team THOR 3D Point Cloud Visualizer (%0.2f FPS)",1/dt)
      )
    end

    if pressed( "Z" ) and pressed( "LSHIFT" ) then
      gl.glRotated( -1*dtScaling*dt, 0, 0, 1 )
    end
    if pressed( "Z" ) and not pressed( "LSHIFT" ) then
      gl.glRotated( dtScaling*dt, 0, 0, 1 )
    end
    if pressed( "UP" ) then
      gl.glRotated( dtScaling*dt, 1, 0, 0 )
    end
    if pressed( "DOWN" ) then
      gl.glRotated( -1*dtScaling*dt, 1, 0, 0 )
    end
    if pressed( "LEFT" ) then
      gl.glRotated( dtScaling*dt, 0, 1, 0 )
    end
    if pressed( "RIGHT" ) then
      gl.glRotated( -1*dtScaling*dt, 0, 1, 0 )
    end
  end -- while
  glfw.glfwCloseWindow()
  glfw.glfwTerminate();
end --main 

main()

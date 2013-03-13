dofile('../../include.lua')
-- Require the right libraries
-- The ffi libraty is with luajit
local ffi  = require( "ffi" )
-- The ffi directory is in Util
local gl   = require( "ffi/OpenGL" )
local glfw = require( "ffi/glfw" )
-- CArray is manually placed in Lib
require 'carray'

-- Convienence functions
local pi   = math.pi
local bor, sin, cos, sqrt, pi = bit.bor, math.sin, math.cos, math.sqrt, math.pi

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
  -- How far away we view the scene
  gl.glTranslatef( 0, 0, -50 );
end

-- Initialize the gears, lighting
local function init()
  -- White background
  gl.glClearColor(1.0, 1.0, 1.0, 0.0);

  -- http://www.opengl.org/discussion_boards/showthread.php/149490-glColorPointer-PROBLEM!!
  -- gl.glColorMaterial(gl.GL_FRONT_AND_BACK, gl.GL_AMBIENT_AND_DIFFUSE);
  -- gl.glEnable( gl.GL_COLOR_MATERIAL );
  -- Not sure what these do
  gl.glEnable(  gl.GL_CULL_FACE  )
  gl.glEnable(  gl.GL_DEPTH_TEST )
  gl.glEnable( gl.GL_NORMALIZE )
end

-- Helper function to translate key presses
local function pressed( key )
  return glfw.glfwGetKey( glfw["GLFW_KEY_" .. key:upper()] ) == glfw.GLFW_PRESS
end

--http://stackoverflow.com/questions/7561165/rendering-kinect-point-cloud-with-vertex-buffer-object-vbo
local dd = carray.int(1)
local dd_star = dd:pointer()
local buffer = ffi.cast('GLuint',dd)
local buffer_star = ffi.cast('GLuint*',dd_star)
local cloud_width = 320;
local cloud_height = 240;
local n_cloud_points = cloud_width * cloud_height;
local cloud_buffer_sz = cloud_width * cloud_height*3;
local point_cloud = {}
point_cloud.points_color = ffi.new('GLfloat[?]', cloud_width * cloud_height * 3, 127)
point_cloud.points_position = ffi.new('GLfloat[?]', cloud_width * cloud_height * 3, 0)
-- Initialize the point cloud
local function init_pointcloud()
  gl.glGenBuffers(1, buffer_star); -- Needs pointer
  gl.glBindBuffer(gl.GL_ARRAY_BUFFER, buffer); -- Needs value
  gl.glBufferData(gl.GL_ARRAY_BUFFER, 
  (ffi.sizeof('GLfloat') * cloud_buffer_sz) --position
  + (ffi.sizeof('GLfloat') * cloud_buffer_sz), --color
  nil, gl.GL_STREAM_DRAW);
  -- Set point size
  gl.glEnable( gl.GL_POINT_SMOOTH );
  gl.glPointSize( .01 );
end
-- Update the point cloud
local function update_pointcloud()
	for j=1,cloud_height do
	  for i=1,cloud_width do
	    local coord = ( (j-1)*cloud_width + (i-1) )*3;
	    local x = (i - cloud_width/2) / 10;
	    local y = (j - cloud_height/2) / 10;
	    local z = 10/(x^2+y^2+1);
	    point_cloud.points_position[coord] = x;
	    point_cloud.points_position[coord+1] = y
	    point_cloud.points_position[coord+2] = z;
	    if x<0 then
	      point_cloud.points_color[coord] = 0
	      point_cloud.points_color[coord+1] = math.random()
	      point_cloud.points_color[coord+2] = 0
	    else
	      point_cloud.points_color[coord] = 0
	      point_cloud.points_color[coord+1] = 0
	      point_cloud.points_color[coord+2] = 1
	    end
	  end
	end
end

local function draw_pointcloud()
  -- save the initial ModelView matrix before modifying ModelView matrix

  gl.glBindBuffer(gl.GL_ARRAY_BUFFER, buffer);
  gl.glBufferSubData(gl.GL_ARRAY_BUFFER, 0,  
  ffi.sizeof('GLfloat') * cloud_buffer_sz, 
  point_cloud.points_color);
  gl.glBufferSubData(gl.GL_ARRAY_BUFFER, 
  ffi.sizeof('GLfloat') * cloud_buffer_sz, --offset of the size of the colors
  (ffi.sizeof('GLfloat') * cloud_buffer_sz), 
  point_cloud.points_position);

  -- enable vertex arrays
  gl.glEnableClientState(gl.GL_VERTEX_ARRAY);
  gl.glEnableClientState(gl.GL_COLOR_ARRAY);

  gl.glColorPointer(3, gl.GL_FLOAT, 0, ffi.cast('void*',0) );
  gl.glVertexPointer(3, gl.GL_FLOAT, 0, ffi.cast('void*',ffi.sizeof('GLfloat')*cloud_buffer_sz) );

  -- Draw the points
  gl.glDrawArrays(gl.GL_POINTS, 0, n_cloud_points);

  -- disable vertex arrays
  gl.glDisableClientState(gl.GL_VERTEX_ARRAY);  
  gl.glDisableClientState(gl.GL_COLOR_ARRAY);

  gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0);

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

		-- Grab updates from SHM
		update_pointcloud();

    -- Clear the scene
    gl.glClear( bor( gl.GL_COLOR_BUFFER_BIT, gl.GL_DEPTH_BUFFER_BIT ) )
    -- Draw the screen
		draw_pointcloud();
    -- Swap the screen into the display
    glfw.glfwSwapBuffers();
    -- Look for keyboard events
    glfw.glfwPollEvents();

    -- Timing
    local dt = glfw.glfwGetTime() - t0;
    t0 = glfw.glfwGetTime()

    -- Dubugging Display
    if math.floor(t0)>disp_t then
      disp_t = math.floor(t0)
      glfw.glfwSetWindowTitle(
      string.format("Team THOR 3D Point Cloud Visualizer (%0.2f FPS)",1/dt)
      )
    end

    -- View rotations
    -- TODO: make more intuitive
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

-- Run the code!
main()

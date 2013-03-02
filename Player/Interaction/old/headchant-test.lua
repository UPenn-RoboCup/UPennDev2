-- Add the required paths
-- uname  = io.popen('uname -s')
-- system = uname:read();
cwd = '.';
package.path = cwd.."/../Util/?.lua;"..package.path;

local ffi = require( "ffi" )
local gl = require( "ffi/OpenGL" ) -- This is required on Windows to work with REGAL
local glfw = require( "ffi/glfw" )
local cr = require ( "ffi/cairo" )
assert( glfw.glfwInit() )

local window = glfw.glfwOpenWindow(320,200, -- width, height
	 0,0,0, -- pixel bits for RGB
	 0,8,0, --alpha, depth, stencil,
	 glfw.GLFW_WINDOW)
--glfw.glfwCloseWindow

--glfw.glfwSetInputMode( window, glfw.GLFW_STICKY_KEYS, 1 )
--glfw.glfwMakeContextCurrent( window );
--glfw.glfwSwapInterval( 0 ) -- 0=nosync 1=60fps

glfw.glfwSetCharCallback(
   ffi.cast( "GLFWcharfun", 
	 function(w,c) 
	 print(w)
	 print(c)
   end
)
)

while glfw.glfwGetKey( glfw.GLFW_KEY_ESC ) ~= glfw.GLFW_PRESS do
    glfw.glfwSwapBuffers()
    glfw.glfwPollEvents() 
 end

window = nil
--collectgarbage()

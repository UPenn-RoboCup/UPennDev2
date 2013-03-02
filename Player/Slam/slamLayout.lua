pwd = '.'
-- Require the relavent libraries
require 'torch'
torch.Tensor = torch.ByteTensor
require 'unix'
require 'tutil'

package.path = pwd..'/../Util/?.lua;'..package.path
local ffi = require 'ffi'
local zmq = require 'ffi/zmq'
require 'ffi/torchffi'

-- Globally accessable QImage
sz = {240,320}
sz = {101,101}
sz = {201,201}
qimage = QImage(sz[2],sz[1],QImage.Format.Format_RGB32)

slam_map = torch.ByteTensor():ones(sz[1],sz[2])*128
slam_map_cdata = torch.data( slam_map )
local tmp1 = slam_map:select(2, 1); -- smooth stride!
local tmp2 = slam_map:select(1, 1); -- smooth stride!
print('Strd',slam_map:stride(1), slam_map:stride(2) )
print('Cont', tmp1:isContiguous(),tmp2:isContiguous() )
print('Sz', tmp1:size()[1],tmp2:size()[1] )
for line_no = 1,sz[1] do
	local map_line = slam_map:select(1, line_no); -- smooth stride!
	tutil.tensor2qimage( map_line, qimage:scanLine(line_no-1) )
end

-- Initialize the context
local n_zmq_threads = 1;
local context_handle = zmq.zmq_init( n_zmq_threads );
assert (context_handle);

-- Set the socket type
local test_socket = zmq.zmq_socket (context_handle, zmq.ZMQ_SUB);
assert (test_socket);

-- Connect to a message pipeline
local rc = zmq.zmq_bind( test_socket, "ipc:///tmp/map_pubsub" )
assert (rc == 0);
zmq.zmq_setsockopt(test_socket, zmq.ZMQ_SUBSCRIBE, '', 0)

print('Sucessfully subscribed to the message pipeline!')

local imageview = function()
  local scene = QGraphicsScene.new()
  local view = QGraphicsView.new(scene)
--  local piximage = QPixmap.new()
--  piximage:fromImage(qimage,Qt.AutoColor)
  pixmapitem = QGraphicsPixmapItem.new( QPixmap.fromImage(qimage) )
  scene:addItem(pixmapitem)
  return view;
end

Widget = function(...)
  local this = QWidget(...)
  this:startTimer(33) -- 30Hz

  -- Set the high level layout
  local hbox = QHBoxLayout(...)
  this.view = imageview()
  hbox:addWidget(this.view)
  -- Set the default widget layout
  this:setLayout(hbox)

  -- Set up the timer event
  function this:timerEvent(e)
	  local n_bytes_recv = zmq.zmq_recv( test_socket, slam_map_cdata, sz[1]*sz[2], 0);
	  --print('Received '..n_bytes_recv.." bytes")
    drawTensor(this)
	-- Only redraws every other timer event??? why?
  end

  return this
end

function drawTensor( widget )
	-- Get the pointer to the Torch data
	local t0 = unix.time()
	for line_no = 1,sz[1] do
		local map_line = slam_map:select(1, line_no); -- smooth stride!
		tutil.tensor2qimage( map_line, qimage:scanLine(line_no-1) )
	end
	local t1 = unix.time()
	--print('Time to copy: ',t1-t0)
pixmapitem:setPixmap( QPixmap.fromImage(qimage) )

end
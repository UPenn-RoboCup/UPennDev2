dofile('../../include.lua')
local simple_ipc = require 'simple_ipc'
require 'qtcore'
require 'qtgui'
require 'torch'
require 'tutil'
local ffi = require 'ffi'
require 'ffi/torchffi'
torch.Tensor = torch.ByteTensor

-- Globally accessable QImage
--sz = {201,201}
--sz = {401,401}
sz = {601,601}
qimage = QImage(sz[2],sz[1],QImage.Format.Format_RGB32)

omap_channel = simple_ipc.setup_subscriber('omap');
slam_map = torch.ByteTensor():ones(sz[1],sz[2])*128
slam_map_cdata = torch.data( slam_map )
timestamp_cdata = ffi.new('double[1]',0);
local tmp1 = slam_map:select(2, 1); -- smooth stride!
local tmp2 = slam_map:select(1, 1); -- smooth stride!
print('Strd',slam_map:stride(1), slam_map:stride(2) )
print('Cont', tmp1:isContiguous(),tmp2:isContiguous() )
print('Sz', tmp1:size()[1],tmp2:size()[1] )
for line_no = 1,sz[1] do
	local map_line = slam_map:select(1, line_no); -- smooth stride!
	tutil.tensor2qimage( map_line, qimage:scanLine(line_no-1) )
end

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
	  local n_bytes_recv, has_more = omap_channel:receive( slam_map_cdata, sz[1]*sz[2] );
		if has_more then 
			local nbytes, has_more = omap_channel:receive(timestamp_cdata)
		else
			print('No omap timestamp received!')
		end
	  --print('Received '..n_bytes_recv.." bytes")
    drawTensor(this)
	-- Only redraws every other timer event??? why?
  end

  return this
end

function drawTensor( widget )
	-- Get the pointer to the Torch data
	for line_no = 1,sz[1] do
		local map_line = slam_map:select(1, line_no); -- smooth stride!
		tutil.tensor2qimage( map_line, qimage:scanLine(line_no-1) )
	end
	local timestamp = timestamp_cdata[0]
	print('Map timestamp',timestamp)
	pixmapitem:setPixmap( QPixmap.fromImage(qimage) )

end





-- Initial Qt Application
app = QApplication(1 + select('#', ...), {arg[0], ...})
app.__gc = app.delete -- take ownership of object

local centralizeWindow = function(window)
  -- Get Screen Size
  local desktop = QApplication.desktop()
  local screenWidth = desktop:width()
  local screenHeight = desktop:height()

  local x = (screenWidth - window.width) / 2
  local y = (screenHeight - window.height) / 2

  window:resize(window.width, window.height)
  window:move(x, y)
end

local createWindow = function(...)
  local this = QMainWindow(...)

  -- add menu
  -- add toolbar
  -- statusbar

  -- Add central Widget
  local widget = Widget()
  this:setCentralWidget(widget)

  return this;
end

-- Set the Window properties
window = createWindow()
window:setWindowTitle("THOR SLAM Visualizer")
window:setWindowIcon(QIcon("favicon.ico"));
-- iphone 4s size
-- http://en.wikipedia.org/wiki/List_of_common_resolutions
window.width = 960
window.height = 640
centralizeWindow(window)
window:show()
app.exec()

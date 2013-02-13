-- Require the relavent libraries
require 'torch'
require 'unix'
require 'tutil'

-- Globally accessable QImage
sz = {240,320}
torch.Tensor = torch.ByteTensor
slam_map = torch.ones(sz[1],sz[2])*128
qimage = QImage(sz[2],sz[1],QImage.Format.Format_RGB32)
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
    drawTensor(this, e)
	-- Only redraws every other timer event??? why?
  end

  return this
end

function drawTensor( widget, e )
	-- Get the pointer to the Torch data
	local t0 = unix.time()
	for line_no = 1,sz[1] do
		local map_line = slam_map:select(1, line_no); -- smooth stride!
		tutil.tensor2qimage( map_line, qimage:scanLine(line_no-1) )
	end
	local t1 = unix.time()
	--print('Time to copy: ',t1-t0)
  slam_map:sub(1,20):fill( math.random(8)*32 );
pixmapitem:setPixmap( QPixmap.fromImage(qimage) )
local t2 = unix.time()
--print('Time to copy: ',t2-t1, 1/(t2-t0)..'fps')

end

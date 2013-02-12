-- Require the relavent libraries
local glue = require'glue'
local ffi = require'ffi'
local bit = require'bit'
local bmpconv = require'bmpconv'
local readfile = glue.readfile

-- Globally accessable QImage
qimage = QImage(640,480,QImage.Format.Format_RGB888)

local imageview = function()
  -- Create GraphicsScene
  scene = QGraphicsScene.new()
  -- Create GraphicView, based on Graphic Scene, widget on GUI
  local view = QGraphicsView.new(scene)
	piximage = QPixmap.new()
	piximage:convertFromImage(qimage, Qt.AutoColor)
	pixmapitem = QGraphicsPixmapItem.new(piximage)
  -- ConvertToPixmap for Graphic Scene
  scene:addItem(pixmapitem)
  return view;
end

Widget = function(...)
  local this = QWidget(...)
  this:startTimer(100) -- 10Hz

  -- Set the high level layout
  local hbox = QHBoxLayout(...)
  this.view = imageview()
  hbox:addWidget(this.view)
  -- Set the default widget layout
  this:setLayout(hbox)

  -- Set up the timer event
  function this:timerEvent(e)
    local qtime = QTime.currentTime()
    drawTensor(this, e)
		scene:update( scene:sceneRect() )
		-- Only redraws every other timerevent??? why?
  end

  return this
end

function drawTensor( widget, e )
  line_no = math.random(480)
  local scanline = ffi.cast('uint8_t*', qimage:scanLine(line_no))
local channel = math.random(3)
  for iii=0,319 do
     scanline[iii*3+channel] = bit.lshift(1, 8)-1;
  end
channel = math.random(3)
  for iii=320,639 do
     scanline[iii*3+channel] = bit.lshift(1, 8)-1;
  end
  --scanline[iii*3] = bit.lshift(1, 8)-1; -- R
  --scanline[iii*3+1] = bit.lshift(1, 7); -- G
  --scanline[iii*3+2] = bit.lshift(1, 7); -- B
	piximage:convertFromImage(qimage, Qt.AutoColor)
	--print('update')
end

local glue = require 'glue'
local ffi = require 'ffi'
local bmpconv = require 'bmpconv'
local readfile = glue.readfile
require 'unit'
local libpng = require 'libpng'
local carray = require 'carray'

--imghandle = carray.byte(640*480*3)
--img = ffi.cast('unsigned char*', imghandle:pointer())
img = ffi.new('unsigned char[?], 640 * 480 * 3)

-- load png with libpng
-- use raw pixel data to create a QImage for display

local  initDraw = function(self, state)
--  print('open file')
  local fileDialog = QFileDialog()
  local fileName = fileDialog:getOpenFileName(
                          "Open File", "", "Log File (*.png)")
  print(fileName:toUtf8())
  local fname = fileName:toUtf8()
  imgload = libpng.load({path = fname})
  ffi.copy(img, imgload.data, imgload.h * imgload.w * 3)
  --img = openimage(fname)
  qimage = QImage(img, imgload.w, imgload.h, imgload.w * 3, QImage.Format.Format_RGB888)
  local piximage = QPixmap.new()
  -- ConvertToPixmap for Graphic Scene
  piximage:convertFromImage(qimage, Qt.AutoColor)
  local pixmapitem = QGraphicsPixmapItem.new(piximage)
  local scene = QGraphicsScene.new()
  scene:addItem(pixmapitem)
  self:setScene(scene)

  self:update(0,0,640,480)
end

local  updateDraw = function(self, state)
  print(state)
  if state == 1 then
    fname = 'image1.png'
  else
    fname = 'image.png'
  end
  imgload = libpng.load({path = fname})
  ffi.copy(img, imgload.data, imgload.h * imgload.w * 3)
--  --img = openimage(fname)
--  qimage = QImage(img, imgload.w, imgload.h, imgload.w * 3, QImage.Format.Format_RGB888)
--  local piximage = QPixmap.new()
--  -- ConvertToPixmap for Graphic Scene
--  piximage:convertFromImage(qimage, Qt.AutoColor)
--  local pixmapitem = QGraphicsPixmapItem.new(piximage)
--  local scene = QGraphicsScene.new()
--  scene:addItem(pixmapitem)
--  self:setScene(scene)

  self:update(0,0,640,480)
  print(img)
end

local updateBackward = function(self, state)
  print('Backward')
  updateDraw(self, 1)
end

local updateBBackward = function(self, state)
  print('BBackward')
  updateDraw(self, 2)
end

local updateForward = function(self, state)
  print('Forward')
  updateDraw(self, 3)
end

local updateFForward = function(self, state)
  print('FForward')
  updateDraw(self, 4)
end

local imageview = function()
    -- Create GraphicsScene
    local scene = QGraphicsScene.new()
    -- Create GraphicView, based on Graphic Scene, widget on GUI
    local view = QGraphicsView.new(scene)
    -- create empty image
    local qimage = QImage(640, 480, QImage.Format.Format_RGB888)
    local piximage = QPixmap.new()
    -- ConvertToPixmap for Graphic Scene
    piximage:convertFromImage(qimage, Qt.AutoColor)
    local pixmapitem = QGraphicsPixmapItem.new(piximage)
    scene:addItem(pixmapitem)
    
    view:setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
    view:setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
    return view
end

Widget = function(...)
  local this = QWidget(...)
  local hbox = QHBoxLayout(...)

  -- left buttons 
  local leftvbox = QVBoxLayout()
    local loadMontage = QPushButton("Load Montage", this)
--    loadMontage:__addMethod('open()', initDraw())
    leftvbox:addWidget(loadMontage)
    
    local colorGroup = QGroupBox("Colors", this)
    leftvbox:addWidget(colorGroup)
    local colorGroupVbox = QVBoxLayout()
    local color = {}
    colorLabel = {"Orange", "Yellow", "Cyan", "Field", "White", "Robot Blue", "Robot Pink"}
    for i = 1 , 7 do
      color[i] = QRadioButton(colorLabel[i], this)
      colorGroupVbox:addWidget(color[i])
    end
    colorGroupVbox:addStretch(1)
    color[1]:setChecked(true)
    colorGroup:setLayout(colorGroupVbox)
    
  
    local toggleView = QPushButton("Toggle View", this)
    leftvbox:addWidget(toggleView)
  
    local clearSelection = QPushButton("Clear Selection", this)
    leftvbox:addWidget(clearSelection)
  
    local threshold = QLabel("Threshold", this)
    threshold:setAlignment(Qt.AlignmentFlag.AlignHCenter)
    leftvbox:addWidget(threshold)
  
    local thresholdSlider = QSlider("Horizontal", this)
    thresholdSlider:setRange(0, 128)
    thresholdSlider:setValue(14)
    leftvbox:addWidget(thresholdSlider)
  
    local saveColors = QPushButton("Save Colors", this)
    leftvbox:addWidget(saveColors)
  
    local saveLut = QPushButton("Save LUT", this)
    leftvbox:addWidget(saveLut)

  local rightvbox = QVBoxLayout()
    -- Image Display
    local view = imageview()
    view:__addmethod("open()", initDraw)
    view:__addmethod("updateBackward()", updateBackward)
    view:__addmethod("updateBBackward()", updateBBackward)
    view:__addmethod("updateForward()", updateForward)
    view:__addmethod("updateFForward()", updateFForward)

    -- Image file Control Buttons
    local fileControlhbox = QHBoxLayout()
      -- add file control buttons
      local prevLog = QPushButton("Prev Log", this)
      local logName = QLabel("No File Selected", this)
      logName:setAlignment(Qt.AlignmentFlag.AlignHCenter + Qt.AlignmentFlag.AlignVCenter)
      local nextLog = QPushButton("Next Log", this)
      fileControlhbox:addWidget(prevLog)
      fileControlhbox:addWidget(logName)
      fileControlhbox:addWidget(nextLog)

    -- Image frame Control Buttons
    local frameControlhbox = QHBoxLayout()
      local bbackward = QPushButton("<<", this)
      local backward = QPushButton("<-", this)
      local frameLabel = QLabel("1", this)
      frameLabel:setAlignment(Qt.AlignmentFlag.AlignHCenter + Qt.AlignmentFlag.AlignVCenter)
      local forward = QPushButton("->", this)
      local fforward = QPushButton(">>", this)
      frameControlhbox:addWidget(bbackward)
      frameControlhbox:addWidget(backward)
      frameControlhbox:addWidget(frameLabel)
      frameControlhbox:addWidget(forward)
      frameControlhbox:addWidget(fforward)

    rightvbox:addWidget(view)
    rightvbox:addLayout(fileControlhbox)
    rightvbox:addLayout(frameControlhbox)

  hbox:addLayout(leftvbox)
  hbox:addLayout(rightvbox)
--  hbox:addSpacing(10)
--  hbox:addWidget(view)
  
  loadMontage:connect('2clicked()', view, '1open()')
  backward:connect('2clicked()', view, '1updateBackward()')
  bbackward:connect('2clicked()', view, '1updateBBackward()')
  forward:connect('2clicked()', view, '1updateForward()')
  fforward:connect('2clicked()', view, '1updateFForward()')

  this:setLayout(hbox)

  return this
end



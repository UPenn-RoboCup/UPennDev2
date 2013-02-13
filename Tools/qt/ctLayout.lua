local ffi = require 'ffi'
local libpng = require 'libpng'
local carray = require 'carray'

-- load png with libpng
-- use raw pixel data to create a QImage for display

local fileList = {}
local currentFile = '' 
local currentFileIdx = 0

local defaultW = 640
local defaultH = 480
local pimage = nil

local loadImageffi = function()
  img = ffi.new('unsigned char[?]', 640 * 480 * 3)
  imgload = libpng.load({path = fname})
  ffi.copy(img, imgload.data, imgload.h * imgload.w * 3)
  --img = openimage(fname)
  qimage = QImage(img, imgload.w, imgload.h, imgload.w * 3, QImage.Format.Format_RGB888)
  local piximage = QPixmap.new()
  -- ConvertToPixmap for Graphic Scene
  piximage:convertFromImage(qimage, Qt.AutoColor)
end

local splitPath = function(str)
  local ptr = 1
  while str:find('/', ptr) ~= nil do
    ptr = ptr + 1
  end
  path = str:sub(1, ptr - 1)
  filename = str:sub(ptr, #str)
  print(path, filename)
  return path, filename
end

local selectPixel = function(o, e)
  print(e:button(), e:pos():x(), e:pos():y(), pimage:height())
end

local  initDraw = function(self, state)
  local fileDialog = QFileDialog()
  local fileName = fileDialog:getOpenFileName(
                          "Open File", "", "Log File (*.png)")
  print(fileName:toUtf8())
  local fullfilename = fileName:toUtf8()
  local path, filename = splitPath(fileName:toUtf8())
  local listFile = assert(io.popen('/bin/ls '..path..'*.png', 'r'))
  local listFileCount = 0
  for file in listFile:lines() do
    listFileCount = listFileCount + 1
    fileList[listFileCount] = file
    if fileList[listFileCount] == fullfilename then
      currentFileIdx = listFileCount
      currentFile = fullfilename
    end
  end

--  local piximage = QPixmap.new(fullfilename)
  pimage:load(fullfilename, 'PNG', Qt.AutoColor)
  if pimage:height() ~= defaultH or pimage:width() ~= defaultW then
    print('scale image')
    pimage = pimage:scaled(defaultW, defaultH, Qt.KeepAspectRatio, Qt.FastTransformation)
  end
  local pixmapitem = QGraphicsPixmapItem.new(pimage)
  local scene = QGraphicsScene.new()
  scene:addItem(pixmapitem)
  self:setScene(scene)

  self:update(0,0,640,480)
end

local updateDraw = function(self, state)
  local idxShift = 0
  if state == 1 then
    idxShift = -10
  elseif state == 2 then
    idxShift = -1
  elseif state == 3 then
    idxShift = 1
  elseif state == 4 then
    idxShift = 10
  end
  if #fileList > 0 then
    local afterShiftIdx = currentFileIdx + idxShift
    if afterShiftIdx > 0 and afterShiftIdx <= #fileList then
      currentFileIdx = afterShiftIdx
      currentFile = fileList[currentFileIdx]
    end
  end
  pimage:load(currentFile, 'PNG', Qt.AutoColor)
  if pimage:height() ~= defaultH or pimage:width() ~= defaultW then
    print('scale image')
    pimage = pimage:scaled(defaultW, defaultH, Qt.IgnoreAspectRatio, Qt.FastTransformation)
  end
  local pixmapitem = QGraphicsPixmapItem.new(pimage)
  local scene = QGraphicsScene.new()
  scene:addItem(pixmapitem)
  self:setScene(scene)

  self:update(0,0,640,480)
--  print(img)
end

local updateBBackward = function(self, state)
  print('BBackward')
  updateDraw(self, 1)
end

local updateBackward = function(self, state)
  print('Backward')
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
    local pixmapitem = QGraphicsPixmapItem.new(pimage)
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
    pimage = QPixmap.new(defaultW, defaultH)
    -- Image Display
    local view = imageview()
    view:__addmethod("open()", initDraw)
    view:__addmethod("updateBackward()", updateBackward)
    view:__addmethod("updateBBackward()", updateBBackward)
    view:__addmethod("updateForward()", updateForward)
    view:__addmethod("updateFForward()", updateFForward)
    view.mousePressEvent = selectPixel

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
      local bbackward = QPushButton("&<< (A)", this)
      local backward = QPushButton("<- (S)", this)
      local frameLabel = QLabel("1", this)
      frameLabel:setAlignment(Qt.AlignmentFlag.AlignHCenter + Qt.AlignmentFlag.AlignVCenter)
      local forward = QPushButton("(D) ->", this)
      local fforward = QPushButton("(F) >>", this)
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



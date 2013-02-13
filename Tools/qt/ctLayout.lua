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

-- load img with libpng + ffi
local loadImageffi = function(filename)
  local imgload = libpng.load({path = filename})
--  window.widget.pcimage = carray.byte(imgload.data, imgload.stride * imgload.h)
--  local qimage = QImage(window.widget.pcimage:pointer(), imgload.w, imgload.h, 
--                  imgload.w * 3, QImage.Format.Format_RGB888)
  local qimage = QImage(imgload.data, imgload.w, imgload.h, 
                  imgload.w * 3, QImage.Format.Format_RGB888)

  window.widget.pimage:convertFromImage(qimage, Qt.AutoColor)
  if window.widget.pimage:height() ~= defaultH or pimage:width() ~= defaultW then
    print('scale image')
    window.widget.pimage = window.widget.pimage:scaled(defaultW, defaultH, Qt.KeepAspectRatio, 
                            Qt.FastTransformation)
  end

end

-- load img with QPixmap constructor
local loadImage = function(filename)
  window.widget.pimage:load(filename, 'PNG', Qt.AutoColor)
  if window.widget.pimage:height() ~= defaultH or pimage:width() ~= defaultW then
    print('scale image')
    window.widget.pimage = window.widget.pimage:scaled(defaultW, defaultH, Qt.KeepAspectRatio, 
                            Qt.FastTransformation)
  end
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
  print(e:button(), e:pos():x(), e:pos():y(), window.widget.pimage:height())
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

  loadImageffi(fullfilename)
--  loadImage(fullfilename)
  window.widget.pixmapitem:setPixmap(window.widget.pimage)
  self:fitInView(0, 0, 640, 480, Qt.IgnoreAspectRatio)
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
--  loadImage(currentFile)
  loadImageffi(currentFile)
  window.widget.pixmapitem:setPixmap(window.widget.pimage)

  self:update(0,0,640,480)
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
    return view
end

Widget = function(...)
  local this = QWidget(...)
  this.hbox = QHBoxLayout(...)

  -- left buttons 
  this.leftvbox = QVBoxLayout()
    this.loadMontage = QPushButton("Load Montage", this)
    this.leftvbox:addWidget(this.loadMontage)
    
    this.colorGroup = QGroupBox("Colors", this)
    this.leftvbox:addWidget(this.colorGroup)
    this.colorGroupVbox = QVBoxLayout()
    this.color = {}
    local colorLabel = {"Orange", "Yellow", "Cyan", "Field", "White", "Robot Blue", "Robot Pink"}
    for i = 1 , 7 do
      this.color[i] = QRadioButton(colorLabel[i], this)
      this.colorGroupVbox:addWidget(this.color[i])
    end
    this.colorGroupVbox:addStretch(1)
    this.color[1]:setChecked(true)
    this.colorGroup:setLayout(this.colorGroupVbox)
    
  
    this.toggleView = QPushButton("Toggle View", this)
    this.leftvbox:addWidget(this.toggleView)
  
    this.clearSelection = QPushButton("Clear Selection", this)
    this.leftvbox:addWidget(this.clearSelection)
  
    this.threshold = QLabel("Threshold", this)
    this.threshold:setAlignment(Qt.AlignmentFlag.AlignHCenter)
    this.leftvbox:addWidget(this.threshold)
  
    this.thresholdSlider = QSlider("Horizontal", this)
    this.thresholdSlider:setRange(0, 128)
    this.thresholdSlider:setValue(14)
    this.leftvbox:addWidget(this.thresholdSlider)
  
    this.saveColors = QPushButton("Save Colors", this)
    this.leftvbox:addWidget(this.saveColors)
  
    this.saveLut = QPushButton("Save LUT", this)
    this.leftvbox:addWidget(this.saveLut)

  this.rightvbox = QVBoxLayout()
    this.pimage = QPixmap.new(defaultW, defaultH)
    -- Create GraphicsScene
    this.scene = QGraphicsScene.new(0, 0, defaultW, defaultH)
    -- Create GraphicView, based on Graphic Scene, widget on GUI
    this.view = QGraphicsView.new(this.scene)
    this.view:fitInView(0, 0, defaultW, defaultH, Qt.IgnoreAspectRatio)
    -- create empty image
    this.pixmapitem = QGraphicsPixmapItem.new(this.pimage)
    this.scene:addItem(this.pixmapitem)
    
    this.view:setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
    this.view:setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

    -- Image Display
    this.view:__addmethod("open()", initDraw)
    this.view:__addmethod("updateBackward()", updateBackward)
    this.view:__addmethod("updateBBackward()", updateBBackward)
    this.view:__addmethod("updateForward()", updateForward)
    this.view:__addmethod("updateFForward()", updateFForward)
    this.view.mousePressEvent = selectPixel

    -- Image file Control Buttons
    this.fileControlhbox = QHBoxLayout()
      -- add file control buttons
      this.prevLog = QPushButton("Prev Log", this)
      this.logName = QLabel("No File Selected", this)
      this.logName:setAlignment(Qt.AlignmentFlag.AlignHCenter + Qt.AlignmentFlag.AlignVCenter)
      this.nextLog = QPushButton("Next Log", this)
      this.fileControlhbox:addWidget(this.prevLog)
      this.fileControlhbox:addWidget(this.logName)
      this.fileControlhbox:addWidget(this.nextLog)

    -- Image frame Control Buttons
    this.frameControlhbox = QHBoxLayout()
      this.bbackward = QPushButton("&<< (A)", this)
      this.backward = QPushButton("<- (S)", this)
      this.frameLabel = QLabel("1", this)
      this.frameLabel:setAlignment(Qt.AlignmentFlag.AlignHCenter + Qt.AlignmentFlag.AlignVCenter)
      this.forward = QPushButton("(D) ->", this)
      this.fforward = QPushButton("(F) >>", this)
      this.frameControlhbox:addWidget(this.bbackward)
      this.frameControlhbox:addWidget(this.backward)
      this.frameControlhbox:addWidget(this.frameLabel)
      this.frameControlhbox:addWidget(this.forward)
      this.frameControlhbox:addWidget(this.fforward)

    this.rightvbox:addWidget(this.view)
    this.rightvbox:addLayout(this.fileControlhbox)
    this.rightvbox:addLayout(this.frameControlhbox)

  this.hbox:addLayout(this.leftvbox)
  this.hbox:addLayout(this.rightvbox)
--  this.hbox:addSpacing(10)
--  this.hbox:addWidget(view)
  
  this.loadMontage:connect('2clicked()', this.view, '1open()')
  this.backward:connect('2clicked()',    this.view, '1updateBackward()')
  this.bbackward:connect('2clicked()',   this.view, '1updateBBackward()')
  this.forward:connect('2clicked()',     this.view, '1updateForward()')
  this.fforward:connect('2clicked()',    this.view, '1updateFForward()')

  this:setLayout(this.hbox)

  return this
end



-- Require the relavent libraries
local glue = require'glue'
local ffi = require'ffi'
local bit = require'bit'
local bmpconv = require'bmpconv'
local readfile = glue.readfile

local  open = function(self, state)
--  print('open file')
  local fileDialog = QFileDialog()
  local fileName = fileDialog:getOpenFileName(
                          "Open File", "", "Log File (*.lua)")
  print(fileName:toUtf8())
end

local imageview = function()
    -- Create GraphicsScene
    local scene = QGraphicsScene.new()
    -- Create GraphicView, based on Graphic Scene, widget on GUI
    local view = QGraphicsView.new(scene)
    local piximage = QPixmap.new()

    -- use raw pixel data to create a QImage for display
    local qimage = QImage(640,480,QImage.Format.Format_RGB888)
		print('ScanLine:',qimage:scanLine(23) )
		local rawdataScan23 = ffi.cast('uint8_t*', qimage:scanLine(23))
		for iii=0,639 do
			rawdataScan23[iii*3] = bit.lshift(1, 8)-1; -- R
--			rawdataScan23[iii*3+1] = bit.lshift(1, 7); -- G
--			rawdataScan23[iii*3+2] = bit.lshift(1, 7); -- B
		end
		-- Send the QImage to a C function?
    -- ConvertToPixmap for Graphic Scene
    piximage:convertFromImage(qimage, Qt.AutoColor)
    local pixmapitem = QGraphicsPixmapItem.new(piximage)
    scene:addItem(pixmapitem)
    return view
end

Widget = function(...)
  local this = QWidget(...)
  local hbox = QHBoxLayout(...)

  -- left buttons 
  local leftvbox = QVBoxLayout()
    local loadMontage = QPushButton("Load Montage", this)
    loadMontage:__addmethod("open()", open)
    loadMontage:connect('2clicked()', loadMontage, '1open()')
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

  this:setLayout(hbox)

  return this
end
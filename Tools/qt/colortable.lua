local pwd = os.getenv('PWD')
package.cpath = pwd..'/lib/?.so;'..package.cpath

require 'qtcore'
require 'qtgui'

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

local Widget = function(...)
  local this = QWidget(...)
  local hbox = QHBoxLayout(...)

  -- left buttons 
  local leftvbox = QVBoxLayout()
    local loadMontage = QPushButton("Load Montage", this)
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
  
    local thresholdSlider = QSlider("Horizontal")
    thresholdSlider:setRange(0, 128)
    thresholdSlider:setValue(14)
    leftvbox:addWidget(thresholdSlider)
  
    local saveColors = QPushButton("Save Colors", this)
    leftvbox:addWidget(saveColors)
  
    local saveLut = QPushButton("Save LUT", this)
    leftvbox:addWidget(saveLut)

  local rightvbox = QVBoxLayout()
    -- Image Display
    local scene = QGraphicsScene.new()
    local piximage = QPixmap('Image-1-10.png')
    local image = QGraphicsPixmapItem(piximage)
    scene:addItem(image)
    local view = QGraphicsView(scene)
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

local createWindow = function(...)
  local this = QMainWindow(...)

  -- add menu
  -- add toolbar
  -- statusbar
  
  -- Add central Widget
  local widget = Widget()
  this:setCentralWidget(widget)

  return this
end

window = createWindow()

window.width = 800
window.height = 600

window:setWindowTitle("UPennalizers Colortable Selection")
--window:setToolTip("QWidget")
window:setWindowIcon(QIcon("favicon.ico"));

centralizeWindow(window)
window:show()

app.exec()


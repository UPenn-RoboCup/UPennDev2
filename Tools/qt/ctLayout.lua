
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
  this.thresholdSlider:setValue(defaultThreshold)
  this.leftvbox:addWidget(this.thresholdSlider)
  
  this.saveColors = QPushButton("Save Colors", this)
  this.leftvbox:addWidget(this.saveColors)
  
  this.saveLut = QPushButton("Save LUT", this)
  this.leftvbox:addWidget(this.saveLut)

  this.rightvbox = QVBoxLayout()
  this.pimage = QPixmap.new(defaultW, defaultH)
  this.pimage:fill(Qt.white)
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
  this.logName:setAlignment(Qt.AlignmentFlag.AlignHCenter + 
                            Qt.AlignmentFlag.AlignVCenter)
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



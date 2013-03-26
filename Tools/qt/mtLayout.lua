
Widget = function(...)
  local this = QWidget(...)
  this.hbox = QHBoxLayout(...)

  this.leftvbox = QVBoxLayout()
 
  -- left buttons 
  this.overLay = QPushButton("OverLay", this)
  this.leftvbox:addWidget(this.overLay)
 
  this.mapType = QPushButton("Map Type", this)
  this.leftvbox:addWidget(this.mapType)
  
  this.debugSwitch = QPushButton("Debug On", this)
  this.leftvbox:addWidget(this.debugSwitch)
  
--  this.threshold = QLabel("Threshold", this)
--  this.threshold:setAlignment(Qt.AlignmentFlag.AlignHCenter)
--  this.leftvbox:addWidget(this.threshold)
--  
--  this.thresholdSlider = QSlider("Horizontal", this)
--  this.thresholdSlider:setRange(0, 128)
--  this.thresholdSlider:setValue(defaultThreshold)
--  this.leftvbox:addWidget(this.thresholdSlider)
  
  this.logging = QPushButton("Logging", this)
  this.leftvbox:addWidget(this.logging)
  
--  this.saveLut = QPushButton("Save LUT", this)
--  this.leftvbox:addWidget(this.saveLut)

  this.labelGroup = QGroupBox("Labels", this)
  this.leftvbox:addWidget(this.labelGroup)
  this.labelGroupVbox = QVBoxLayout()
  this.label = {}
  local labels = {"Label A", "Label B"}
  for i = 1 , #labels do
    this.label[i] = QRadioButton(labels[i], this)
    this.labelGroupVbox:addWidget(this.label[i])
  end
--  this.labelGroupVbox:addStretch(1)
  this.label[1]:setChecked(true)
  this.labelGroup:setLayout(this.labelGroupVbox)

  this.leagueGroup = QGroupBox("Field Size", this)
  this.leftvbox:addWidget(this.leagueGroup)
  this.leagueGroupVbox = QVBoxLayout()
  this.league = {}
  local leagues = {"SPL", "NSL KidSize", "NSL TeenSize", "NSL AdultSize"}
  for i = 1 , #leagues do
    this.league[i] = QRadioButton(leagues[i], this)
    this.leagueGroupVbox:addWidget(this.league[i])
  end
--  this.leagueGroupVbox:addStretch(1)
  this.league[1]:setChecked(true)
  this.leagueGroup:setLayout(this.leagueGroupVbox)


  this.rightvbox = QVBoxLayout()
  this.pimage = QPixmap.new(defaultW, defaultH)
  this.pimage:fill(Qt.white)
  -- Create GraphicsScene
--  this.scene = QGraphicsScene.new(0, 0, defaultW, defaultH)
  this.scene = QGraphicsScene.new()
  -- Create GraphicView, based on Graphic Scene, widget on GUI
  this.view1 = QGraphicsView.new(this.scene)
--    this.view1:fitInView(0, 0, defaultW, defaultH, Qt.IgnoreAspectRatio)
    -- create empty image
    this.pixmapitem = QGraphicsPixmapItem.new(this.pimage)
    this.scene:addItem(this.pixmapitem)
    
    this.view1:setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
    this.view1:setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
  
    -- Image Display
    this.view1:__addmethod("open()", initDraw)
    this.view1:__addmethod("updateBackward()", updateBackward)
    this.view1:__addmethod("updateBBackward()", updateBBackward)
    this.view1:__addmethod("updateForward()", updateForward)
    this.view1:__addmethod("updateFForward()", updateFForward)
    this.view1.mousePressEvent = selectPixel

  this.view2 = QGraphicsView.new(this.scene)
    this.view2:fitInView(0, 0, defaultW, defaultH, Qt.IgnoreAspectRatio)
    -- create empty image
    this.pixmapitem = QGraphicsPixmapItem.new(this.pimage)
    this.scene:addItem(this.pixmapitem)
    
    this.view2:setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
    this.view2:setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
  
    -- Image Display
    this.view2:__addmethod("open()", initDraw)
    this.view2:__addmethod("updateBackward()", updateBackward)
    this.view2:__addmethod("updateBBackward()", updateBBackward)
    this.view2:__addmethod("updateForward()", updateForward)
    this.view2:__addmethod("updateFForward()", updateFForward)
    this.view2.mousePressEvent = selectPixel

  this.view3 = QGraphicsView.new(this.scene)
    this.view3:fitInView(0, 0, defaultW, defaultH, Qt.IgnoreAspectRatio)
    -- create empty image
    this.pixmapitem = QGraphicsPixmapItem.new(this.pimage)
    this.scene:addItem(this.pixmapitem)
    
    this.view3:setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
    this.view3:setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
  
    -- Image Display
    this.view3:__addmethod("open()", initDraw)
    this.view3:__addmethod("updateBackward()", updateBackward)
    this.view3:__addmethod("updateBBackward()", updateBBackward)
    this.view3:__addmethod("updateForward()", updateForward)
    this.view3:__addmethod("updateFForward()", updateFForward)
    this.view3.mousePressEvent = selectPixel

  this.view4 = QGraphicsView.new(this.scene)
    this.view4:fitInView(0, 0, defaultW, defaultH, Qt.IgnoreAspectRatio)
    -- create empty image
    this.pixmapitem = QGraphicsPixmapItem.new(this.pimage)
    this.scene:addItem(this.pixmapitem)
    
    this.view4:setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
    this.view4:setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
  
    -- Image Display
    this.view4:__addmethod("open()", initDraw)
    this.view4:__addmethod("updateBackward()", updateBackward)
    this.view4:__addmethod("updateBBackward()", updateBBackward)
    this.view4:__addmethod("updateForward()", updateForward)
    this.view4:__addmethod("updateFForward()", updateFForward)
    this.view4.mousePressEvent = selectPixel


  this.views = QGridLayout()
  this.views:addWidget(this.view1, 1, 1, 1, 1)
  this.views:addWidget(this.view2, 1, 2, 1, 1)
  this.views:addWidget(this.view3, 2, 1, 1, 1)
  this.views:addWidget(this.view4, 2, 2, 1, 1)

--  -- Image file Control Buttons
--  this.fileControlhbox = QHBoxLayout()
--  -- add file control buttons
--  this.prevLog = QPushButton("Prev Log", this)
--  this.logName = QLabel("No File Selected", this)
--  this.logName:setAlignment(Qt.AlignmentFlag.AlignHCenter + 
--                            Qt.AlignmentFlag.AlignVCenter)
--  this.nextLog = QPushButton("Next Log", this)
--  this.fileControlhbox:addWidget(this.prevLog)
--  this.fileControlhbox:addWidget(this.logName)
--  this.fileControlhbox:addWidget(this.nextLog)
--
--  -- Image frame Control Buttons
--  this.frameControlhbox = QHBoxLayout()
--  this.bbackward = QPushButton("&<< (A)", this)
--  this.backward = QPushButton("<- (S)", this)
--  this.frameLabel = QLabel("1", this)
--  this.frameLabel:setAlignment(Qt.AlignmentFlag.AlignHCenter + Qt.AlignmentFlag.AlignVCenter)
--  this.forward = QPushButton("(D) ->", this)
--  this.fforward = QPushButton("(F) >>", this)
--  this.frameControlhbox:addWidget(this.bbackward)
--  this.frameControlhbox:addWidget(this.backward)
--  this.frameControlhbox:addWidget(this.frameLabel)
--  this.frameControlhbox:addWidget(this.forward)
--  this.frameControlhbox:addWidget(this.fforward)

  this.debugInfo = QLabel("Debugging Infomation", this)

--  this.rightvbox:addWidget(this.view)
  this.rightvbox:addLayout(this.views)
  this.rightvbox:addWidget(this.debugInfo)
--  this.rightvbox:addLayout(this.fileControlhbox)
--  this.rightvbox:addLayout(this.frameControlhbox)

  this.hbox:addLayout(this.leftvbox)
  this.hbox:addLayout(this.rightvbox)
--  this.hbox:addSpacing(10)
--  this.hbox:addWidget(view)
  
--  this.loadMontage:connect('2clicked()', this.view1, '1open()')
--  this.backward:connect('2clicked()',    this.view1, '1updateBackward()')
--  this.bbackward:connect('2clicked()',   this.view1, '1updateBBackward()')
--  this.forward:connect('2clicked()',     this.view1, '1updateForward()')
--  this.fforward:connect('2clicked()',    this.view1, '1updateFForward()')

  this:setLayout(this.hbox)

  return this
end



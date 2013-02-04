local pwd = os.getenv('PWD')
package.cpath = pwd..'/lib/?.so;'..package.cpath

require 'qtcore'
require 'qtgui'
--require 'tch'

app = QApplication(1 + select('#', ...), {arg[0], ...})

local new_gui = function(...)
  local this = QGraphicsWidget.new(...);
	local quit = QPushButton.new('Quit')
	quit:setFont(QFont('Times', 18, 75))
	QObject.connect(quit, '2clicked()', QCoreApplication.instance(), '1quit()')

	local lcd = QLCDNumber.new()
	lcd:setSegmentStyle'Filled'

  local slider = QSlider.new'Horizontal'
	slider:setRange(0, 99)
	slider:setValue(0)
  slider:__addmethod("mySlot(int)", slot)
  slider:connect('2valueChanged(int)', slider, '1mySlot(int)')

	QObject.connect(slider, '2valueChanged(int)', lcd, '1display(int)')

  local scene = QGraphicsScene.new()
--  local glcd = scene.addWidget(lcd);
--  local gbutton = scene:addWidget(quit);
--  local gslider = scene:addWidget(slider);
  local piximage = QPixmap('Image-1-10.png')
  local image = QGraphicsPixmapItem(piximage)
  scene:addItem(image)
  
  view = QGraphicsView(scene)
  local layout = QGraphicsGridLayout.new()
--  layout:addItem(lcd)
--  layout:addItem(gbutton)
--  layout:addItem(gslider)
--  layout:addItem(glcd)
  this:setLayout(layout)
  return this
end

gui = new_gui()
gui:show()

app.exec()



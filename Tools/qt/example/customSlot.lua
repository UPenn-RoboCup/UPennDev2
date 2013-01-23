local pwd = os.getenv('PWD')
package.cpath = pwd..'/lib/?.so;'..package.cpath

require 'qtcore'
require 'qtgui'
-- Require torch example to integration testing
require 'tch'

app = QApplication(1 + select('#', ...), {arg[0], ...})
app.__gc = app.delete -- take ownership of object

local displaynum = function()
  print(1);
end

local LCD_Range = function(...)
	local this = QWidget.new(...)

	local lcd = QLCDNumber.new()
	lcd:setSegmentStyle'Filled'

	local slider = QSlider.new'Horizontal'
	slider:setRange(0, 99)
	slider:setValue(0)

	QObject.connect(slider, '2valueChanged(int)', lcd, '1display(int)')

	local layout = QVBoxLayout.new()
	layout:addWidget(lcd)
	layout:addWidget(slider)
	this:setLayout(layout)
	return this
end

local slot = function(self, int)
  print(int)
end



local new_gui = function(...)
  -- Create a GUI(QWidget object)
  local this = QWidget.new(...);

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

	local layout = QVBoxLayout.new()
	layout:addWidget(lcd)
	layout:addWidget(quit)
  layout:addWidget(slider)
	this:setLayout(layout)
	return this

end

-- use QWidgets for GUI
monitor = new_gui()
monitor:show();

app.exec()



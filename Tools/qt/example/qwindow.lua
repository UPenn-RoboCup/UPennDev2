local pwd = os.getenv('PWD')
package.cpath = pwd..'/lib/?.so;'..package.cpath

require 'qtcore'
require 'qtgui'

app = QApplication(1 + select('#', ...), {arg[0], ...})
app.__gc = app.delete -- take ownership of object

local onplus = function(self)
  local val = QString.toInt(QLabel.text(self))
  val = val + 1;
  QLabel.setText(self, QString.number(val))
end

local new_window = function(...)
  width = 800
  height = 600
  
  local this = QWidget.new(...)
  
  local button = QPushButton("Quit", this)
  -- [horizontal, vertical, width, height]
  button:setGeometry(0, 0, 75, 30);
  button:connect("2clicked()", app, "1quit()")
  
  local frame1 = QFrame(this, ...)
  frame1:setFrameStyle(1)
  frame1:setCursor('SizeAllCursor')
  --
  local frame2 = QFrame(this, ...)
  frame2:setFrameStyle(1)
  frame2:setCursor('WaitCursor')
  --
  local frame3 = QFrame(this, ...)
  frame3:setFrameStyle(1)
  frame3:setCursor('PointingHandCursor')

  local label = QLabel(this, ...)
  label:setGeometry(190, 80, 20, 30)
  label:setText(QString.number(40))
  label:__addmethod("OnPlus()", onplus)
  local plusbutton = QPushButton("+", this)
  plusbutton:connect('2clicked()', label, '1OnPlus()');


  layout = QGridLayout.new()
  layout:addWidget(button, 0, 0)
  layout:addWidget(frame1, 0, 1)
  layout:addWidget(frame2, 1, 0)
  layout:addWidget(frame3, 0, 2)
  layout:addWidget(label, 1, 1)
  layout:addWidget(plusbutton, 1, 2)
  this:setLayout(layout)
  
  return this
end

window = new_window()
-- Get Screen Size
desktop = QApplication.desktop()
screenWidth = desktop:width()
screenHeight = desktop:height()

x = (screenWidth - width) / 2
y = (screenHeight - height) / 2

window:resize(width, height)
window:move(x, y)
window:setWindowTitle("UPennalizers")
window:setToolTip("QWidget")
window:setWindowIcon(QIcon("favicon.ico"));

window:show()

app.exec()


local pwd = os.getenv('PWD')
package.cpath = pwd..'/lib/?.so;'..package.cpath

require 'qtcore'
require 'qtgui'

app = QApplication(1 + select('#', ...), {arg[0], ...})
app.__gc = app.delete -- take ownership of object

for k,v in pairs(QFrame) do
  print(k, v)
end

local new_window = function(...)
  width = 800;
  height = 600;
  
  local this = QWidget.new(...)
  
  local button = QPushButton("Quit", this)
  -- [horizontal, vertical, width, height]
  button:setGeometry(0, 0, 75, 30);
  button:connect("2clicked()", app, "1quit()")
  
  local frame1 = QFrame(this, ...);
  frame1:setFrameStyle(1)
  frame1:setCursor('SizeAllCursor')
  --
  local frame2 = QFrame(this, ...);
  frame2:setFrameStyle(1)
  frame2:setCursor('WaitCursor')
  --
  local frame3 = QFrame(this, ...);
  frame3:setFrameStyle(1)
  frame3:setCursor('PointingHandCursor')
  
  layout = QGridLayout.new()
  layout:addWidget(button, 0, 0)
  layout:addWidget(frame1, 0, 1)
  layout:addWidget(frame2, 1, 0)
  layout:addWidget(frame3, 0, 2)
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


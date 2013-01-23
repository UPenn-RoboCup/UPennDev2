local pwd = os.getenv('PWD')
package.cpath = pwd..'/lib/?.so;'..package.cpath

require 'qtcore'
require 'qtgui'

require ('ctLayout')

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


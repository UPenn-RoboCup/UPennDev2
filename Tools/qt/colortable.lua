local pwd = os.getenv('PWD')
package.cpath = pwd..'/lib/qt/?.so;'..package.cpath
package.path = pwd..'/lib/ffi/?.lua;'..package.path

require 'qtcore'
require 'qtgui'

require ('ctLayout')

-- Initial Qt Application
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
  this.widget = Widget()
  this:setCentralWidget(this.widget)

  return this
end

window = createWindow()
window.resizeEvent = function(e)
--  print(e:size():width(), e:size():height())
end

window.width = 800
window.height = 550

window:setWindowTitle("UPennalizers Colortable Selection")
--window:setToolTip("QWidget")
window:setWindowIcon(QIcon("favicon.ico"));

centralizeWindow(window)
window:show()

app.exec()


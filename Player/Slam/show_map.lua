-- Add the FFI bindings and the lqt library
--local pwd = os.getenv('PWD')
pwd = '.'
package.cpath = pwd..'/../Lib/qt/?.so;'..package.cpath
package.cpath = pwd..'/../Lib/?.so;'..package.cpath
--package.cpath = pwd..'/../Util/?.lua;'..package.cpath
--package.path = pwd..'/../Lib/ffi/?.lua;'..package.path

require 'qtcore'
require 'qtgui'
-- Help definitions
require ('layout_slam')

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
  local widget = Widget()
  this:setCentralWidget(widget)

  return this;
end

-- Set the Window properties
window = createWindow()
window:setWindowTitle("THOR SLAM Visualizer")
window:setWindowIcon(QIcon("favicon.ico"));
window.width = 640
window.height = 480
centralizeWindow(window)
window:show()
app.exec()

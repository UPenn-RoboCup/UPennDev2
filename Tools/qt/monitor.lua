local pwd = os.getenv('PWD')
package.cpath = pwd..'/lib/qt/?.so;'..package.cpath
package.cpath = pwd..'/lib/util/?.so;'..package.cpath
package.cpath = pwd..'/../../Player/Lib/?.so;'..package.cpath
package.cpath = pwd..'/?.so;'..package.cpath
package.path = pwd..'/../../Player/Util/?.lua;'..package.path
package.path = pwd..'/../../Player/Util/ffi/?.lua;'..package.path

require 'unix'
require 'qtcore'
require 'qtgui'

require ('mtLayout')
require ('ctImageProc')
require ('ctCommon')
require ('ctEvent')

-- Setup IPC
-- -- -- -- --
local simple_ipc = require 'simple_ipc'
local img_channel = simple_ipc.setup_subscriber('img');
img_channel.callback = function()
  local img_data, has_more = img_channel:receive();
  print('Have an image of size:',#img_data);
end
local wait_channels = { img_channel }
local channel_poll = simple_ipc.wait_on_channels( wait_channels );
-- -- -- -- --

-- Initial Qt Application
app = QApplication(1 + select('#', ...), {arg[0], ...})
app.__gc = app.delete -- take ownership of object

centralizeWindow = function(window)
  -- Get Screen Size
  local desktop = QApplication.desktop()
  local screenWidth = desktop:width()
  local screenHeight = desktop:height()
   
  local x = (screenWidth - window.width) / 2
  local y = (screenHeight - window.height) / 2
  
  window:resize(window.width, window.height)
  window:move(x, y)
end

createWindow = function(...)
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

window:setWindowTitle("UPennalizers Robot Monitor")
--window:setToolTip("QWidget")
window:setWindowIcon(QIcon("favicon.ico"));

centralizeWindow(window)
window:show()

--app.exec()
--local channel_timeout = 1e3;
local channel_timeout = 0;
while 1 do
  app.processEvents()
  channel_poll:poll(channel_timeout)
  --print(unix.time())
end

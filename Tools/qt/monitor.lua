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
  img = cjpeg.uncompress(img_data)
--  window.widget.pimage:convertFromImage(imgg, Qt.AutoColor)
----  window.widget.pixmapitem:setPixmap(window.widget.pimage)
--  window.widget.view1:update()
--  window.widget.view2:update()

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

local file = io.open('Image-1-10.jpg', 'r')
img_str = file:read('*a')
img1 = cjpeg.uncompress(img_str)
local file1 = io.open('img11.jpg', 'r')
img_str1 = file1:read('*a')
img2 = cjpeg.uncompress(img_str1)

local qimage = {}
qimage[1] = QImage(img1:pointer(), defaultW, defaultH, 
                defaultW * 3, QImage.Format.Format_RGB888)
qimage[2] = QImage(img2:pointer(), defaultW, defaultH, 
                defaultW * 3, QImage.Format.Format_RGB888)


local channel_timeout = 1e3;
local counter = 1
while 1 do
--  local img_data, has_more = img_channel:receive();
--  print(img_data)
--  img = cjpeg.uncompress(img_data)
--
  channel_poll:poll(channel_timeout)

--  local qimage = QImage(img:pointer(), defaultW, defaultH, 
--                  defaultW * 3, QImage.Format.Format_RGB888)
--  window.widget.pimage:convertFromImage(_G['qimage'..counter], Qt.AutoColor)
--  window.widget.pimage:convertFromImage(qimage[counter + 1], Qt.AutoColor)
--  imgg = QImage(img:pointer(), 160, 120, 160 * 3, QImage.Format.Format_RGB888)
--  window.widget.pimage:convertFromImage(imgg, Qt.AutoColor)
----  window.widget.pimage:loadFromData(img_handle:pointer(), #jpg_str, "JPG")
--  window.widget.pixmapitem:setPixmap(window.widget.pimage)
--  window.widget.view1:update()

  counter = 1 - counter

  app.processEvents()
--  app.exec()
  print(counter)
--  unix.usleep(1e6)
  --print(unix.time())
end

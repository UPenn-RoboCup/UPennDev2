local pwd = os.getenv('PWD')
package.cpath = pwd..'/lib/?.so;'..package.cpath

require 'qtcore'
require 'qtgui'

local glue = require'glue'
local ffi = require'ffi'
local bmpconv = require'bmpconv'
local readfile = glue.readfile
require'unit'
local libpng = require'libpng'
local Q = require 'Q'

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
  
  local label = QLabel(this, ...)
  label:setGeometry(190, 80, 20, 30)
  label:setText(QString.number(40))
  label:__addmethod("OnPlus()", onplus)
  local filename = 'Image-1-10.png'
  local piximage = QPixmap.new()
  local img = libpng.load({path = filename})
  local qimage = QImage(img.data, img.w, img.h, 
                        img.w * 3, QImage.Format.Format_RGB888)

  piximage:convertFromImage(qimage, Qt.AutoColor)
  label:setPixmap(piximage)

  layout = QGridLayout.new()
  layout:addWidget(label, 1, 1)
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


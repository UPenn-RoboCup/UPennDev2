local ffi = require 'ffi'
local libpng = require 'libpng'
local carray = require 'carray'

defaultW = 640
defaultH = 480

-- load img with libpng + ffi
loadImageffi = function(filename)
  local imgload = libpng.load({path = filename})
--  window.widget.pcimage = carray.byte(imgload.data, imgload.stride * imgload.h)
--  local qimage = QImage(window.widget.pcimage:pointer(), imgload.w, imgload.h, 
--                  imgload.w * 3, QImage.Format.Format_RGB888)
  -- use raw pixel data to create a QImage for display
  local qimage = QImage(imgload.data, imgload.w, imgload.h, 
                  imgload.w * 3, QImage.Format.Format_RGB888)

  window.widget.pimage:convertFromImage(qimage, Qt.AutoColor)
  if window.widget.pimage:height() ~= defaultH or 
    window.widget.pimage:width() ~= defaultW then
    print('scale image')
    window.widget.pimage = window.widget.pimage:scaled(defaultW, defaultH, 
                              Qt.KeepAspectRatio, Qt.FastTransformation)
  end

end

-- load img with QPixmap constructor
loadImage = function(filename)
  window.widget.pimage:load(filename, 'PNG', Qt.AutoColor)
  if window.widget.pimage:height() ~= defaultH or pimage:width() ~= defaultW then
    print('scale image')
    window.widget.pimage = window.widget.pimage:scaled(defaultW, defaultH, 
                              Qt.KeepAspectRatio, Qt.FastTransformation)
  end
end



local ffi = require 'ffi'
local libpng = require 'libpng'
local carray = require 'carray'
rgb = require 'rgbselect'

defaultW = 640
defaultH = 480
defaultThreshold = 14

-- load img with libpng + ffi
loadImageffi = function(filename)
  local imgload = libpng.load({path = filename})
  window.widget.imgload = imgload
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

rgbselect = function(data, w, h, ptx, pty, threshold)
  print(data[0], data[1], data[2], ptx, pty, threshold)
  dd = rgb.select(data, w, h, ptx, pty, threshold)
  df = ffi.cast('uint8_t*', dd)
  count = 0
  for x = 0, h - 1 do
    for y = 0, w - 1 do
     if df[x * w + y] ~= 0 then
       print(df[x * w + y], x, y)
       count = count + 1
     end
    end
  end
  print('dddddd', count)
end

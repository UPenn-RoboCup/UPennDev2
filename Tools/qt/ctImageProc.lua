local ffi = require 'ffi'
--local libpng = require 'libpng'
local carray = require 'carray'
local cpng = require 'cpng'
--rgb = require 'rgbselect'

defaultW = 640
defaultH = 480
defaultThreshold = 14
img = carray.byte(defaultW * defaultH * 3)

loadImageCPNG = function(filename)
  cpng.load(filename, img:pointer())

  local qimage = QImage(img:pointer(), defaultW, defaultH, 
                  defaultW * 3, QImage.Format.Format_RGB888)

  window.widget.pimage:convertFromImage(qimage, Qt.AutoColor)
--  if window.widget.pimage:height() ~= defaultH or 
--    window.widget.pimage:width() ~= defaultW then
--    print('scale image')
--    window.widget.pimage = window.widget.pimage:scaled(defaultW, defaultH, 
--                              Qt.KeepAspectRatio, Qt.FastTransformation)
--  end


end

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

loadIndexImg = function(filename)
  local imgload = ffi.new('uint8_t[?]', 640 * 480)
  for i = 0, 640 * 480 - 1 do
    imgload[i] = math.random(4) * 2
  end
  local indexImg = QImage(imgload, 640, 480, 640, QImage.Format.Format_Indexed8)
  local rgb = _G['QVector<QRgb>'].new(4)
  rgb[1] = QColor(0,255,0):rgb()
  rgb[2] = QColor(0,0,255):rgb()
  rgb[3] = QColor(255,0,0):rgb()
  rgb[4] = QColor(0,255,255):rgb()
  indexImg:setColorTable(rgb)
  window.widget.pimage:convertFromImage(indexImg, Qt.AutoColor)
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
--  print(data[0], data[1], data[2], ptx, pty, threshold)
--  dd = rgb.select(data, w, h, ptx, pty, threshold)
--  df = ffi.cast('uint8_t*', dd)
--  count = 0
--  print('dddddd', count)
end

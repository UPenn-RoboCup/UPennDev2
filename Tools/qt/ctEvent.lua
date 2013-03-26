require 'unix'

fileList = {}
currentFile = '' 
currentFileIdx = 0

initDraw = function(self, state)
  local fileDialog = QFileDialog()
  local fileName = fileDialog:getOpenFileName( "Open File", "", "Log File (*.png *.jpg)")
--  print(fileName:toUtf8())
  local fullfilename = fileName:toUtf8()
  local path, filename = splitPath(fileName:toUtf8())
  local listFile = unix.readdir(path)
  local listFileCount = 0
  for k, v in pairs(listFile) do
    if v:find('%.png') or v:find('%.jpg') then
      fileList[#fileList+1] = file
      if fileList[#fileList] == fullfilename then
        currentFileIdx = fileList
        currentFile = fullfilename
      end
    end
  end

  loadImageJPEG(fullfilename)
--  loadImageCPNG(fullfilename)
--  loadIndexImg(fullfilename)
--  loadImage(fullfilename)
  window.widget.pixmapitem:setPixmap(window.widget.pimage)
  self:fitInView(0, 0, 640, 480, Qt.IgnoreAspectRatio)
  self:update(0,0,640,480)
end

updateDraw = function(self, state)
  local idxShift = 0
  if state == 1 then
    idxShift = -10
  elseif state == 2 then
    idxShift = -1
  elseif state == 3 then
    idxShift = 1
  elseif state == 4 then
    idxShift = 10
  end
  if #fileList > 0 then
    local afterShiftIdx = currentFileIdx + idxShift
    if afterShiftIdx > 0 and afterShiftIdx <= #fileList then
      currentFileIdx = afterShiftIdx
      currentFile = fileList[currentFileIdx]
    end
  end
--  loadImage(currentFile)
--  loadImageffi(currentFile)
--  loadIndexImg(currentFile)
  loadImageCPNG(currentFile)
  window.widget.pixmapitem:setPixmap(window.widget.pimage)
  self:update(0,0,640,480)
end

updateBBackward = function(self, state)
  print('BBackward')
  updateDraw(self, 1)
end

updateBackward = function(self, state)
  print('Backward')
  updateDraw(self, 2)
end

updateForward = function(self, state)
  print('Forward')
  updateDraw(self, 3)
end

updateFForward = function(self, state)
  print('FForward')
  updateDraw(self, 4)
end

selectPixel = function(self, e)
  print('You clicked on',e:button(), e:pos():x(), e:pos():y())
  local imageW = window.widget.pimage:size():width()
  local imageH = window.widget.pimage:size():height()
  --print(imageW, imageH)
  print("Running imageproc?",window.widget.pimage)
  if window.widget.pimage ~= nil then
    local threshold = window.widget.thresholdSlider:value()
    rgbselect( e:pos():x(), e:pos():y(), threshold );
    window.widget.pixmapitem:setPixmap(window.widget.pimage)
    self:update(0,0,640,480)
  end
end

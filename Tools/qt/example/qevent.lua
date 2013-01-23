local pwd = os.getenv('PWD')
package.cpath = pwd..'/lib/?.so;'..package.cpath

require 'qtcore'
require 'qtgui'

app = QApplication(1 + select('#', ...), {arg[0], ...})
app.__gc = app.delete -- take ownership of object


local onCheck = function (self, state)
  if (state == Qt.CheckState.Checked) then
    print("checked")
    QObject.connect(self.click, '2clicked()', self, '1onClick()')
  else
    print("not checked")
    QObject.disconnect(self.click, '2clicked()', self, '1onClick()')
  end
end

local KeyEvent = function(...)
  local this = QWidget(...)

  this.click = QPushButton("Click", this)
  this.click:setGeometry(50, 40, 75, 30)
  
  this.cb = QCheckBox("Connect", this)
  this.cb:setCheckState("Checked")
  this.cb:move(150,40)

  this:__addmethod("onClick()", function(self, state) print("Button Clicked") end)
  this:__addmethod("onCheck(int)", onCheck)

  QObject.connect(this.click, '2clicked()', this, '1onClick()')
  QObject.connect(this.cb, '2stateChanged(int)', this, '1onCheck(int)')

  this.qtime = QTime.currentTime()
  this.label = QLabel("", this)
  this.label:move(50, 50)

  this:startTimer(1000)

  return this
end


function window:timerEvent(e)
  local qtime = QTime.currentTime()
--  print(qtime:toNumber())
  self.label:setText(qtime:toString())
  print("update time")
end

function window:keyPressEvent(e)
  print("I'm triggered")
  if (e:key() == Qt.Key.Key_Escape) then
    app:quit()
  end
end

function window:moveEvent(e)
  local x = e:pos():x()
  local y = e:pos():y()
  local text = string.format("%d, %d", x, y);
  self:setWindowTitle(text)
end

window = KeyEvent()

window:setWindowTitle("UPennalizers")

window:show()

app.exec()


local pwd = os.getenv('PWD')
package.cpath = pwd..'/../lib/qt/?.so;'..package.cpath

require 'qtcore'
require 'qtgui'
require 'qtopengl'

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
  
  this.button = QPushButton("Quit", this)
  -- [horizontal, vertical, width, height]
  this.button:setGeometry(0, 0, 75, 30);
  this.button:connect("2clicked()", app, "1quit()")
  
  this.gl1 = QGLWidget(this, ...)
  this.gl2 = QGLWidget(this, ...)
  glBegin(GL_PLOYGON)

  this.frame1 = QFrame(this, ...)
  this.frame1:setFrameStyle(1)
  this.frame1:setCursor('SizeAllCursor')
  --
  this.frame2 = QFrame(this, ...)
  this.frame2:setFrameStyle(1)
  this.frame2:setCursor('WaitCursor')
  --
  this.frame3 = QFrame(this, ...)
  this.frame3:setFrameStyle(1)
  this.frame3:setCursor('PointingHandCursor')

  this.label = QLabel(this, ...)
  this.label:setGeometry(190, 80, 20, 30)
  this.label:setText(QString.number(40))
  this.label:__addmethod("OnPlus()", onplus)
  this.plusbutton = QPushButton("+", this)
  this.plusbutton:connect('2clicked()', this.label, '1OnPlus()');


  this.layout = QGridLayout.new()
  this.layout:addWidget(this.button, 0, 0)
  this.layout:addWidget(this.gl1, 0, 1)
  this.layout:addWidget(this.gl2, 1, 0)
  this.layout:addWidget(this.frame3, 0, 2)
  this.layout:addWidget(this.label, 1, 1)
  this.layout:addWidget(this.plusbutton, 1, 2)
  this:setLayout(this.layout)
  
  return this
end

window = new_window()
window.label:setText(QString.number(50))
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


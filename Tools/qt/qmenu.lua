local pwd = os.getenv('PWD')
package.cpath = pwd..'/lib/?.so;'..package.cpath

require 'qtcore'
require 'qtgui'

app = QApplication(1 + select('#', ...), {arg[0], ...})
app.__gc = app.delete -- take ownership of object

local simple_menu = function(...)
 
  local this = QWidget.new(...)
  
  local quit = QAction("Quit", this)
  local file = QMenu(this, ...)
  local menuBar = QMenuBar(this, ...)
  file = QMenuBar.addMenu(menuBar, "File")
  QMenu.addAction(file, quit)
  QObject.connect(quit, '2triggered()', app, '1quit()')
 
  return this
end

local toggleStatusBar = function(self)
  if (QAction.isChecked(self)) then
    print("Checked")
    QWidget.show(statusBar)
  else
    print("not Checked")
    QWidget.hide(statusBar)
  end
end

local another_menu = function(...)
  local this = QMainWindow.new(...)

  local pnewpix = QIcon('favicon.ico')
  local openpix = QIcon('favicon.ico')
  local quitpix = QIcon('favicon.ico')
 
  local newa = QAction(pnewpix, "New", this)
  local open = QAction(openpix, "Open", this)
  local quit = QAction(quitpix, "Quit", this)
  QAction.setShortcut(quit, QKeySequence(Qt.Modifier.CTRL +Qt.Key.Key_P))

  local viewist = QAction("View statusbar", this)
  viewist:__addmethod("ToggleStatusBar()", toggleStatusBar)
  QAction.setCheckable(viewist, true)
  QAction.setChecked(viewist, false)

  statusBar = this.statusBar(this)
  QWidget.hide(statusBar)

  viewist:connect("2triggered()", viewist, '1ToggleStatusBar()')

  local menuBar = QMenuBar(this, ...)
  local file = QMenu(this, ...)
  file = QMenuBar.addMenu(menuBar, "File")
  QMenu.addAction(file, newa)
  QMenu.addAction(file, open)
  QMenu.addSeparator(file)
  QMenu.addAction(file, quit)
  QMenu.addAction(file, viewist)
  local edit = QMenu(this, ...)
  edit = QMenuBar.addMenu(menuBar, "Edit")

  QObject.connect(quit, '2triggered()', app, '1quit()')

  return this
end

local skeleton = function(...)
  this = QMainWindow(...)

  local pnewpix = QIcon('favicon.ico')
  local openpix = QIcon('favicon.ico')
  local quitpix = QIcon('favicon.ico')
 
  local quit = QAction(quitpix, "Quit", this)
  local open = QAction(openpix, "Open", this)

  local menuBar = this.menuBar(this)
  local file = QMenu(this, ...)
  file = QMenuBar.addMenu(menuBar, "File")
  QMenu.addAction(file, quit)

  QObject.connect(quit, '2triggered()', app, '1quit()')

  local toolbar = this.addToolBar(this, "Main toolbar")
  QToolBar.addAction(toolbar, pnewpix, "New File")
  QToolBar.addAction(toolbar, openpix, "Open File")
  QToolBar.addSeparator(toolbar)
  
  local edit = QTextEdit(this, ...)
  this.setCentralWidget(this, edit)

  local statusbar = this.statusBar(this)
  statusbar.showMessage(statusbar, "Ready")

  return this
end

window = skeleton()
--window = simple_menu()
--window = another_menu()
-- Get Screen Size
desktop = QApplication.desktop()
screenWidth = desktop:width()
screenHeight = desktop:height()
width = 800
height = 600
 
x = (screenWidth - width) / 2
y = (screenHeight - height) / 2

window:resize(width, height)
window:move(x, y)
window:setWindowTitle("UPennalizers")
window:setToolTip("QWidget")
window:setWindowIcon(QIcon("favicon.ico"));

window:show()

app.exec()


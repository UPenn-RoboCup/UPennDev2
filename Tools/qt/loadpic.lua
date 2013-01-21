local pwd = os.getenv('PWD')
package.cpath = pwd..'/lib/?.so;'..package.cpath

require 'qtcore'
require 'qtgui'
require 'tch'

app = QApplication(1 + select('#', ...), {arg[0], ...})

scene = QGraphicsScene();
piximage = QPixmap('Image-1-10.png');
image = QGraphicsPixmapItem(piximage);
scene:addItem(image);

view = QGraphicsView(scene);

view:show()

---- the conversion from Lua string to QString is automatic
--hello = QPushButton.new("Hello World!")
---- but not the other way round
--print(hello:text():toUtf8())
--
--file = QFile.new("Image-1-10.png");
--
--hello:resize(100, 30)
--hello:show()

app.exec()



local pwd = os.getenv('PWD')
package.cpath = pwd..'/lib/?.so;'..package.cpath

require 'qtcore'
require 'qtgui'

app = QApplication(1 + select('#', ...), {arg[0], ...})
app.__gc = app.delete -- take ownership of object

local simple_layout = function(...)
  local this = QWidget.new(...)
  local edit = QTextEdit(this)
  edit:setGeometry(5, 5, 200, 150)

  return this
end

local VerticalBox = function(...)
  local this = QWidget.new(...)
  local vbox = QVBoxLayout.new()
  vbox:setSpacing(1)

  local setting = QPushButton("Setting", this)
  setting:setSizePolicy("Expanding", "Expanding")
  local accounts = QPushButton("Accounts", this)
  accounts:setSizePolicy("Expanding", "Expanding")
  local loans = QPushButton("Loans", this)
  loans:setSizePolicy("Expanding", "Expanding")
  local cash = QPushButton("Cash", this)
  cash:setSizePolicy("Expanding", "Expanding")
  local debts = QPushButton("Debts", this)
  debts:setSizePolicy("Expanding", "Expanding")

  vbox:addWidget(setting)
  vbox:addWidget(accounts)
  vbox:addWidget(loans)
  vbox:addWidget(cash)
  vbox:addWidget(debts)

  this:setLayout(vbox)

  return this
end

local Button = function(...)
  local this = QWidget.new(...)
  
  local ok = QPushButton("OK", this)
  local apply = QPushButton("Apply", this)

  local vbox = QVBoxLayout.new(this)
  local hbox = QHBoxLayout.new(this)

  hbox:addWidget(ok, 0, Qt.AlignRight)
  hbox:addWidget(apply, 1, Qt.AlignRight)

  vbox:addStretch(1)
  vbox:addLayout(hbox)

  return this
end

local Layout = function(...)
  local this = QWidget(...)

  local vbox = QVBoxLayout()
  local hbox = QHBoxLayout(...)

  local lw = QListWidget(this);
  lw:addItem("The Omen")
  lw:addItem("The Exorcist")
  lw:addItem("Nodes on a scandal")
  lw:addItem("Fargo")
  lw:addItem("Capote")

  local add = QPushButton("Add", this)
  local rename = QPushButton("Rename", this)
  local remove = QPushButton("Remove", this)
  local removeall = QPushButton("Remove All", this)

  vbox:setSpacing(3)
  vbox:addStretch(1)
  vbox:addWidget(add)
  vbox:addWidget(rename)
  vbox:addWidget(remove)
  vbox:addWidget(removeall);
  vbox:addStretch(1)

  hbox:addWidget(lw)
  hbox:addSpacing(10)
  hbox:addLayout(vbox)

  this:setLayout(hbox)

  return this
end

local Calculator = function(...)
  local this = QWidget(...)
   
  local grid = QGridLayout.new(...)
  grid:setSpacing(2)

  local value = {"7", "8", "9", "/", "4" ,"5" ,"6", "*", "1", "2",
        "3", "-", "0", ".", "=" ,"+"}
  local pos = 1
  for i = 0 , 3 do
    for j = 0 , 3 do
      local btn = QPushButton(value[pos], this);
      btn:setFixedSize(40,40)
      grid:addWidget(btn, i, j)
      pos = pos + 1;
    end
  end

  this:setLayout(grid)
  return this
end

local Karenina = function(...)
  local this = QWidget(...)
  local grid = QGridLayout()
  grid:setSpacing(20)
  
  local title = QLabel("Title", this)
  grid:addWidget(title, 0, 0, 1, 1)
  local author = QLabel("Author", this)
  grid:addWidget(author, 1, 0, 1, 1)
  local review = QLabel("Review", this)
  grid:addWidget(review, 2, 0, 1, 1)
  local edit1 = QLineEdit(this)
  grid:addWidget(edit1, 0, 1, 1, 1)
  local edit2 = QLineEdit(this)
  grid:addWidget(edit2, 1, 1, 1, 1)
  local te = QTextEdit(this)
  grid:addWidget(te, 2, 1, 6, 1)


  this:setLayout(grid)
  
  return this
end

--window = simple_layout()
--window = VerticalBox()
--window = Button()
--window = Layout()
window = Calculator()
window = Karenina()

window:setWindowTitle("UPennalizers")

window:show()

app.exec()


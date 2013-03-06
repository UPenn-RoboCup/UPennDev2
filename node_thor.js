var zmq = require('zmq'), 
	sock = zmq.socket('sub'),
	app = require('http').createServer(handler),
  io = require('socket.io').listen(app),
  fs = require('fs'),
	mp = require('msgpack');

app.listen(8080);

sock.connect('tcp://localhost:5555');
sock.subscribe('');
console.log('Worker connected to test');

// Global Variables
var test_obj = null;

// Listen
sock.on('message', function(msg){
	test_obj = mp.unpack(msg)
  console.log('work: %s', test_obj.toString());
});

function handler (req, res) {
  fs.readFile(__dirname + '/index.html',
  function (err, data) {
    if (err) {
      res.writeHead(500);
      return res.end('Error loading index.html');
    }
    res.writeHead(200);
    res.end(data);
  });
}

io.sockets.on('connection', function (socket) {
  socket.emit('news', test_obj);
  socket.on('my other event', function (data) {
    console.log(data);
  });
});
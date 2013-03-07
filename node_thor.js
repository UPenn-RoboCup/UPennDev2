var app = require('http').createServer(handler);
// Use BinaryJS as well
var BinaryServer = require('binaryjs').BinaryServer;
var bserver = BinaryServer({server: app});
console.log( 'bserver: %s', bserver.toString() );

var fs = require('fs');

var mp = require('msgpack');
var zmq = require('zmq');
var sock = zmq.socket('sub');
var sock_lidar = zmq.socket('sub');


app.listen(8080);
//sock.connect('tcp://localhost:5555');
sock.connect('ipc:///tmp/imu');
sock.subscribe('');
console.log('Worker connected to imu');
sock_lidar.connect('ipc:///tmp/lidar');
sock_lidar.subscribe('');
console.log('Worker connected to lidar');

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

/*
// Set up websockets
var io = require('socket.io').listen(app);
io.sockets.on('connection', function (socket) {
  // Send an initial packet
  //socket.emit('news', 'hour');
  
  // Get data from the browser
  //socket.on('my other event', function (data) {
  //  console.log(data);
  //});
  // Listen to zmq
  sock.on('message', function(msg){
    var imu = mp.unpack(msg)
    console.log('imu: %s', imu.toString());
    socket.emit('imu', imu);
  });
  // Listen to zmq lidar
  sock_lidar.on('message', function(msg){
//    console.log('lidar: %s', msg.toString());
//    socket.emit('lidar', msg);
  });
});
*/
bserver.on('connection', function(client){
  sock_lidar.on('message', function(msg){
    // Send the lidar data
    client.send(msg)
    //console.log('sending msg')
  });
  console.log('hi binary!')
});

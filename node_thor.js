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

var nconnections = 0;
// Size of float times number of lidar returns
//var lidar_msg = new Buffer(1081*4);
bserver.on('connection', function(client){
  var lidar_stream = client.createStream('lidar');
  sock_lidar.on('message', function(msg){
    //console.log('Received '+msg.length+' lidar bytes. '+msg.readFloatLE(0))
    lidar_stream.write(msg);
  });
});


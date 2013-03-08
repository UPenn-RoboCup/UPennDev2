// Require the appropriate modules
var BinaryServer = require('binaryjs').BinaryServer;
var fs = require('fs');
var mp = require('msgpack');
var zmq = require('zmq');

// Set up a Websocket server on 9000
var bserver = BinaryServer({port: 9000});
console.log( 'bserver: %s', bserver.toString() );

// Listen to IPC sensor messages
var sock_imu = zmq.socket('sub');
var sock_lidar = zmq.socket('sub');
sock_imu.connect('ipc:///tmp/imu');
sock_imu.subscribe('');
console.log('Worker connected to imu');
sock_lidar.connect('ipc:///tmp/lidar');
sock_lidar.subscribe('');
console.log('Worker connected to lidar');

// Listen to binary websockets
bserver.on('connection', function(client){
  var lidar_stream = client.createStream('lidar');
  sock_lidar.on('message', function(msg){
    lidar_stream.write(msg);
  });
});


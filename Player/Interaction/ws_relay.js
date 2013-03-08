// Require the appropriate modules
var BinaryServer = require('binaryjs').BinaryServer;
var fs = require('fs');
var mp = require('msgpack');
var zmq = require('zmq');

// Set up a Websocket server on 9000
var bserver = BinaryServer({port: 9000});

// Listen to IPC sensor messages
var sock_imu = zmq.socket('sub');
sock_imu.connect('ipc:///tmp/arduimu');
sock_imu.subscribe('');
console.log('Worker connected to imu');
var sock_lidar = zmq.socket('sub');
sock_lidar.connect('ipc:///tmp/lidar');
sock_lidar.subscribe('');
console.log('Worker connected to lidar');
var sock_flir = zmq.socket('sub');
sock_flir.connect('ipc:///tmp/flir');
sock_flir.subscribe('');
console.log('Worker connected to flir');

// Listen to binary websockets
var lidar_streams = [];
var imu_streams = [];
var flir_streams = [];
var counter = 0;
bserver.on('connection', function(client){
  console.log('A client is Connnected!');
  var lidar_stream = client.createStream('lidar');
  lidar_stream.readable = false;
  lidar_streams.push(lidar_stream);
  var imu_stream = client.createStream('imu');
  imu_stream.readable = false;
  imu_streams.push(imu_stream);
  var flir_stream = client.createStream('flir');
  flir_stream.readable = false;
  flir_streams.push(flir_stream);
  //console.log(lidar_streams)
});

// Listen to IPC messages
var last_lidar_cntr = counter;
sock_lidar.on('message', function(msg){
  if( counter>last_lidar_cntr ) {
    for(var s=0;s<lidar_streams.length;s++) {
      var ret = lidar_streams[s].write( msg );
      if(ret==false){
        console.log(counter+' lidar:' +msg.readFloatLE(0));
      }
    }
    last_lidar_cntr = counter;
  }
});

var last_imu_cntr = counter;
sock_imu.on('message', function(msg){
  if( counter>last_imu_cntr ) {
    var send_msg = mp.unpack(msg);
    for(var s=0;s<imu_streams.length;s++) {
      var ret = imu_streams[s].write( send_msg );
      if(ret==false){
        console.log(counter+' imu:', send_msg);
      }
    }
    last_imu_cntr = counter;
  }
});

var last_flir_cntr = counter;
sock_flir.on('message', function(msg){
  var send_msg = mp.unpack( msg )
  if( counter>last_flir_cntr ) {
    for(var s=0;s<flir_streams.length;s++) {
      var ret = flir_streams[s].write( send_msg );
      if(ret==false){
        console.log('flir: ' +send_msg.t );
      }
    }
    last_flir_cntr = counter;
  }
});

// Send data to clients at a set interval
// For now, this is 15fps
var fps = 1;
setInterval(  function(){
  counter++;
}, 1000/fps);


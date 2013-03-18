// Require the appropriate modules
var mp = require('msgpack');
var zmq = require('zmq');
var WebSocketServer = require('ws').Server;

// Globals
var wskts = []
var counter = 0;

// Send data to clients at a set interval
// For now, this is 15fps
var fps = 15;
setInterval(  function(){
  counter++;
}, 1000/fps);

// Listen to IPC sensor messages
var sock_lidar = zmq.socket('sub');
sock_lidar.connect('ipc:///tmp/lidar');
sock_lidar.subscribe('');
console.log('IPC: lidar');
// Process lidar
var last_lidar_cntr = counter;
sock_lidar.on('message', function(msg){
//  console.log('got message!')
  if( counter>last_lidar_cntr ) {
    for(var s=0;s<wskts.length;s++) {
      wskts[s].send(msg,{binary:true},function(){
        //console.log(counter+' lidar:' +msg.readFloatLE(0));
      });
    }
    last_lidar_cntr = counter;
  }
});

// Set up a Websocket server on 9000
var wss = new WebSocketServer({port: 9000});
// Listen to binary websockets
wss.on('connection', function(ws) {
  console.log('A client is Connnected!');
  ws.on('message', function(message) {
    console.log('received: %s', message);
  });
  wskts.push(ws)

  ws.send('something');
});



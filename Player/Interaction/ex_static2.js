var express = require('express');
var app = express();
var http = require('http')
app.use(express.static(__dirname + '/public'));

var server = http.createServer(app);
server.listen(8080);

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
var zmq_img = zmq.socket('sub');
zmq_img.connect('ipc:///tmp/img');
zmq_img.subscribe('');
console.log('IPC | Connected to img');
// Process lidar
var last_img_cntr = counter;
zmq_img.on('message', function(msg){
  //console.log('IPC | Got img message!')
  if( counter>last_img_cntr ) {
    for(var s=0;s<wskts.length;s++) {
      wskts[s].send(msg,{binary:true},function(){
      });
    }
    last_img_cntr = counter;
  }
});

// Set up a Websocket server on 9001
var wss = new WebSocketServer({server: server});
// Listen to binary websockets
wss.on('connection', function(ws) {
  console.log('A client is Connnected!');
  // Client message?
  ws.on('message', function(message) {
    console.log('received: %s', message);
  });
  wskts.push(ws)
//  ws.send('something');
});



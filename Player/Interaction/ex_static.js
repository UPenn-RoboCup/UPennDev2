// Require the appropriate modules
var mp = require('msgpack');
var zmq = require('zmq');
var WebSocketServer = require('ws').Server;
var express = require('express');
var app = express();
var http = require('http')
app.use(express.static(__dirname + '/public'));

// HTTP static server
var server = http.createServer(app);
server.listen(8080);

// Globals
var wskts   = [];
var wskts9  = [];
var counter = 0;

// Send data to clients at a set interval
// For now, this is 15fps
var fps = 5;
setInterval(  function(){
  counter++;
}, 1000/fps);

// Listen to IPC sensor messages
var zmq_prime = zmq.socket('sub');
zmq_prime.connect('ipc:///tmp/prime');
zmq_prime.subscribe('');
console.log('IPC | Connected to prime');
// Process lidar
var last_prime_cntr = counter;
zmq_prime.on('message', function(msg){
  //console.log('IPC | Got prime message!')
  if( counter>last_prime_cntr ) {
    for(var s=0;s<wskts.length;s++) {
      wskts[s].send(msg,{binary:true},function(){
      });
    }
    last_prime_cntr = counter;
  }
});

// Set up a Websocket server on the HTTP server
var wss = new WebSocketServer({server: server});
// Listen to binary websockets
wss.on('connection', function(ws) {
  console.log('A client is Connnected!');
  // Client message?
  ws.on('message', function(message) {
    console.log('received: %s', message);
  });
  wskts.push(ws)
});

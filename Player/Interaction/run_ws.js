// Check for channels to listen on
if( process.argv.length <= 2 ) {
  console.log('No channels listening!');
  return;
}
var channels = [];
// Publish someting
process.argv.forEach(function (val, index, array) {
  if( index>1 ) {
//    console.log(index + ': ' + val);
    console.log('Listening to ' + val);
    channels.push( val );
  }
});

// Require the appropriate modules
var mp = require('msgpack');
var zmq = require('zmq');
var WebSocketServer = require('ws').Server;

// Globals
var wskts = []
var counter = 0;

// Send data to clients at a set interval
// For now, this is 15fps
var fps = 5;
setInterval(  function(){
  counter++;
}, 1000/fps);

// Listen to IPC sensor messages
var zmq_img = zmq.socket('sub');
zmq_img.connect('ipc:///tmp/'+channels[0]);
zmq_img.subscribe('');
console.log('ZeroMQ IPC | Connected to '+channels[0]);
// Process lidar
var last_cntr = counter;
zmq_img.on('message', function(msg){
  // console.log('ZeroMQ IPC | Got'+channels[0]+' message!')
  if( counter>last_img_cntr ) {
    for(var s=0;s<wskts.length;s++) {
      wskts[s].send(msg,{binary:true},function(){
      });
    }
    last_cntr = counter;
  }
});

// Set up a Websocket server on 9001
var wss = new WebSocketServer({port: 9000});
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


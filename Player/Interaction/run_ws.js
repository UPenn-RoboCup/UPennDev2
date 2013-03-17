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
var colorData;
var depthData;

// Send data to clients at a set interval
// For now, this is 15fps
var fps = 15;
setInterval(  function(){
  for(var s=0;s<wskts.length;s++) {
    if( counter%2==0 ){
      //console.log("sending color");
      wskts[s].send(colorData,{binary:true});
    } else {
      //console.log("sending depth");
      wskts[s].send(depthData,{binary:true});
    }
    //wskts[s].send( Buffer.concat( [type,raw]),{binary:true});
  }
  counter++;
}, 1000/fps);

// Listen to IPC sensor messages
var zmq_skt = zmq.socket('sub');
zmq_skt.connect('ipc:///tmp/'+channels[0]);
zmq_skt.subscribe('');
console.log('ZeroMQ IPC | Connected to '+channels[0]);
zmq_skt.on('message', function(type,raw){
  if(type=='c'){
    //console.log('Color');
    if( raw.length>15600 ){
      colorData = raw;
    }
  } else {
    //console.log('Depth');
    if(raw.length<=15600){
      depthData = raw;
    }
  }
});

// Set up a Websocket server on 9001
var wss = new WebSocketServer({port: 9002});
// Listen to binary websockets
wss.on('connection', function(ws) {
  console.log('A client is Connnected!');
  // Client message?
  ws.on('message', function(message) {
    console.log('received: %s', message);
  });
  wskts.push(ws)
});


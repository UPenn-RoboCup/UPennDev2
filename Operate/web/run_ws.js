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
var fps = 1;
var c_id = Buffer([13,13]);
var d_id = Buffer([15,12]);
var s = 0;
setInterval(  function(){
  for(s=0;s<wskts.length;s++) {
    if( wskts[s].readyState==1 ){ //1 is OPEN
      if( colorData!==undefined ){
        wskts[s].send( Buffer.concat([colorData,c_id]), {binary:true} );
      }
      if( depthData!==undefined ){    
        wskts[s].send( Buffer.concat([depthData,d_id]) ,{binary:true} );
      }
    }
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
    //console.log('Color: '+raw.length+' bytes.');
    colorData = raw;
  } else {
    //console.log('Depth: '+raw.length+" bytes.");
    depthData = raw;
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


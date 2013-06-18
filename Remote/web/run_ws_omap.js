var channels = ['omap'];
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
var omapData;

// Send data to clients at a set interval
// For now, this is 15fps
var fps = 1;
var omap_id = Buffer([1,14]);
var s = 0;
setInterval(  function(){
  for(s=0;s<wskts.length;s++) {
    if( wskts[s].readyState==1 ){ //1 is OPEN
      if( omapData!==undefined ){    
        wskts[s].send( Buffer.concat([omapData,d_id]) ,{binary:true} );
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
//  console.log(type.length)
  omapData = type;
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


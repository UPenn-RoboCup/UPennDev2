// Check for channels to listen on
if( process.argv.length <= 2 ) {
	console.log('No channels listening!');
	return;
}
var channels = [];
// Publish someting
process.argv.forEach(function (val, index, array) {
	if( index>1 ) {
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

// Listen to IPC sensor messages
var zmq_skt = zmq.socket('sub');
zmq_skt.connect('ipc:///tmp/'+channels[0]);
zmq_skt.subscribe('');
console.log('ZeroMQ IPC | Connected to '+channels[0]);
zmq_skt.on('message', function(raw){
	console.log(raw.length+' bytes.');
	for( var s=0;s<wskts.length;s++) {
		if( wskts[s].readyState==1 ){ //1 is OPEN
			wskts[s].send( raw, {binary:true} );
			counter++;
			console.log('Sent to clients '+counter+' times.');
		}
	} //console.log('('+raw.readFloatLE(200*4)+','+raw.readFloatLE(201*4)+','+raw.readFloatLE(202*4)+')');
});

// Set up a Websocket server on 9876
var wss = new WebSocketServer({port: 9876});
// Listen to binary websockets
wss.on('connection', function(ws) {
	console.log('A client is Connnected!');
	// Client message?
	ws.on('message', function(message) {
		console.log('received: %s', message);
	});
	wskts.push(ws)
});


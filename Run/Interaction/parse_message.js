var URL = window.URL || window.webkitURL; // Compatibility

var process_message = function (event) {
	var t0_raw = Date.now()
	var t0 = t0_raw/1000000;
	var vertices = new Float32Array(event.data);
	/* Debugging information */
	$("#debug").append(
		"<p>Received "+event.data.byteLength+" at "+t0+" raw time: "+t0_raw+'</p>'
	);
	$("#debug").append(
		"<p>Converted to "+vertices.length+" element float array</p>"
	);
	$("#debug").append(
		"<p>Elements:"+vertices[0]+","+vertices[1]+","+vertices[2]+")</b>"
	);
	/* Put into THREE.js */
	update_faces(vertices);
	
};

// Event listeners
var ws_listen = function(){
	// Connect to the websocket server
	var host = window.document.location.host.replace(/:.*/, '');
	if( host.length==0 ){
		host = "localhost";
	}
	var ws_k = new WebSocket('ws://' + host + ':9876');
	ws_k.binaryType = "arraybuffer"; //"blob"
	ws_k.onmessage = process_message;
	
	$("#debug").append('Debugging console');
	
};

document.addEventListener( "DOMContentLoaded", ws_listen, false );
document.addEventListener( "DOMContentLoaded", init_faces, false );
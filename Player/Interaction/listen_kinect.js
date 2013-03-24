// Websocket variables
var ws_k;
var mydata;
var myid;
// Color/depth canvas elements
var cCanvas;
var c_ctx;
var dCanvas;
var d_ctx;
// Image elements
var c_imgdata;
var d_imgdata;
var c_img = new Image;
c_img.alt="color";
var d_img = new Image;
d_img.alt="depth";
var URL = window.URL || window.webkitURL;
// Update the cloud colors
c_img.onload = function(){
  c_ctx.drawImage( c_img, 0, 0 );
  URL.revokeObjectURL( this.src );
  this.src = '';
  c_imgdata = c_ctx.getImageData(0, 0, 320, 240);
  if(use_cloud == 1){
    update_cloud_image( new Uint8Array(c_imgdata.data.buffer) );
  }
  if(use_mesh == 1){
    update_mesh_image( new Uint8Array(c_imgdata.data.buffer) );
  }
}; //c onload
// Update the cloud depths    
d_img.onload = function(e){
  d_ctx.drawImage( d_img, 0, 0 );
  URL.revokeObjectURL( this.src );
  this.src = '';
  d_imgdata = d_ctx.getImageData(0, 0, 320, 240);
  if(use_cloud == 1){
    update_cloud_depth( new Uint8ClampedArray(d_imgdata.data.buffer) );
  } else if(use_mesh == 1){
    update_mesh_depth( new Uint8ClampedArray(d_imgdata.data.buffer) );
  }
}; //d onload

// Process Websocket messages
var process_kmsg = function (event) {
  mymeta = new Uint8Array(event.data, event.data.byteLength-2, 2 );
  // Act on color vs. depth data
  if( mymeta[0] == 13 ){
    c_img.src = URL.createObjectURL( new Blob( [event.data], { type: "image\/jpeg" } ) );
  } else {
    d_img.src = URL.createObjectURL( new Blob( [event.data], { type: "image\/jpeg" } ) );
  }
};

// After the elements are loaded
var init_kinect = function(){
  cCanvas = document.getElementById('kcolor');
  cCanvas.width = 320;
  cCanvas.height = 240;
  c_ctx = cCanvas.getContext('2d');
  dCanvas = document.getElementById('kdepth');
  dCanvas.width = 320;
  dCanvas.height = 240;
  d_ctx = dCanvas.getContext('2d');
  // Connect to the websocket server
  var host = window.document.location.host.replace(/:.*/, '');
  if( host.length==0 ){
    host = "localhost";
  }
  ws_k = new WebSocket('ws://' + host + ':9002');
  ws_k.binaryType = "arraybuffer"; //"blob"
  ws_k.onmessage = process_kmsg;
}
document.addEventListener( "DOMContentLoaded", init_kinect );

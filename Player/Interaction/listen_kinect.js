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
var myjpeg;
var mypng;
var jpeg_url_name;
var c_imgdata;
var d_imgdata;
var c_img = new Image;
var d_img = new Image;
// Update the cloud colors
c_img.onload = function(){
  c_ctx.drawImage( c_img, 0, 0 );
  c_imgdata = c_ctx.getImageData(0, 0, 320, 240);
  if(use_cloud == 1){
    update_cloud_image( new Uint8Array(c_imgdata.data.buffer) );
  }
  if(use_mesh == 1){
    update_mesh_image( new Uint8Array(c_imgdata.data.buffer) );
  }
  // Revoke the Object URL
  if(window.webkitURL!==undefined) {
    window.webkitURL.revokeObjectURL( this.src );
  } else {
    window.URL.revokeObjectURL( this.src );
  }
}; //c onload
// Update the cloud depths    
d_img.onload = function(e){
  d_ctx.drawImage( d_img, 0, 0 );
  d_imgdata = d_ctx.getImageData(0, 0, 320, 240);
  if(use_cloud == 1){
    update_cloud_depth( new Uint8ClampedArray(d_imgdata.data.buffer) );
  }
  if(use_mesh == 1){
    update_mesh_depth( new Uint8ClampedArray(d_imgdata.data.buffer) );
  }
  // Revoke the Object URL
  if(window.webkitURL!==undefined) {
    window.webkitURL.revokeObjectURL( this.src );
  } else {
    window.URL.revokeObjectURL( this.src );
  }
}; //d onload

// Process Websocket messages
var process_kmsg = function (event) {
  mymeta = new Uint8Array(event.data, event.data.byteLength-2, 2 );
  //var mydata = new Uint8Array(event.data, 2, event.data.byteLength-2 );
  // Act on color vs. depth data
  if( mymeta[0] == 13 ){
    myjpeg = new Blob( [event.data], { type: "image\/jpeg" } );
    //myjpeg = new Blob( [mydata], { type: "image\/jpeg" } );
    c_img.alt="color"
    // Load the image
    if(window.webkitURL!==undefined) {
      c_img.src = window.webkitURL.createObjectURL( myjpeg );
    } else {
      c_img.src = window.URL.createObjectURL( myjpeg );
    }
  } else {
    mypng = new Blob( [event.data], { type: "image\/jpeg" } );
    //mypng = new Blob( [mydata], { type: "image\/jpeg" } );
    d_img.alt="depth"
    // Load the image
    if(window.webkitURL!==undefined) {
      d_img.src = window.webkitURL.createObjectURL( mypng );
    } else {
      d_img.src = window.URL.createObjectURL( mypng );
    }
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
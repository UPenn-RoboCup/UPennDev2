var URL = window.URL || window.webkitURL; // Compatibility
// Event listeners
document.addEventListener( "DOMContentLoaded", function(){
  // Connect to the websocket server
  var host = window.document.location.host.replace(/:.*/, '');
  if( host.length==0 ){
    host = "localhost";
  }
  var ws_k = new WebSocket('ws://' + host + ':9002');
  ws_k.binaryType = "arraybuffer"; //"blob"
  ws_k.onmessage = function (event) {
    var mymeta = new Uint8Array(event.data, event.data.byteLength-2, 2 );
    // Act on color vs. depth data
    if( mymeta[0] == 13 ){
      var c_img = new Image();
      c_img.alt="color";
      c_img.src = URL.createObjectURL( new Blob( [event.data], { type: "image\/jpeg" } ) );
      // Update the cloud colors
      c_img.onload = function(e){
        var cCanvas = document.getElementById('kcolor');
        cCanvas.width = 320;
        cCanvas.height = 240;
        var c_ctx = cCanvas.getContext('2d')
        c_ctx.drawImage( this, 0, 0 );
        URL.revokeObjectURL( this.src );
        this.src = '';
        var c_imgdata = c_ctx.getImageData(0, 0, 320, 240);
        if(use_cloud == 1){
          update_cloud_image( c_imgdata );
        } else if(use_mesh == 1){
          update_mesh_image( c_imgdata );
        }
      }; //c onload
    } else {
      var d_img = new Image();
      d_img.alt="depth";
      d_img.src = URL.createObjectURL( new Blob( [event.data], { type: "image\/png" } ) );
      // Update the cloud depths
      d_img.onload = function(e){
        var dCanvas = document.getElementById('kdepth');
        dCanvas.width = 320;
        dCanvas.height = 240;
        var d_ctx = dCanvas.getContext('2d');
        d_ctx.drawImage( d_img, 0, 0 );
        URL.revokeObjectURL( this.src );
        this.src = '';
        var d_imgdata = d_ctx.getImageData(0, 0, 320, 240);
        if(use_cloud == 1){
          update_cloud_depth( d_imgdata );
        } else if(use_mesh == 1){
          update_mesh_depth( d_imgdata );
        }
      }; //d onload
    }
  };
});

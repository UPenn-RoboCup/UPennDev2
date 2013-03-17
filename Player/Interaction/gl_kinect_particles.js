if ( ! Detector.webgl ) Detector.addGetWebGLMessage();

var ctxwidth = 320;
var ctxheight = 240;
var kwidth = 320;
var kheight = 240;
var nparticles = kwidth*kheight;
var nchunks = Math.floor( nparticles / 2^16 );
console.log("nchunks: "+nchunks);

// Globals
var particleSystem;
var container;
var camera, controls, scene, renderer;
var positions, colors, geometry;
//https://github.com/OpenNI/OpenNI2/blob/master/Source/Core/OniStream.cpp#L362
var hlut = new Float32Array( kwidth  );
var vlut = new Float32Array( kheight );
var hFOV = 58;
var vFOV = 45;

for( var i=0;i<kwidth;i++ ){
  hlut[i] = Math.tan(hFOV/2)*2*(i/kwidth -.5); // float?
}
for( var j=0;j<kheight;j++ ){
  vlut[j] = Math.tan(vFOV/2)*2*(j/kheight-.5); // float math?
}
console.log("Done setting up the fov tables.")

init();
animate();

function init() {

  // Set up the camera
  camera = new THREE.PerspectiveCamera();
  camera.position.x = 0;//kwidth/2;
  camera.position.y = 0;// kheight/2;
  camera.position.z = 750;
  //camera.position.z = -500;
  camera.lookAt(0,0,0);

  // Set up the mouse controls
  controls = new THREE.TrackballControls( camera );
  controls.rotateSpeed = 1.0;
  controls.zoomSpeed = 1.2;
  controls.panSpeed = 0.8;
  controls.noZoom = false;
  controls.noPan = false;
  controls.staticMoving = true;
  controls.dynamicDampingFactor = 0.3;
  controls.keys = [ 65, 83, 68 ];
  controls.addEventListener( 'change', render );

  // Initialize the particles
  // TODO: chunk it!
  geometry = new THREE.BufferGeometry();
  geometry.attributes = {
    position: {
      itemSize: 3,
      array: new Float32Array( nparticles * 3 ),
      numItems: nparticles * 3
    },
    color: {
      itemSize: 3,
      array: new Float32Array( nparticles * 3 ),
      numItems: nparticles * 3
    }
  }
  geometry.dynamic = true;
  // Set the colors and positions
  positions = geometry.attributes.position.array;
  colors = geometry.attributes.color.array;
  var cnt = 0;
  for( var j=0; j<240; j++ ){
    for( var i=0; i<320; i++ ){
      positions[cnt] = i;
      positions[cnt+1] = j;
      positions[cnt+2] = 0;
      colors[cnt] = 0;
      colors[cnt+1] = 255;
      colors[cnt+2] = 0;
      cnt = cnt+3;
    }
  }
  geometry.verticesNeedUpdate = true;
  // TODO: is this necessary?
  geometry.computeBoundingSphere();

  var material = new THREE.ParticleBasicMaterial( { size: 4, vertexColors: true } );
  particleSystem = new THREE.ParticleSystem( geometry, material );

  // Set up the world
  scene = new THREE.Scene();
  scene.add( particleSystem );

  // Renderer
  //renderer = new THREE.WebGLRenderer( { antialias: false, clearColor: 0x333333, clearAlpha: 1, alpha: false } );
  renderer = new THREE.WebGLRenderer( { antialias: false, alpha: false } );
  renderer.setSize( ctxwidth, ctxheight );
  container = $('#kgl')[0];
  container.appendChild( renderer.domElement );

}

function animate() {
  requestAnimationFrame( animate );
  controls.update();
}

function render() {
  renderer.render( scene, camera );
}

function update_kinect_image( c_buffer ) {

  var d_idx = 0;
  var c_idx = 0;
  for(var j=0; j<kheight; j++ ){
    for (var i = 0; i<kwidth; i++ ){
      // TODO: div by 255 may be in int space, and not float space
      colors[ c_idx ]     = c_buffer[d_idx] / 255;
      colors[ c_idx + 1 ] = c_buffer[d_idx+1] / 255;
      colors[ c_idx + 2 ] = c_buffer[d_idx+2] / 255;
      c_idx+=3;
      d_idx+=4; //RGBA
    }
  }

  particleSystem.geometry.colorsNeedUpdate = true;
  animate();
  render();
}

function update_kinect_depth( d_buffer ) {
  var d_idx = 0;
  var p_idx = 0;
  var tmp = 0;
  // TODO: cache it good
  for(var j=0; j<kheight; j++ ){
    for (var i = 0; i<kwidth; i++ ){
      tmp = d_buffer[d_idx]+d_buffer[d_idx+1]+d_buffer[d_idx+2];
      tmp = tmp / 3;
      positions[ p_idx ]     = tmp*hlut[i];
      positions[ p_idx + 1 ] = tmp*vlut[j];
      positions[ p_idx + 2 ] = tmp;
      p_idx+=3;
      d_idx+=4;
    }
  }
  particleSystem.geometry.verticesNeedUpdate = true;

  animate();
  render();
}


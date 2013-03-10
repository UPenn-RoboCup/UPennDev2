if ( ! Detector.webgl ) Detector.addGetWebGLMessage();

var container, stats;

var camera, controls, scene, renderer;

var positions, colors;
//var particles = 1081; // # of ranges
var particles = 320*240; // # of ranges
var color = new THREE.Color();
var particleSystem
var cross;

var plane;
var init_texture;
var width = 640;
var height = 480;

init();
animate();

function init() {

  // Set up the camera
  camera = new THREE.PerspectiveCamera();
  camera.position.x = 160;
  camera.position.y = 120;
  camera.position.z = 1000;

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

  // Set up the world
  scene = new THREE.Scene();
  //scene.fog = new THREE.FogExp2( 0xcccccc, 0.002 );
  scene.fog = new THREE.Fog( 0x050505, 2000, 3500 );

  // Initialize the particles
  geometry = new THREE.BufferGeometry();
  geometry.attributes = {
    position: {
      itemSize: 3,
      array: new Float32Array( particles * 3 ),
      numItems: particles * 3
    },
    color: {
      itemSize: 3,
      array: new Float32Array( particles * 3 ),
      numItems: particles * 3
    }
  }
  geometry.dynamic = true;

  // Set the colors and positions
  positions = geometry.attributes.position.array;
  colors = geometry.attributes.color.array;

  color.setRGB( 0, 1, 0 );
  var cnt = 0;
  for( var j=0; j<240; j++ )
    for( var i=0;i<320;i++ ){
      positions[cnt] = i;
      positions[cnt+1] = j;
      positions[cnt+2] = 0;
      cnt = cnt+3;
    }

  for ( var i = 0; i < positions.length; i += 3 ) {
    // colors
    colors[ i ]     = color.r;
    colors[ i + 1 ] = color.g;
    colors[ i + 2 ] = color.b;
  }


  scene.add( plane );
  geometry.computeBoundingSphere();

  // Add the particle system to the scene
  var material = new THREE.ParticleBasicMaterial( { size: 1, vertexColors: true } );
  particleSystem = new THREE.ParticleSystem( geometry, material );
  scene.add( particleSystem );
  var meshMaterial = new THREE.MeshBasicMaterial( { wireframe: true, vertexColors: true, color: 0xff0000 } );
  //meshSystem = new THREE.Mesh( geometry, meshMaterial );
  //lineSystem = new THREE.Mesh( geometry );
  //scene.add( meshSystem );
  //scene.add( lineSystem );
  plane = new THREE.Mesh(
      new THREE.PlaneGeometry( 320, 190, 160, 95 ),
      meshMaterial
  );


  // renderer
  renderer = new THREE.WebGLRenderer( { antialias: false, clearColor: 0x333333, clearAlpha: 1, alpha: false } );
  renderer.setClearColor( scene.fog.color, 1 );
  renderer.setSize( 640,480 );

  container = document.getElementById( 'container' );
  container.appendChild( renderer.domElement );

  stats = new Stats();
  stats.domElement.style.position = 'absolute';
  stats.domElement.style.top = '0px';
  stats.domElement.style.zIndex = 100;
  container.appendChild( stats.domElement );


  window.addEventListener( 'resize', onWindowResize, false );

}

function onWindowResize() {
  /*
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize( window.innerWidth, window.innerHeight );
  controls.handleResize();
  */
  render();
}

function animate() {
  requestAnimationFrame( animate );
  controls.update();
}

function render() {
  renderer.render( scene, camera );
  stats.update();
}

function update_kinect( data_buffer ) {
  var vertices = plane.geometry.vertices;
  for (var i = 0; i<vertices.length; i++ ) {
    vertices[i].z = Math.floor( Math.random()*10 );
//    vertices[i].z = 10/(1+(i-vertices.length/2)^2);
  }
  plane.geometry.verticesNeedUpdate = true;
  var new_texture = new THREE.DataTexture( data_buffer, 320, 190);
  plane_mat.map.image.data = data_buffer;
  plane_mat.map.needsUpdate = true;
  //plane_mat.map = new_texture;
  //plane_mat.map.needsUpdate = true;
//  console.log( plane_mat );
  //console.log( data_buffer );
  animate();
  render();
}

function update_particles( d ) {
  for ( var i = 0; i < positions.length; i += 3 )
    positions[ i + 2 ] = d[i];
  //console.log( positions );
  particleSystem.geometry.verticesNeedUpdate = true;
  animate();
  render();
}

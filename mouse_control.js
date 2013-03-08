if ( ! Detector.webgl ) Detector.addGetWebGLMessage();

var container, stats;

var camera, controls, scene, renderer;

var positions, colors;
var particles = 1081; // # of ranges
var color = new THREE.Color();
var particleSystem
var cross;

init();
animate();

function init() {

  // Set up the camera
  camera = new THREE.PerspectiveCamera( 60, window.innerWidth / window.innerHeight, 1, 1000 );
  camera.position.z = 10;

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

  // Set up the lights
/*
  light = new THREE.DirectionalLight( 0xffffff );
  light.position.set( 1, 1, 1 );
  scene.add( light );
  light = new THREE.DirectionalLight( 0x002288 );
  light.position.set( -1, -1, -1 );
  scene.add( light );
  light = new THREE.AmbientLight( 0x222222 );
  scene.add( light );
*/

  // Initialize the particles
  var geometry = new THREE.BufferGeometry();
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
  geometry.computeBoundingSphere();

  // Add the particle system to the scene
  var material = new THREE.ParticleBasicMaterial( { size: .5, vertexColors: true } );
  particleSystem = new THREE.ParticleSystem( geometry, material );
  var meshMaterial = new THREE.MeshBasicMaterial( { wireframe: true, vertexColors: true, color: 0xff0000 } );
  meshSystem = new THREE.Mesh( geometry, meshMaterial );
  scene.add( particleSystem );
  scene.add( meshSystem );

  // renderer
  renderer = new THREE.WebGLRenderer( { antialias: false, clearColor: 0x333333, clearAlpha: 1, alpha: false } );
  renderer.setClearColor( scene.fog.color, 1 );
  renderer.setSize( window.innerWidth, window.innerHeight );

  container = document.getElementById( 'container' );
  container.appendChild( renderer.domElement );

  stats = new Stats();
  stats.domElement.style.position = 'absolute';
  stats.domElement.style.top = '0px';
  stats.domElement.style.zIndex = 100;
  container.appendChild( stats.domElement );


  window.addEventListener( 'resize', onWindowResize, false );
  console.log('set up...')

}

function onWindowResize() {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize( window.innerWidth, window.innerHeight );
  controls.handleResize();
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

function update_particles( ranges ) {
  
  //color.setRGB( Math.random(), Math.random(), Math.random() );
  color.setRGB( .5,.5,.5 );
  var range_id = 0;
  for ( var i = 0; i < positions.length; i += 3 ) {

    // positions
    var x = Math.cos( 3.14/180*(range_id/1081*270-135) ) * ranges[range_id];
    var y = Math.sin( 3.14/180*(range_id/1081*270-135) ) * ranges[range_id];
    range_id = range_id+1;
    var z = 0;
    positions[ i ]     = x;
    positions[ i + 1 ] = y;
    positions[ i + 2 ] = z;

    // colors
    colors[ i ]     = color.r;
    colors[ i + 1 ] = color.g;
    colors[ i + 2 ] = color.b;
  }

    particleSystem.geometry.verticesNeedUpdate = true;
    particleSystem.geometry.colorsNeedUpdate = true;
    animate();
    render();
}

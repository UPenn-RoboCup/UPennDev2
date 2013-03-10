if ( ! Detector.webgl ) Detector.addGetWebGLMessage();

var container, stats;

var camera, controls, scene, renderer;

var positions, colors;
var particles = 1081; // # of ranges
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
  //camera = new THREE.PerspectiveCamera( 60, width / height, 1, 1000 );
  camera = new THREE.PerspectiveCamera();
  camera.position.z = 500;

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

  // grid
  var image = new Uint8Array(320*190*4)
  for (var i = 0;i<320*190; i=i+4) {
    image[i] = 255;
    image[i+1] = 0;
    image[i+2] = 0;
    image[i+3] = 255;
  }
  init_texture = new THREE.DataTexture( image, 320, 190);

  plane_mat = new THREE.MeshBasicMaterial( { color: 0xffffff, wireframe: false,
    transparent: false, opacity : 1,
    map: init_texture  } );
//  });
  init_texture.needsUpdate = true;
  plane_mat.needsUpdate = true;
  console.log( plane_mat );

  plane = new THREE.Mesh(
      new THREE.PlaneGeometry( 320, 190, 160, 95 ),
      plane_mat
  );
  scene.add( plane );

  // Lights!
  var ambientLight = new THREE.AmbientLight( 0xffffff );
  scene.add( ambientLight );

  var directionalLight = new THREE.DirectionalLight( 0xffffff );
  directionalLight.position.set( 1, 0.75, 0.5 ).normalize();
  scene.add( directionalLight );

  // renderer
  renderer = new THREE.WebGLRenderer();
  renderer.setSize( width, height );

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

if ( ! Detector.webgl ) Detector.addGetWebGLMessage();

var ctxwidth = window.innerWidth;//400;
var ctxheight = window.innerHeight;400;
var kwidth = 320;
var kheight = 240;
var particles = kwidth*kheight;

// Globals
var particleSystem, plane;
var container;
var camera, controls, scene, renderer;
var positions, colors, vertices, geometry, faces;
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

/*
  // Initialize the particles
  geometry = new THREE.BufferGeometry();
  geometry.attributes = {
    position: {
      itemSize: 3,
      array: new Float32Array( particles * 3 ),
      numItems: particles * 3
    }
  }
  geometry.dynamic = true;
  // Set the colors and positions
  positions = geometry.attributes.position.array;
  var cnt = 0;
  for( var j=0; j<240; j++ ){
    for( var i=0; i<320; i++ ){
      positions[cnt] = i;
      positions[cnt+1] = j;
      positions[cnt+2] = 0;
      cnt = cnt+3;
    }
 }
geometry.verticesNeedUpdate = true;
geometry.computeBoundingSphere();
geometry.computeTangents();
geometry.computeVertexNormals()
*/

  // Add the Image plane
  var image = new Uint8Array(320*190*4)
  for (var i = 0;i<kwidth*kheight; i=i+4) {
    image[i] = 255; // R
    image[i+1] = 0; // G
    image[i+2] = 0; // B
    image[i+3] = 255; // A
  }
  var plane_texture = new THREE.DataTexture( image, kwidth, kheight);
  plane_mat = new THREE.MeshBasicMaterial( {
    color: 0xffffff,
    wireframe: true,
    transparent: false,
    opacity : 1,
    map: plane_texture
  } );
  plane = new THREE.Mesh(
    new THREE.PlaneGeometry( kwidth, kheight, kwidth, kheight ),
    //geometry,
    plane_mat
  );

//plane.geometry.applyMatrix( new THREE.Matrix4().makeRotationX( - Math.PI / 2 ) );
vertices = plane.geometry.vertices;
faces = plane.geometry.faces;
/*
var cnt = 0;
  for( var j=0; j<240; j++ ){
    for( var i=0; i<320; i++ ){
//    vertices[cnt].x = i;
//    vertices[cnt].y = j;
    vertices[ faces[cnt].a ].z = 10*(i/320);
    vertices[ faces[cnt].b ].z = 10*(i/320);
    vertices[ faces[cnt].c ].z = 10*(i/320);
    vertices[ faces[cnt].d ].z = 10*(i/320);
faces
cnt = cnt+1;
    }
  }
*/
plane.geometry.computeVertexNormals()
plane.geometry.computeFaceNormals()

console.log(plane.geometry)

plane.geometry.dynamic = true;
  plane_texture.needsUpdate = true;
  plane_mat.needsUpdate = true;
  plane.geometry.verticesNeedUpdate = true;
//  plane.geometry.elementsNeedUpdate = true;
//  plane.geometry.uvsNeedUpdate = true;
  plane.geometry.normalsNeedUpdate = true;

  // Set up the world
  scene = new THREE.Scene();
  scene.add( plane );

  // Renderer
  renderer = new THREE.WebGLRenderer( { antialias: false, clearColor: 0x333333, clearAlpha: 1, alpha: false } );
  renderer.setSize( ctxwidth, ctxheight );
  container = document.getElementById( 'kgl' );
  container.appendChild( renderer.domElement );

window.addEventListener( 'resize', onWindowResize, false );

// Double click
  container.ondblclick = function( event ){
    var projector = new THREE.Projector();
    var directionVector = new THREE.Vector3();
    var SCREEN_HEIGHT = window.innerHeight;
    var SCREEN_WIDTH = window.innerWidth;
    // The following will translate the mouse coordinates into a number
    // ranging from -1 to 1, where
    //      x == -1 && y == -1 means top-left, and
    //      x ==  1 && y ==  1 means bottom right
    var x = ( event.clientX / SCREEN_WIDTH ) * 2 - 1;
    var y = -( event.clientY / SCREEN_HEIGHT ) * 2 + 1;
    directionVector.set(x, y, 1);
    // Unproject the vector
    projector.unprojectVector(directionVector, camera);
    // Substract the vector representing the camera position
    directionVector.sub(camera.position);
    // Normalize the vector, to avoid large numbers from the
    // projection and substraction
    directionVector.normalize()
//    directionVector.setZ( directionVector.z*-1) ;
      
    // Now our direction vector holds the right numbers!
    var ray = new THREE.Raycaster(camera.position, directionVector);
    var intersects = ray.intersectObjects(scene.children);
    //console.log( intersects );
    //console.log( ray );
    if (intersects.length) {
      // Ordered by distance
      console.log( intersects[0] );
    }
    
    console.log(event)
  }

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
}

function update_kinect_image( data_buffer ) {
  // Redo the RGB data for the texture
  plane_mat.map.image.data = data_buffer;
  //var new_texture = new THREE.DataTexture( data_buffer, kwidth, kheight);
  //plane_mat.map = new_texture;
  plane_mat.map.needsUpdate = true;

  animate();
  render();
}

function update_kinect_depth( d_buffer ) {

var tmp,fdx,ddx;
ddx = 0;
for(var j=0; j<240; j++ ){
  for (var i = 0; i<320; i++ ){
fdx = j*320+i;
//tmp = ( d_buffer[ddx] + d_buffer[ddx+1] + d_buffer[ddx+2] )/3;
tmp = d_buffer[ddx];
ddx = ddx+4;
if(tmp>4 && tmp < 250) {
    vertices[faces[fdx].a].x = tmp*hlut[i];
    vertices[faces[fdx].a].y = tmp*vlut[j];
    vertices[faces[fdx].a].z = tmp;

    vertices[faces[fdx].b].x = tmp*hlut[i];
    vertices[faces[fdx].b].y = tmp*vlut[j];
    vertices[faces[fdx].b].z = tmp;

    vertices[faces[fdx].c].x = tmp*hlut[i];
    vertices[faces[fdx].c].y = tmp*vlut[j];
    vertices[faces[fdx].c].z = tmp;

    vertices[faces[fdx].c].x = tmp*hlut[i];
    vertices[faces[fdx].c].y = tmp*vlut[j];
    vertices[faces[fdx].d].z = tmp;
} 
/*
else {
    vertices[faces[fdx].a].z = 255;
    vertices[faces[fdx].b].z = 255;
    vertices[faces[fdx].c].z = 255;
    vertices[faces[fdx].d].z = 255;
}
*/

  }
}
  plane.geometry.verticesNeedUpdate = true;
//plane.geometry.elementsNeedUpdate = true;
//plane.geometry.uvsNeedUpdate = true;
plane.geometry.normalsNeedUpdate = true;
plane.geometry.computeVertexNormals()
plane.geometry.computeFaceNormals()

  animate();
  render();
}

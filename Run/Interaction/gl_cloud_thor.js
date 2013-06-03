// Global Variables
var use_mesh = 1;
var use_cloud = 0;
var ctxwidth;
var ctxheight;
var kwidth = 320;
var kheight = 240;
var nparticles = kwidth*kheight;
var nchunks = Math.floor( nparticles / 2^16 );
//console.log("nchunks: "+nchunks);
var particleSystem, plane;
var container;
var camera, controls, scene, renderer;
var positions, colors, geometry;

// Set up LUT for real xyz positions
//https://github.com/OpenNI/OpenNI2/blob/master/Source/Core/OniStream.cpp#L362
var hFOV = 58;
var vFOV = 45;
var hlut = new Float32Array( kwidth  );
var vlut = new Float32Array( kheight );
for( var i=0;i<kwidth;i++ ){
  hlut[i] = Math.tan(hFOV/2)*2*(i/kwidth -.5); // float?
}
for( var j=0;j<kheight;j++ ){
  vlut[j] = Math.tan(vFOV/2)*2*(j/kheight-.5); // float math?
}
//console.log("Done setting up the fov tables.")

function set_ctx_dim(){
  ctxwidth = window.innerWidth;
  ctxheight = $('#content').height()-40;//window.innerHeight;
}

function init_gl_cloud() {
  container = document.getElementById('pickr');
  $('#kcolor').hide()
  $('#kdepth').hide()
  set_ctx_dim();
  // Set up the camera
  camera = new THREE.PerspectiveCamera();
  camera.position.x = 0;
  camera.position.y = 300;
  camera.position.z = 300;
  camera.lookAt( 0, 0, -200 );

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
  
  /////////////////////
  // Set up the world
  scene = new THREE.Scene();
  // Renderer
  renderer = new THREE.WebGLRenderer( { antialias: false, clearColor: 0x558855, clearAlpha: 1, alpha: false } );
  renderer.setSize( ctxwidth, ctxheight );
  container.appendChild( renderer.domElement );

  /////////////////////
  // Initialize the particles
  // TODO: chunk it!
  if( use_cloud==1 ){
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
        positions[cnt] = i - kwidth/2;
        positions[cnt+1] = j - kheight/2;
        positions[cnt+2] = 0;
        colors[cnt] = 0;
        colors[cnt+1] = 255;
        colors[cnt+2] = 0;
        cnt = cnt+3;
      }
    }
    //particleSystem.geometry.attributes[ "position" ].needsUpdate = true;
    // TODO: is this necessary?
    geometry.computeBoundingSphere();
    var material = new THREE.ParticleBasicMaterial( { size: 3, vertexColors: true } );
    particleSystem = new THREE.ParticleSystem( geometry, material );
    scene.add( particleSystem );
  }
  /////////////////////
  // Add the plane mesh
  if( use_mesh==1 ){
    var size = kwidth * kheight;
    var data = new Uint8Array( 4 * size );
    for ( var i = 0; i < size; i ++ ) {
      data[ i * 4 ] 	  = 255;
      data[ i * 4 + 1 ] = 0;
      data[ i * 4 + 2 ] = 0;
      data[ i * 4 + 3 ] = 255;
    }
    var meshmap = new THREE.DataTexture( data, kwidth, kheight, THREE.RGBAFormat );
    meshmap.needsUpdate = true;
    //var plane_texture = new THREE.DataTexture( image, kwidth, kheight);
    var plane_mat = new THREE.MeshBasicMaterial( {
      map: meshmap,
      wireframe: false
    } );
    plane = new THREE.Mesh(
      new THREE.PlaneGeometry( kwidth, kheight, kwidth, kheight ),
      plane_mat
    );
    scene.add( plane );
  }

  /////////////////////
  // Set up the cylinder to move around
  var cylGeo = new THREE.CubeGeometry(20, 20, 20, 1,1,1 );
  cylMesh = new THREE.Mesh( cylGeo, new THREE.MeshBasicMaterial( { color: 0xff0000 } ) );
  scene.add( cylMesh );

  // Draw
  animate();
  render();

}

function animate() {
  requestAnimationFrame( animate );
  controls.update();
}

function render() {
  renderer.render( scene, camera );
}

function onWindowResize() {
  set_ctx_dim();
  camera.aspect = ctxwidth / ctxheight;
  camera.updateProjectionMatrix();
  renderer.setSize( ctxwidth, ctxheight );
  controls.handleResize();
  render();
}

var update_cloud_image = function( c_buffer ) {
  var d_idx = 0;
  var c_idx = 0;
  for(var j=0; j<kheight; j++ ){
    for (var i = 0; i<kwidth; i++ ){
      // TODO: div by 255 may be in int space, and not float space
      colors[ c_idx ]     = c_buffer.data[d_idx] / 255;
      colors[ c_idx + 1 ] = c_buffer.data[d_idx+1] / 255;
      colors[ c_idx + 2 ] = c_buffer.data[d_idx+2] / 255;
      c_idx+=3;
      d_idx+=4; //RGBA
    }
  }
  particleSystem.geometry.attributes[ "color" ].needsUpdate = true;
  animate();
  render();
}

var update_cloud_depth = function( d_buffer ) {
  var d_idx = 0;
  var p_idx = 0;
  var tmp = 0;
  // TODO: cache it good
  for(var j=0; j<kheight; j++ ){
    for (var i = 0; i<kwidth; i++ ){
      tmp = d_buffer.data[d_idx];
      if( tmp>32 ) {
        tmp = -1*tmp
        positions[ p_idx ]     = tmp*hlut[i];
        positions[ p_idx + 1 ] = tmp*vlut[j];
        positions[ p_idx + 2 ] = tmp;
      } else {
        positions[ p_idx ]     = 0;
        positions[ p_idx + 1 ] = 0;
        positions[ p_idx + 2 ] = 0;
      }
      p_idx+=3;
      d_idx+=4;
    }
  }
  particleSystem.geometry.attributes[ "position" ].needsUpdate = true;
  animate();
  render();
}

function update_mesh_image( c_buffer ) {
  //plane.material.map.image.data = c_buffer;
  var size = kwidth * kheight;
  var tmp = 0;
  for ( var i = 0; i < size; i++ ) {
    tmp = i*4;
    plane.material.map.image.data[ tmp ] 	 = c_buffer[ tmp ];
    plane.material.map.image.data[ tmp+1 ] = c_buffer[ tmp+1 ];
    plane.material.map.image.data[ tmp+2 ] = c_buffer[ tmp+2 ];
//    plane.material.map.image.data[ tmp + 3 ] = 255;
  }
  plane.material.needsUpdate = true;
  plane.material.map.needsUpdate = true;
  animate();
  render();
}

function update_mesh_depth( d_buffer ) {
  var vertices = plane.geometry.vertices;
  var faces = plane.geometry.faces;
  var tmp,fdx,ddx,tmpH,tmpV;
  ddx = 0;
  for(var j=0; j<kheight; j++ ){
    for (var i = 0; i<kwidth; i++ ){
      fdx = j*kwidth+i;
      ddx = ddx+4;
      // Range check
      //tmp = 255;
      if(d_buffer[ddx]>20) {
        tmp = d_buffer[ddx];
      }
      tmpH = tmp*hlut[i];
      tmpV = tmp*vlut[j];
      vertices[faces[fdx].a].x = tmpH;
      vertices[faces[fdx].a].y = tmpV;
      vertices[faces[fdx].a].z = tmp;
      vertices[faces[fdx].b].x = tmpH;
      vertices[faces[fdx].b].y = tmpV;
      vertices[faces[fdx].b].z = tmp;
      vertices[faces[fdx].c].x = tmpH;
      vertices[faces[fdx].c].y = tmpV;
      vertices[faces[fdx].c].z = tmp;
      vertices[faces[fdx].c].x = tmpH;
      vertices[faces[fdx].c].y = tmpV;
      vertices[faces[fdx].d].z = tmp;
    }
  }
//  plane.geometry.computeVertexNormals()
//  plane.geometry.computeFaceNormals()
  plane.geometry.normalsNeedUpdate = true;
  plane.geometry.verticesNeedUpdate = true;
  animate();
  render();
}

// Set the event listeners
window.addEventListener( 'resize', onWindowResize );
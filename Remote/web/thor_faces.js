// Guide: http://threejs.org/examples/webgl_buffergeometry.html
// Global Variables
var ctxwidth, ctxheight;
//var ntriangles = 160000; // TODO: check with SJ what a good limit is
var ntriangles = 1166;

var container, mesh;
var camera, controls, scene, renderer;

function set_ctx_dim(){
	ctxwidth  = window.innerWidth /2;
	ctxheight = window.innerHeight/2;
}

function animate() {
	requestAnimationFrame( animate );
	//controls.update();
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
// Set the event listeners
window.addEventListener( 'resize', onWindowResize );

function init_faces() {
	
	/////////////////////
	// Find the canvas element for the webgl scene
	container = document.getElementById('content');
	$('#faces').hide();
	set_ctx_dim();
	/////////////////////
	
	/////////////////////
	// Set up the world
	scene = new THREE.Scene();
	// Renderer
	renderer = new THREE.WebGLRenderer( 
		/*{ antialias: false, clearColor: 0x558855, clearAlpha: 1, alpha: false } */
	);
	renderer.setSize( ctxwidth, ctxheight );
	container.appendChild( renderer.domElement );
	/////////////////////
	
	/////////////////////
	// Set up the camera
	camera = new THREE.PerspectiveCamera();
	camera.position.x = 0;
	camera.position.y = 300;
	camera.position.z = 300;
	camera.lookAt( 0, 0, -200 );
	// Override from the example
	// PerspectiveCamera( fov, aspect, near, far )
	camera = new THREE.PerspectiveCamera( 60, ctxwidth/ctxheight, 1, 3500 );
	camera.position.z = 2750;
	/////////////////////
	
	/////////////////////
	// Lighting
	scene.add( new THREE.AmbientLight( 0x444444 ) );
	var light1 = new THREE.DirectionalLight( 0xffffff, 0.5 );
	light1.position.set( 1, 1, 1 );
	scene.add( light1 );
	var light2 = new THREE.DirectionalLight( 0xffffff, 1.5 );
	light2.position.set( 0, -1, 0 );
	scene.add( light2 );
	/////////////////////

	/////////////////////
	// Initialize the faces
	var fgeometry = new THREE.BufferGeometry();
	// Dynamic, because we will update it (holds buffers)
	fgeometry.dynamic = true;
	// Set the attribute buffers
	fgeometry.attributes = {
		index: {
			itemSize: 1,
			array: new Uint16Array( ntriangles * 3 ),
			numItems: ntriangles * 3
		},
		position: {
			itemSize: 3,
			array: new Float32Array( ntriangles * 3 * 3 ),
			numItems: ntriangles * 3 * 3
		},
		normal: {
			itemSize: 3,
			array: new Float32Array( ntriangles * 3 * 3 ),
			numItems: ntriangles * 3 * 3
		},
		color: {
			itemSize: 3,
			array: new Float32Array( ntriangles * 3 * 3 ),
			numItems: ntriangles * 3 * 3
		}
	}
	/////////////////////
	
	/////////////////////
	// Initialize the indices and offsets for chunks of triangles
	// This is because you can only index by uint16, so need to overcome this
	// From the reference: "break geometry into
	// chunks of 21,845 triangles (3 unique vertices per triangle)
	// for indices to fit into 16 bit integer number
	// floor(2^16 / 3) = 21845"
	var chunkSize = 21845;
	var indices = fgeometry.attributes.index.array;
	for ( var i = 0; i < indices.length; i ++ ) {
		indices[ i ] = i % ( 3 * chunkSize );
	}
	var offsets = ntriangles / chunkSize;
	fgeometry.offsets = [];
	for ( var i = 0; i < offsets; i ++ ) {
		var offset = {
			start: i * chunkSize * 3,
			index: i * chunkSize * 3,
			count: Math.min( ntriangles - ( i * chunkSize ), chunkSize ) * 3
		};
		fgeometry.offsets.push( offset );
	}
	/////////////////////
	
	/////////////////////
	// Initialize the colors and positions
	var positions = fgeometry.attributes.position.array;
	var normals = fgeometry.attributes.normal.array;
	var colors = fgeometry.attributes.color.array;

	var color = new THREE.Color();

	var n = 800, n2 = n/2;	// triangles spread in the cube
	var d = 12, d2 = d/2;	// individual triangle size

	var pA = new THREE.Vector3();
	var pB = new THREE.Vector3();
	var pC = new THREE.Vector3();

	var cb = new THREE.Vector3();
	var ab = new THREE.Vector3();

	for ( var i = 0; i < positions.length; i += 9 ) {

		// positions

		var x = Math.random() * n - n2;
		var y = Math.random() * n - n2;
		var z = Math.random() * n - n2;

		var ax = x + Math.random() * d - d2;
		var ay = y + Math.random() * d - d2;
		var az = z + Math.random() * d - d2;

		var bx = x + Math.random() * d - d2;
		var by = y + Math.random() * d - d2;
		var bz = z + Math.random() * d - d2;

		var cx = x + Math.random() * d - d2;
		var cy = y + Math.random() * d - d2;
		var cz = z + Math.random() * d - d2;

		positions[ i ]     = ax;
		positions[ i + 1 ] = ay;
		positions[ i + 2 ] = az;

		positions[ i + 3 ] = bx;
		positions[ i + 4 ] = by;
		positions[ i + 5 ] = bz;

		positions[ i + 6 ] = cx;
		positions[ i + 7 ] = cy;
		positions[ i + 8 ] = cz;

		// flat face normals

		pA.set( ax, ay, az );
		pB.set( bx, by, bz );
		pC.set( cx, cy, cz );

		cb.subVectors( pC, pB );
		ab.subVectors( pA, pB );
		cb.cross( ab );

		cb.normalize();

		var nx = cb.x;
		var ny = cb.y;
		var nz = cb.z;

		normals[ i ]     = nx;
		normals[ i + 1 ] = ny;
		normals[ i + 2 ] = nz;

		normals[ i + 3 ] = nx;
		normals[ i + 4 ] = ny;
		normals[ i + 5 ] = nz;

		normals[ i + 6 ] = nx;
		normals[ i + 7 ] = ny;
		normals[ i + 8 ] = nz;

		// colors

		var vx = ( x / n ) + 0.5;
		var vy = ( y / n ) + 0.5;
		var vz = ( z / n ) + 0.5;

		color.setRGB( vx, vy, vz );

		colors[ i ]     = color.r;
		colors[ i + 1 ] = color.g;
		colors[ i + 2 ] = color.b;

		colors[ i + 3 ] = color.r;
		colors[ i + 4 ] = color.g;
		colors[ i + 5 ] = color.b;

		colors[ i + 6 ] = color.r;
		colors[ i + 7 ] = color.g;
		colors[ i + 8 ] = color.b;

	}
	/////////////////////
	
	/////////////////////
	// Massage the geometry to be ready for the scene
	// fgeometry.computeFaceNormals();
	// TODO: not included
	fgeometry.computeBoundingSphere();
	/////////////////////

	/////////////////////
	// Set a the initial colors (from fgeometry) and material (standard)
	var material = new THREE.MeshPhongMaterial( {
		color: 0xaaaaaa, ambient: 0xaaaaaa, specular: 0xffffff, shininess: 250,
		side: THREE.DoubleSide, vertexColors: THREE.VertexColors
	} );
	/////////////////////

	/////////////////////
	// Make the mesh from our geometry, and add it to the scene
	mesh = new THREE.Mesh( fgeometry, material );
	scene.add( mesh );
	/////////////////////

	/////////////////////
	// Set up the cylinder to move around
	var cylGeo = new THREE.CubeGeometry(20, 20, 20, 1,1,1 );
	cylMesh = new THREE.Mesh( 
		cylGeo, new THREE.MeshBasicMaterial( { color: 0xff0000 } )
	);
	scene.add( cylMesh );
	/////////////////////
	
	/////////////////////
	// Set up the mouse controls
	/*
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
	*/
	/////////////////////

	/////////////////////
	// Draw the scene
	animate();
	render();
	/////////////////////

}

var update_faces = function( vertices ){
	mesh.geometry.attributes[ 'position' ].array = vertices;
	// TODO: change the face normals
	mesh.geometry.normalsNeedUpdate = true;
	mesh.geometry.verticesNeedUpdate = true;
	mesh.geometry.attributes[ 'color' ].needsUpdate = true;
	animate();
	render();
}

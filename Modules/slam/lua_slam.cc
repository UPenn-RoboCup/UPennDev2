/* NOTE: Torch C access begins with 0, not 1 as in Torch's Lua access */
#include <lua.hpp>
#ifdef __cplusplus
extern "C" {
#endif
#include <torch/luaT.h>
#include <torch/TH/TH.h>
#ifdef __cplusplus
}
#endif

#include <stdint.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <stdbool.h>
#include <float.h>
#include <iostream>

#define SKIP 2
#define DEFAULT_RESOLUTION 0.05
#define DEFAULT_INV_RESOLUTION 20

/* Candidate indices helpers for speed */
unsigned int xis[1081];
unsigned int yis[1081];
/* Laser points in map coordinates */
double lxs_map[1081];
double lys_map[1081];

	template<typename TT>
int isContiguous(TT *self)
{
	long z = 1;
	int d;
	for(d = self->nDimension-1; d >= 0; d--)
	{
		if(self->size[d] != 1)
		{
			if(self->stride[d] == z)
				z *= self->size[d];
			else
				return 0;
		}
	}
	return 1;
}

/* Store the min and max values of the map */
/* TODO: either make as a module, or pass as arguments. */
/* TODO: multiple maps of various resolution? */
double xmin, ymin, xmax, ymax;
double invxmax, invymax;
double res = DEFAULT_RESOLUTION, invRes = DEFAULT_INV_RESOLUTION;

/* Set the boundaries for scan matching and map updating */
int lua_set_boundaries(lua_State *L) {
	xmin = luaL_checknumber(L, 1);
	ymin = luaL_checknumber(L, 2);
	xmax = luaL_checknumber(L, 3);
	ymax = luaL_checknumber(L, 4);
	invymax = ymax * DEFAULT_INV_RESOLUTION;
	invxmax = xmax * DEFAULT_INV_RESOLUTION;
	return 0;
}

/* Set the resolution of the map */
int lua_set_resolution(lua_State *L) {
	res = luaL_checknumber(L, 1);
	invRes = 1.0 / res;
	return 0;
}

/* Update the map with the laser scan points */
int lua_update_map(lua_State *L) {
	static int i = 0, mapLikelihood = 0;
	static double x=0,y=0;
	static unsigned int xi = 0, yi = 0;
	static uint8_t newVal = 0;

	/* Get the map, which is a ByteTensor */
	const THByteTensor * map_t =
		(THByteTensor *) luaT_checkudata(L, 1, "torch.ByteTensor");
#ifdef DEBUG
	int is_con = isContiguous<THByteTensor>((THByteTensor *)map_t);
	if (!is_con)
		return luaL_error(L, "map_t input must be contiguous");
#endif
	uint8_t * map_tp = (uint8_t *)(map_t->storage->data + map_t->storageOffset);

	/*
		 const long sizex = map_t->size[0];
		 const long sizey = map_t->size[1];
		 */

	/* Get the updating table */
	const THDoubleTensor * update_t =
		(THDoubleTensor *) luaT_checkudata(L, 4, "torch.DoubleTensor");
#ifdef DEBUG
	is_con = isContiguous<THDoubleTensor>((THDoubleTensor *)update_t);
	if (!is_con)
		return luaL_error(L, "update_t input must be contiguous");
#endif
	double * update_tp = (double *)(update_t->storage->data + update_t->storageOffset);


	/* Grab the updated points */
	const THDoubleTensor * ps_t =
		(THDoubleTensor *) luaT_checkudata(L, 2, "torch.DoubleTensor");
	const long nps  = ps_t->size[0];
#ifdef DEBUG
	is_con = isContiguous<THDoubleTensor>((THDoubleTensor *)ps_t);
	if (!is_con)
		return luaL_error(L, "ps_t input must be contiguous");
#endif
	//  double * ps_tp = (double *)(ps_t->storage->data + ps_t->storageOffset);

	/* Grab the increment value */
	const int inc = luaL_checknumber(L, 3);

	// TODO: Assume x/y stride relationship (Make sure to TH_Assert this or something)
	/*
		 printf("W | Offset (%ld), Stride0 (%ld), Stride1 (%ld), Size0 (%ld), Size1 (%ld)\n",
		 ps_t->storageOffset, ps_t->stride[0], ps_t->stride[1], 
		 ps_t->size[0], ps_t->size[1]);
	//x = THTensor_fastGet2d( ps_t, i, 0 );
	//y = THTensor_fastGet2d( ps_t, i, 1 );
	*/
	double* pts_ptr = ps_t->storage->data;
	long mystride = ps_t->stride[0];
	//#pragma omp parallel for
	for( i=0; i<nps; i++ ) {
		x = *(pts_ptr);
		y = *(pts_ptr+1);
		pts_ptr += mystride;
		if( x>xmax || y>ymax || x<xmin || y<ymin ) continue;

		/* TODO: ceil or floor these? map_t bounds check? */
		xi = (unsigned long)( ( x - xmin ) * invRes );
		yi = (unsigned long)( ( y - ymin ) * invRes );

		/* Check if this cell has been updated before */
		/* if (THTensor_fastGet2d(update_t,xi,yi) == 1) continue; */
		if (update_tp[xi * update_t->stride[0] + yi] == 1) continue;

		/* mapLikelihood = THTensor_fastGet2d(map_t,xi,yi) + inc; */
		mapLikelihood = map_tp[xi * map_t->stride[0] + yi] + inc;

		/* --------------------------- */
		/* bit hack for range limit */
		/* http://graphics.stanford.edu/~seander/bithacks.html#IntegerMinOrMax */
		/* newVal = min(mapLikelihood, 255) */
		int range_max = 255;
		int range_min = 0;
		mapLikelihood = range_max ^ ((mapLikelihood ^ range_max) & -(mapLikelihood < range_max));
		mapLikelihood = mapLikelihood ^ ((mapLikelihood ^ range_min) & -(mapLikelihood < range_min));
		newVal = mapLikelihood;

		/* --------------------------- */

		/*
			 THTensor_fastSet2d( map_t, xi, yi, newVal );
			 THTensor_fastSet2d( update_t, xi, yi, 1 );
			 */
		map_tp[xi * map_t->stride[0] + yi] = newVal;
		update_tp[xi * update_t->stride[0] + yi] = 1;
	}
	return 0;
}

/* Update the height map with the laser scan points */
/* Currently use same resolution and size as OMAP*/
int lua_update_hmap(lua_State *L) {
	static int i = 0;
	static double x=0,y=0, height=0;
	static unsigned int xi = 0, yi = 0;

	/* Get the map, which is a ByteTensor */
	const THByteTensor * map_t =
		(THByteTensor *) luaT_checkudata(L, 1, "torch.ByteTensor");
#ifdef DEBUG
	int is_con = isContiguous<THByteTensor>((THByteTensor *)map_t);
	if (!is_con)
		return luaL_error(L, "map_t input must be contiguous");
#endif
	uint8_t * map_tp = (uint8_t *)(map_t->storage->data + map_t->storageOffset);


	/* Grab the updated points */
	const THDoubleTensor * ps_t =
		(THDoubleTensor *) luaT_checkudata(L, 2, "torch.DoubleTensor");
	const long nps  = ps_t->size[0];
#ifdef DEBUG
	is_con = isContiguous<THDoubleTensor>((THDoubleTensor *)ps_t);
	if (!is_con)
		return luaL_error(L, "ps_t input must be contiguous");
#endif
	double * ps_tp = (double *)(ps_t->storage->data + ps_t->storageOffset);

	/* Grab the height value */
	//const double height = luaL_checknumber(L, 3);

	//#pragma omp parallel for
	double *ps_tp_temp = NULL;
	for( i=0; i<nps; i++ ) {
		/*
			 x = THTensor_fastGet2d( ps_t, i, 0 );
			 y = THTensor_fastGet2d( ps_t, i, 1 );
			 height = THTensor_fastGet2d( ps_t, i, 2 );
			 */
		ps_tp_temp = ps_tp + i * ps_t->stride[0];
		x = *(ps_tp_temp++);
		y = *(ps_tp_temp++);
		height = *(ps_tp_temp++);

		if( x>xmax || y>ymax || x<xmin || y<ymin ) continue;

		/*  and OMAP.data_update[ xis[i] ][ yis[i] ]==0 */
		/*  OMAP.data_update[ xis[i] ][ yis[i] ] = 1 */
		/* TODO: ceil or floor these? map_t bounds check? */
		xi = (unsigned long)( ( x - xmin ) * invRes );
		yi = (unsigned long)( ( y - ymin ) * invRes );

		//currentHeight = THTensor_fastGet2d(map_t,xi,yi);

		//if (height > currentHeight){
		/* THTensor_fastSet2d( map_t, xi, yi, height); */
		map_tp[xi * map_t->stride[0] + yi]= height;
		//}

	}
	return 0;
}


/* Merge hmap and omap together */
int lua_update_smap(lua_State *L) {
	static int i = 0, j=0;
	static uint8_t omap_val=0, hmap_val=0;

	/* Get the omap, which is a ByteTensor */
	const THByteTensor * omap_t =
		(THByteTensor *) luaT_checkudata(L, 1, "torch.ByteTensor");
#ifdef DEBUG
	int is_con = isContiguous<THByteTensor>((THByteTensor *)omap_t);
	if (!is_con)
		return luaL_error(L, "omap_t input must be contiguous");
#endif
	uint8_t * omap_tp = (uint8_t *)(omap_t->storage->data + omap_t->storageOffset);

	/* Get the hmap, which is a ByteTensor */
	const THByteTensor * hmap_t =
		(THByteTensor *) luaT_checkudata(L, 2, "torch.ByteTensor");
#ifdef DEBUG
	is_con = isContiguous<THByteTensor>((THByteTensor *)hmap_t);
	if (!is_con)
		return luaL_error(L, "hmap_t input must be contiguous");
#endif
	uint8_t * hmap_tp = (uint8_t *)(hmap_t->storage->data + hmap_t->storageOffset);

	/* Get the smap, which is a ByteTensor */
	const THByteTensor * smap_t =
		(THByteTensor *) luaT_checkudata(L, 3, "torch.ByteTensor");
#ifdef DEBUG
	is_con = isContiguous<THByteTensor>((THByteTensor *)smap_t);
	if (!is_con)
		return luaL_error(L, "smap_t input must be contiguous");
#endif
	uint8_t * smap_tp = (uint8_t *)(smap_t->storage->data + smap_t->storageOffset);

	/* Get the size of maps */
	const long sizex = omap_t->size[0];
	const long sizey = omap_t->size[1];

	/* Get pointers
		 double* omap_ptr = omap_t->storage->data;
		 long ostride = omap_t->stride[0];

		 double* hmap_ptr = hmap_t->storage->data;
		 long hstride = hmap_t->stride[0];

		 double* smap_ptr = smap_t->storage->data;
		 long sstride = smap_t->stride[0];
		 */

	//#pragma omp parallel for
	for( i=0; i<sizex; i++ ) {
		for( j=0; j<sizey; j++){

			/*
				 ox = *(omap_ptr);
				 oy = *(omap_ptr+1);
				 omap_ptr += ostride;

				 hx = *(hmap_ptr);
				 hy = *(hmap_ptr+1);
				 hmap_ptr += hstride;

				 sx = *(smap_ptr);
				 sy = *(smap_ptr+1);
				 smap_ptr += sstride;
				 */

			/* omap_val = THTensor_fastGet2d(omap_t,i,j); */
			omap_val = omap_tp[i * omap_t->stride[0] + j];
			/* hmap_val = THTensor_fastGet2d(hmap_t,i,j); */
			hmap_val = hmap_tp[i * hmap_t->stride[0] + j];

			if(omap_val>200){
				/* THTensor_fastSet2d( smap_t, i, j, 255 ); */
				smap_tp[i * smap_t->stride[0] + j] = 255;
			}else{
				/* THTensor_fastSet2d( smap_t, i, j, hmap_val ); */
				smap_tp[i * smap_t->stride[0] + j] = hmap_val;
			}

		}

	}
	return 0;
}



/* 2D scan match, with resulting maximum correlation information */
int lua_match(lua_State *L) {

	/* Get the map, which is a ByteTensor */
	const THByteTensor * map_t =
		(THByteTensor *) luaT_checkudata(L, 1, "torch.ByteTensor");
	// Assume offset of zero for map
	uint8_t * map_ptr = map_t->storage->data;
	const unsigned int sizex = map_t->size[0];
	const unsigned int sizey = map_t->size[1];
	const unsigned int map_xstride = map_t->stride[0];
	// Assume ystride of 1
	//const unsigned int map_ystride = map_t->stride[1];

	/* Grab the xs and ys from the last laser scan*/
	const THDoubleTensor * lY_t =
		(THDoubleTensor *) luaT_checkudata(L, 2, "torch.DoubleTensor");
	/* Grab the scanning values for theta, x, y */
	const THDoubleTensor * pxs_t =
		(THDoubleTensor *) luaT_checkudata(L, 3, "torch.DoubleTensor");
	const THDoubleTensor * pys_t =
		(THDoubleTensor *) luaT_checkudata(L, 4, "torch.DoubleTensor");
	const THDoubleTensor * pths_t =
		(THDoubleTensor *) luaT_checkudata(L, 5, "torch.DoubleTensor");
	/* The number of laser points and number of candidate x/y to match */
	unsigned int nps, npxs, npys;
	unsigned int npths = pths_t->size[0];
	unsigned int th_stride = pths_t->stride[0];

	/* Grab the output Tensor */
	THDoubleTensor * likelihoods_t = 
		(THDoubleTensor *) luaT_checkudata(L, 6, "torch.DoubleTensor");
	double* likestorage = 
		likelihoods_t->storage->data + likelihoods_t->storageOffset;
	const unsigned int nlikes = likelihoods_t->storage->size;

	/* Grab the number of points to process */
	npxs = pxs_t->size[0];
	npys = pys_t->size[0];
	nps = lY_t->size[0];

	/* Ensure that the liklihood map matches the candidate sizes */
	if (likelihoods_t->size[1] != npxs || 
			likelihoods_t->size[2]!=npys || 
			likelihoods_t->size[0]!=npths)
		return luaL_error(L, "Likelihood output wrong");

	/* Precalculate the indices for candidate x and y */
	unsigned int ii=0;
	//TODO: invRes should not be semaphored... that would be slow
#pragma omp parallel default(shared) private(ii) firstprivate(invRes)
	{
#pragma omp sections nowait
		{ /* sections */
#pragma omp section
			{
				double* pxs_ptr = pxs_t->storage->data + pxs_t->storageOffset;
				unsigned int* tmpx = xis;
				unsigned int xstride1 = pxs_t->stride[0];
				for( ii=0; ii<npxs; ii++ ){
					//xis[ii] = (THTensor_fastGet1d(pxs_t,ii)-xmin)*invRes;
					*tmpx = ( *pxs_ptr - xmin )*invRes;
					pxs_ptr += xstride1;
					tmpx++;
				}
			} /*section*/
#pragma omp section
			{
				double* pys_ptr = pys_t->storage->data + pys_t->storageOffset;
				unsigned int* tmpy = yis;
				unsigned int ystride1 = pys_t->stride[0];
				for(unsigned int ii=0;ii<npys;ii++){
					//yis[ii] = (THTensor_fastGet1d(pys_t,ii)-ymin)*invRes;
					*tmpy = ( *pys_ptr - ymin )*invRes;
					pys_ptr+=ystride1;
					tmpy++;
				}
			}/*section*/
#pragma omp section
			{
				/* Convert the laser points to the map coordinates */
				double* pls_ptr = lY_t->storage->data + lY_t->storageOffset;
				unsigned int pstride = lY_t->stride[0];
				double * tmplx = lxs_map, * tmply = lys_map;
				for(unsigned int ii=0;ii<nps;ii++){
					*tmplx = *(pls_ptr) * invRes;
					*tmply = *(pls_ptr+1) * invRes;
					pls_ptr += pstride;
					tmplx++;
					tmply++;
					/*
						 double lx = THTensor_fastGet2d( lY_t, ii, 0) * invRes;
						 double ly = THTensor_fastGet2d( lY_t, ii, 1) * invRes;
						 printf("lx (%f) tmplx (%f)\t",lx,*tmplx);
						 printf("lx (%f) tmplx (%f)\n",ly,*tmply);
						 */
				}
			} /*section*/
		}/*sections*/
	}/*omp parallel*/


	/* Loop indices */
	/* Iterate over all search angles */
	/* Shared variables for each thread */
	/* Private variables to be used within each thread */
	unsigned int * tmp_xi, * tmp_yi;
	double * tmp_lx_map, * tmp_ly_map, * tmp_like_with;
	uint8_t * map_ptr_with_x;
	unsigned int pi, pyi, pxi, xi, yi, pthi;
	double theta, costh, sinth, lx_map, ly_map, x_map, y_map;
	/* Variables for each distribution */
	double* th_ptr = pths_t->storage->data;
	double* tmp_like = likestorage;
	#pragma omp parallel for default(none) private(tmp_xi,tmp_yi,tmp_lx_map,tmp_ly_map, tmp_like_with, map_ptr_with_x, pi, pyi, pxi, xi, yi, theta, costh, sinth, lx_map, ly_map, x_map, y_map, npths, pthi, nps, npxs, npys )
	for ( pthi=0; pthi < npths; pthi++ ) {

		/* Matrix transform for each theta */
		theta = *th_ptr;
		th_ptr += th_stride;
		costh = cos( theta );
		sinth = sin( theta );

		/* Reset the pointers to the candidate points and liklihoods */
		tmp_xi = xis;
		tmp_yi = yis;

		/* Reset the pointers to the laser points */
		tmp_lx_map = lxs_map;
		tmp_ly_map = lys_map;
		/* Iterate over all laser points */
		// TODO: +=2 or adaptive filtering of the laser points
		for ( pi=0; pi<nps; pi+=SKIP ) {

			/* Grab the laser readings in map coordinates */
			/* Rotate them by the candidate theta */
			/* TODO: Do in bulk with torch? */

			lx_map = *tmp_lx_map;
			ly_map = *tmp_ly_map;
			x_map = lx_map*costh - ly_map*sinth;
			y_map = lx_map*sinth + ly_map*costh;
			/* Increment the pointer */
			tmp_lx_map+=SKIP;
			tmp_ly_map+=SKIP;

			/* Iterate over all candidate x's */
			tmp_like_with = tmp_like;
			tmp_xi = xis;
			for ( pxi=0; pxi<npxs; pxi++ ) {
				/* Use unsigned int - don't have to check < 0 */
				/* TODO: is this really a safe assumption at map edges? */
				xi = x_map + *(tmp_xi++);
				/*
					 int xi_old = x_map + (THTensor_fastGet1d(pxs_t,pxi)-xmin)*invRes;
					 printf("xi: %d -> xi: %u\n",xi_old,xi);
					 */
				if ( xi >= sizex ) {
					tmp_xi ++;
					tmp_like_with ++;
					continue;
				}
				map_ptr_with_x = map_ptr + xi*map_xstride;

				/* Iterate over all search y's */
				tmp_yi = yis;
				for ( pyi=0; pyi<npys; pyi++ ) {
					/* Use unsigned int - don't have to check < 0 */
					yi = y_map + *(tmp_yi++);
					/*
						 int yi_old = y_map + (THTensor_fastGet1d(pys_t,pyi)-ymin)*invRes;
						 printf("yi: %d -> yi: %u\n",yi_old,yi);
						 */
					if ( yi >= sizey ){
						tmp_like_with ++;
						continue;
					}
					// Increment likelihoods
					*tmp_like_with += *( map_ptr_with_x + yi );
					// Increment likelihood pointer
					tmp_like_with ++;

				} /* For pose ys */
			} /* For pose xs */
		} /* For laser scan points */
		/* Update the liklihood pointers AFTER each iteration */
		tmp_like = tmp_like_with;
	} /* For yaw values */

	/* Initialize max correlation value */
	double hmax = 0;
	double* tmplikestorage = likestorage; // Assume 0 for storageOffset
	int ilikestorage = 0;
	/* TODO: Use OpenMP to find the max */
	// http://stackoverflow.com/questions/978222/openmp-c-algorithms-for-min-max-median-average
	// http://msdn.microsoft.com/en-us/magazine/cc163717.aspx#S6
	for(unsigned int ii=0;ii<nlikes;ii++){
		//#pragma omp critical 
		{
			if( *tmplikestorage > hmax ){
				ilikestorage = ii;
				hmax = *tmplikestorage;
			}		
		}
		tmplikestorage++;
	}
	unsigned int ithmax = ilikestorage / likelihoods_t->stride[0];
	unsigned int ixmax  = (ilikestorage - ithmax*likelihoods_t->stride[0]) / likelihoods_t->stride[1];
	unsigned int iymax = ilikestorage - ithmax*likelihoods_t->stride[0] - ixmax*likelihoods_t->stride[1];

	/* Push maximum correlation results to Lua */
	lua_pushnumber(L,hmax);
	/* Lua indices start at 1, rather than 0, so +1 each index */
	lua_pushinteger(L,ixmax+1);
	lua_pushinteger(L,iymax+1);
	lua_pushinteger(L,ithmax+1);
	return 4;
}


/************************************************************************/


int lua_binStats(lua_State *L)
{
	static std::vector<int> Count;
	static std::vector<double> sumY, sumYY, maxY, minY;
	static double * prXp = NULL;
	static double * prYp = NULL;
	static double * prBp = NULL;
	static double * prTp = NULL;

	/* Grab the inputs */

	// #1 input: Distances from chest lidar
	const THDoubleTensor * prX =
		(THDoubleTensor *) luaT_checkudata(L, 1, "torch.DoubleTensor");
	const long nX  = prX->size[0];

#ifdef DEBUG
	int is_con = isContiguous<THDoubleTensor>((THDoubleTensor *)prX);
	if (!is_con)
		return luaL_error(L, "prX input must be contiguous");
#endif
	prXp = (double *)(prX->storage->data + prX->storageOffset);

	// #2 input: Heights from chest lidar
	const THDoubleTensor * prY =
		(THDoubleTensor *) luaT_checkudata(L, 2, "torch.DoubleTensor");
	const long nY  = prY->size[0];

#ifdef DEBUG
	is_con = isContiguous<THDoubleTensor>((THDoubleTensor *)prY);
	if (!is_con)
		return luaL_error(L, "prY input must be contiguous");
#endif
	prYp = (double *)(prY->storage->data + prY->storageOffset);

	// #3 input: Number of bins
	const int n = luaL_checkinteger(L, 3);

	// #4 input: Bin Table containing: "count", "mean", "max", "min", "std"
	const THDoubleTensor * prT =
		(THDoubleTensor *) luaT_checkudata(L, 4, "torch.DoubleTensor");
#ifdef DEBUG
	is_con = isContiguous<THDoubleTensor>((THDoubleTensor *)prT);
	if (!is_con)
		return luaL_error(L, "prT input must be contiguous");
#endif
	prTp = (double *)(prT->storage->data + prT->storageOffset);

	// #5 input: Bins
	const THDoubleTensor * prB =
		(THDoubleTensor *) luaT_checkudata(L, 5, "torch.DoubleTensor");
#ifdef DEBUG
	is_con = isContiguous<THDoubleTensor>((THDoubleTensor *)prB);
	if (!is_con)
		return luaL_error(L, "prB input must be contiguous");
#endif
	prBp = (double *)(prB->storage->data + prB->storageOffset);

	//fprintf(stdout, "nX, nY, n, BinTable, Bins: %ld \t %ld \t %d \t %ldx%ld \t %ldx%ld \n"
	//	,nX, nY, n, prT->size[0], prT->size[1], prB->size[0], prB->size[1]);

	if (nX != nY)
		luaL_error(L, "Number of elements in inputs should match");


	/* Initialize statistics vectors */
	if ( (int)Count.size() != n) {
		Count.resize(n);
		sumY.resize(n);
		sumYY.resize(n);
		maxY.resize(n);
		minY.resize(n);
	}

	for (int i = 0; i < n; i++) {
		Count[i] = 0;
		sumY[i] = 0;
		sumYY[i] = 0;
		maxY[i] = -__FLT_MAX__;
		minY[i] = __FLT_MAX__;
	}


	/* Calculate all the variables needed for the statistics of the points */		
	double tmpX, tmp;
	for (int i = 0; i < nX; i++) {
		tmpX = *(prXp + i);
		int j = round(tmpX) - 1;
		if ((j >= 0) && (j < n)) {
			Count[j]++;
			tmp = *(prYp + i);
			sumY[j] += tmp;
			sumYY[j] += tmp*tmp;
			if (tmp > maxY[j]) maxY[j] = tmp;
			if (tmp < minY[j]) minY[j] = tmp;
			prBp[i] = j + 1;
		}
	} 


	/* Calculate the statistics of each bin and set the returning Tensor */
	double mean = 0, std = 0, max = 0, min = 0;    
	double * prTp_temp = NULL;
	int * Countp = &(*Count.begin());
	for (int i = 0; i < n; i++) { 
		mean = 0; std = 0; max = 0; min = 0;
		if (Count[i] > 0) {
			mean = sumY[i]/Count[i];
			std = sqrt((sumYY[i]-sumY[i]*sumY[i]/Count[i])/Count[i]);
			max = maxY[i];
			min = minY[i];
		} 	  
		/*
			 THTensor_fastSet2d(prT, i, 0, Count[i]);
			 THTensor_fastSet2d(prT, i, 1, mean);
			 THTensor_fastSet2d(prT, i, 2, max);
			 THTensor_fastSet2d(prT, i, 3, min);
			 THTensor_fastSet2d(prT, i, 4, std);
			 */
		prTp_temp = prTp + i * prT->stride[0];
		*(prTp_temp++) = *(Countp ++);
		*(prTp_temp++) = mean;
		*(prTp_temp++) = max;
		*(prTp_temp++) = min;
		*(prTp_temp++) = std;

	}

	return 0;
}



/* Get the ground points from chest lidar points */
int lua_get_ground_points(lua_State *L) {
	static int i = 0;
	static double x=0,y=0,z=0,w=0;

	/* Get the torch to set */
	const THDoubleTensor * ps_x =
		(THDoubleTensor *) luaT_checkudata(L, 1, "torch.DoubleTensor");
#ifdef DEBUG
	int is_con = isContiguous<THDoubleTensor>((THDoubleTensor *)ps_x);
	if (!is_con)
		return luaL_error(L, "ps_x input must be contiguous");
#endif
	double * ps_xp = (double *)(ps_x->storage->data + ps_x->storageOffset);

	/* Grab the lidar points */
	const THDoubleTensor * ps_t =
		(THDoubleTensor *) luaT_checkudata(L, 2, "torch.DoubleTensor");
	const long nps_t  = ps_t->size[0];
#ifdef DEBUG
	is_con = isContiguous<THDoubleTensor>((THDoubleTensor *)ps_t);
	if (!is_con)
		return luaL_error(L, "ps_t input must be contiguous");
#endif
	double * ps_tp = (double *)(ps_t->storage->data + ps_t->storageOffset);

	/* Grab the total number of points to set */
	const double nps = luaL_checknumber(L, 3);

	//#pragma omp parallel for
	double * ps_tp_temp = NULL;
	double * ps_xp_temp = NULL;
	for( i=0; i<nps; i++ ) {
		/*
			 x = THTensor_fastGet2d( ps_t, nps_t-i-1, 0 );
			 y = THTensor_fastGet2d( ps_t, nps_t-i-1, 1 );
			 z = THTensor_fastGet2d( ps_t, nps_t-i-1, 2 );
			 w = THTensor_fastGet2d( ps_t, nps_t-i-1, 3 );
			 */
		ps_tp_temp = ps_tp + (nps_t-i-1) * ps_t->stride[0];
		x = *(ps_tp_temp++);
		y = *(ps_tp_temp++);
		z = *(ps_tp_temp++);
		w = *(ps_tp_temp++);

		ps_xp_temp = ps_xp + i * ps_x->stride[0];
		*(ps_xp_temp++) = x;
		*(ps_xp_temp++) = y;
		*(ps_xp_temp++) = z;
		*(ps_xp_temp++) = w;

	}
	return 0;
}



/* Get the points from chest lidar points 
	 to update the height map later */
int lua_get_height_points(lua_State *L) {
	static int i = 0;
	static double x=0,y=0,z=0,w=0; 
	double height = 0;

	/* Get the torch to set */
	const THDoubleTensor * ps_x =
		(THDoubleTensor *) luaT_checkudata(L, 1, "torch.DoubleTensor");
#ifdef DEBUG
	int is_con = isContiguous<THDoubleTensor>((THDoubleTensor *)ps_x);
	if (!is_con)
		return luaL_error(L, "ps_x input must be contiguous");
#endif
	double * ps_xp = (double *)(ps_x->storage->data + ps_x->storageOffset);

	/* Grab the lidar points */
	const THDoubleTensor * ps_t =
		(THDoubleTensor *) luaT_checkudata(L, 2, "torch.DoubleTensor");
	const long nps_t  = ps_t->size[0];
#ifdef DEBUG
	is_con = isContiguous<THDoubleTensor>((THDoubleTensor *)ps_t);
	if (!is_con)
		return luaL_error(L, "ps_t input must be contiguous");
#endif
	double * ps_tp = (double *)(ps_t->storage->data + ps_t->storageOffset);

	/* Grab the minimum height of the height map */
	const double min_height = luaL_checknumber(L, 3);

	/* Grab the maximum height of the height map */
	const double max_height = luaL_checknumber(L, 4);

	/* Grab the chest lidar height */
	const double offset = luaL_checknumber(L, 5);

	//#pragma omp parallel for
	double * ps_tp_temp = NULL;
	double * ps_xp_temp = NULL;
	for( i=0; i<nps_t; i++ ) {
		ps_tp_temp = ps_tp + i * ps_t->stride[0];
		x = *(ps_tp_temp++);
		y = *(ps_tp_temp++);
		z = *(ps_tp_temp++);
		w = *(ps_tp_temp++);

		/* Transformation to range [0,255] */
		ps_xp_temp = ps_xp + i * ps_x->stride[0];
		if (z >= max_height - offset){
			*(ps_xp_temp++) = x;
			*(ps_xp_temp++) = y;
			*(ps_xp_temp++) = 255;
			*(ps_xp_temp++) = w;
			continue;
		}
		else if (z <= min_height - offset) {
			*(ps_xp_temp++) = x;
			*(ps_xp_temp++) = y;
			*(ps_xp_temp++) = 0;
			*(ps_xp_temp++) = w;
			continue;
		}

		height = (z + offset - min_height)*(255/(max_height - min_height));

		ps_xp_temp = ps_xp + i * ps_x->stride[0];
		*(ps_xp_temp++) = x;
		*(ps_xp_temp++) = y;
		*(ps_xp_temp++) = height;
		*(ps_xp_temp++) = w;

	}
	return 0;
}


/* Get the chest lidar points and find the gnd pts and obs pts  */
int lua_mask_points(lua_State *L) {
	static int i = 0;
	int gndIdx=-1, obsIdx=-1;

	/* Get counts in each bin */
	const THDoubleTensor * c_t =
		(THDoubleTensor *) luaT_checkudata(L, 1, "torch.DoubleTensor");
	const long nps  = c_t->size[0];
#ifdef DEBUG
	int is_con = isContiguous<THDoubleTensor>((THDoubleTensor *)c_t);
	if (!is_con)
		return luaL_error(L, "c_t input must be contiguous");
#endif
	double * c_tp = (double *)(c_t->storage->data + c_t->storageOffset);

	/* Get zMaxmin */
	const THDoubleTensor * mm_t =
		(THDoubleTensor *) luaT_checkudata(L, 2, "torch.DoubleTensor");
	const long nps_mm  = mm_t->size[0];
	if (nps_mm!=nps)
		luaL_error(L,"Wrong Number of elements in zMaxMin!");
#ifdef DEBUG
	is_con = isContiguous<THDoubleTensor>((THDoubleTensor *)mm_t);
	if (!is_con)
		return luaL_error(L, "mm_t input must be contiguous");
#endif
	double * mm_tp = (double *)(mm_t->storage->data + mm_t->storageOffset);

	/* Get zMean */
	const THDoubleTensor * mean_t =
		(THDoubleTensor *) luaT_checkudata(L, 3, "torch.DoubleTensor");
	const long nps_mean  = mean_t->size[0];
	if (nps_mean!=nps)
		luaL_error(L,"Wrong Number of elements in zMean!");
#ifdef DEBUG
	is_con = isContiguous<THDoubleTensor>((THDoubleTensor *)mean_t);
	if (!is_con)
		return luaL_error(L, "mean_t input must be contiguous");
#endif
	double * mean_tp = (double *)(mean_t->storage->data + mean_t->storageOffset);

	/* Get xBin */
	const THDoubleTensor * xbin_t =
		(THDoubleTensor *) luaT_checkudata(L, 4, "torch.DoubleTensor");
	const long nps_xbin  = xbin_t->size[0];
	if (nps_xbin!=nps)
		luaL_error(L,"Wrong Number of elements in xBin!");
#ifdef DEBUG
	is_con = isContiguous<THDoubleTensor>((THDoubleTensor *)xbin_t);
	if (!is_con)
		return luaL_error(L, "xbin_t input must be contiguous");
#endif
	double * xbin_tp = (double *)(xbin_t->storage->data + xbin_t->storageOffset);

	/* Get container for iGnd */
	const THDoubleTensor * gnd_t =
		(THDoubleTensor *) luaT_checkudata(L, 5, "torch.DoubleTensor");
	const long nps_gnd  = gnd_t->size[0];
	if (nps_gnd!=nps)
		luaL_error(L,"Wrong Number of elements in iGnd!");
#ifdef DEBUG
	is_con = isContiguous<THDoubleTensor>((THDoubleTensor *)gnd_t);
	if (!is_con)
		return luaL_error(L, "gnd_t input must be contiguous");
#endif
	double * gnd_tp = (double *)(gnd_t->storage->data + gnd_t->storageOffset);

	/* Get container for iObs */
	const THDoubleTensor * obs_t =
		(THDoubleTensor *) luaT_checkudata(L, 6, "torch.DoubleTensor");
	const long nps_obs  = obs_t->size[0];
	if (nps_obs!=nps)
		luaL_error(L,"Wrong Number of elements in iObs!");
#ifdef DEBUG
	is_con = isContiguous<THDoubleTensor>((THDoubleTensor *)obs_t);
	if (!is_con)
		return luaL_error(L, "obs_t input must be contiguous");
#endif
	double * obs_tp = (double *)(obs_t->storage->data + obs_t->storageOffset);

	//#pragma omp parallel for
	for( i=0; i<nps; i++ ) {

		double counts = *(c_tp + i);
		double zMaxmin = *(mm_tp + i);
		double zMean = *(mean_tp + i);
		double xBin = *(xbin_tp + i);

		if( counts >= 1 && zMaxmin <=0.3 && zMean<(0.3*xBin+0.05-0.93)
				&& zMean>(-0.3*xBin-0.05-0.93) ){
			gndIdx += 1;
			gnd_tp[gndIdx] = i + 1;
		}

		if( counts >= 1 && zMaxmin >0.3 && zMean>(0.4*xBin+0.10-0.93) ){
			obsIdx += 1;
			obs_tp[obsIdx] = i + 1;
		}
	}
	lua_pushinteger(L, gndIdx+1);
	lua_pushinteger(L, obsIdx+1);

	return 2;
}


/* Get the chest lidar points 
	 and find the last free ground point before the first obstacle  */
int lua_find_last_free_point(lua_State *L) {
	static int i = 0;
	static double x=0;

	/* Get the torch that contains in which bin each point belongs to */
	const THDoubleTensor * ps_x =
		(THDoubleTensor *) luaT_checkudata(L, 1, "torch.DoubleTensor");
	const long nps  = ps_x->size[0];
#ifdef DEBUG
	int is_con = isContiguous<THDoubleTensor>((THDoubleTensor *)ps_x);
	if (!is_con)
		return luaL_error(L, "ps_x input must be contiguous");
#endif
	double * ps_xp = (double *)(ps_x->storage->data + ps_x->storageOffset);

	/* Grab the lidar points 
		 const THDoubleTensor * ps_t =
		 (THDoubleTensor *) luaT_checkudata(L, 2, "torch.DoubleTensor");
		 const long nps_t  = ps_t->size[0];*/

	/* Grab the total number of bins */
	const double nbins = luaL_checknumber(L, 2);

	/* Grab the bin that contains the 1st obstacle */
	const double iFirstObs = luaL_checknumber(L, 3);

	/* Grab the safe distance that we want to keep from the obstacle */
	const double safe_distance = luaL_checknumber(L, 4);


	bool not_found_last_point = true;
	int last_point = 1;
	//#pragma omp parallel for
	for( i=0; i<nps; i++ ) {
		/* x = THTensor_fastGet2d( ps_x, nps-i-1, 0 ); */
		x = ps_xp[(nps-i-1) * ps_x->stride[0] + 0];

		if (x<1) {
			/* THTensor_fastSet2d( ps_x, nps-i-1, 0, nbins ); */
			ps_xp[(nps-i-1) * ps_x->stride[0] + 0] = nbins;
		}

		if ((x>iFirstObs-safe_distance)&&(not_found_last_point)){
			last_point = i + 1;
			not_found_last_point = false;
		}
	}
	lua_pushinteger(L,last_point);
	return 1;
}




/* Decay the map around a region from the robot  */
int lua_decay_map(lua_State *L) {
	int i=0, j=0;
	uint8_t val = 0;
	uint8_t * ps_mapp = NULL;

	/* Get the torch that contains the map data */
	const THByteTensor * ps_map =
		(THByteTensor *) luaT_checkudata(L, 1, "torch.ByteTensor");
#ifdef DEBUG
	int is_con = isContiguous<THByteTensor>((THByteTensor *)ps_map);
	if (!is_con)
		return luaL_error(L, "ps_map input must be contiguous");
#endif
	ps_mapp = (uint8_t *)(ps_map->storage->data + ps_map->storageOffset);

	const long nps_x  = ps_map->size[0];
	const long nps_y  = ps_map->size[1];
	
	//fprintf(stdout, "local map stride0: %d \n", uint8_t(ps_map->stride[0]));

	/* Grab the threshold on likelihood */
	const double thres = luaL_checknumber(L, 2);

	/* Grab by how much to deacy the map */
	const double dec = luaL_checknumber(L, 3);

	for( i=0; i<nps_x; i++ ) {
		for ( j=0; j<nps_y; j++){
			val = *(ps_mapp + i * nps_y + j);

			/* If super certain: remain at high level */
			if ( val>=thres ){
				ps_mapp[i * nps_y + j] = thres;
				// *(ps_mapp+j) = thres;
			}

			/* Decay */
			else {
				val = val * dec;
				ps_mapp[i * nps_y + j] = val;
				// *(ps_mapp+j) = val;
			}
		}
	}

	return 0;
}


/**************
 * range_filter(key,min,max,value1,value2)
 * This function removes, in memory, points outside of the min/max band
 * Removes the points from both value1 and value2, but not key
 * Does not perform a resize
 * */
int lua_range_filter(lua_State *L) {
	THFloatTensor * k_t = 
		(THFloatTensor *) luaT_checkudata(L, 1, "torch.FloatTensor");
	THArgCheck(k_t->nDimension == 1, 1, "Ranges should be 1 dimensional");

	// The key must lie within min and max
	const double min = luaL_checknumber(L,2);
	const double max = luaL_checknumber(L,3);

	/* 
	Grab the Value tensors
	Check that Key and Value tensors have the same number of elements 
	*/
	THDoubleTensor * v1_t = 
		(THDoubleTensor *) luaT_checkudata(L, 4, "torch.DoubleTensor");
	THArgCheck(v1_t->nDimension == 1, 4, "Data1 should be 1 dimensional");
	const long nKeys = k_t->size[0];
	const long nData1 = v1_t->size[0];
	if(nKeys!=nData1)
		return luaL_error(L, "Keys and Data1 dimension mismatch");

	THDoubleTensor * v2_t = 
		(THDoubleTensor *) luaT_checkudata(L, 5, "torch.DoubleTensor");
	THArgCheck(v2_t->nDimension == 1, 5, "Data2 should be 1 dimensional");
	const long nData2 = v2_t->size[0];
	if(nKeys!=nData2)
		return luaL_error(L, "Keys and Data2 dimension mismatch");

	// Filter points with range outside of band
	float* k_ptr = k_t->storage->data + k_t->storageOffset;
	double* v1_ptr = v1_t->storage->data + v1_t->storageOffset;
	long v1_stride = v1_t->stride[0];
	double* v2_ptr = v2_t->storage->data + v2_t->storageOffset;
	long v2_stride = v2_t->stride[0];
	float tmpR;
	int i,curIndex;
	for(i = 0, curIndex = 0; i<nKeys; i++){
		tmpR = *(k_ptr + i);
		if(tmpR>=min && tmpR<=max) {
			//printf("%lf %lf %lf\n",key,min,max);
			//fflush(stdout);
			if(i!=curIndex){
				*(v1_ptr+curIndex*v1_stride) = *(v1_ptr + i*v1_stride );
				*(v2_ptr+curIndex*v2_stride) = *(v2_ptr + i*v2_stride );
			}
			curIndex++;
		}
	}
	
	// Return the new size of the array, without resizing
	lua_pushinteger(L,curIndex+1);
	return 1;
}

/************************************************************************/



static const struct luaL_Reg slam_lib [] = {
	{"set_boundaries", lua_set_boundaries},	
	{"set_resolution", lua_set_resolution},
	{"match", lua_match},
	{"update_map", lua_update_map},
	{"binStats", lua_binStats},
	{"update_hmap", lua_update_hmap},
	{"update_smap", lua_update_smap},
	{"get_ground_points", lua_get_ground_points},
	{"get_height_points", lua_get_height_points},
	{"get_last_free_point",lua_find_last_free_point},
	{"mask_points", lua_mask_points},
	{"decay_map", lua_decay_map},
	{"range_filter", lua_range_filter},
	{NULL, NULL}
};


#ifdef __cplusplus
extern "C"
#endif


int luaopen_slam (lua_State *L) {
#if LUA_VERSION_NUM == 502
	luaL_newlib(L, slam_lib);
#else
	luaL_register(L, "slam", slam_lib);
#endif
	return 1;
}

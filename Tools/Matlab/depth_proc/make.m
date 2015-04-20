dbclear all;

mex('mexFitPlane_uv1d.cpp','fitPlane.cpp','-I/usr/local/include/eigen3');
disp('====== mexFitPlane_uv1d done');

mex('mexComputeGeometry.cpp','computeGeo.cpp', '-I/usr/local/include/eigen3');
disp('====== mexComputeGeometry done');

dbclear all;

mex('mexFitPlane_uv1d.cpp','fitPlane.cpp','-I/usr/include/eigen3');

disp('====== mexFitPlane_uv1d done');

if 0
mex('mexComputeGeometry.cpp','computeGeo.cpp',...
    '-I/usr/include/eigen3');

disp('====== mexComputeGeometry done');
end

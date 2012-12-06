require('Slam');

for k,v in pairs(Slam) do
  print(k,v);
end

-- state = Slam.binStats(xClip./xBinLength, zClip, xBinMax);
-- state, bins  = Slam.binStats(xClip./xBinLength, zClip, xBinMax);
--
-- eix, eiy = Slam.getMapCellsFromRay(xl,yl,xis(indGood),yis(indGood));
--
-- h_trans = HoughTransformAPI(xsh,ysh,a_center,a_range,a_res,r_center, r_range, r_res);
-- h_trans, angles = HoughTransformAPI(xsh,ysh,a_center,a_range,a_res,r_center, r_range, r_res);
-- h_trans, angles, rhos = HoughTransformAPI(xsh,ysh,a_center,a_range,a_res,r_center, r_range, r_res);
-- h_trans = HoughTransformAPI(xsh,ysh);

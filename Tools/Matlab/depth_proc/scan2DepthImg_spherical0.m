function [ Scans_, X_, Y_ ] = scan2DepthImg_spherical0( Scans, azi, scanAng )
  
    nzElem0 = find(sum(Scans,1) > 0);
    azi_ = azi(nzElem0);
    
    nzElem = find(azi_(2:end)-azi_(1:end-1));
    if nzElem < 10
        Scans_ = [];
        return;
    end
    azi_ = azi_(nzElem); 
    scanAng_ = scanAng(:);%(4:4:end);     
    
    X_ = repmat(azi_,[length(scanAng_) 1]);
    Y_ = repmat(scanAng_,[1 length(nzElem)]);
    
    Scans_ = Scans(:,nzElem0(nzElem));     
    %Scans_ = Scans_(4:4:end,:);  
     
end


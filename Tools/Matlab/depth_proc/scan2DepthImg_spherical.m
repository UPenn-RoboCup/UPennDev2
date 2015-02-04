function [ Scans_, X_, Y_ ] = scan2DepthImg_spherical( Scans, azi, scanAng )
  
    nzElem = find(azi(2:end)-azi(1:end-1));
    if nzElem < 10
        Scans_ = [];
        return;
    end
    azi_ = azi(nzElem); 
    scanAng_ = scanAng(:);%(4:4:end);     
    
    X_ = repmat(azi_,[length(scanAng_) 1]);
    Y_ = repmat(scanAng_,[1 length(nzElem)]);
    
    Scans_ = Scans(:,nzElem);     
    %Scans_ = Scans_(4:4:end,:);  
     
end


function [finalMean,clusterXYcell,nMembers] = sphericalMeanShiftxyB(xy, normals, bandWidth, weights, init_pts)
% MeanShift Clustering using a flat kernel
% 
% <input>
% xy                         - spatial data 
% normals                    - input directional data, ([ dim x numPtsOfInterest ])
% bandWidth                  - bandwidth parameter (scalar)

 
% <output>
% finalMean         - is locations of cluster centers (numDim x numClust)
% data2cluster      - for every data point which cluster it belongs to (numPts)
% cluster2dataCell  - for every cluster which points are in it (numClust)
% 
% by Bhoram Lee 01/20/14
%
% Reference
% Comaniciu, D.; Meer, P., "Mean shift: a robust approach toward feature
% space analysis," PAMI, IEEE Transactions on, 24(5), pp.603,619, May 2002.


nPts = size(normals,2);
if (size(xy,2) ~= nPts)
    error('Dimension mismatch!');
end

nClust        = 0;        
finalMean = [];   
clusterXYcell = [];
nMembers = [];
initPtIdx  = [1:nPts];
numInitPts      = nPts;               
visitedFlag = zeros(1,nPts); 
votes    = zeros(1,nPts);   
ite1 = 0;
Ninit_pts = 0;
if nargin > 4
    if ~isempty(init_pts)
        Ninit_pts = size(init_pts,2);        
    end
end

while numInitPts > 0
    if Ninit_pts > 0
        myMean = init_pts(:,Ninit_pts);
        Ninit_pts = Ninit_pts - 1;
    else
        tempInd         = ceil( (numInitPts-1e-6)*rand);        %pick a random seed point
        stInd           = initPtIdx(tempInd);                 
        myMean          = [xy(:,stInd); normals(:,stInd)];   
    end
    myMean(3:5)          = myMean(3:5)/norm(myMean(3:5));
    myMembers       = [];                                                          
    thisClusterVotes    = zeros(1,nPts);        

    ite1 = ite1 + 1;
    ite2 = 0;
    prevdel = -1;
    while 1     %loop untill convergence
              
        sqDistToAll = weights(1)*sum((xy-repmat(myMean(1:2),1,nPts)).^2)'  + weights(2)*acos(abs(normals'*myMean(3:5)));
        %sqDistToAll = sum((repmat(myMean,1,nPts) - normals).^2);  
        inInds      = find(sqDistToAll < bandWidth^2);              
       
        if isempty(inInds)   
            %disp('empty');
            break;
        end
        
        thisClusterVotes(inInds) = thisClusterVotes(inInds)+1;  
                
        myOldMean   = myMean;                        
        myMean(1:2) = mean(xy(:,inInds),2);
        myMean(3:5)      = mean(normals(:,inInds),2);        
        myMean(3:5)          = myMean(3:5)/norm(myMean(3:5));
        myMembers   = [myMembers; inInds];                       
        visitedFlag(myMembers) = 1;     
        
        del = (weights(1)*norm(myMean(1:2)-myOldMean(1:2)) + weights(2)*acos(abs(myMean(3:5)'*myOldMean(3:5))));
        if isnan(del)   
            disp('Nan');
            break;
        elseif del < 0.5*bandWidth || (ite2 > 100 && abs(prevdel - del) < 1e-6)
            
            %check for merge posibilities
            mergeWith = 0;
            for cN = 1:nClust
                distToOther = weights(1)*norm(myMean(1:2)-finalMean(1:2,cN)) + weights(2)*acos(abs(myMean(3:5)'*finalMean(3:5,cN)));     
                if distToOther < bandWidth/2                    
                    mergeWith = cN;
                    break;
                end
            end            
            
            if mergeWith > 0    
                finalMean(:,mergeWith)       = 0.5*(myMean + finalMean(:,mergeWith));       
                finalMean(3:5,mergeWith)       = finalMean(3:5,mergeWith)/norm(finalMean(3:5,mergeWith));
                votes(mergeWith,:)    = votes(mergeWith,:) + thisClusterVotes;    
            else    
                nClust                    = nClust+1 ;            
                finalMean(:,nClust)       = myMean;                     
                votes(nClust,:)    = thisClusterVotes;
            end

            break;
        end
        
        ite2 = ite2 + 1;
        prevdel = del;
        
        if ite2 > 1000
            finalMean = [];
            return;
        end
    end    
    
    initPtIdx      = find(visitedFlag == 0);           
    numInitPts      = length(initPtIdx);                 

end

[val,data2cluster] = max(votes,[],1);              
clusterXYcell = cell(nClust,1);
for cN = 1:nClust
    myMembers = find(data2cluster == cN);
    nMembers(cN) = length(myMembers);
    clusterXYcell{cN} = [myMembers];
end




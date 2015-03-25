function [A, S] = computeNormal_lidar(X0, Y0, Z0, mask, wnd,stepsz)

w = size(mask,2);
h = size(mask,1);
A = zeros(4,w*h);
S = zeros(4,w*h);
for k=(wnd+1):1:(w-wnd)
    for j=(stepsz*wnd+1):stepsz:(h-stepsz*wnd)
        
        X1_ = (j-stepsz*wnd):stepsz:(j+stepsz*wnd);
        X2_  = (k-wnd):(k+wnd);
        
        nbhood = mask(X1_,X2_);
        if ~sum(sum( ~(nbhood & 1)))
            
            X = reshape(X0(X1_,X2_),[], 1);
            Y = reshape(Y0(X1_,X2_),[], 1);
            Z = reshape(Z0(X1_,X2_),[], 1);
            I = ones(size(X));
            
            H = [X Y Z I];
            [U, s] = svd(H');
            
            u = U(:,4);
            u = u./norm(u(1:3));
            
            A(:,(k-1)*h+j) = u;
            S(:,(k-1)*h+j) = diag(s(1:4,1:4));
        end
    end
end       

end


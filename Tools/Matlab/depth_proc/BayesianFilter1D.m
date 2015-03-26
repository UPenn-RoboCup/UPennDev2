% by Bhoram Lee
% Apr 2014
% modified Mar 2015
classdef BayesianFilter1D
    
    properties
        x       % scalar state       
        P 
    end
    
    methods
        function fp = initialize(fp, x0, sigx2)
            fp.x = x0;
            fp.P = sigx2; 
        end
        
        function [fp, x_post, sig_post] = update(fp, meas)           
            meas.value;
            sigy2 = Sig2_y(meas.param);
            sum_sx_sy = (fp.P + sigy2);
            fp.x = (sigy2*fp.x + fp.P*meas.value)/sum_sx_sy;
            fp.P = sigy2*fp.P/sum_sx_sy;

            x_post = fp.x;
            sig_post = sqrt(fp.P);
        end
        
        function [fp, x_post, sig_post]= propagate(fp, u)
            sigx2 = 0.01;
            if nargin == 2
                fp.x = fp.x + u;
                sigx2 = Sig2_x(u);
            end
                
            fp.P = fp.P + sigx2;
            x_post = fp.x;
            sig_post = sqrt(fp.P);           
        end
    end
    
    methods (Access = private)       
               
        function s = Sig2_y(param)
            s = (1*param + 0.03)^2; % sig ~ linear             
        end
        
        function s = Sig2_x(param)
            s = (1*param + 0.01)^2; % sig ~ linear             
        end
        
    end
    
end


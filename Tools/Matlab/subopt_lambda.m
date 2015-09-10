function [ dlambda ] = subopt_lambda(lambda)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

nNull = size(lambda, 2);

%% Optimization Tuning
% Relative weight of acceleration (vs null space accuracy)
alpha = 1e4;
%alpha = 1e2; % Snaps to the next goal
%alpha = 0; % Instant if possible (Verified)
% Closeness to previous trajectory
epsilon = 0.2;
% Constraint the joints to be close on how many iterations...
% More skips makes the formulation of the problem easier
% Only works with proper acceleration weight
nSkip = 0; % Default on all
%nSkip = 10; % One constraint per second
%nSkip = 3;
nSkip = max(floor(nSkip), 0) + 1;
% TODO: Truncate the path if possible, once the difference is small
% Then further optimization steps just make the path smaller

%% Joint angles on the path
% Number of joints angles in total
nl = numel(lambda);
lambdat = lambda';
lambda0 = lambdat(:);
% Number of trajectory points
np = size(lambda, 1);
clear lambdat;

%% Acceleration matrix
d2 = 2*ones(nl, 1);
% Proper doundary condition (Central difference 2nd order):
%http://www.mathematik.uni-dortmund.de/~kuzmin/cfdintro/lecture4.pdf
d2(1:nNull) = 1;
d2(end-nNull+1:end) = 1;
d1 = ones(nl-nNull, 1);
A0 = diag(-d2);
A1 = diag(d1, nNull);
A = (A0 + A1 + A1');
A = sparse(A);
ATA = A' * A;
clear d1 d2 A0 A1;

%% Optimization Variables
P0 = eye(nl) + alpha * ATA;

%% CVX Solver
%fprintf(1, 'Computing the optimal value of the QCQP and its dual...\n');
tic;
cvx_begin quiet
    cvx_precision low
    %cvx_precision medium
    variable dlambda(nl)
    % This seems faster
    minimize( quad_form(dlambda, P0))
    % This seems slower...
    %minimize( norm( P0sqrt * q - b ) )
    % Keep the first point the same
    % TODO: For lambda, this is not necessary
    dlambda(1:nNull) == lambda0(1:nNull);
    % Last point the same
    dlambda(nl-nNull+1:nl) == lambda0(nl-nNull+1:nl);
    % Keep the paths somewhat close, due to jacobian linearity
    for k = nNull+1 : nSkip*nNull : nl-nNull,
        norm(dlambda(k:k+nNull-1) - lambda0(k:k+nNull-1)) <= epsilon;
    end
cvx_end
toc
cvx_cputime

dlambda = reshape(dlambda, [nNull, np])';

end


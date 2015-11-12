function [ dlambda, dt_opt, opt_val ] = subopt_lambda(lambda, ds)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

nNull = size(lambda, 2);

%% Optimization Tuning
% Relative weight of acceleration (vs null space accuracy)
alpha = 10 * 10 * 1e2 * 0;
beta = 10;

% Closeness to previous trajectory
%epsilon = deg2rad(0.25);
%epsilon = deg2rad(0.5);
%epsilon = deg2rad(1);
epsilon = deg2rad(10);
% Constraint the joints to be close on how many iterations...

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

%% Velocity matrix
v0 = ones(nl-nNull, 1);
v0(1:nNull) = 2;
v1 = zeros(nl, 1);
v1(1:nNull) = -2;
v1(end-nNull+1:end) = 2;
V = diag(v0, nNull) + diag(-flip(v0), -nNull) + diag(v1);
V = 0.5*V;
clear v0 v1;

V = eye(nl) + diag(-1*ones(nl-1,1), -nNull);
V(1,1) = 0;

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

%% CVX Solver
%fprintf(1, 'Computing the optimal value of the QCQP and its dual...\n');
tstart = tic;
cvx_begin
    cvx_solver gurobi
    cvx_precision low
    %cvx_precision medium
    variable dlambda(nl)
    minimize( quad_form(dlambda, eye(nl)) + ...
        beta * quad_form(V * dlambda, eye(nl)) ... %alpha * quad_form(dlambda - lambda0, ATA) + ...
        )
    % Keep the first point the same
    dlambda(1:nNull) == lambda0(1:nNull);
    % Last point the same
    dlambda(nl-nNull+1:nl) == lambda0(nl-nNull+1:nl);
    % Keep the paths somewhat close, due to jacobian linearity
    for k = 2:np-1,
        norm(...
            dlambda((k-1)*nNull+1:k*nNull) - ...
            lambda0((k-1)*nNull+1:k*nNull) ...
            ) <= epsilon ;%* ds(k);
    end
    dtformulate = toc(tstart);
cvx_end
%% Finish the timing
dt_form_n_solve = toc(tstart);
dt_solve = dt_form_n_solve - dtformulate;
dt_cvx = cvx_cputime;

%% Output
dlambda = reshape(dlambda, [nNull, np])';
dt_opt = [dt_solve, dt_cvx, dt_form_n_solve];
opt_val = cvx_optval;

end


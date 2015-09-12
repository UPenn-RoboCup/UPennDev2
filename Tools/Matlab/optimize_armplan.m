function [ qw, dt_opt, opt_val ] = ...
    optimize_armplan(qwPath0, vwPath0, nullPath0, jacobianPath0, qwStar)

%% Tuning
% Relative weight of acceleration (vs null space accuracy)
alpha = 1e3;
%alpha = 1e2; % Snaps to the next goal
%alpha = 0; % Instant if possible (Verified)
% How much to care about the Jtask Jacobian
%beta = 25;
%beta = 1e2;
beta = 1e-3;
% Closeness to previous trajectory
epsilon = deg2rad(10);
% Constraint the joints to be close on how many iterations...
% More skips makes the formulation of the problem easier
% Only works with proper acceleration weight
nSkip = 0; % Default on all
%nSkip = 10; % One constraint per second
%nSkip = 3;
nSkip = max(floor(nSkip), 0) + 1;
% TODO: Truncate the path if possible, once the difference is small
% Then further optimization steps just make the path smaller

%% Reshape the path for optimization
% Number of trajectory points
np = size(qwPath0, 1);
% Number of joints
nq = size(qwPath0, 2);
% Number of joints angles in total
n = numel(qwPath0);
qw0 = reshape(qwPath0', [n, 1]);
vw0 = reshape(vwPath0', [numel(vwPath0), 1]);
qwStar = repmat(qwStar(:), np, 1);

%% Velocity matrix
v0 = ones(n-nq, 1);
v0(1:nq) = 2;
v1 = zeros(n, 1);
v1(1:nq) = -2;
v1(end-nq+1:end) = 2;
V = diag(v0, nq) + diag(-flip(v0), -nq) + diag(v1);
V = 0.5*V;
clear v0 v1;

%% Acceleration matrix
d2 = 2*ones(n, 1);
% Proper doundary condition (Central difference 2nd order):
%http://www.mathematik.uni-dortmund.de/~kuzmin/cfdintro/lecture4.pdf
d2(1:nq) = 1;
d2(end-nq+1:end) = 1;
d1 = ones(n-nq,1);

A0 = diag(-d2);
A1 = diag(d1, nq);
A = (A0 + A1 + A1');
A = sparse(A);
ATA = A' * A;
clear d2 A0 A1;

%% Nullspace
%{
JTJs = cell(np);
for i=1:np
    % MATLAB/torch transpose...
    JTJs{i} = pinv(jacobianPath0{i}') * jacobianPath0{i}';
end
N = sparse(blkdiag(JTJs{:}));
N = eye(size(N, 1)) - N;
%}

% Use the weird null space
N = sparse(blkdiag(nullPath0{:}))';

NTN = N' * N;

%% Jacobians
J = sparse(blkdiag(jacobianPath0{:}))';
%JTJ = J' * J;
JV = J * V;
%VJJV = JV' * JV;

%% CVX Solver
tmpName = [tempname, '.dat'];
diary(tmpName);
tic;
cvx_begin
cvx_precision low
variable qw(n)
%%{
minimize( quad_form(qw - qwStar, NTN) + ...
    alpha * quad_form(qw, ATA) + ...
    beta * norm(JV * qw - vw0)^2 )
%}
%minimize(quad_form(q, ATA))
% Keep the first point the same
qw(1:nq) == qw0(1:nq);
% Last point the same
qw(n-nq+1:n) == qw0(n-nq+1:n);
% Keep the paths somewhat close, due to jacobian linearity
for k = nq+1 : nSkip*nq : n-nq,
    norm(qw(k:k+nq-1) - qw0(k:k+nq-1)) <= epsilon;
end
cvx_end

%% Complete the timing
dt_cvx = cvx_cputime;
dt_tictoc = toc;
diary off;
[~, cmdout] = unix(['grep ', 'Total ', tmpName]);
cmdout = strsplit(cmdout);
dt_solver = str2double(cmdout(7));
clear cmdout;
delete(tmpName);

%% Form again
qw = reshape(qw, [nq, np])';
dt_opt = [dt_solver, dt_cvx, dt_tictoc];
opt_val = cvx_optval;

end
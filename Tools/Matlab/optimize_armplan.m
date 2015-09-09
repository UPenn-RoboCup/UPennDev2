%% Tuning
% Relative weight of acceleration (vs null space accuracy)
alpha = 1e3;
%alpha = 1e2; % Snaps to the next goal
%alpha = 0; % Instant if possible (Verified)
% How much to care about the Jtask Jacobian 
%beta = 25;
beta = 1e2;
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

%% Joint angles on the path
qwPath0 = cell2mat(qwPath);
% Number of trajectory points
np = numel(qwPath);
% Number of joints
nq = size(qwPath{1}, 1);
% Number of joints angles in total
n = numel(qwPath0);
% If given a guess, else the last point
if exist('qWaistArmGuess', 'var')
    qStar = repmat(qWaistArmGuess, np, 1);
else
    qStar = repmat(qwPath{end}, np, 1);
end

%% Task space path
vwPath0 = cell2mat(vwPath);
nt = numel(vwPath0);

%% Velocity matrix
v0 = ones(n-nq, 1);
v0(1:nq) = 2;
v1 = zeros(n, 1);
v1(1:nq) = -2;
v1(end-nq+1:end) = 2;
V = diag(v0, nq) + diag(-flip(v0), -nq) + diag(v1);
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
N = sparse(blkdiag(nulls{:}))';
NTN = N' * N;

%% Jacobians
J = sparse(blkdiag(Js{:}))';
JTJ = J' * J;
JV = J * V;
VJJV = JV' * JV;

%% Remain tidy
%clear N A qPath qGoal NTN ATA;

%% CVX Solver
fprintf(1, 'Computing the optimal value of the QCQP and its dual... ');
cvx_begin
    cvx_precision low
    %cvx_precision medium
    variable q(n)
    dual variables lam1 lam2 %y{np}
    minimize( quad_form(q - qStar, NTN) + ...
        alpha * quad_form(q, ATA) + ...
        beta * norm(JV*q - vwPath0) )
    % Keep the first point the same
    lam1: q(1:nq) == qwPath0(1:nq);
    % Last point the same
    lam2: q(n-nq+1:n) == qwPath0(n-nq+1:n);
    % Keep the paths somewhat close, due to jacobian linearity
    for k = nq+1 : nSkip*nq : n-nq,
    %for k = nq+1 : nq : n,
        norm(q(k:k+nq-1) - qwPath0(k:k+nq-1)) <= epsilon;
    end
cvx_end

%% Form again
q = reshape(q, [nq, np])';
qwPath0 = reshape(qwPath0, [nq, np])';
%% Tuning
% Relative weight of acceleration (vs null space accuracy)
alpha = 1e3;
%alpha = 1e2; % Snaps to the next goal
%alpha = 0; % Instant if possible (Verified)
% Closeness to previous trajectory
epsilon = deg2rad(10);
% Number of joints
nq = 7;
% Constraint the joints to be close on how many iterations...
% More skips makes the formulation of the problem easier
% Only works with proper acceleration weight
%nSkip = 0; % Default on all
%nSkip = 10; % One constraint per second
nSkip = 3;
nSkip = max(floor(nSkip), 0) + 1;
% TODO: Truncate the path if possible, once the difference is small
% Then further optimization steps just make the path smaller

%% Joint angles on the path
qPath0 = cell2mat(qPath);
n = numel(qPath0);
np = n / nq;
qStar = repmat(qGoal, np, 1);

%% Acceleration matrix
d2 = 2*ones(n,1);
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

%% Nullspace
N = sparse(blkdiag(nulls{:}));
% TODO: Just each block times itself?
NTN = N' * N;

%% Optimization Variables
P0 = NTN + alpha * ATA;
q0 = qStar' * NTN;
% NOTE: This constant is probably not needed
%r0 = qStar' * NTN * qStar;
% NOTE: Flip dimensions...
P0 = P0';
q0 = q0';

%% Attempt to avoid the quad_form
% NOTE: Purported to be faster, but it is slower...
% http://cvxr.com/cvx/doc/advanced.html
%b = sqrtm(full(P0)) \ q0;
%[ P0sqrt, p, S  ] = chol( P0 );
%P0sqrt = P0sqrt * S;
% Close to zero:
%norm(sqrtm(full(P0))'*sqrtm(full(P0)) - full(P0), 'fro')
%P0sqrt = sparse(sqrtm(full(P0)));
%b = P0sqrt' \ q0;


%% Remain tidy
%clear N A qPath qGoal NTN ATA;

%% CVX Solver
fprintf(1, 'Computing the optimal value of the QCQP and its dual... ');
cvx_begin
    cvx_precision low
    %cvx_precision medium
    variable q(n)
    dual variables lam1 lam2 %y{np}
    %minimize( quad_form(q, P0) -2 * q0'*q + r0 )
    % This seems faster
    minimize( quad_form(q, P0) -2 * q0'*q )
    % This seems slower...
    %minimize( norm( P0sqrt * q - b ) )
    % Keep the first point the same
    lam1: q(1:nq) == qPath0(1:nq);
    % Last point the same
    lam2: q(n-nq+1:n) == qPath0(n-nq+1:n);
    % Keep the paths somewhat close, due to jacobian linearity
    for k = nq+1 : nSkip*nq : n-nq,
    %for k = nq+1 : nq : n,
        norm(q(k:k+nq-1) - qPath0(k:k+nq-1)) <= epsilon;
    end

% TODO: Keep the difference in human space close...
%lam1: quad_form(q, NTN) + q1'*q + r1 <= epsilon;
cvx_end

% obj1 = cvx_optval;
%
% P_lam = P0 + lam1;
% q_lam = q0 + lam1 * q1;
% r_lam = r0 + lam1 * r1;
%
% obj2 = -0.5*q_lam'*inv(P_lam)*q_lam + r_lam;
%
fprintf(1,'Done! \n');
show_armplan;
%
% % Displaying results
% disp('------------------------------------------------------------------------');
% disp('The duality gap is equal to ');
% disp(obj1-obj2);

%clear P0 q0 r0;
%clear q1 r1;
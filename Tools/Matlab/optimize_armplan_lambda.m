%% Tuning
% Relative weight of acceleration (vs null space accuracy)
alpha = 1e3;
%alpha = 1e2; % Snaps to the next goal
%alpha = 0; % Instant if possible (Verified)
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
% Number of joints angles in total
n = numel(dlambda0);
% Number of joints
nNull = size(dlambda0, 1);
% Number of trajectory points
np = size(dlambda0, 2);
dlambda0 = dlambda0(:);

%% Acceleration matrix
d2 = 2*ones(n,1);
% Proper doundary condition (Central difference 2nd order):
%http://www.mathematik.uni-dortmund.de/~kuzmin/cfdintro/lecture4.pdf
d2(1:nNull) = 1;
d2(end-nNull+1:end) = 1;
d1 = ones(n-nNull,1);

A0 = diag(-d2);
A1 = diag(d1, nNull);
A = (A0 + A1 + A1');
A = sparse(A);
ATA = A' * A;

%% Optimization Variables
P0 = eye(n) + alpha * ATA;
% NOTE: Flip dimensions...
P0 = P0';

%% Remain tidy
%clear N A qPath qGoal NTN ATA;

%% CVX Solver
fprintf(1, 'Computing the optimal value of the QCQP and its dual... ');
cvx_begin
    cvx_precision low
    %cvx_precision medium
    variable dlambda(n)
    dual variables lam1 lam2 %y{np}
    % This seems faster
    minimize( quad_form(dlambda, P0))
    % This seems slower...
    %minimize( norm( P0sqrt * q - b ) )
    % Keep the first point the same
    lam1: dlambda(1:nNull) == dlambda0(1:nNull);
    % Last point the same
    lam2: dlambda(n-nNull+1:n) == dlambda0(n-nNull+1:n);
    % Keep the paths somewhat close, due to jacobian linearity
    for k = nNull+1 : nSkip*nNull : n-nNull,
        norm(dlambda(k:k+nNull-1) - dlambda0(k:k+nNull-1)) <= epsilon;
    end
cvx_end
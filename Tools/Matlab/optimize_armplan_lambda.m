%% Tuning
% Relative weight of acceleration (vs null space accuracy)
alpha = 1e3;
%alpha = 1e2; % Snaps to the next goal
%alpha = 0; % Instant if possible (Verified)
% Closeness to previous trajectory
epsilon = deg2rad(5);
% Constraint the joints to be close on how many iterations...
% More skips makes the formulation of the problem easier
% Only works with proper acceleration weight
nSkip = 0; % Default on all
%nSkip = 10; % One constraint per second
%nSkip = 3;
nSkip = max(floor(nSkip), 0) + 1;
% TODO: Truncate the path if possible, once the difference is small
% Then further optimization steps just make the path smaller
% Number of null space dimensions
nNull = 1;

%% Filter the nullspace in time
gamma = 1;
Ns = cell(size(nulls));
% Torch comes in transposed!
Ns{1} = nulls{1}';
for i=2:numel(nulls)
    Ns{i} = (1-gamma)*Ns{i-1} + gamma*nulls{i}';
end

%% Save the SVD and form the null space coordinates
Ss = zeros(np, nq);
lambda = zeros(np, nNull);
Vs = cell(np, 1);
Us = cell(np, 1);

for i=1:numel(Ns)
    [U, S, V] = svd(Ns{i});
    Vs{i} = V(:, 1:nNull);
    Us{i} = U(:, 1:nNull);
    Ss(i, :) = diag(S);
    lambda(i, :) = S(1:nNull, 1:nNull) * V(:, 1:nNull)' * (qwPath{i} - qwPath{end});
end

%% Determine the signs from the SVD for consistent basis directions
% NOTE: This should be slow, but not *too* bad ;)
swapidx = [1];
for iN=1:nNull
    for i=2:numel(nulls)
        dirlambda = dot(Vs{i}(:, iN), Vs{i-1}(:, iN));
        if abs(dirlambda) > 0.9 % Pretty much the same dir...
            if dirlambda < 0
                Vs{i}(:, iN) = -Vs{i}(:, iN);
                Us{i}(:, iN) = -Us{i}(:, iN);
                lambda(i, iN) = -lambda(i, iN);
            end
        else
            swapidx = [swapidx, i];
        end
    end
end
clear dirlambda;
swapidx = [swapidx, numel(nulls)];

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
fprintf(1, 'Computing the optimal value of the QCQP and its dual... ');
cvx_begin
    cvx_precision low
    %cvx_precision medium
    variable dlambda(nl)
    dual variables lam1 lam2 %y{np}
    % This seems faster
    minimize( quad_form(dlambda, P0))
    % This seems slower...
    %minimize( norm( P0sqrt * q - b ) )
    % Keep the first point the same
    % TODO: For lambda, this is not necessary
    lam1: dlambda(1:nNull) == lambda0(1:nNull);
    % Last point the same
    %lam2: dlambda(nl-nNull+1:nl) == lambda0(nl-nNull+1:nl);
    % Keep the paths somewhat close, due to jacobian linearity
    for k = nNull+1 : nSkip*nNull : nl-nNull,
        norm(dlambda(k:k+nNull-1) - lambda0(k:k+nNull-1)) <= epsilon;
    end
cvx_end

%% Apply the change in delat back to the original q
dlambda = reshape(dlambda, [nNull, np])';
qLambda = zeros(np, nq);
dqLambda = zeros(np, nq);
%%
for i=1:size(lambda, 1)
    % TODO: Us should be a cell array and then choose nNull columns
    dqLambda(i, :) = Us{i} * (dlambda(i, :) - lambda(i, :))';
    qLambda(i, :) = qwPath{i} + dqLambda(i, :)';
end
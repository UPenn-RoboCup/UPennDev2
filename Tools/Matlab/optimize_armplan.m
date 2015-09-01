% Boyd & Vandenberghe, "Convex Optimization"
% Joëlle Skaf - 08/23/05
%
% Solved a QCQP with 3 inequalities:
%           minimize    1/2 x'*P0*x + q0'*r + r0
%               s.t.    1/2 x'*Pi*x + qi'*r + ri <= 0   for i=1,2,3
% and verifies that strong duality holds.

% Input data
load plan;

% Acceleration matrix
nq = 7;
n = numel(bigQ);
np = n / nq;
A0 = zeros(nq, nq*3);
a = zeros(1, nq*3);
a(1) = -1;
a(nq+1) = 2;
a(2*nq+1) = -1;
for i=1:nq
    A0(i,:) = circshift(a, i-1, 2);
end
A = zeros(size(bigNulls));
for i=1:np-2
    A( (i-1)*nq+1 : i*nq, (i-1)*nq+1 : (i+2)*nq) = A0;
end
A( (np-2)*nq+1 : (np-1)*nq, (np-2)*nq+1 : n ) = A0(:,1:2*nq);
A( (np-1)*nq+1 : n, (np-1)*nq+1 : n ) = A0(:,1:nq);
% Remove temporary matrices
clear A0 a;

NTN = bigNulls' * bigNulls;
ATA = A' * A;
epsilon = deg2rad(5);
%
P0 = NTN + ATA;
q0 = -2 * bigQstar' * NTN;
r0 = bigQstar' * NTN * bigQstar;
%
q1 = -2 * bigQ';
r1 = bigQ' * bigQ;

% Flip dimensions...
P0 = P0';
q0 = q0';
q1 = q1';

fprintf(1,'Computing the optimal value of the QCQP and its dual... ');

cvx_begin
    variable q(n)
    dual variables lam1
    minimize( 0.5*quad_form(q, P0) + q0'*q + r0 )
    % Now this works:
    lam1: (q' * q) + q1'*q + r1 <= epsilon;
cvx_end

obj1 = cvx_optval;

P_lam = P0 + lam1;
q_lam = q0 + lam1 * q1;
r_lam = r0 + lam1 * r1;

obj2 = -0.5*q_lam'*inv(P_lam)*q_lam + r_lam;

fprintf(1,'Done! \n');

% Displaying results
disp('------------------------------------------------------------------------');
disp('The duality gap is equal to ');
disp(obj1-obj2);
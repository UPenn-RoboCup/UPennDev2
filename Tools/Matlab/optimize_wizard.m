% Boyd & Vandenberghe, "Convex Optimization"
% Joëlle Skaf - 08/23/05
%
% Solved a QCQP with 3 inequalities:
%           minimize    1/2 x'*P0*x + q0'*r + r0
%               s.t.    1/2 x'*Pi*x + qi'*r + ri <= 0   for i=1,2,3
% and verifies that strong duality holds.

%% Initialization
% Listen for requests
clear all;
ch = zmq('reply', 'ipc', 'armopt');

%% Tuning
% Relative weight of acceleration (vs null space accuracy)
alpha = 0.5;
% Closeness to previous trajectory
epsilon = deg2rad(5);

%% Acceleration helper
nq = 7;
A0 = zeros(nq, nq*3);
a = zeros(1, nq*3);
a(1) = -1;
a(nq+1) = 2;
a(2*nq+1) = -1;
for i=1:nq
    A0(i,:) = circshift(a, i-1, 2);
end

%% Main loop
while 1
    p = zmq('poll', 1000);
    if numel(p)>0
        %% Load new plan
        tmpfile = zmq('receive', ch);
        tmpfile = char(tmpfile);
        %packed_msg = zmq('receive', ch);
        %msg = msgpack('unpack', packed_msg);
        fprintf(1, 'Optimizing %s\n', tmpfile);
        % Input data
        load(char(tmpfile));
        
        %% Joint angles on the path
        qPath0 = cell2mat(qPath);
        n = numel(qPath0);
        np = n / nq;
        qStar = repmat(qGoal, np, 1);
        
        %% Acceleration matrix
        % NOTE: There is a *much* simpler way to do this
        A = zeros(n, n);
        for i=1:np-2
            A( (i-1)*nq+1 : i*nq, (i-1)*nq+1 : (i+2)*nq) = A0;
        end
        A( (np-2)*nq+1 : (np-1)*nq, (np-2)*nq+1 : n ) = A0(:,1:2*nq);
        A( (np-1)*nq+1 : n, (np-1)*nq+1 : n ) = A0(:,1:nq);
        A = sparse(A);
        ATA = A' * A;
        
        %% Nullspace
        N = sparse(blkdiag(nulls{:}));
        % TODO: Just each block times itself?
        NTN = N' * N;
        
        %% Optimization Variables
        P0 = NTN + alpha * ATA;
        q0 = -2 * qStar' * NTN;
        r0 = qStar' * NTN * qStar;
        % NOTE: Flip dimensions...
        P0 = P0';
        q0 = q0';
        % Cleanup
        clear NTN;
        clear ATA;
        
        %% CVX Solver
        fprintf(1, 'Computing the optimal value of the QCQP and its dual... ');
        cvx_begin
            %cvx_precision low
            cvx_precision medium
            variable q(n)
            dual variables lam1
            minimize( quad_form(q, P0) + q0'*q + r0 )
            % Precomputing:
            %lam1: (q' * q) + q1'*q + r1 <= epsilon;
            % Norm may be faster than precomputing
            lam1: norm(q - qPath0) <= epsilon;
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
        save(tmpfile, 'q');
        y = zmq('send', ch, tmpfile);
        %
        % % Displaying results
        % disp('------------------------------------------------------------------------');
        % disp('The duality gap is equal to ');
        % disp(obj1-obj2);
        
        clear P0 q0 r0;
        %clear q1 r1;
    %else disp('Timeout...');
    end
    
end
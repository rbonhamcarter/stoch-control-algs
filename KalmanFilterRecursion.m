%% Kalman filter recursion

% define system parameters
A = [1 1;
    0 1];
C = [2 1];
sigma{1,1} = eye(2); %indexing is -2 of what it is
W = eye(2);
V = 1;

% Riccati recursion
for i=2:10
    sigma{1,i} = A*sigma{1,i-1}*A' + W - (A*sigma{1,i-1}*C')*inv((C*sigma{1,i-1}*C' + V))*(C*sigma{1,i-1}*A');
end

%Fixed point (convergence at t = 7)
fixed_point = sigma{1,7}

% Confirming fixed point is unique by confirming (A,B) controllable and 
% (A,C) observable 
B = eye(2);
rank_1 = rank(ctrb(A,B))
rank_2 = rank(obsv(A,C))

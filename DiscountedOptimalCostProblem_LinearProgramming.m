%% Discounted Optimal Cost Problem
beta = rand(1); % discount coefficient
alpha = 1/2;

X = [1 2];
U = [1 2];

% transition kernel
P{1,1,1} = 0.1; %x_t+1 = 1, x_t = 1, u_t = 1
P{1,1,2} = 0.5;
P{1,2,1} = 0.8;
P{1,2,2} = 0.2;
P{2,2,2} = 0.8;
P{2,2,1} = 0.2;
P{2,1,2} = 0.5;
P{2,1,1} = 0.9;

%% Linear Programming

% looking to minimize <v,cost^T> over distributions v contained within a
% convex set that can be described by a number of linear constraints

% Cost vector
cost = [CostCompute(1,1);
        CostCompute(1,2);
        CostCompute(2,1);
        CostCompute(2,2)];

% Define the matrice/vector to be used for linear constraints
Aeq = [1-P{1,1,1} 1-P{1,1,2} -P{1,2,1} -P{1,2,2};
       -P{2,1,1} -P{2,1,2} 1-P{2,2,1} 1-P{2,2,2};
       1 1 1 1;
       0 0 0 0];
beq = [0;0;1;0];
lb = zeros(4,1);
ub = [];
A=[];
b=[];

% Use linprog to solve the linear program and find the minimum v
v_min = linprog(cost,A,b,Aeq,beq,lb,ub);
v_min = reshape(v_min,2,2);

% Use v_min to find the optimal_policy(x|u)
optimal_policy = zeros(2,2);
for i=1:2
    for j=1:2
    optimal_policy(i,j) = v_min(j,i)/(v_min(j,1)+v_min(j,2));
    end
end
% display the optimal policy
optimal_policy




%% Discounted Optimal Cost Problem
beta = rand(1); % discount coefficient
alpha = 1/2;

X = [1 2];
U = [1 2];

% initialize vector for containing cost function
c = ones(2,1);

% transition kernel
P{1,1,1} = 0.1; %x_t+1 = 1, x_t = 1, u_t = 1
P{1,1,2} = 0.5;
P{1,2,1} = 0.8;
P{1,2,2} = 0.2;
P{2,2,2} = 0.8;
P{2,2,1} = 0.2;
P{2,1,2} = 0.5;
P{2,1,1} = 0.9;

% set of possible policies
policies{1,1} = [1 1];
policies{1,2} = [1 2];
policies{1,3} = [2 1];
policies{1,4} = [2 2];

%% Policy Iteration
% initial policy
gamma{1,1} = policies{1,1};

% Doing the recursion
T = zeros(4,2);

for k=1:5
    % Finding new W
    c(1) = CostCompute(1,gamma{1,k}(1));
    c(2) = CostCompute(2,gamma{1,k}(2));
    W{1,k} = inv((eye(2,2)-beta*P_gamma(gamma{1,k}, P)))*c;
    % Finding new gamma
    for i=1:4
        c(1) = CostCompute(1,policies{1,i}(1));
        c(2) = CostCompute(2,policies{1,i}(2));
        T(i,:) = c + beta*P_gamma(policies{1,i}, P)*W{1,k};
    end
    [minT1, argmin1] = min(T(:,1));
    [minT2, argmin2] = min(T(:,2));
    gamma{1,k+1} = [argmin1, argmin2];
end
% Convergence at k=3
fixed_point_policy_iteration = gamma{1,3};

%% Value Iteration

% set initial value
v{1,1} = zeros(2,1);

% initial policy
gamma_value{1,1} = policies{1,1};

% Doing the recusion
T_value = zeros(4,2);

for k=1:5
    % Finding new value
    c(1) = CostCompute(1,gamma_value{1,k}(1));
    c(2) = CostCompute(2,gamma_value{1,k}(2));

    for i=1:4
        c(1) = CostCompute(1,policies{1,i}(1));
        c(2) = CostCompute(2,policies{1,i}(2));
        T_value(i,:) = c + beta*P_gamma(policies{1,i}, P)*v{1,k};
    end
    [minT1, argmin1] = min(T_value(:,1));
    [minT2, argmin2] = min(T_value(:,2));
    gamma_value{1,k+1} = [argmin1, argmin2];
    v{1,k+1} = [minT1; minT2];
end
% Convergence at k=3
fixed_point_value_iteration = gamma_value{1,3};

%% Q-Learning
% specify number of iterations
iterations = 5000;
% define arbitrary intial Q_0 and intialise other matrices
for i=1:iterations
    Q{1,i} = zeros(2,2);
end

% initize vectors for storing state and control values over time
x_t = zeros(1,iterations+1);
u_t = zeros(1,iterations+1);
% define arbitrary initial x and u
x_t(1,1) = randi(X);
u_t(1,1) = randi(U);
% initialize alpha as a matrix for storing the coefficient values of each 
% possible choice of x and u
% initilise cost matrix
cost = zeros(2,2);
cost(1,1) = CostCompute(1,1);
cost(1,2) = CostCompute(1,2);
cost(2,1) = CostCompute(2,1);
cost(2,2) = CostCompute(2,2);
% initialize control action vector
v_min = ones(1,iterations+1);

for i=1:size(X)
    for j=1:size(U)
        for k=1:iterations
            x = X(i);
            x_t(1,k) = x;
            u = v_min(1,k);
            u_t(1,k) = u;
            % find coefficients
            alpha_coeff{1,k}(x,u) = alphaCompute(k,x_t,u_t,x,u); 
            % next state value
            x_t(1,k+1) = randsrc(1,1,[[1 2];[P{:,x_t(1,k),u_t(1,k)}]]);
            % minimize over the control actions
            [min_over_u(x,u), v_min(1,k+1)] = min(Q{1,k}(x_t(1,k+1),:));
            Q{1,k+1}(x,u) = Q{1,k}(x,u) + alpha_coeff{1,k}(x,u)...
                *(cost(x,u)+ beta*min_over_u(x,u) - Q{1,k}(x,u));
        end
    end
end

% Q_k converges to a fixed point by k=5 for all numerical simulations
% Perform final minimization over U to determine the optimal policy
[min1, optimal_u1] = min(Q{1,iterations}(1,:));
[min2, optimal_u2] = min(Q{1,iterations}(2,:));
Q_learning_optimal_policy = [optimal_u1; optimal_u2]

function [ alpha ] = alphaCompute(t,x_t,u_t,x,u)
% Computes the coefficients in the Q-learning algorithm for a given time t,
% matrix of states from time 0 to time t x_t, and matrix of control 
% actions from time 0 to time t u_t, the state x and the 
% control action u correspoding to the coefficient
% Uses the most commonly used choice of coefficient equations
count = 0;
for k=1:t+1
    if x_t(k) == x && u_t(k) == u
        count = count+1;
    end
end
alpha = 1/(1+count);
end


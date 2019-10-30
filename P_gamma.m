function [ P_gamma ] = P_gamma( policy, P )
temp = ones(2,2);
for i=1:2
    for j=1:2
        temp(i,j) = P{i,j,policy(j)};
    end
end
P_gamma = temp';
end


function [c] = CostCompute(x,u)
    alpha = 1/2;
    if x==2 && u==2
        c = -1 + alpha*(u - 1);
    else
        c = alpha*(u - 1);
    end
end


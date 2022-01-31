function [derivative] = getDerivative(delta_x,delta_y)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    if (delta_x == 0) && (delta_y >= 0)
        derivative = 10000;
    elseif (delta_x == 0) && (delta_y < 0)
        derivative = -10000;
    else
        derivative = delta_y/delta_x;
    end
    
    % need to limit derivative????
end


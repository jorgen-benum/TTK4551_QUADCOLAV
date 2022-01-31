function [derivative] = getDerivative(delta_x,delta_y)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    if (delta_x == 0) && (delta_y >= 0)
        derivative = 10000;
    elseif (delta_x == 0) && (delta_y < 0)
        derivative = -10000;
    else
        unsat_derivative = delta_y/delta_x;
        
        if unsat_derivative > 10000
            derivative = 10000;
        elseif unsat_derivative < -10000
            derivative = -10000;
        else
            derivative = unsat_derivative;
        end
    end
    
end


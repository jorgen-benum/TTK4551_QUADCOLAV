function [soft_angle, soft_delta, soft_b] = generateSoftContraints(inflated_vector)
%   Detailed explanation goes here
        
        % angle of hard constraints for the cone
        soft_angle = atan2(inflated_vector(2), inflated_vector(1));
        
        % calculate the derivative of the hard constraint
        soft_delta = getDerivative(inflated_vector(1), inflated_vector(2));
        % caulate b to get correct offset for constraint: b = y - delta*x
        soft_b = 0;
        
        
end


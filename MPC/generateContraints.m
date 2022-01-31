function [hard_angle, hard_delta, hard_b, soft_delta, soft_b] = generateHardContraints(edge_angle, inflated_vector, neighbor_cc_vector)
%take free cone edge, neighbor collision cone mean angle and shortest
%intersection from the collision cone, generate the constraints in terms of
% delta and b in y = delta*x + b
%   Detailed explanation goes here
        
        % vector from neighbor collision cone center/mean to inflated cone start of free cone
        delta_vector = inflated_vector - neighbor_cc_vector
        
        % angle of hard constraints for the cone
        hard_angle = atan2(delta_vector(2), delta_vector(1))
        
        % calculate the derivative of the hard constraint
        hard_delta = getDerivative(delta_vector(1), delta_vector(2))
        % caulate b to get correct offset for constraint: b = y - delta*x
        hard_b = inflated_vector(2) - hard_delta*inflated_vector(1)
        
        % calulate the derivative of the soft contraint
        soft_delta = tan(edge_angle)
        soft_b = 0
        
end


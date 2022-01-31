function [soft_angle, soft_delta, soft_b] = generateSoftContraintsSlack(inflated_vector, start)
%   Detailed explanation goes here
        
        % angle of hard constraints for the cone
        soft_angle = atan2(inflated_vector(2), inflated_vector(1));
        
        % calculate the derivative of the hard constraint
        soft_delta = getDerivative(inflated_vector(1), inflated_vector(2));
        
        soft_b = getSlackSign(soft_angle, start)
        
%         % calulate sign of b in order for slack varables to work in correct
%         % direction 
%         
%         if (-pi/2 <= soft_angle) && (soft_angle <= pi/2)
%             % in quadrant 1 or 4
%             if start == true
%                 soft_b = -1;
%             elseif start == false
%                 soft_b = 1;
%             else
%                 k % crash
%             end
%         elseif (-pi > soft_angle) || (soft_angle > pi)
%             k % crash here due to invalid value of angle
%         else
%             % in quadrant 2 or 4
%             if start == true
%                 soft_b = 1;
%             elseif start == false 
%                 soft_b = -1;
%             else
%                 k % crash
%             end   
%         end
        
        
end


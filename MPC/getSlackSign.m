function [soft_b] = getSlackSign(angle,start)
%   Detailed explanation goes here
        % calulate sign of b in order for slack varables to work in correct
        % direction 
        
        if (-pi/2 <= angle) && (angle <= pi/2)
            % in quadrant 1 or 4
            if start == true
                soft_b = -1;
            elseif start == false
                soft_b = 1;
            else
                k % crash
            end
        elseif (-pi > angle) || (angle > pi)
            k % crash here due to invalid value of angle
        else
            % in quadrant 2 or 3
            if start == true
                soft_b = 1;
            elseif start == false 
                soft_b = -1;
            else
                k % crash
            end   
        end
end


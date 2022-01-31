function [slack_variable] = getSlackVar(angle,derivative,start)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
    
    % Y = x when derivative = 1
    if abs(derivative) > 1
        slack_factor = abs(derivative)
    else
        slack_factor = 1
    end
    
    slack_sign = getSlackSign(angle,start)
    
    slack_variable = slack_factor*slack_sign
        
        
end


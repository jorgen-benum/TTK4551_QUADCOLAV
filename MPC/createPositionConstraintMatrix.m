function [alpha, beta, gamma] = createPositionConstraintMatrix(angle, delta, b, start)
%take angle of constraint, delta, b and the bool indicating if it is start


    if (-pi/2 <= angle) && (angle <= pi/2)
        % in quadrant 1 or 4
        half_section = 1; 
    elseif (-pi > angle) || (angle > pi)
        k % crash here due to invalid value of angle
    else
        % in quadrant 2 or 4
        half_section = 2;
    end
    
    % MPC constraint are implemented:
    % alpha >= beta + gamma
    
    if half_section == 1
        if start == true
            alpha = [ 0 1 0 0 0 0]; % *[x y u v phi theta]
            beta = [ delta 0 0 0 0 0]; % *[x y u v phi theta]
            gamma = b;
        elseif start == false
            alpha = [delta 0 0 0 0 0];% *[x y u v phi theta]
            beta = [0 1 0 0 0 0]; % *[x y u v phi theta]
            gamma = -b;
        else
            k % crash
        end
    elseif half_section == 2
        if start == true
            alpha = [delta 0 0 0 0 0]; %*[x y u v phi theta]
            beta = [0 1 0 0 0 0]; % *[x y u v phi theta]
            gamma = -b;
        elseif start == false
            alpha = [0 1 0 0 0 0];
            beta = [delta 0 0 0 0 0];
            gamma = b;
        else
            k % crash
        end
    end

end


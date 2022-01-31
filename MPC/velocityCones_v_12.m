function [Alpha_h, Beta_h, Gamma_h, Alpha_s, Beta_s, Gamma_s] = velocityCones_v_12(vehiclePose,detection_range, angle_array, map, expand_distance, state, constraints)


%% Parameters
constraints_on = false
max_velocity = sqrt(constraints.x_lower(3)^2 + constraints.x_upper(3)^2)


%% Defenitions

deg2rad = pi/180;
rad2deg = 180/pi;


%% Ray iditification settings

number_of_rays = size(angle_array,2)


%% Get sensor distance data for each ray

% get meansurements [x, y]
intsectionPts = rayIntersection(map,vehiclePose,angle_array(1,1:number_of_rays),detection_range);
% transform to body
intsectionPts_body = intsectionPts - [vehiclePose(1), vehiclePose(2)];


%% Identify collision cones

single_collision_cone = [];
in_collision_cone = false;
collison_cones = [];
cc_number = 1;

for ray_number = 1:number_of_rays
    % check if there is a intersecting ray
    if isnan(intsectionPts_body(ray_number)) == false
        if in_collision_cone == true % continue collision cone
            collison_cones{cc_number}.ray_indexes = [collison_cones{cc_number}.ray_indexes, ray_number];
        else
            in_collision_cone = true; % collision cone start
            collison_cones{cc_number} = CollisionCone;
            collison_cones{cc_number}.ray_indexes = ray_number;
        end
            
    else
        if in_collision_cone == true % go to next collision cone
            cc_number = cc_number + 1;
        end 
        in_collision_cone = false; % not in a collision cone 
    end
    
end

for cone_number = 1:size(collison_cones,2)
   collison_cones{cone_number}.intersection_points = getIntersections(collison_cones{cone_number}, intsectionPts_body);
   collison_cones{cone_number}.shortest_intsec = min(sqrt(collison_cones{cone_number}.intersection_points(:,1).^2 + collison_cones{cone_number}.intersection_points(:,2).^2));
   % length?
end


%% Calculate free cones 

last_cc = max(1,size(collison_cones,2)); %
fcone_number = 1; % 

for ccone_number = 1:last_cc
    free_cones{fcone_number} = FreeCone;
    
    % check if all is blocked (that is all rays are in a single collision cone)
    if (last_cc == 1) && (isempty(collison_cones) == false)
        % first check if a single non-empty exsist
        % then check if all rays are blocked
        if size(collison_cones{1,1}.ray_indexes,2) == number_of_rays
            fprintf( 'all blocked crash into we have a solution implemented \n');
            k % crash by setting undefined varaiable
            % all blocked! % reduce detection-length and max velocity ??
            % change to LQR at current position or a position known to be safe in past?
        end
    end
    
    % Get the outer points of the free cones
    % use previous index. If first index, use last index to complete circle
    if ccone_number ~= 1
        prev_cc = ccone_number-1;
    else
        prev_cc = last_cc;
    end
    if isempty(collison_cones) == false
        free_cones{fcone_number}.start_index = max(collison_cones{prev_cc}.ray_indexes);
        free_cones{fcone_number}.start_mean_index = floor(mean(collison_cones{prev_cc}.ray_indexes));
        free_cones{fcone_number}.start_shortest_intsec = collison_cones{prev_cc}.shortest_intsec;
        
        free_cones{fcone_number}.end_index = min(collison_cones{ccone_number}.ray_indexes);
        free_cones{fcone_number}.end_mean_index = ceil(mean(collison_cones{ccone_number}.ray_indexes));
        free_cones{fcone_number}.end_shortest_intsec = collison_cones{ccone_number}.shortest_intsec;
    else 
        % special case when there is no obsticals
        free_cones{fcone_number}.start_index = 1;
        free_cones{fcone_number}.start_mean_index = 1;
        free_cones{fcone_number}.start_shortest_intsec = nan;
        
        free_cones{fcone_number}.end_index = number_of_rays;
        free_cones{fcone_number}.end_mean_index = number_of_rays;
        free_cones{fcone_number}.end_shortest_intsec = nan;
    end
    
    index_difference = 0
    % check if cone is larger than 180, if so, devide into 2 cones
    if free_cones{fcone_number}.start_index <= free_cones{fcone_number}.end_index
        index_difference = abs(free_cones{fcone_number}.end_index - free_cones{fcone_number}.start_index)
    elseif free_cones{fcone_number}.start_index > free_cones{fcone_number}.end_index
        index_difference = number_of_rays - free_cones{fcone_number}.start_index + free_cones{fcone_number}.end_index
    end
    
    if index_difference > number_of_rays/2
        % find where to devide into 2 sections
%         end_first_cone = floor(mean([free_cones{fcone_number}.start_index
%         free_cones{fcone_number}.end_index])); % bug here
        end_first_cone = free_cones{fcone_number}.start_index + floor(index_difference/2)
        if end_first_cone > number_of_rays
            end_first_cone = end_first_cone - number_of_rays;
        end
        
        start_second_cone = end_first_cone + 1;
        % ensure that the index i still inside the array of rays
        if start_second_cone > number_of_rays
            start_second_cone = start_second_cone - number_of_rays
        end
        end_second_cone = free_cones{fcone_number}.end_index;
        end_mean_second_cone = free_cones{fcone_number}.end_mean_index;
        end_shortest_intsec_second_cone = free_cones{fcone_number}.end_shortest_intsec;
        
        % rewrite end of first sector
        free_cones{fcone_number}.end_index = end_first_cone;
        free_cones{fcone_number}.end_mean_index = end_first_cone;
        free_cones{fcone_number}.end_shortest_intsec = nan;
        % make a flag for this? when end and mean are equal?
        
        % create new sector
        fcone_number = fcone_number + 1;
        free_cones{fcone_number} = FreeCone;
        free_cones{fcone_number}.start_index = start_second_cone;
        free_cones{fcone_number}.start_mean_index = start_second_cone;
        free_cones{fcone_number}.start_shortest_intsec = nan;
        
        free_cones{fcone_number}.end_index = end_second_cone;
        free_cones{fcone_number}.end_mean_index = end_mean_second_cone;
        free_cones{fcone_number}.end_shortest_intsec = end_shortest_intsec_second_cone;
    end
    
    % check for empty cone at the start/end of circle
    % it is only a single possiblity for this to happen for a totaly free
    % circle start index will ve 1 and end index = number of rays
    if (free_cones{fcone_number}.start_index == number_of_rays) && (free_cones{fcone_number}.end_index == 1)
        free_cones{fcone_number}.empty_cone = true;
    end
    
    % check if cone is empty, if so, overwrite it
    if free_cones{fcone_number}.empty_cone == false
        fcone_number = fcone_number + 1; % make a new free cone
    else
        % overwrite the current cone as it is empty
        fprintf( 'overwriting empty cone \n');
    end
    
end

%% Calculate constraints

% last_fc = max(1,size(free_cones,2));
last_fc = size(free_cones,2);

for fcone_number = 1:last_fc
    
    % get the interaction points for the edges of the cone
%     [intsec_start, intsec_mean_start, intsec_end, intsec_mean_end] = getConeEdges(free_cones{fcone_number}, intsectionPts_body)
%     free_cones{fcone_number}.intsec_start = intsec_start
%     free_cones{fcone_number}.intsec_mean_start = intsec_mean_start
%     free_cones{fcone_number}.intsec_end = intsec_end
%     free_cones{fcone_number}.intsec_mean_end = intsec_mean_end
    
    % get angles of edges of the cone
    free_cones{fcone_number}.start_angle = angle_array(free_cones{fcone_number}.start_index);
    free_cones{fcone_number}.start_mean_angle = angle_array(free_cones{fcone_number}.start_mean_index);
    free_cones{fcone_number}.end_angle = angle_array(free_cones{fcone_number}.end_index);
    free_cones{fcone_number}.end_mean_angle = angle_array(free_cones{fcone_number}.end_mean_index);
    
    
    % constraints of start of free cone
    start_shortest_intsec = free_cones{fcone_number}.start_shortest_intsec;
    start_angle = free_cones{fcone_number}.start_angle;
    start_mean_angle = free_cones{fcone_number}.start_mean_angle; 
    
%     safety_distance_start = ((current_velocity)/max_velocity)*start_shortest_intsec
    
    if isnan(free_cones{fcone_number}.start_shortest_intsec) == false
        % calculate vectors to artificial intersections using the shortest distance 
        intsec_vector_start = start_shortest_intsec.*[cos(start_angle); sin(start_angle)];
        inflated_vector_start = inflateObject(intsec_vector_start, expand_distance, true);
%         safety_distance_start = max(((current_velocity)/max_velocity)*start_shortest_intsec, expand_distance)
        safety_distance_start = max(((3)/8)*start_shortest_intsec, expand_distance)
        prev_cc_center_vector = (start_shortest_intsec - safety_distance_start).*[cos(start_mean_angle); sin(start_mean_angle)];

        % generate soft and hard constraint varaibles   
        inflated_vector_lenght_start = sqrt(inflated_vector_start(1)^2 + inflated_vector_start(2)^2)
        inflated_vector_angle_start = atan2(inflated_vector_start(2), inflated_vector_start(1))

        [hard_angle_start, hard_delta_start, hard_b_start] = generateHardContraints(inflated_vector_start, prev_cc_center_vector);
        
        % NB sign of soft_b gives need to be removed from figure to get correct visualization
        [soft_angle_start, soft_delta_start, soft_b_start] = generateSoftContraintsSlackV(inflated_vector_start, true);
        
        figure(1)
        hold on
        plot(inflated_vector_start(1) + vehiclePose(1), inflated_vector_start(2) + vehiclePose(2), '*g')
        plot(prev_cc_center_vector(1) + vehiclePose(1), prev_cc_center_vector(2) + vehiclePose(2), '*b')
        
    else
        soft_angle_start = start_angle;
        soft_delta_start = tanSat(start_angle); % use saturation to avoid too large derivative
        soft_b_start = getSlackVar(soft_angle_start, soft_delta_start, true); % to avoide feasible region disconnection from inital condition
        
        hard_angle_start = soft_angle_start;
        hard_delta_start = soft_delta_start;

        % when splitting a region, add some slack
        hard_b_start = 0.5*getSlackVar(hard_angle_start, hard_delta_start, true) 
    end
    
    
    % constraints of end of free cone
    end_shortest_intsec = free_cones{fcone_number}.end_shortest_intsec;
    end_angle = free_cones{fcone_number}.end_angle;
    end_mean_angle = free_cones{fcone_number}.end_mean_angle;
    
    if isnan(free_cones{fcone_number}.end_shortest_intsec) == false
        % calculate vectors to artificial intersections using the shortest distance
        intsec_vector_end = end_shortest_intsec.*[cos(end_angle); sin(end_angle)];
        inflated_vector_end = inflateObject(intsec_vector_end, expand_distance, false);
%         safety_distance_end = max(((current_velocity)/max_velocity)*end_shortest_intsec, expand_distance)
        safety_distance_end = max(((3)/8)*end_shortest_intsec, expand_distance)
        post_cc_center_vector = (end_shortest_intsec - safety_distance_end).*[cos(end_mean_angle); sin(end_mean_angle)];
        
        % generate soft and hard constraint varaibles

        [hard_angle_end, hard_delta_end, hard_b_end] = generateHardContraints(inflated_vector_end, post_cc_center_vector);
         
        % NB sign of soft_b gives need to be removed from figure to get correct visualization
        [soft_angle_end, soft_delta_end, soft_b_end] = generateSoftContraintsSlackV(inflated_vector_end, false);
        
        figure(1)
        hold on
        plot(inflated_vector_end(1) + vehiclePose(1), inflated_vector_end(2) + vehiclePose(2), '*g')
        plot(post_cc_center_vector(1) + vehiclePose(1), post_cc_center_vector(2) + vehiclePose(2), '*b')
        
    else
        soft_angle_end = end_angle;
        soft_delta_end = tanSat(end_angle); % use saturation to avoid too large derivative
%         soft_b_end = 0;
        soft_b_end = getSlackVar(soft_angle_end,soft_delta_end,false) % to avoide feasible region disconnection from inital condition
        
        hard_angle_end = soft_angle_end;
        hard_delta_end = soft_delta_end;

        % to avoide feasible region disconnection from inital condition
        % when splitting a region add some slack
        hard_b_end = 0.5*getSlackVar(hard_angle_end, hard_delta_end, false)
    end
    
    x_var = linspace(-10, 10, 4);
    y_start_soft = soft_delta_start*x_var + soft_b_start;
    y_start_hard = hard_delta_start*x_var + hard_b_start;
    y_end_soft = soft_delta_end*x_var + soft_b_end;
    y_end_hard = hard_delta_end*x_var + hard_b_end;
    
    if constraints_on == true
        figure(1)
        hold on
    %     plot(x_var + vehiclePose(1), y_start_soft + vehiclePose(2), 'y')
        plot(x_var + vehiclePose(1), y_start_hard + vehiclePose(2), 'y')
    %     plot(x_var + vehiclePose(1), y_end_soft + vehiclePose(2), 'c')
        plot(x_var + vehiclePose(1), y_end_hard + vehiclePose(2), 'c')
    end
    
    % save values 
    free_cones{fcone_number}.hard_angle_start = hard_angle_start;
    free_cones{fcone_number}.hard_delta_start = hard_delta_start;
    free_cones{fcone_number}.hard_b_start = hard_b_start;
    free_cones{fcone_number}.soft_angle_start = soft_angle_start;
    free_cones{fcone_number}.soft_delta_start = soft_delta_start;
    free_cones{fcone_number}.soft_b_start = soft_b_start;
    
    free_cones{fcone_number}.hard_angle_end = hard_angle_end;
    free_cones{fcone_number}.hard_delta_end = hard_delta_end;
    free_cones{fcone_number}.hard_b_end = hard_b_end;
    free_cones{fcone_number}.soft_angle_end = soft_angle_end;
    free_cones{fcone_number}.soft_delta_end = soft_delta_end;
    free_cones{fcone_number}.soft_b_end = soft_b_end;
  
    % replace inf with high number
    
    % Create constraints for optimalization
%     [h_alpha_start, h_beta_start, h_gamma_start] = createConstraintMatrix(hard_angle_start, hard_delta_start, hard_b_start, true);
    [h_alpha_start, h_beta_start, h_gamma_start] = createPositionConstraintMatrix(hard_angle_start, hard_delta_start, hard_b_start, true);
    [s_alpha_start, s_beta_start, s_gamma_start] = createConstraintMatrix(soft_angle_start, soft_delta_start, soft_b_start, true);

%     [h_alpha_end, h_beta_end, h_gamma_end] = createConstraintMatrix(hard_angle_end, hard_delta_end, hard_b_end, false);
    [h_alpha_end, h_beta_end, h_gamma_end] = createPositionConstraintMatrix(hard_angle_end, hard_delta_end, hard_b_end, false);
    [s_alpha_end, s_beta_end, s_gamma_end] = createConstraintMatrix(soft_angle_end, soft_delta_end, soft_b_end, false);
    
    Alpha_h{fcone_number} = [h_alpha_start; h_alpha_end];
    Beta_h{fcone_number} = [h_beta_start; h_beta_end];
    Gamma_h{fcone_number} = [h_gamma_start; h_gamma_end];
    
    Alpha_s{fcone_number} = [s_alpha_start; s_alpha_end];
    Beta_s{fcone_number} = [s_beta_start; s_beta_end];
    Gamma_s{fcone_number} = [s_gamma_start; s_gamma_end];
    
end

%% Plot regions

cond = 0
cond_2 = 0
for i = 1:size(Alpha_h,2)
    
    v = -10:0.2:10;
    [x,y] = meshgrid(v);  % create a grid
    
    if Alpha_h{1,i}(1,4) == 0
        a_s = Alpha_h{1,i}(1,3);
        b_s = Beta_h{1,i}(1,4);
        g_s = Gamma_h{1,i}(1,1);
        ineq = a_s.*x >= y.*b_s + g_s;
    else
        a_s = Alpha_h{1,i}(1,4);
        b_s = Beta_h{1,i}(1,3);
        g_s = Gamma_h{1,i}(1,1);
        ineq = a_s.*y >= x.*b_s + g_s;    % some inequality
    end
    
    if Alpha_h{1,i}(2,3) == 0
        a_e = Alpha_h{1,i}(2,4);
        b_e = Beta_h{1,i}(2,3);
        g_e = Gamma_h{1,i}(2,1);
        ineq2 = y.*a_e >= x.*b_e + g_e;
    else
        a_e = Alpha_h{1,i}(2,3);
        b_e = Beta_h{1,i}(2,4);
        g_e = Gamma_h{1,i}(2,1);
        ineq2 = x.*a_e >= y.*b_e + g_e;
    end
    start = double(ineq);
    end_ = double(ineq2);

    cond = start.*end_ + cond;
    
    figure(21)
    surf(x,y,cond);
    view(0,90);   

    % plot soft constraints
%     v = -5:0.1:5;
    [x,y] = meshgrid(v);  % create a grid
    
    if Alpha_s{1,i}(1,4) == 0
        a_s = Alpha_s{1,i}(1,3);
        b_s = Beta_s{1,i}(1,4);
        g_s = Gamma_s{1,i}(1,1);
%         ineq3 = a_s*x >= y*b_s + g_s;
        ineq3 = a_s*x >= y*b_s;
    else
        a_s = Alpha_s{1,i}(1,4);
        b_s = Beta_s{1,i}(1,3);
        g_s = Gamma_s{1,i}(1,1);
%         ineq3 = a_s*y >= x*b_s + g_s;    % some inequality
        ineq3 = a_s*y >= x*b_s;
    end
    
    if Alpha_s{1,i}(2,3) == 0
        a_e = Alpha_s{1,i}(2,4);
        b_e = Beta_s{1,i}(2,3);
        g_e = Gamma_s{1,i}(2,1);
%         ineq4 = y*a_e >= x*b_e + g_e;
        ineq4 = y*a_e >= x*b_e;
    else
        a_e = Alpha_s{1,i}(2,3);
        b_e = Beta_s{1,i}(2,4);
        g_e = Gamma_s{1,i}(2,1);
%         ineq4 = x*a_e >= y*b_e + g_e;
        ineq4 = x*a_e >= y*b_e;
    end
    start_2 = double(ineq3);
    end_2 = double(ineq4);

    cond_2 = start_2.*end_2 + cond_2;
    

end

% plot each open region, where overlapping regions are defined by different hight
figure(21)
title('hard contraints')
surf(x,y,cond);
view(0,90)

figure(22)
title('soft contraints')
surf(x,y,cond_2);
view(0,90)

figure(23)
surf(x,y,cond + cond_2);
view(0,90)



%% Plot intersection with objects

% intersection
intsectionPts_plot = intsectionPts_body + vehiclePose(1:2);

figure(1)
hold on
plot(intsectionPts_plot(:,1),intsectionPts_plot(:,2),'*r') % Intersection points
plot(vehiclePose(1),vehiclePose(2),'ob') % Vehicle pose
for i = 1:number_of_rays
    if (ceil(number_of_rays/2) <= i)
        plot([vehiclePose(1),intsectionPts_plot(i,1)],...
        [vehiclePose(2),intsectionPts_plot(i,2)],'-b') % Plot intersecting rays
    else
        plot([vehiclePose(1),intsectionPts_plot(i,1)],...
        [vehiclePose(2),intsectionPts_plot(i,2)],'-g') % Plot intersecting rays
    end
        
end

end
            








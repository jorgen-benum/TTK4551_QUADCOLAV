classdef FreeCone
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        start_index
        start_mean_index
        end_index
        end_mean_index
        empty_cone = false
        
        intsec_start
        intsec_mean_start
        intsec_end
        intsec_mean_end
        
        start_angle
        start_mean_angle
        end_angle
        end_mean_angle
        
        start_shortest_intsec
        end_shortest_intsec
        
        
        hard_angle_start
        hard_delta_start
        hard_b_start
        soft_angle_start
        soft_delta_start
        soft_b_start

        hard_angle_end
        hard_delta_end
        hard_b_end
        soft_angle_end
        soft_delta_end
        soft_b_end
%         cone_start_intersection
%         cone_end_intersection
%         intersection_points
%         rayes
%         ray_indexes
%         contain_first_ray = false
%         contain_last_ray = false
    end
    
%     methods
%         function obj = getIntersections(obj, intersection_array)
%             %UNTITLED2 Construct an instance of this class
%             %   Detailed explanation goes here
%             
%             minimum = min(obj.ray_indexes)
%             maximum = max(obj.ray_indexes)
%             obj.intersection_points = intersection_array(minimum:maximum, :);
%             
%         end
% %         
%         function obj = set.intersection_points(obj,inputArg)
%             obj.intersection_points = inputArg
%  
%         end
%     end
end


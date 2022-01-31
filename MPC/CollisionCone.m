classdef CollisionCone
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        cone_start_angle
        cone_end_angle
        cone_start_intersection
        cone_end_intersection
        intersection_points
        rayes
        ray_indexes
        contain_first_ray = false
        contain_last_ray = false
        shortest_intsec
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


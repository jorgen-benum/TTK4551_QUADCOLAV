function [intersection_points_infObsticals] = inflateObstical(intersection_points, distance)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    % for all rays reduce the ray intersection point with the inflate
    % distance
    for i = 1:size(intersection_points,1)
        if isnan(intersection_points(i,1)) == false
            vector_angle = atan2(intersection_points(i,2), intersection_points(i,1));
            inflate_vector = distance * [cos(vector_angle+pi);sin(vector_angle+pi)];
            intersection_points(i,:) = intersection_points(i,:) + inflate_vector';
        end
    end
    
    
    intersection_points_infObsticals = intersection_points;
end


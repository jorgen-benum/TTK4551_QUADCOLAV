function [inter_sections_cc] = getIntersections(collision_cone,intersections_points)
%take a collision cone and a array of intersections points return the
%intersection point for the cone
%   Detailed explanation goes here
maximum = max(collision_cone.ray_indexes);
minimum = min(collision_cone.ray_indexes);

inter_sections_cc = intersections_points(minimum:maximum, :);
end


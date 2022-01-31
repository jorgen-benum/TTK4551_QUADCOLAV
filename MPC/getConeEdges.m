function [intsec_start, intsec_mean_start, intsec_end, intsec_mean_end] = getConeEdges(free_cone,intersections_points)
%take a frre cone and intersection points and return the value of all egde
%indexes spesified
%   Detailed explanation goes here

    intsec_start = intersections_points(free_cone.start_index,:);
    intsec_mean_start = intersections_points(free_cone.start_mean_index,:);
    intsec_end = intersections_points(free_cone.end_index,:);
    intsec_mean_end = intersections_points(free_cone.end_mean_index,:);

end


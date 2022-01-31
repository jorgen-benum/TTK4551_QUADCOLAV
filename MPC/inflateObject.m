function [inflated_intersection_vector] = inflateObject(intersection_body_vector, distance, start_vector)
%UNTITLED Summary of this function goes here
% take the intersection point in the body and move it x meters normal to
% the vector to inflate the object

    % find the angle in body frame
    vector_angle = atan2(intersection_body_vector(2),intersection_body_vector(1));
    
    % start or end vector of free cone
    if start_vector == true
        inflate_direction = vector_angle + pi/2;
    else 
        inflate_direction = vector_angle - pi/2;
    end
    
    inflate_vector = distance * [cos(inflate_direction);sin(inflate_direction)];
    
    inflated_intersection_vector = intersection_body_vector + inflate_vector;
    
end


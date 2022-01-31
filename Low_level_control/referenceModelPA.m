function [x_refmod_dot] = referenceModelPA(reference, zeta_ref, wn_ref, x_refmod)
% refernce model for attitude and position
% take the reference, reference model parameters and the reference model state
% reference
% xref = reference model state
% calculate the desired position, velocity and acceleration

Ad = [ 0                        1                    0;
       0                        0                    1;
     -wn_ref^3  -(2*zeta_ref + 1)*wn_ref^2  -(2*zeta_ref+1)*wn_ref ];

Bd = [ 0 0 wn_ref^3 ]';
    
x_refmod_dot = Ad * x_refmod + Bd * reference;
end


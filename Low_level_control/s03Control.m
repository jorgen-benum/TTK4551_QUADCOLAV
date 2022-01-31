function [M_vector] = s03Control(roll_pitch_yaw_d, roll_pitch_yaw, omega)
%S03 Summary of this function goes here
% omega is the angular velocity in the body fixed frame

% controller gains
% attitude gain
k_R = [1; 1; 0.03];
% k_R = [1.5; 1.5; 0.03];
% angular rate gain 
k_omega = [0.2; 0.2; 0.04];

% inertia tensor
I_xx = 0.01;
I_yy = 0.01;
I_zz = 0.01;

I__b_CG = diag([I_xx, I_yy, I_zz]);

% definitions
phi_d = roll_pitch_yaw_d(1);
theta_d = roll_pitch_yaw_d(2);
psi_d = roll_pitch_yaw_d(3);

% assuming setpoint control, where the MAW should hold a spesific attitude,
% hence desired angular velocity and acceleration is 0
omega_d = [0;0;0];

phi = roll_pitch_yaw(1);
theta = roll_pitch_yaw(2);
psi = roll_pitch_yaw(3);

% R = R__i_b
R_d = Rzyx(phi_d, theta_d, psi_d);
R = Rzyx(phi, theta, psi);

% orientation error
e_R = (1/2)*vex(transpose(R_d)*R - transpose(R)*R_d); 

e_omega = omega - transpose(R)*R_d*omega_d;

% M_vector = [M_x; M_y; M_z] which is 3 of the 4 controll input to thrust
% allocation

% short version for setpoint control
M_vector = -k_R.*e_R - k_omega.*e_omega + cross(omega,(I__b_CG*omega));
end


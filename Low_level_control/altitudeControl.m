function [F_T, dot_integrator_z] = altitudeControl(z_d,z, zdot, integrator_z, roll_pitch_yaw)
% MAV SISO linear PID for altitude control (double integrator system)
% take the desired height, actual height and z velocity and calulate the
% desired thrust

%% Controller gain

omega_n = 10 % natural frequency give omega_bandwith 
zeta = 1 % damping ratio

m = 0.526685 % mass of MAV

K_p = (omega_n^2)*m
K_d = 2*zeta*omega_n*m
K_i = (omega_n/40)*K_p

%%
phi = roll_pitch_yaw(1)
theta = roll_pitch_yaw(2)

%% Gravity compensation feed forward 
g = 9.81 % m/s^2

f_gff = g*m


%% Controller

z_error = z - z_d
dot_integrator_z = z_error

tau_z = -(K_p*z_error + K_d*zdot + K_i*integrator_z) 
% tau_z = -(K_p*z_error + K_d*zdot) 

% F_T = f_gff + tau_z

% controller with nonlinearity compensation
F_T = (f_gff + tau_z)/(cos(phi)*cos(theta))

end


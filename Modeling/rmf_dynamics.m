function [xdot, f__b_Tall] = rmf_dynamics(t, x,tau)
% RMF_DYNAMICS returns the time derivative of the state vector
% x = [p__i_ib; theta_ib; v__b_ib; omega__b_ib]

%% input config
% p__i_ib = x(1:3)
% theta_ib = x(4:6) 
% v__b_ib = x(7:9) 
% omega__b_ib = x(10:12)
% 
% eta = [p__i_ib; theta_ib]
% nu = [v__b_ib; omega__b_ib]


x__i = x(1)
y__i = x(2);
z__i = x(3);
phi = x(4);
theta = x(5);
psi = x(6);
u = x(7);
v = x(8);
w = x(9);
p = x(10);
q = x(11);
r = x(12);

f__b_x_aero = 0;
f__b_y_aero = 0;
f__b_z_aero = 0;

f__b_x_ext = 0;
f__b_y_ext = 0;
f__b_z_ext = 0;

%% rmf parameters
%inertia [kg m^2]
I_xx = 0.01;
I_yy = 0.01;
I_zz = 0.01;

% propeller inertia
I__b_p_zz = 0;

%mass [m]
m = 0.526685;

% acceleration due to gravity
g = 9.81;

% motor position [rad]
arm_angle4 = -0.785398;
arm_angle2 = 2.35619;
arm_angle1 = 0.78539;
arm_angle3 = -2.35619;

arm_length1 = 0.1; %[m]
arm_length2 = 0.1;
arm_length3 = 0.1;
arm_length4 = 0.1;

% vector from cg to motors
r__b_bP1 = arm_length1*[cos(arm_angle1); sin(arm_angle1); 0];
r__b_bP2 = arm_length2*[cos(arm_angle2); sin(arm_angle2); 0];
r__b_bP3 = arm_length3*[cos(arm_angle3); sin(arm_angle3); 0];
r__b_bP4 = arm_length4*[cos(arm_angle4); sin(arm_angle4); 0];

r__b_xP1 = r__b_bP1(1)
r__b_yP1 = r__b_bP1(2)
r__b_zP1 = r__b_bP1(3)

r__b_xP2 = r__b_bP2(1)
r__b_yP2 = r__b_bP2(2)
r__b_zP2 = r__b_bP2(3)

r__b_xP3 = r__b_bP3(1)
r__b_yP3 = r__b_bP3(2)
r__b_zP3 = r__b_bP3(3)

r__b_xP4 = r__b_bP4(1)
r__b_yP4 = r__b_bP4(2)
r__b_zP4 = r__b_bP4(3)

% motor constants
% force constant
k_n = 8.54858e-06; %0.02246
% moment constant
k_m = 1.6e-2;

%% Thrust allocation
% tau = TKu
% u_ci = inv(K)*inv(T)*tau
% tau = controller output

T =     [1,         1,         1,         1;
    r__b_bP1(2),  r__b_bP2(2),  r__b_bP3(2),  r__b_bP4(2);
    -r__b_bP1(1), -r__b_bP2(1), -r__b_bP3(1), -r__b_bP4(1);
    -k_m,       k_m,      -k_m,       k_m];

K = diag([k_n, k_n, k_n, k_n]);

% system control input u_ci
u_ci = inv(K)*inv(T)*tau;

% no negative inputs (motors run only on way)
u_ci =[max(u_ci(1),0);
       max(u_ci(2),0);
       max(u_ci(3),0);
       max(u_ci(4),0)];

% saturation of motors
nomial_thrust = g*m/4
max_thrust = nomial_thrust*2 % hover at 50% motor speed
   
u_ci =[min(u_ci(1), max_thrust/k_n);
       min(u_ci(2), max_thrust/k_n);
       min(u_ci(3), max_thrust/k_n);
       min(u_ci(4), max_thrust/k_n)];

% input as motor speed vector
n = sign(u_ci).*sqrt(abs(u_ci));

n_1 = n(1)
n_2 = n(2)
n_3 = n(3)
n_4 = n(4)

% induced force
f__b = K*u_ci;

f__b_T1 = f__b(1);
f__b_T2 = f__b(2);
f__b_T3 = f__b(3);
f__b_T4 = f__b(4);

f__b_Tall = [f__b_T1; f__b_T2; f__b_T3; f__b_T4];

%% Dynamics                                                                                                                                                                                                                                                                                                     (I_xx*p*q - I_yy*p*q)/I_zz - (k_m*(f__b_T1 - f__b_T2 + f__b_T3 - f__b_T4))/I_zz]

xdot = [
w*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - v*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + u*cos(psi)*cos(theta);
v*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - w*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + u*cos(theta)*sin(psi);
w*cos(phi)*cos(theta) - u*sin(theta) + v*cos(theta)*sin(phi);
p + r*cos(phi)*tan(theta) + q*sin(phi)*tan(theta);
q*cos(phi) - r*sin(phi);
(r*cos(phi))/cos(theta) + (q*sin(phi))/cos(theta);
(f__b_x_ext + f__b_x_aero - m*(q*w - r*v) + g*m*sin(theta))/m;
(f__b_y_ext + f__b_y_aero + m*(p*w - r*u) - g*m*cos(theta)*sin(phi))/m;
(f__b_T1 + f__b_T2 + f__b_T3 + f__b_T4 + f__b_z_ext + f__b_z_aero - m*(p*v - q*u) - g*m*cos(phi)*cos(theta))/m;
(f__b_T1*r__b_yP1 + f__b_T2*r__b_yP2 + f__b_T3*r__b_yP3 + f__b_T4*r__b_yP4 - I__b_p_zz*n_1*q + I__b_p_zz*n_2*q - I__b_p_zz*n_3*q + I__b_p_zz*n_4*q)/I_xx + (I_yy*q*r - I_zz*q*r)/I_xx;
- (I_xx*p*r - I_zz*p*r)/I_yy - (f__b_T1*r__b_xP1 + f__b_T2*r__b_xP2 + f__b_T3*r__b_xP3 + f__b_T4*r__b_xP4 - I__b_p_zz*n_1*p + I__b_p_zz*n_2*p - I__b_p_zz*n_3*p + I__b_p_zz*n_4*p)/I_yy;
(I_xx*p*q - I_yy*p*q)/I_zz + ( -f__b_T1*k_m + f__b_T2*k_m - f__b_T3*k_m + f__b_T4*k_m)/I_zz];









end


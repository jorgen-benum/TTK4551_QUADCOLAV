function [controller, constraint_container] = initMPC9_Q4(Hz, N)
%UNTITLED Summary of this function goes here
% N = Predition horizon

%% General variables
deg2rad = pi/180;

%% MPC initialization
deg2rad = pi/180;
rad2deg = 180/pi;


%% Model parameters
g = 9.81;

K_phi = 1;
T_phi = 0.112;
% T_phi = 10;  % works well with slove dynamics?


K_theta = 1;
T_theta = 0.112;
% T_theta = 10; % works well with slove dynamics?

Hz = Hz;
Ts = 1/Hz;

nx = 6; % Number of states
nu = 2; % Number of inputs
ns = 2; % Number of slack variables
nvs = 2; % Number of velocity max slack varables

nr = 2 % Number of inputs (waypoint)

% N = 25; % Predition horizon

%% Model data
A_c = [0  0   1   0   0   0;
       0  0   0   1   0   0;
       0  0   0   0   0   g;
       0  0   0   0   -g   0;
       0  0   0   0   -1/T_phi  0;
       0  0   0   0   0   -1/T_theta];

B_c = [0              0;
       0              0;
       0              0;
       0              0;
       K_phi/T_phi    0;
       0              K_theta/T_theta];

rank(ctrb(A_c,B_c))

% decrete model
A_d = A_c*Ts + eye(6);
B_d = B_c*Ts;

C_p = [1 0 0 0 0 0;
       0 1 0 0 0 0];

C_v = [0 0 1 0 0 0;
       0 0 0 1 0 0];
 
 % state and error weights
Q_p = 200*eye(nr);

Q_v = 0*[1 1];

 % input weights
R = 200*eye(nu);

% Slack weight
Sq_v = 1000*eye(ns);

S_maxVel_W = 22*eye(nvs);
 
 % bounds
 constraint_container = ConstraintContainer;
 
 % states:
 xl = -Inf*ones(nx,1);
 xu = Inf*ones(nx,1);
 

% velocity constraints
 xl(3) = -2;
 xu(3) = 2;
 xl(4) = -2;
 xu(4) = 2;

constraint_container.x_lower = xl;
constraint_container.x_upper = xu;

 % inputs
 
phi_max = 25*deg2rad;
theta_max = 25*deg2rad;
 
ul = [-phi_max; -theta_max];
uu = [phi_max; theta_max];
 
constraint_container.u_lower = ul;
constraint_container.u_upper = uu;
 
% slack
sl = [0; 0];
svl = [0; 0];

%% collsion constraints
% soft
Alpha_s = sdpvar(2,6);
Beta_s = sdpvar(2,6);
Gamma_s = sdpvar(2,1);

% hard
Alpha_h = sdpvar(2,6);
Beta_h = sdpvar(2,6);
Gamma_h = sdpvar(2,1);

%% cost and constraints shapers
% Scaling
scaling = sdpvar(1,1);

% Constraint shift
soft_constraint_distance = sdpvar(1,1);

%% The optimalization problem
u = sdpvar(repmat(nu,1,N),repmat(1,1,N));
x = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
r = sdpvar(repmat(nr,1,N+1),repmat(1,1,N+1));
s = sdpvar(repmat(ns,1,N+1),repmat(1,1,N+1));
s_maxVel = sdpvar(repmat(nvs,1,N+1),repmat(1,1,N+1));

constraints = []
objective = 0;
dropp_number_VO = 0; % number of steps that is unconstrained 
for k = 1:N
    objective = objective + (C_p*x{k}-r{k})'*Q_p*(C_p*x{k}-r{k})/(scaling^2) + (u{k})'*R*u{k} - Q_v*abs(C_v*x{k}) + (s{k}')*Sq_v*s{k} + (s_maxVel{k}')*S_maxVel_W*s_maxVel{k};
    constraints = [constraints, x{k+1} == A_d*x{k}+B_d*u{k}];
    constraints = [constraints, ul <= u{k} <= uu, (xl - [0; 0; s_maxVel{k}; 0; 0]) <= x{k+1} <=( xu + [0; 0; s_maxVel{k}; 0; 0])];
    constraints = [constraints, sl <= s{k}];
    constraints = [constraints, svl <= s_maxVel{k}];
    if k > dropp_number_VO
        constraints = [constraints, Alpha_s*x{k+1} >= Beta_s*x{k+1} + Gamma_s.*s{k} - Gamma_s.*[soft_constraint_distance; soft_constraint_distance]];% tull med ujevnt nummer
        constraints = [constraints, Alpha_h*x{k+1} >= Beta_h*x{k+1} + Gamma_h];
    else
        constraints = [constraints, 0.*Alpha_h*x{k+1} >= 0.*Beta_h*x{k+1} + 0.*Gamma_h];
    end
end
objective = objective + (C_p*x{k}-r{k})'*Q_p*(C_p*x{k}-r{k})/(scaling^2) - Q_v*abs(C_v*x{k}) + (s{k}')*Sq_v*s{k} + (s_maxVel{k}')*S_maxVel_W*s_maxVel{k};

parameters_in = {x{1},[r{:}], Alpha_h, Beta_h, Gamma_h, Alpha_s, Beta_s, Gamma_s, scaling, soft_constraint_distance};
solutions_out = {[u{:}], [x{:}], objective, [s{:}]}; 

controller = optimizer(constraints, objective,sdpsettings('solver','quadprog'),parameters_in,solutions_out);
end


clear all
close all
clc

%% simulate RMF using MPC for navigation and collision avoidence

%% paths
addpath(genpath('C:\Users\jorge\local_repo\TTK4551_QUADCOLAV\Low_level_control'))
addpath(genpath('C:\Users\jorge\local_repo\TTK4551_QUADCOLAV\MPC'))
addpath(genpath('C:\Users\jorge\local_repo\TTK4551_QUADCOLAV\Modeling'))

%% ploting 
plot_square = false;

%% constants
deg2rad = pi/180;
rad2deg = 180/pi;

g = 9.81;

%% parameters

% low level control sample time
Hz_low_level = 100;
Ts = 1/Hz_low_level; % sample time [s]
% high level control sample time
Hz_high_level = 10;
Ts_h = 1/Hz_high_level;

% simulation time
sim_time = 21;
Ns = Hz_low_level*sim_time;

% sensor setting
detection_range = 6; % [m]
resulution = 6; % [deg]

% constraint setting
expand_distance = 0.25;% [m] expansion of obstacle 
soft_constraint_distance = 0.01; % [m] move the soft constraints in the normal direction 


% parameter ONLY for visualization
phi_max = 25*deg2rad;
theta_max = 25*deg2rad;

%% MPC setting and init

N = 20; % Predition horizon
[controller, constraint_container] = initMPC9_Q4(Hz_high_level,N) %

distance_to_switch = 1; % [m] when to stop scaling the Q matrix

%% initial condition

% goal position in inital frame
goal = [25; 47; 1];

p__i_ib_0 = [25; 2; 1];
theta_ib_0 = [0; 0; 0]; 
v__b_ib_0 = [0; 0; 0];
omega__b_ib_0 = [0; 0; 0];

x0 = [p__i_ib_0; theta_ib_0; v__b_ib_0; omega__b_ib_0];

% integrators
integrator_z = 0;

%for estimation of angular acceleration (attitude controller)
omega_d_prev = 0;

%% reference model
wn_ref = 24;                        % reference model nataural frequnecy
zeta_ref = 1;

phi_ref = [0; 0; 0];                % initial reference model states (3rd order)
theta_ref = [0; 0; 0];              % initial reference model states (3rd order)

roll_pitch_yaw_ref = [0; 0; 0];

%% MAP
map = binaryOccupancyMap(50, 50, 100); %size and resolution in cells per meter

% set obstacles

x_random = randi([20 30],20, 1)
y_random = randi([5 45],20, 1)

x_random = [25;25;25;25;31;25;25;23;23;23;23;23;23;27;27;27;27;27;27;27]
y_random = [ 8;20;17;28;33;35;44; 5;10;14;23;27;35; 7;20;16;24;31;36;40]

x_random2 = [20;21;22;23;24;25;26;27;28;29;30;29;28;27;26;25;24;23;22;21]
y_random2 = [ 8;13;17;25;33;35;39; 5;10;14;23;27;35; 7;13;16;24;31;36;40]

x_random2_5 = [17;17;18;19;20;17;16;16;18;19]
y_random2_5 = [5;9;13;17;19;25;29;31;37;39]

x_random2_6 = [34;33;32;32;31;32;30;32;33;30]
y_random2_6 = [7;11;14;23;27;29;31;34;41;43]
    
x_random2 = [x_random2; x_random2_5; x_random2_6]
y_random2 = [y_random2; y_random2_5; y_random2_6]

x_random3 = 16
y_random3 =  5

x_random4 = 34
y_random4 = 5

setOccupancy(map, [x_random y_random], ones(20,1));
inflate(map, 0.05);

setOccupancy(map, [x_random2 y_random2], ones(40,1));
inflate(map, 0.2);

setOccupancy(map, [[x_random3; x_random4] [y_random3; y_random4]], ones(2,1));
inflate(map, 0.1);

%% Ray identification settings

number_of_rays = round(360/resulution);
angle_array = zeros(1,number_of_rays);

for i = 1:number_of_rays
    angle_array(1,i) = -pi + resulution*deg2rad*(i-1);
end


%% Main Loop

% setup
nx = 6; % Number of states
nu = 2; % Number of inputs

% allocate memory
simdata = zeros(Ns+1,length(x0)+1+4+3+1+4); 
simdataMPC = zeros(Ns/Hz_high_level+1, (1 + nx + nu + 1));

% init values
x = x0;
z_d = goal(3);
roll_pitch_yaw_d = [0;0;0];

% counter for MPC
k_l = 0;


for k = 1:Ns+1
    % time
    t = (k-1)*Ts;
    
    %% definitions
    % position 
    x__i = x(1);
    y__i = x(2);
    z__i = x(3);
    
    % orientation
    phi = x(4);
    theta = x(5);
    psi = x(6);
    roll_pitch_yaw = x(4:6);
    
    % velocity
    v__b_ib = x(7:9);
    R__i_b = Rzyx(phi, theta, psi);
    v__i_ib = R__i_b*v__b_ib;
    
    xdot__i = v__i_ib(1);
    ydot__i = v__i_ib(2);
    zdot__i = v__i_ib(3);
    
    % angular velocity
    omega_b = x(10:12);
    
    %% MPC
    if mod(1,(Hz_low_level/Hz_high_level)/k) == 0 % run MPC and detection at 10 HZ
        k_l = 1 + k_l;

        % position and orientation of body in inertial frame, basis for
        % constraint generation
        pose = [x(1) x(2) psi];
        
        %%%%%%% SETUP FOR probelm in horzontal bodyframe frame
        % Transform from inertial frame to horizontal body frame called mpc
        % frame
        R__mpc_i = transpose(Rzyx(0, 0, psi))

        vel__mpc = R__mpc_i*[xdot__i; ydot__i; 0]
%         
        x_mpc = [0;
                 0;
                 vel__mpc(1);
                 vel__mpc(2);
                 phi;
                 theta];
             
        
%         x_mpc = [0;
%                  0;
%                  xdot__i;
%                  ydot__i;
%                  phi;
%                  theta];
        % feasible velocities 
        horizon = 1; % need to integrate if not 1?
        possible_acc_xb = theta_max*Ts_h*horizon*g;
        possible_acc_yb = phi_max*Ts_h*horizon*g;

        %%%%%%% SETUP FOR problem in horzontal bodyframe frame
        
        V_next_max_x = [x(1) + x_mpc(1) + x_mpc(3) - possible_acc_xb;
                        x(1) + x_mpc(1) + x_mpc(3) + possible_acc_xb;
                        x(1) + x_mpc(1) + x_mpc(3) + possible_acc_xb;
                        x(1) + x_mpc(1) + x_mpc(3) - possible_acc_xb];

        V_next_max_y = [x(2) + x_mpc(2) + x_mpc(4) - possible_acc_yb;
                        x(2) + x_mpc(2) + x_mpc(4) - possible_acc_yb;
                        x(2) + x_mpc(2) + x_mpc(4) + possible_acc_yb;
                        x(2) + x_mpc(2) + x_mpc(4) + possible_acc_yb];

    %     plot(V_next_max_x', V_next_max_y')

        
        figure(1)
        show(map)
        grid on
            
        hold on
        if plot_square == true
            plot(V_next_max_x', V_next_max_y')
        end

    %     tic
        % calulate the position error
        position_error = sqrt((goal(1) - x(1))^2 + (goal(2) - x(2))^2)
        position_error_scaling = max(position_error, distance_to_switch) 
%         position_error_scaling = 1% 

        % calulate and return open cones
        [Alpha_models_h, Beta_models_h, Gamma_models_h, Alpha_models_s, Beta_models_s, Gamma_models_s] = velocityCones_v_12(pose, detection_range, angle_array, map, expand_distance, x, constraint_container);

        solution_list = {};
        cost_list = [];
        % reference and solution
        tic
        for cone_number = 1:size(Alpha_models_s,2)
            %%%%%%% SETUP FOR problem in inertial frame
%             ref = goal(1:2)*ones(1,N+1); %using inertial frame
            %%%%%%% SETUP FOR problem in horzontal bodyframe frame
            ref = (goal(1:2)-[x(1);x(2)])*ones(1,N+1); % in horizontal bodyframe
            ref = [cos(x(6)) sin(x(6)); -sin(x(6)) cos(x(6))]*ref;

            % Obtain constraints (first with only soft constraints)
%             inputs = {x_mpc,ref, Alpha_models_h{1,cone_number}.*0, Beta_models_h{1,cone_number}.*0, Gamma_models_h{1,cone_number}.*0, ...
%                              Alpha_models_s{1,cone_number}, Beta_models_s{1,cone_number}, Gamma_models_s{1,cone_number}, position_error_scaling, soft_constraint_distance};
            inputs = {x_mpc,ref, Alpha_models_h{1,cone_number}, Beta_models_h{1,cone_number}, Gamma_models_h{1,cone_number}, ...
                             Alpha_models_s{1,cone_number}, Beta_models_s{1,cone_number}, Gamma_models_s{1,cone_number}, position_error_scaling, soft_constraint_distance};
            [solutions,diagnostics] = controller{inputs};

            solution_list{cone_number} = solutions; 
            cost_list = [cost_list, solutions(3)];
        end

        % find the best option    
        opt_index = nan; 
        opt_cost = nan;
        for solution_index = 1:length(solution_list)
            if ((cost_list{1,solution_index} < opt_cost) || (isnan(opt_cost)))
                opt_index = solution_index;
                opt_cost = cost_list{1,solution_index};
            end
        end
        
        variabletime{k_l} = toc;

        u_opt = solution_list{1,opt_index}{1,1};
        x_opt = solution_list{1,opt_index}{1,2};
        
        if isnan(u_opt(1,1)) == true
            crash % crash
        end

        x_mpc_plan = x_opt(:,2)+[x(1); x(2); 0; 0; 0; 0]; % first is initial condition
%         pose = [x(1), x(2), 0];

        % reference for low-level control
        roll_pitch_yaw_ref = [u_opt(:,1) ; 0]; % input to be used

        simdataMPC(k_l,:) = [t x_mpc_plan' roll_pitch_yaw_ref'];
    
        % extract data
        x_log = simdataMPC(1:k_l,2);
        y_log = simdataMPC(1:k_l,3);
        time_log = simdataMPC(1:k_l,1);
        roll_log = simdataMPC(1:k_l,6);
        pitch_log = simdataMPC(1:k_l,7);
        roll_command_log = simdataMPC(1:k_l,8);
        pitch_command_log = simdataMPC(1:k_l,9);
        u_vel_log = simdataMPC(1:k_l,4);
        v_vel_log = simdataMPC(1:k_l,5);

        %%%%% MPC plotting %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % plot position from optimalization
        figure(1)
        hold on
        plot(x(1)+x_opt(1,:), x(2)+x_opt(2,:))

        % goal
        plot(goal(1),goal(2), '--or')

        % plot optional paths to take
        for i = 1:cone_number
            x_opt_ = solution_list{1,i}{1,2};
            plot(x_opt_(1,:)+x(1), x_opt_(2,:)+x(2), 'm')
        end

        % plot path taken

        plot(x_log, y_log, 'b')
        xlim([10 40])
        hold off


        % plot log
        % plot current solution for u
        figure(2)
        subplot(3,1,1)
    %     plot((1:size(u_opt,2))*Ts, u_opt.*rad2deg)
        stairs((1:size(u_opt,2))*Ts, u_opt'.*rad2deg)
        subtitle('optimal input sequence')
        ylim([-60 60])
        xlabel('time [s]')
        ylabel('attitude command [deg]')
        legend('roll_{d}', 'pitch_{d}')
        grid on

        subplot(3,1,2)
    %     plot(time_log, roll_log*rad2deg, time_log, pitch_log*rad2deg)
        stairs(time_log, roll_command_log'*rad2deg)
        hold on
        stairs(time_log, pitch_command_log'*rad2deg)
        subtitle('input history')
        ylim([-60 60])
        xlabel('time [s]')
        ylabel('attitude command [deg]')
        legend('roll_{d}', 'pitch_{d}')
        grid on
        hold off

        subplot(3,1,3)
        plot(time_log, u_vel_log, time_log, v_vel_log)
        subtitle('velocity history')
        ylim([-8 8])
        ylabel('velocity [m/s]')
        xlabel('time [s]')
        legend('u_{d}', 'v_{d}')
        grid on
      
    end
    
    
    %% attitude refernce models     
    phi_refmod_dot = referenceModelPA(roll_pitch_yaw_ref(1), zeta_ref, wn_ref, phi_ref);
    theta_refmod_dot = referenceModelPA(roll_pitch_yaw_ref(2), zeta_ref, wn_ref, theta_ref); 
    
    phi_ref = euler2(phi_refmod_dot, phi_ref, Ts);
    theta_ref = euler2(theta_refmod_dot, theta_ref, Ts);
    
    % with reference model
    roll_pitch_yaw_d = [phi_ref(1); theta_ref(1); 0];  %commented out for testing
    omega_d = inv(Tzyx(phi_ref(1), theta_ref(1)))*[phi_ref(2); theta_ref(2); 0]; %commented out for testing 
    omega_d_dot = 0.99*(omega_d-omega_d_prev)/Ts;
    omega_d_prev = omega_d;

    %% altitude control
    % anti-windup

%     [F_T, dot_integrator_z] = altitudeControl(z_d, z__i, zdot__i, integrator_z, roll_pitch_yaw) %without anti-windup 
    [F_T, dot_integrator_z, integrator_z] = altitudeControlAW(z_d, z__i, zdot__i, integrator_z, roll_pitch_yaw);

    %% attitude control    
%     M = s03Control(roll_pitch_yaw_d, roll_pitch_yaw, omega) % setpoint problem
    M = s03ControlTrack(roll_pitch_yaw_d, omega_d, omega_d_dot, roll_pitch_yaw, omega_b); % attitude tracking
    
    tau = [F_T; M];

    %% system dyanmics
    [xdot, f__b_Tall] = rmf_dynamics(0,x,tau);
   
    %% integration
    x = euler2(xdot,x,Ts);
    integrator_z = euler2(dot_integrator_z,integrator_z,Ts)
    simdata(k,:) = [t, x', tau', roll_pitch_yaw_d', z_d, f__b_Tall']; 
    
    
end



%% plots

% extract data
time = simdata(:,1)
position = simdata(:,2:4)
orientation = simdata(:,5:7)
velocity = simdata(:,8:10)
angular_velocity = simdata(:,11:13)
altitude_force_control = simdata(:,14)
attitude_moment_control = simdata(:,15:17)
desired_attitude = simdata(:,18:20)
desired_altitude = simdata(:,21)
actuator_forces = simdata(:,22:25)

figure(100)
plot(time,position)
title('Position')
legend('x','y','z')
ylim([-1.5 1.5])
grid on
ylabel('position [m]')
xlabel('time [s]')

figure(111)
plot(time,position(:,3), time, desired_altitude)
title('Altitude')
legend('z', 'z_d')
ylim([0.8 1.2])
grid on
ylabel('altitude [m]')
xlabel('time [s]')

figure(112)
plot(time, altitude_force_control)
title('Thurst force command')
ylabel('force [N]')
xlabel('time [s]')
legend('F_T')
grid on

figure(101)
plot(time, orientation(:,1).*rad2deg, time, desired_attitude(:,1).*rad2deg)
title('Attitude')
% legend('roll','pitch','yaw', 'roll_d','pitch_d','yaw_d')
legend('roll','roll_d')
ylabel('attitude [deg]')
xlabel('time [s]')
ylim([-5 15])
grid on

figure(102)
subplot(2,1,1)
plot(time, altitude_force_control)
title('Thurst force command')
ylabel('force [N]')
xlabel('time [s]')
legend('F_T')
grid on

subplot(2,1,2)
plot(time, attitude_moment_control)
title('Moment command')
ylabel('moment [Nm]')
xlabel('time [s]')
legend('M_x', 'M_y', 'M_z')
grid on

figure(110)
plot(time, attitude_moment_control)
title('Moment command')
ylabel('moment [Nm]')
xlabel('time [s]')
legend('M_x', 'M_y', 'M_z')
grid on

figure(103)
plot(time,velocity, time_log, u_vel_log, time_log, v_vel_log)
title('Velocities')
legend('u^{hb}', 'v^{hb}', 'w^{hb}', 'predicted u^{hb}', 'predicted v^{hb}')
xlabel('time [s]')
ylabel('velocity [m/s]')
grid on

figure(104)
plot(time, orientation(:,1).*rad2deg)
hold on
stairs(time_log, roll_command_log'*rad2deg)
stairs(time_log, roll_log'*rad2deg)
plot(time, orientation(:,2).*rad2deg)
stairs(time_log, pitch_command_log'*rad2deg)
stairs(time_log, pitch_log'*rad2deg)
plot(time, orientation(:,3).*rad2deg)
legend('roll','roll_d', 'predicted roll','pitch', 'pitch_d','predicted pitch','yaw')
ylabel('attitude [deg]')
xlabel('time [s]')
grid on
hold off

figure(105)
plot(time, orientation.*rad2deg)
hold on
stairs(time_log, roll_command_log'*rad2deg)
stairs(time_log, pitch_command_log'*rad2deg)
% stairs(time_log, roll_log'*rad2deg)
% stairs(time_log, pitch_log'*rad2deg)
legend('roll','pitch','yaw', 'roll_d', 'pitch_d')
ylabel('attitude [deg]')
xlabel('time [s]')
grid on
hold off

figure(107)
plot(time, actuator_forces)
title('Actuator force commands')
ylabel('Force [N]')
xlabel('time [s]')
legend('f^{b}_{T1}', 'f^{b}_{T2}', 'f^{b}_{T3}', 'f^{b}_{T4}')
grid on



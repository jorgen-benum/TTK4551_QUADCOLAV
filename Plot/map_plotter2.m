% %% initial condition

% goal position in inital frame
% goal = [48; 10; 1]
goal = [25; 47; 1];

% velocity in body or inertial frame?

% % p__i_ib_0 = [3; 10; 1];
% p__i_ib_0 = [27; 2; 1];
% % p__i_ib_0 = [21; 2; 1]; % good
% 
% 
% % %% MAP
% map = binaryOccupancyMap(50, 50, 100); %size and resolution in cells per meter
% 
% x_random = randi([20 30],20, 1)
% y_random = randi([5 45],20, 1)
% % 
% % x_random = [29;18;17;23;22;19;27;35;22;22;10;6;16;18;31;44;43;23;14;36]
% % y_random = [23;23;23;16;22;20;17;16;24;16;16;22;24;20;22;16;25;20;22;15]
% 
% x_random = [25;25;25;25;31;25;25;23;23;23;23;23;23;27;27;27;27;27;27;27]
% y_random = [ 8;20;17;28;33;35;44; 5;10;14;23;27;35; 7;20;16;24;31;36;40]
% 
% x_random2 = [20;21;22;23;24;25;26;27;28;29;30;29;28;27;26;25;24;23;22;21]
% y_random2 = [ 8;13;17;25;33;35;39; 5;10;14;23;27;35; 7;13;16;24;31;36;40]
% 
% x_random2_5 = [17;17;18;19;20;17;16;16;18;19]
% y_random2_5 = [5;9;13;17;19;25;29;31;37;39]
% 
% x_random2_6 = [34;33;32;32;31;32;30;32;33;30]
% y_random2_6 = [7;11;14;23;27;29;31;34;41;43]
%     
% x_random2 = [x_random2; x_random2_5; x_random2_6]
% y_random2 = [y_random2; y_random2_5; y_random2_6]
% 
% x_random3 = [1;1;1;1;1;1;1;1;1;1;1;1;1;1;1;1;1;1;1;1].*16
% y_random3 = [ 5;7;9;11;13;15;17;19;21;23;25;27;29;31;33;35;37;39;41;43]
% 
% x_random4 = [1;1;1;1;1;1;1;1;1;1;1;1;1;1;1;1;1;1;1;1].*34
% y_random4 = [ 5;7;9;11;13;15;17;19;21;23;25;27;29;31;33;35;37;39;41;43]
% 
% setOccupancy(map, [x_random y_random], ones(20,1));
% inflate(map, 0.05);
% 
% % setOccupancy(map, [x_random2 y_random2], ones(20,1));
% % inflate(map, 0.2);
% 
% setOccupancy(map, [x_random2 y_random2], ones(40,1));
% inflate(map, 0.2);
% 
% setOccupancy(map, [[x_random3; x_random4] [y_random3; y_random4]], ones(40,1));
% inflate(map, 0.1);

%% load data
% 
maxspeed = 3.5

position1 = simdata1(:,2:3)
velocity1 = simdata1(:,8:9)
speed1 = sqrt(velocity1(:,1).^2 + velocity1(:,2).^2)
v1 = speed1/maxspeed;
maxvel1 = max(speed1)
avg1 = mean(speed1)

% position2 = simdata2(:,2:3);
% velocity2 = simdata2(:,8:9);
% speed2 = sqrt(velocity2(:,1).^2 + velocity2(:,2).^2);
% v2 = speed2/maxspeed;
% maxvel2 = max(speed2)
% avg2 = mean(speed2)
% 
% position3 = simdata3(:,2:3)
% velocity3 = simdata3(:,8:9)
% speed3 = sqrt(velocity3(:,1).^2 + velocity3(:,2).^2)
% v3 = speed3/maxspeed;
% maxvel3 = max(speed3)
% avg3 = mean(speed3)

% position4 = simdata4(:,2:3)
% velocity4 = simdata4(:,8:9)
% speed4 = sqrt(velocity4(:,1).^2 + velocity4(:,2).^2)
% v4 = speed4/maxspeed;
% maxvel4 = max(speed4)
% avg4 = mean(speed4)

figure(34)
show(map)
hold on

% plot(position1(:,1)',position1(:,2)','r') ;  % plot trajcetory 
scatter(position1(:,1)',position1(:,2)',1,v1,'filled') ; % velcoity as a scatter plot
% scatter(position2(:,1)',position2(:,2)',1,v2,'filled') ; % velcoity as a scatter plot
% scatter(position3(:,1)',position3(:,2)',1,v3,'filled') ; % velcoity as a scatter plot
% scatter(position4(:,1)',position4(:,2)',1,v4,'filled') ; % velcoity as a scatter plot



c = colorbar
% c.Ticks=[0, 0.5, 1]
xlim([10 40])
plot(goal(1), goal(2), '--or')


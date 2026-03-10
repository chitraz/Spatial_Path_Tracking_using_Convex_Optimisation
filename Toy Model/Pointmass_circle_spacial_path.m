clc; clear all; close all;

%--------------------------(Problem parameters)----------------------------
f_max = 50; f_min = -50;    %force limit
f_s1 = 0; f_s2 = 0;
accel_max = 10; accel_min = -10;    %accleration limits
b_0 = 0; b_N = 0;   %B.C. on b(S)
b_max = 20; %s_dot^2, velocity bound
g = 9.8;
mass = 2;
radius = 2;     %path parameters
angle = 3*pi/2;
phase = pi/2;

x1_0 = radius*cos(phase); x2_0 = radius*sin(phase); %intial bc
x1_1 = radius*cos(angle+phase); x2_1 = radius*sin(angle+phase);  %end bc

%--------------------------(linear program)--------------------------------
N = 200;
options = optimoptions('linprog','Algorithm','dual-simplex'); %use Simplex Method
%options = optimoptions('linprog','Algorithm','interior-point-Legacy'); %use Interior Point Method
[a,b,F_1,F_2] = LinearProgram(N,g,angle,phase,radius,mass,f_max,f_min,...
                        accel_max,accel_min,b_max,b_0,b_N,options,f_s1,f_s2);
t_s = t(b,N); %calculate t-s relation)
T = t_s(N) %get final time
[F_1_t,F_2_t] = ConstructSource(N,t_s,F_1,F_2); %convert into time domain, using t-s relation
sim('ToyModel_sliding_friction');

% %--------------------------(convex program)--------------------------------
% sigma = 0; %work optimal term factor
% gamma = 1; %time optimal term factor
% omega = 0; %energy optimal term factor (not convex?)
% N = 100;   
% cvx_solver sedumi %select solver
% [a,b,F_1,F_2] = ConvexProgram(N,g,angle,phase,radius,mass,f_max,f_min,...
%     accel_max,accel_min,b_max,b_0,b_N,sigma,gamma,omega,f_s1,f_s2);
% plot_solution(N,a,b,F_1,F_2);
% t_s = t(b,N); %calculate t-s relation
% T = t_s(N) %get final time
% [F_1_t,F_2_t] = ConstructSource(N,t_s,F_1,F_2); %convert into time domain, using t-s relation
% sim('ToyModel_sliding_friction'); %run simulink simulation

%----------------------(plot simulated trajectory)-------------------------
figure; grid on; grid minor; hold on;
title('Minimum Work Done Trajectory');
plot(x_2_simout.data,x_1_simout.data,'linewidth',1); %plot trajector

%---------------------(cal/plot geometric path)----------------------------
xlim([-radius*3/2 radius*3/2]); ylim([-radius*3/2 radius*3/2]);
angles = 0:pi/100:angle;
arc_x1 = radius*cos(angles+phase); arc_x2 = radius*sin(angles+phase);
plot(arc_x2, arc_x1,'k','Linewidth',2); %draw the geometric path
xlabel('x_2'); ylabel('x_1');
scatter(x2_0 ,x1_0,'x','k'); scatter(x2_1 ,x1_1,'o','k');
figure; hold on;
F_1_t.plot; F_2_t.plot;
title('Optimal Control Signal')
xlabel('time (t)'); ylabel('force (N)');
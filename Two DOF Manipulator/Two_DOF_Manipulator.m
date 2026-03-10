clear all; clc; close all;
syms theta_1 theta_2

%problem parameters
m_1 = 1; m_2 = 2; L_1 = 1; L_2 = 0.5; g = 9.8; 
fs_1 = 0.1; fs_2 = 0.1; %sliding friction magnitudes

theta_1_max = pi; theta_1_min = 0;   
theta_2_max = 3*pi/4; theta_2_min = -3*pi/4; %joint angle bounds

b_0 = 0; b_N = 0; b_max = 20; %speed constriants: b(s)=(s_dot)^2 

theta1_ddot_min = -5; theta2_ddot_min = -5; %acceleration bounds
theta1_ddot_max = 5; theta2_ddot_max = 5;   

tau1_min = -40; tau1_max = 40; tau2_min = -40; tau2_max = 40; %torque bounds

gamma = theta_1_max - theta_1_min; beta = theta_1_min;  
phi = theta_2_max - theta_2_min; alpha = theta_2_min; %path parameters
theta1_0 = beta; theta2_0 = alpha;
theta1_1 = gamma + beta; theta2_1 = phi + alpha; %B.C.s 

%Tool Matrix
A_1 = [cos(theta_1) -sin(theta_1) 0 L_1*cos(theta_1);...
     sin(theta_1) cos(theta_1) 0 L_1*sin(theta_1);...
     0 0 1 0;0 0 0 1];
A_2 = [cos(theta_2) -sin(theta_2) 0 L_2*cos(theta_2);...
     sin(theta_2) cos(theta_2) 0 L_2*sin(theta_2);...
     0 0 1 0;0 0 0 1];
Tool = A_1*A_2;
Tool = subs(Tool);

x_2 = Tool(1,4); y_2 = Tool(2,4);     %extract tool coordinate ; forward kinematics 
x_1 = A_1(1,4); y_1 = A_1(2,4);

%create admissable set for joint angles (only used to to find the workspace)
theta_q1 = 100; theta_q2 = 100; 
t1 = theta_1_min:(theta_1_max-theta_1_min)/theta_q1: theta_1_max;
t2 = theta_2_min:(theta_2_max-theta_2_min)/theta_q2: theta_2_max;
[theta_1,theta_2]=ndgrid(t1,t2);

%find the workspace
x_2 = subs(x_2);y_2 = subs(y_2);
x_1 = subs(x_1);y_1 = subs(y_1);
% hold on; axis([-(L_2+L_1) (L_2+L_1) -L_2 (L_2+L_1)]);
% scatter(x_2(:),y_2(:),'.') %plot each of the now transformed points
% xlabel('X-base (m)'); ylabel('Y-base (m)'); title('Workspace - Transforming admissable set theta');
figure; hold on; grid on;
k=boundary(double(x_2(:)),double(y_2(:)),0.5);
plot(x_2(k),y_2(k),'k','LineWidth',2) %draw the boundary of workspace

% %animation of the manipulators tracing out the workspace
% scatter(0,0,'k','filled'); flip = 1;
% xlabel('X-base (m)');ylabel('Y-base (m)'); title('Workspace - with manipulator');
% for i=1:(theta_q2+1)
%     flip = -1*flip;
%     flip_ = (flip + 1)/2; %fix glitch
%     for j=1:(theta_q1+1)
%         link_1 =plot([0 x_1(i,flip_*(theta_q1+2)-flip*j)],... 
%             [0 y_1(i,flip_*(theta_q1+2)-flip*j)],'k','LineWidth',1);
%         link_2 =plot([x_1(i,flip_*(theta_q1+2)-flip*j) x_2(i,flip_*(theta_q1+2)-flip*j)], ...
%             [y_1(i,flip_*(theta_q1+2)-flip*j) y_2(i,flip_*(theta_q1+2)-flip*j)],'k','LineWidth',1); 
%         joint_2 = scatter(x_1(i,flip_*(theta_q1+2)-flip*j),...
%             y_1(i,flip_*(theta_q1+2)-flip*j),'k','filled');
%         tool = scatter(x_2(i,flip_*(theta_q1+2)-flip*j),...
%             y_2(i,flip_*(theta_q1+2)-flip*j),'k','filled');
%         pause(0.0001); 
%         delete(link_1); delete(link_2); delete(joint_2); delete(tool);
%     end
% end

%draw the geometric path
s = 0:0.01:1;
theta_s = [gamma*s+beta;phi*s+alpha];  %the path
theta_1 = theta_s(1,:); theta_2 = theta_s(2,:);
x_2 = Tool(1,4); y_2 = Tool(2,4);  %extract fk agian
x_2 = subs(x_2); y_2 = subs(y_2); 
plot(x_2,y_2,'color','r'); %geometric path in the workspace

%construct optmisation problem
N=50;
cvx_solver sedumi %select solver
[a,b,tau1,tau2] = ConvexProgram_elbow(m_1,m_2,L_1,L_2,gamma,beta,phi,alpha,fs_1,fs_2...
    ,N,g,b_max,tau1_max,tau1_min,tau2_max,tau2_min,b_0, b_N, ...
    theta1_ddot_min,theta1_ddot_max,theta2_ddot_min,theta2_ddot_max);
figure
plot_solution(N,a,b,tau1,tau2);
t_s = t(b,N); %calculate t-s relation
T = t_s(N) %get trajectory period
[tau1_t,tau2_t] = ConstructSource(N,t_s,tau1,tau2); %convert into time domain, using t-s relation   figure; hold on;
figure; hold on
tau1_t.plot; tau2_t.plot; figure;

%simulate simulink model 
sim('elbow_manipulator_model');
theta_1 = theta1_simout.data; theta_2 =theta2_simout.data;
x_2 = Tool(1,4); y_2 = Tool(2,4);  %extract fk agian
x_2 = subs(x_2); y_2 = subs(y_2); 
plot3(x_2,y_2,theta1_simout.time,'--','color','b'); %actual path in the workspace
xlabel('x'); ylabel('y'); title('path-tracking in workspace');
legend('boundary of workspace','geometric path', 'actual path');
figure; hold on; grid on;
rectangle('Position',[theta1_0 theta2_0 theta1_1-theta1_0 theta2_1-theta2_0]... 
    ,'EdgeColor','k','LineWidth',2); 
plot(theta1_simout.data, theta2_simout.data,'--','color','b'); %actual path
plot(theta_s(1,:),theta_s(2,:),'color','r'); %geometric path in configuration space
xlabel('theta 1'); ylabel('theta 2'); title('path-tracking in jointspace');
legend('geometric path', 'actual path');

close all
clear all
clc

%% system params
I1 = 1;  %rotational inertia of link one 
I2 = 1;  %rotational inertia of link two
m1 = 1;  %mass of link one
m2 = 1;  %mass of link two
mb = 1;  %mass of the ball
L1 = 1;  %length of link1
L2 = 1;  %length of link2
g  = 10; %acceleration due to gravity


%% define dynamcis system matrix values
M  = @(t,X) Mfun(I1,I2,m1,m2,mb,L1,L2,g,X(1),X(2),X(3),t)
C  = @(t,X) Cfun(I1,I2,m1,m2,mb,L1,L2,g,X(1),X(2),X(3),X(4),X(5),X(6),t)
G  = @(t,X) Gfun(I1,I2,m1,m2,mb,L1,L2,g,X(1),X(2),X(3),t)
Ke = @(t,X) KEfun(I1,I2,m1,m2,mb,L1,L2,g,X(1),X(2),X(3),X(4),X(5),X(6),t)
Pe = @(t,X) PEfun(I1,I2,m1,m2,mb,L1,L2,g,X(1),X(2),X(3),t)
dX = @(t,X) dXfun(I1,I2,m1,m2,mb,L1,L2,g,X(1),X(2),X(3),X(4),X(5),X(6),t)
%% run the simulation
ODEFUN = @(t,X) GeneralODEfun(M,C,G,X,t,L1,mb,dX)

X0 = [pi/6 pi 0.5 0.5 -3 0]'; %[th1 th2 s dth1 dth2 ds];
TSPAN = [0 3];
[TOUT,Xout] = ode45(ODEFUN,TSPAN,X0);
figure(1);
subplot(3,1,1)
plot(TOUT,Xout(:,1))
ylabel('\theta_1')
subplot(3,1,2)
plot(TOUT,Xout(:,2))
ylabel('\theta_2')
subplot(3,1,3)
plot(TOUT,Xout(:,3))
ylabel('s')
xlabel('time [s]')

for i = 1:length(Xout)
KE_data(i) = Ke(1,Xout(i,:));
PE_data(i) = Pe(1,Xout(i,:));
end 
figure(2);
% KE_data(1)
plot(TOUT,[KE_data;PE_data;KE_data+PE_data])
ylabel('Energy')
xlabel('time [s]')
legend('Pe','Ke','Ke+Pe')


%% Animation

% simulation parameters
FPS = 30; % frames per second
tspan_anim = TOUT; % simulation time span
%nframes = diff(tspan_anim)*FPS; % total number of frames
v = VideoWriter('BallLevelUncontr3.mp4','MPEG-4');
v.FrameRate = FPS;
open(v)
% ReSet Initial Conditions for Animation 
%tspan_anim = 0:1/FPS:3; % simulation time span
th1_0 = X0(1);          % Theta1
th2_0 = X0(2);          % Theta2
s_0 = X0(3);            % s0 (Ball position)
dth1_0 = X0(4);         % dtheta1/dt    
dth2_0 = X0(5);         % dtheta2/dt

% create figure and axes
figure(3);
axes('XLim',[-2,2],'YLim',[-2,2]);
axis equal
set(gcf,"Color", [1 1 1]);
title('\theta_1 = pi/6, \theta_2 = \pi, s = 0.5, d\theta_1 = 0.5, d\theta_2 = -3, ds = 0');
hold on

%linkagesim = interp1(TOUT,Xout, tspan_anim);

% draw links and ball
link1 = plot([0, L1*cos(th1_0)], [0, L1*sin(th1_0)], 'k', 'LineWidth',2);
link2 = plot([L1*cos(th1_0), L1*cos(th1_0) + L2*cos(th2_0)], [L1*sin(th1_0), L1*sin(th1_0)+ L2*sin(th2_0)], 'g', 'LineWidth',2);
ball = plot(L1*cos(th1_0) + s_0*cos(th2_0), L1*sin(th1_0) + s_0*sin(th2_0), 'o', 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b', 'MarkerSize', 10);

% % animate system
for i = 1:numel(tspan_anim)-1
    % calculate positions of links and ball using current values of joint angles and ball position
    x1 = L1*cos(Xout(i,1));
    y1 = L1*sin(Xout(i,1));
    x2 = L1*cos(Xout(i,1)) + L2*cos(Xout(i,2));
    y2 = L1*sin(Xout(i,1)) + L2*sin(Xout(i,2));
    xb = L1*cos(Xout(i,1)) + Xout(i,3)*cos(Xout(i,2));
    yb = L1*sin(Xout(i,1)) + Xout(i,3)*sin(Xout(i,2));  
  
    % update positions of links and ball
    link1.XData = [0, x1];
    link1.YData = [0, y1];
    link2.XData = [x1, x2];
    link2.YData = [y1, y2];
    ball.XData = xb;
    ball.YData = yb;    
   
% pause for specified frame rate
    pause(1/FPS)
    frame = getframe(gcf);
    writeVideo(v,frame);
end

close(v);
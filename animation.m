function [] = animation(tsim,xsim,param)
[I1,I2,m1,m2,mb,L1,L2,g] = decodeParams(param);
%% Animation

% simulation parameters
FPS = 60; % frames per second
tspan_anim = tsim(1):1/FPS:tsim(end); % simulation time span
Xout = interp1(tsim,xsim,tspan_anim); %interpolate the needed frames
%nframes = diff(tspan_anim)*FPS; % total number of frames

% ReSet Initial Conditions for Animation 
%tspan_anim = 0:1/FPS:3; % simulation time span
th1_0   = Xout(1,1);         % Theta1
th2_0   = Xout(1,2);         % Theta2
s_0     = Xout(1,3);         % s0 (Ball position)
dth1_0  = Xout(1,4);         % dtheta1/dt    
dth2_0  = Xout(1,5);         % dtheta2/dt
ds_0    = Xout(1,6);




% create figure and axes
figure();
% axes('XLim',[-2,2],'YLim',[-2,2]);
axis equal
set(gcf,"Color", [1 1 1]);
X0string = ['\theta_1 =  '+string(round(th1_0,2))+...
    ', \theta_2 = '+string(round(th2_0,2))+...
    ', s = '+string(round(s_0,2))+...
    ', d\theta_1 = '+string(round(dth1_0,2))+...
    ', d\theta_2 = '+string(round(dth2_0,2))+...
    ', ds = '+string(round(ds_0,2))]
title(X0string);

fileLocName = X0string;
fileLocName = strrep(fileLocName,'.','p');
fileLocName = strrep(fileLocName,'=','');
fileLocName = strrep(fileLocName,'\theta','th');
fileLocName = strrep(fileLocName,' ','');
fileLocName = strrep(fileLocName,'th_1','Th1-');
fileLocName = strrep(fileLocName,'th_2','Th2-');
fileLocName = strrep(fileLocName,'s','S-');
fileLocName = strrep(fileLocName,',','');
fileLocName = ['./VideoFiles/'+fileLocName+'.mp4']
v = VideoWriter(fileLocName,'MPEG-4');
v.FrameRate = FPS;
open(v)


hold on

%linkagesim = interp1(TOUT,Xout, tspan_anim);

% draw links and ball
link1 = plot([0, L1*cos(th1_0)], [0, L1*sin(th1_0)], 'k', 'LineWidth',2);
link2 = plot([L1*cos(th1_0), L1*cos(th1_0) + L2*cos(th2_0)], [L1*sin(th1_0), L1*sin(th1_0)+ L2*sin(th2_0)], 'g', 'LineWidth',2);
ball = plot(L1*cos(th1_0) + s_0*cos(th2_0), L1*sin(th1_0) + s_0*sin(th2_0), 'o', 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b', 'MarkerSize', 10);
axis([-2 2 -2 2])
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

close all
clear all
clc
 % test git with new nameS
%% system params
I1 = 1;         %rotational inertia of link one 
I2 = 1;         %rotational inertia of link two
m1 = 1;         %mass of link one
m2 = 1;         %mass of link two
mb = 1e-12;     %mass of the ball
L1 = 1;         %length of link1
L2 = 1;         %length of link2
g  = 10;        %acceleration due to gravity

param = encodeParams(I1,I2,m1,m2,mb,L1,L2,g);





%% define dynamcis system matrix values
M  = @(t,X) Mfun(I1,I2,m1,m2,mb,L1,L2,g,X(1),X(2),X(3),t);
C  = @(t,X) Cfun(I1,I2,m1,m2,mb,L1,L2,g,X(1),X(2),X(3),X(4),X(5),X(6),t);
G  = @(t,X) Gfun(I1,I2,m1,m2,mb,L1,L2,g,X(1),X(2),X(3),t);
Ke = @(t,X) KEfun(I1,I2,m1,m2,mb,L1,L2,g,X(1),X(2),X(3),X(4),X(5),X(6),t);
Pe = @(t,X) PEfun(I1,I2,m1,m2,mb,L1,L2,g,X(1),X(2),X(3),t);
dX = @(t,X) dXfun(I1,I2,m1,m2,mb,L1,L2,g,X(1),X(2),X(3),X(4),X(5),X(6),t);
%% run the simulation
ODEFUN = @(t,X) GeneralODEfun(M,C,G,X,t,L1,mb,dX)

X0 = [-pi/4 -pi/4 0.5 0 0 0]' %[th1 th2 s dth1 dth2 ds];
TSPAN = [0 10];
[TOUT,Xout] = ode45(ODEFUN,TSPAN,X0);
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
figure
for i = 1:length(Xout)
KE_data(i) = Ke(1,Xout(i,:));
PE_data(i) = Pe(1,Xout(i,:));
end 

% KE_data(1)
plot(TOUT,[KE_data;PE_data;KE_data+PE_data])
ylabel('Energy')
xlabel('time [s]')
legend('Ke','Pe','Ke+Pe')

animation(TOUT,Xout,param);

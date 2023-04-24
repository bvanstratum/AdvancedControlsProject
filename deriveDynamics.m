

%this script derives the eoms for a robot arm balancing a ball
close all
clear all
clc
syms t I1 I2 m1 m2 mb L1 L2 g...params
     th1(t) th2(t)  s(t)  ... time dep vars
     Th1    Th2     dTh1    dTh2    S    dS   ddS ddTh1 ddTh2 %non time dep vars

 dX = [dTh1 dTh2 ddTh1 ddTh2 ].'
 
 
% timeDepVars = [th1 th2 dth1 dth2 s ds diff(dth1(t), t) diff(dth2(t), t)]
timeDepVars = {diff(th1(t), t, 2), diff(th2(t), t, 2), diff(s(t),t,2),...
              diff(th1(t), t), diff(th2(t), t), diff(s(t),t),...
              th1(t), th2(t) s(t)}
stateVars   = {ddTh1,               ddTh2,      ddS,...
                dTh1,                dTh2,       dS,...
                Th1,    Th2, S}


R1 = [L1/2*cos(th1)
      L1/2*sin(th1)]
 
v1 = diff(R1,t)
 
R2 = [L1*cos(th1)+L2/2*cos(th2)
      L1*sin(th1)+L2/2*sin(th2)]
v2 = diff(R2,t)

R_ball = [L1*cos(th1)+s*cos(th2)
          L1*sin(th1)+s*sin(th2)]
v_ball = diff(R_ball)    

T =   m1*(v1.'*v1)/2 +I1*(diff(th1,t))^2/2 ... first link kinetic nrg
    + m2*(v2.'*v2)/2 +I2*(diff(th2,t))^2/2 ... second link kin. nrg
    + mb*(v_ball.'*v_ball)/2 % ball kin. nrg

V =  m1*g*L1/2*sin(th1(t))...
    +m2*g*(L1*sin(th1)+L2/2*sin(th2))...
    +mb*g*(L1*sin(th1)+   s*sin(th2)) 

L = T-V

f3 = simplify(diff(L, diff(th1(t),t)));
f4 = simplify(diff(L, diff(th2(t),t)));
f5 = simplify(diff(L, diff(  s(t),t)));

f3 = diff(f3,t);
f4 = diff(f4,t);
f5 = diff(f5,t);


% partial partial q
f3 = f3-diff(L,th1(t));
f4 = f4-diff(L,th2(t));
f5 = f5-diff(L,  s(t));

f3 = simplify(subs(f3,timeDepVars,stateVars));
f4 = simplify(subs(f4,timeDepVars,stateVars));
f5 = simplify(subs(f5,timeDepVars,stateVars));

Sln = solve([f3 f4 f5],{ddTh1 ddTh2 ddS});
dX = [Sln.ddTh1
      Sln.ddTh2
      Sln.ddS];
matlabFunction(dX,'File','dXfun','Vars',{I1 I2 m1 m2 mb L1 L2 g...
                                       Th1 Th2 S ...
                                       dTh1 dTh2 dS t})

M = jacobian([f3, f4, f5],[ddTh1 ddTh2 ddS]);
% C = jacobian([f3, f4, f5],[dTh1   dTh2  dS]);
% M = jacobian([f3, f4],[ddTh1 ddTh2]);

g1 = subs(f3,{ddTh1,ddTh2, ddS, dTh1, dTh2, dS},{0 0 0 0 0 0});
g2 = subs(f4,{ddTh1,ddTh2, ddS, dTh1, dTh2, dS},{0 0 0 0 0 0});
g3 = subs(f5,{ddTh1,ddTh2, ddS, dTh1, dTh2, dS},{0 0 0 0 0 0});
G = [g1 g2 g3].';


%  one these elements I am plugging in 1 but I know that the dX term is
%  squared
c11 = subs(f3,{ddTh1,ddTh2, ddS, dTh1, dTh2, dS},{0 0 0 dTh1    0   0}) -g1; %zero
c12 = subs(f3,{ddTh1,ddTh2, ddS, dTh1, dTh2, dS},{0 0 0    0 dTh2   0}) -g1; %fun of dth1^2
c13 = subs(f3,{ddTh1,ddTh2, ddS, dTh1, dTh2, dS},{0 0 0    0    0   dS}) -g1;

c21 = subs(f4,{ddTh1,ddTh2, ddS, dTh1, dTh2, dS},{0 0 0 dTh1    0   0}) -g2;
c22 = subs(f4,{ddTh1,ddTh2, ddS, dTh1, dTh2, dS},{0 0 0    0 dTh2   0}) -g2;
c23 = subs(f4,{ddTh1,ddTh2, ddS, dTh1, dTh2, dS},{0 0 0    0    0  dS}) -g2;

c31 = subs(f5,{ddTh1,ddTh2, ddS, dTh1, dTh2, dS},{0 0 0 dTh1    0   0}) -g3;
c32 = subs(f5,{ddTh1,ddTh2, ddS, dTh1, dTh2, dS},{0 0 0    0 dTh2   0}) -g3;
c33 = subs(f5,{ddTh1,ddTh2, ddS, dTh1, dTh2, dS},{0 0 0    0    0  dS}) -g3;

% these are all a function only every of [dth1 dth2 ds].^2
C = [c11 c12 c13
     c21 c22 c23
     c31 c32 c33];

%  one these elements I am plugging in 1 but I know that the dX term is
%  squared
 C = simplify(subs(C,{dTh1 dTh2 dS},{1 1 1}));

M = simplify(M);
% C = simplify(C);


G = [g1 g2 g3].';
latex(M)
latex(C)
latex(G)

pretty(M)
pretty(C)
pretty(G)
matlabFunction(M,'File','Mfun','Vars',{I1 I2 m1 m2 mb L1 L2 g Th1 Th2 S t})
matlabFunction(C,'File','Cfun','Vars',{I1 I2 m1 m2 mb L1 L2 g...
                                       Th1 Th2 S ...
                                       dTh1 dTh2 dS t})
matlabFunction(G,'File', 'Gfun','Vars',{I1 I2 m1 m2 mb L1 L2 g Th1 Th2 S t})



T = simplify(subs(T,timeDepVars,stateVars));
V = simplify(subs(V,timeDepVars,stateVars));
matlabFunction(T,'File','KEfun','Vars',{I1 I2 m1 m2 mb L1 L2 g...
                                       Th1 Th2 S ...
                                       dTh1 dTh2 dS t})

matlabFunction(V,'File','PEfun','Vars',{I1 I2 m1 m2 mb L1 L2 g...
                                       Th1 Th2 S t})


 

%%
% for checking
params = {I1 I2 m1 m2 mb L1 L2 g}
X =      {Th1 Th2 S dTh1 dTh2 dS}
% paramsNStatesDoub={ 1, 1, 1, 1, 1, 1,1, 10, pi/4, pi, 0.5,   1, 2, 3}
paramsNStatesDoub={ 1, 2,3,4,5,6,7,8,9,10,11,12,13,14}

%                                     {I1 I2 m1 m2 mb L1 L2 g   Th1 Th2     S dTh1 dTh2 dS}
Cdoub = double(subs(C,cat(2,params,X),paramsNStatesDoub))
Gdoub = double(subs(G,cat(2,params,X),paramsNStatesDoub))
Mdoub = double(subs(M,cat(2,params,X),paramsNStatesDoub))
%%
syms theta_ddot_1 theta_ddot_2 s_ddot theta_dot_1 theta_dot_2 s_dot ...
    theta_1 theta_2 s
latex(simplify(expand(subs(f3,{ddTh1,ddTh2, ddS,...
           dTh1, dTh2, dS Th1 Th2 S},...
         {theta_ddot_1 theta_ddot_2 s_ddot...
           theta_dot_1 theta_dot_2 s_dot theta_1 theta_2 s}))))

latex(simplify(expand(subs(f4,{ddTh1,ddTh2, ddS,...
           dTh1, dTh2, dS Th1 Th2 S},...
         {theta_ddot_1 theta_ddot_2 s_ddot...
           theta_dot_1 theta_dot_2 s_dot theta_1 theta_2 s}))))

latex(simplify(expand(subs(f5,{ddTh1,ddTh2, ddS,...
           dTh1, dTh2, dS Th1 Th2 S},...
         {theta_ddot_1 theta_ddot_2 s_ddot...
           theta_dot_1 theta_dot_2 s_dot theta_1 theta_2 s}))))

latex(simplify(expand(subs(M,{ddTh1,ddTh2, ddS,...
           dTh1, dTh2, dS Th1 Th2 S},...
         {theta_ddot_1 theta_ddot_2 s_ddot...
           theta_dot_1 theta_dot_2 s_dot theta_1 theta_2 s}))))

latex(simplify(expand(subs(C,{ddTh1,ddTh2, ddS,...
           dTh1, dTh2, dS Th1 Th2 S},...
         {theta_ddot_1 theta_ddot_2 s_ddot...
           theta_dot_1 theta_dot_2 s_dot theta_1 theta_2 s}))))


latex(simplify(expand(subs(G,{ddTh1,ddTh2, ddS,...
           dTh1, dTh2, dS Th1 Th2 S},...
         {theta_ddot_1 theta_ddot_2 s_ddot...
           theta_dot_1 theta_dot_2 s_dot theta_1 theta_2 s}))))


%% now compute the a matrix 
%  Xstar = [pi/4 0 0 0 0 0].'; 
%          [A_lin, B_lin, Ustar]  = findUstarGivenXstar(params,f3,f4,f5,Xstar,debug)

function C = Cfun(I1,I2,m1,m2,mb,L1,L2,g,Th1,Th2,S,dTh1,dTh2,dS,t)
%Cfun
%    C = Cfun(I1,I2,M1,M2,MB,L1,L2,G,Th1,Th2,S,dTh1,dTh2,dS,T)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    27-Apr-2023 12:07:27

t2 = L2.*m2;
t3 = S.*mb.*2.0;
t4 = -Th2;
t5 = Th1+t4;
t7 = t2+t3;
t6 = sin(t5);
t8 = (L1.*t6.*t7)./2.0;
C = reshape([0.0,-t8,-L1.*mb.*cos(t5),t8,0.0,-S.*mb,0.0,0.0,0.0],[3,3]);

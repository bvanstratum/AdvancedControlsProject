function dX = dXfun(I1,I2,m1,m2,mb,L1,L2,g,Th1,Th2,S,dTh1,dTh2,dS,t)
%dXfun
%    dX = dXfun(I1,I2,M1,M2,MB,L1,L2,G,Th1,Th2,S,dTh1,dTh2,dS,T)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    27-Apr-2023 12:07:25

t2 = cos(Th1);
t3 = cos(Th2);
t4 = sin(Th2);
t5 = L1.^2;
t6 = L1.^3;
t7 = L2.^2;
t8 = L2.^3;
t9 = S.^2;
t10 = S.^3;
t11 = dTh1.^2;
t12 = dTh2.^2;
t13 = m2.^2;
t14 = mb.^2;
t15 = -Th2;
t16 = I1.*I2.*1.6e+1;
t17 = Th1+t15;
t18 = I2.*m1.*t5.*4.0;
t19 = I1.*m2.*t7.*4.0;
t22 = I2.*m2.*t5.*1.6e+1;
t23 = I2.*mb.*t5.*1.6e+1;
t24 = I1.*mb.*t9.*1.6e+1;
t29 = m1.*m2.*t5.*t7;
t30 = m2.*mb.*t5.*t7.*4.0;
t31 = m1.*mb.*t5.*t9.*4.0;
t32 = t5.*t7.*t13.*4.0;
t33 = m2.*mb.*t5.*t9.*1.6e+1;
t34 = t5.*t9.*t14.*1.6e+1;
t20 = cos(t17);
t21 = sin(t17);
t25 = t20.^2;
t26 = t20.^3;
t27 = t21.^2;
t28 = t21.^3;
t35 = t23.*t27;
t36 = I2.*mb.*t5.*t27.*-1.6e+1;
t37 = L2.*S.*m2.*mb.*t5.*t25.*1.6e+1;
t38 = t27.*t30;
t39 = t25.*t32;
t41 = m2.*mb.*t5.*t7.*t27.*-4.0;
t42 = t5.*t7.*t13.*t25.*-4.0;
t43 = t25.*t34;
t44 = t27.*t34;
t45 = t5.*t9.*t14.*t25.*-1.6e+1;
t46 = t5.*t9.*t14.*t27.*-1.6e+1;
t40 = -t37;
t47 = t16+t18+t19+t22+t23+t24+t29+t30+t31+t32+t33+t34+t36+t40+t41+t42+t45+t46;
t48 = 1.0./t47;
et1 = I2.*L1.*g.*m1.*t2.*4.0+I2.*L1.*g.*m2.*t2.*8.0+I2.*L1.*g.*mb.*t2.*8.0+L1.*g.*t2.*t7.*t13.*2.0+L1.*g.*t2.*t9.*t14.*8.0+L1.*t8.*t12.*t13.*t21+t5.*t7.*t11.*t13.*t20.*t21.*2.0+I2.*L1.*L2.*m2.*t12.*t21.*4.0+I2.*L1.*dS.*dTh2.*mb.*t20.*1.6e+1+I2.*L1.*g.*mb.*t4.*t21.*8.0+L1.*g.*m1.*m2.*t2.*t7+L1.*g.*m2.*mb.*t2.*t7.*2.0+L1.*g.*m1.*mb.*t2.*t9.*4.0+L1.*g.*m2.*mb.*t2.*t9.*8.0-L1.*g.*t3.*t7.*t13.*t20.*2.0-L1.*g.*t3.*t9.*t14.*t20.*8.0+L1.*g.*t4.*t9.*t14.*t21.*8.0-I2.*mb.*t5.*t11.*t20.*t21.*8.0+L1.*L2.*m2.*mb.*t9.*t12.*t21.*4.0+L1.*dS.*dTh2.*m2.*mb.*t7.*t20.*4.0+L1.*g.*m2.*mb.*t4.*t7.*t21.*2.0-m2.*mb.*t5.*t7.*t11.*t20.*t21.*2.0-L1.*L2.*S.*dS.*dTh2.*m2.*mb.*t20.*8.0;
et2 = L1.*L2.*S.*g.*m2.*mb.*t3.*t20.*-8.0+L2.*S.*m2.*mb.*t5.*t11.*t20.*t21.*8.0;
et3 = I1.*S.*dS.*dTh2.*mb.*-1.6e+1-I1.*L2.*g.*m2.*t3.*4.0-I1.*S.*g.*mb.*t3.*8.0-S.*dS.*dTh2.*t5.*t14.*1.6e+1-L2.*g.*t3.*t5.*t13.*4.0-S.*g.*t3.*t5.*t14.*8.0+L2.*t6.*t11.*t13.*t21.*4.0+S.*t6.*t11.*t14.*t21.*8.0-S.*t6.*t11.*t14.*t28.*8.0+t5.*t7.*t12.*t13.*t20.*t21.*2.0+I1.*L1.*L2.*m2.*t11.*t21.*4.0+I1.*L1.*S.*mb.*t11.*t21.*8.0-S.*dS.*dTh2.*m1.*mb.*t5.*4.0-S.*dS.*dTh2.*m2.*mb.*t5.*1.6e+1-L2.*g.*m1.*m2.*t3.*t5-L2.*g.*m2.*mb.*t3.*t5.*4.0+S.*dS.*dTh2.*t5.*t14.*t25.*1.6e+1+S.*dS.*dTh2.*t5.*t14.*t27.*1.6e+1-S.*g.*m1.*mb.*t3.*t5.*2.0-S.*g.*m2.*mb.*t3.*t5.*8.0+L2.*m1.*m2.*t6.*t11.*t21+L2.*m2.*mb.*t6.*t11.*t21.*4.0-L2.*m2.*mb.*t6.*t11.*t28.*4.0;
et4 = L2.*g.*t2.*t5.*t13.*t20.*4.0+S.*m1.*mb.*t6.*t11.*t21.*2.0+S.*m2.*mb.*t6.*t11.*t21.*8.0+S.*g.*t2.*t5.*t14.*t20.*8.0+S.*g.*t3.*t5.*t14.*t27.*8.0-S.*t6.*t11.*t14.*t21.*t25.*8.0+L2.*dS.*dTh2.*m2.*mb.*t5.*t25.*8.0+L2.*g.*m1.*m2.*t2.*t5.*t20.*2.0+L2.*g.*m2.*mb.*t2.*t5.*t20.*4.0+L2.*g.*m2.*mb.*t3.*t5.*t27.*4.0+S.*g.*m1.*mb.*t2.*t5.*t20.*4.0+S.*g.*m2.*mb.*t2.*t5.*t20.*8.0-L2.*m2.*mb.*t6.*t11.*t21.*t25.*4.0+S.*g.*t4.*t5.*t14.*t20.*t21.*8.0+L2.*S.*m2.*mb.*t5.*t12.*t20.*t21.*4.0+L2.*g.*m2.*mb.*t4.*t5.*t20.*t21.*4.0;
et5 = -S.*t12.*t29+S.*t12.*t35+S.*t12.*t38+S.*t12.*t39+g.*t4.*t16+g.*t4.*t18+g.*t4.*t19+g.*t4.*t22+g.*t4.*t23+g.*t4.*t24+g.*t4.*t29+g.*t4.*t30+g.*t4.*t31+g.*t4.*t32+g.*t4.*t33+g.*t4.*t34+g.*t4.*t42+g.*t4.*t45+L2.*t12.*t25.*t33+g.*t2.*t21.*t22+g.*t2.*t21.*t23+g.*t2.*t21.*t29.*2.0+g.*t2.*t21.*t30+g.*t2.*t21.*t32+g.*t2.*t21.*t33+g.*t2.*t21.*t34-t5.*t10.*t12.*t14.*1.6e+1-I1.*I2.*S.*t12.*1.6e+1-I1.*mb.*t10.*t12.*1.6e+1-I1.*I2.*L1.*t11.*t20.*1.6e+1-I2.*S.*m1.*t5.*t12.*4.0-I2.*S.*m2.*t5.*t12.*1.6e+1-I1.*S.*m2.*t7.*t12.*4.0-I2.*S.*mb.*t5.*t12.*1.6e+1-I2.*m1.*t6.*t11.*t20.*4.0-I2.*m2.*t6.*t11.*t20.*1.6e+1;
et6 = I2.*mb.*t6.*t11.*t20.*-1.6e+1-S.*t5.*t7.*t12.*t13.*4.0-m1.*mb.*t5.*t10.*t12.*4.0-m2.*mb.*t5.*t10.*t12.*1.6e+1-t6.*t7.*t11.*t13.*t20.*4.0-t6.*t9.*t11.*t14.*t20.*1.6e+1+t6.*t7.*t11.*t13.*t26.*4.0+t5.*t8.*t12.*t13.*t27.*2.0+t5.*t10.*t12.*t14.*t25.*1.6e+1+t6.*t9.*t11.*t14.*t26.*1.6e+1+t5.*t10.*t12.*t14.*t27.*1.6e+1-m1.*m2.*t6.*t7.*t11.*t20-m2.*mb.*t6.*t7.*t11.*t20.*4.0-m1.*mb.*t6.*t9.*t11.*t20.*4.0-m2.*mb.*t6.*t9.*t11.*t20.*1.6e+1+t6.*t7.*t11.*t13.*t20.*t27.*4.0+t6.*t9.*t11.*t14.*t20.*t27.*1.6e+1-I1.*L1.*m2.*t7.*t11.*t20.*4.0+I2.*L2.*m2.*t5.*t12.*t27.*8.0-I1.*L1.*mb.*t9.*t11.*t20.*1.6e+1+I2.*g.*m1.*t2.*t5.*t21.*8.0-S.*m2.*mb.*t5.*t7.*t12.*4.0+L2.*S.*m2.*mb.*t6.*t11.*t26.*1.6e+1;
et7 = I2.*dS.*dTh2.*mb.*t5.*t20.*t21.*3.2e+1+L2.*m2.*mb.*t5.*t9.*t12.*t27.*8.0+g.*m1.*mb.*t2.*t5.*t9.*t21.*8.0-g.*t3.*t5.*t7.*t13.*t20.*t21.*4.0-g.*t3.*t5.*t9.*t14.*t20.*t21.*1.6e+1-L2.*S.*g.*m2.*mb.*t4.*t5.*t25.*1.6e+1+L2.*S.*m2.*mb.*t6.*t11.*t20.*t27.*1.6e+1+dS.*dTh2.*m2.*mb.*t5.*t7.*t20.*t21.*8.0-L2.*S.*g.*m2.*mb.*t3.*t5.*t20.*t21.*1.6e+1-L2.*S.*dS.*dTh2.*m2.*mb.*t5.*t20.*t21.*1.6e+1;
dX = [t48.*(et1+et2).*-2.0;t48.*(et3+et4).*2.0;-t48.*(et5+et6+et7)];

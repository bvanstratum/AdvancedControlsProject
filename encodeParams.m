function param = encodeParams(I1,I2,m1,m2,mb,L1,L2,g)
param.I1 = I1;  %rotational inertia of link one 
param.I2 = I2;  %rotational inertia of link two
param.m1 = m1;  %mass of link one
param.m2 = m2;  %mass of link two
param.mb = mb;  %mass of the ball
param.L1 = L1;  %length of link1
param.L2 = L2;  %length of link2
param.g  = g; %acceleration due to gravity
end
close all; clear all; clc
addpath rtb common smtb

%% Compute workspace of robot
L1 = 0.285;
L2 = 0.265;

DH(1) = Link([0 0 L1 0]);
DH(2) = Link([0 0 L2 0]);
th1 = 0:0.05:pi;
th2 = 0:0.05:pi; % 
% th2 = -pi/2:0.05:pi/2;

q = {th1,th2};
 
plotworkspace(DH,q)
hold on

xi = -0.08; yi = -0.15; size = 0.39;
% Square work area corresponding to GUI window
plot([xi, xi, xi - size, xi - size, xi],....
     [yi, yi + size, yi + size, yi, yi],'LineWidth',2,'color','k')
 
plot(xi - size/2, yi + size/2, "b*",'LineWidth',2)

 
 %% Inverse kinematics for initial position
 
 syms L_1 L_2 theta1 theta2 XE YE

XE_RHS = -L_1*sin(theta1) - L_2*cos(theta1+theta2);
YE_RHS = L_1*cos(theta1) - L_2*sin(theta1+theta2);

XE_MLF = matlabFunction(XE_RHS,'Vars',[L_1 L_2 theta1 theta2]);
YE_MLF = matlabFunction(YE_RHS,'Vars',[L_1 L_2 theta1 theta2]);

XE_EQ = XE == XE_RHS;
YE_EQ = YE == YE_RHS;

S = solve([XE_EQ YE_EQ], [theta1 theta2]);

simplify(S.theta1)

TH1_MLF{1} = matlabFunction(S.theta1(1),'Vars',[L_1 L_2 XE YE]);
TH1_MLF{2} = matlabFunction(S.theta1(2),'Vars',[L_1 L_2 XE YE]);
TH2_MLF{1} = matlabFunction(S.theta2(1),'Vars',[L_1 L_2 XE YE]);
TH2_MLF{2} = matlabFunction(S.theta2(2),'Vars',[L_1 L_2 XE YE]);

tmp_th1 = TH1_MLF{1}(L1,L2,xi - size/2, yi + size/2)
tmp_th2 = TH2_MLF{1}(L1,L2,xi - size/2, yi + size/2)

t_step = 0.01;
f = 0.05;
a = 2*pi*f;
t = 0:t_step:1/f;
a_per_b = 0.5;
b = a/a_per_b;
window_size = 400; % square window
d = pi/4;

x = size/2.2*sin(a*t+d) + xi - size/2;
y = size/2.2*cos(b*t) + yi + size/2;

plot(x, y, 'LineWidth',1,'color','b')



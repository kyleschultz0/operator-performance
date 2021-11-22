close all; clear all; clc
addpath rtb common smtb

%% Compute workspace of robot
L1 = 0.285;
L2 = 0.265;

window_size = 1000; % pixels

DH(1) = Link([0 0 L1 0]);
DH(2) = Link([0 0 L2 0]);
th1 = 0:0.05:pi;
th2 = 0:0.05:pi; % 
% th2 = -pi/2:0.05:pi/2;

q = {th1,th2};
 
plotworkspace(DH,q)
hold on

%% Rectangular workspace

xpos = [-0.5 -0.1 0.25 -0.2];
ypos = [-0.15 -0.15 0.48 0.48];

% wrap-around for closed region
xpos(5) = xpos(1);
ypos(5) = ypos(1);

plot(xpos, ypos, 'LineWidth',2,'color','g')

%% Square workspace
xc = -0.1; yc = -0.17; size = 0.37;    % center and size of square

plot([xc, xc, xc - size, xc - size, xc],....
     [yc, yc + size, yc + size, yc, yc],'LineWidth',2,'color','k')
 
 
trajOffx = (window_size/2-100)/window_size*size;

xi = xc - size/2 + size/2.2;
yi = yc + size/2 - size/2.2;
plot(xi, yi, "b*",'LineWidth',2)

 
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

tmp_th1 = TH1_MLF{1}(L1,L2,xi,yi)
tmp_th2 = TH2_MLF{1}(L1,L2,xi,yi)


T = 60;
t_step = 0.01;
t = 0:t_step:10;

type = "chirp";

if type == "circle"
    f = 0.05;
    a = 2*pi*f;
    a_per_b = 0.5;
    b = a/a_per_b;
    d = pi/4;
    
    x = size/2.2*sin(a*t+d) + xc - size/2;
    y = size/2.2*cos(b*t) + yc + size/2;
end

if type == "chirp"
    f0 = 0.001;
    f1 = 0.1;
    fa = f0 + (f1 - f0) * t / T;
    fb = f1 - (f1 - f0) * t / T;
    x = size/2.2.*cos(pi.*fa.*t) + xi - size/2.2;
    y = yi - size/2.2.*cos(pi.*fb.*(T-t)) + size/2.2;
end
    
    

plot(x, y, 'LineWidth',1,'color','b')



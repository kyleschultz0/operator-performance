clc; clear all; close all;
addpath rtb common smtb


%% Trajectory

T = 120;
fs = 100;
num = 30;

t = 0:1/fs:T;


A1 = [0.610, 0.610, 0.610, 0.610, 0.610, 0.610, 0.610, 0.610];

A2 = [0.145, 0.145, 0.145, 0.145, 0.145, 0.145, 0.145, 0.145, 0.145,...
      0.145, 0.145, 0.145]./4;

A = [A1, A2];
 
fd = [0.105, 0.157, 0.419, 0.471, 0.733, 0.785, 1.361, 1.414, 2.094, 2.147,...
    4.084, 4.136, 5.760, 5.812, 7.749, 7.802, 9.268, 9.320, 11.519, 11.572];


rng(2)
phi = 2*pi*rand(30, 20);

x = zeros(num, length(t));

for i = 1:num
    for j = 1:length(fd)
        x(i, :) = x(i, :) + A(j)*sin(fd(j)*t + phi(i, j));
    end
end

xd = 1.2*x./(sum(A));    % normalize to one, low probability of being >1
xd = [t; xd];

writematrix(xd, "sines.csv")


figure;
plot(t, xd(2:end, :))


%% Power spectrum of signals (should match)

figure;
N = length(x(2:end, :));
freq = 0:fs/length(x(2:end, :)):fs/2;

for i = 2:num+1
    xdft = fft(xd(i, :));
    xdft = xdft(1:N/2+1);
    psdx = (1/(fs*N)) * abs(xdft).^2;
    psdx(2:end-1) = 2*psdx(2:end-1);
    power(i - 1, :) = psdx;
end

loglog(freq,power)
title('Periodogram')
xlabel('Frequency (rad/s)')
ylabel('Power/Frequency')
xlim([0, max(fd)+1])
ylim([10^-3, 5*10^1])


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




%% Compute workspace of robot
L1 = 0.285;
L2 = 0.265;

window_size = 1000; % pixels

DH(1) = Link([0 0 L1 0]);
DH(2) = Link([0 0 L2 0]);
th1 = 0:0.05:pi;
th2 = 0:0.05:pi;

q = {th1,th2};

figure;
plotworkspace(DH,q)
hold on


%% Square workspace
xc = -0.1; yc = -0.16; size = 0.37;    % center and size of square

plot([xc, xc, xc - size, xc - size, xc],....
     [yc, yc + size, yc + size, yc, yc],'LineWidth',2,'color','k')
 
axis equal
 
 
trajOffx = (window_size/2-100)/window_size*size;
thetai = zeros(2, num);
for i = 2:num+1
    [theta1i, theta2i, x, y] = ikInit(xd(i, :), L1, L2, TH1_MLF, TH2_MLF);
    plot(x(1), y(1), "b*",'LineWidth',2)
    plot(x(1, :), y(1, :))
    thetai(1, i-1) = theta1i;
    thetai(2, i-1) = theta2i;
end

writematrix(thetai, "thetai.csv")

% t_test = 120; 
% 
% hold on
% figure;
% xlim([-1, 1]);
% ylim([-1, 1]);
% for i = 1:t_test*fs
%     hold on
%     cla
%     plot(xd(2, i), -xd(2, i), 'ro')
%     pause(1/fs)
% end


%% Functions

function [theta1i, theta2i, x, y] = ikInit(xd, L1, L2, TH1_MLF, TH2_MLF)
xc = -0.1; yc = -0.16; size = 0.37;
x = (xc - size/2) - size*xd/2;
y = (yc + size/2) + size*xd/2;
theta1i = TH1_MLF{1}(L1,L2,x(1),y(1));
theta2i = TH2_MLF{1}(L1,L2,x(1),y(1));
end

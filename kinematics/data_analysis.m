clc; clear all; close all


hebi_vel = importfile("C:\Users\Kyle\source\repos\hebi-performance\csv\hebi_0025_uncompensated_6.csv");



figure;
plot(hebi_vel.t, hebi_vel.pos_in1, 'r', hebi_vel.t, hebi_vel.pos1, 'r--',...
     hebi_vel.t, hebi_vel.pos_in2, 'b',  hebi_vel.t, hebi_vel.pos2, 'b--');


legend("Desired x", "Output x", "Desired y", "Output y")
ylim([0, 1100])

figure;
plot(hebi_vel.t, hebi_vel.error)
title("Error Between Desired and Actual Ouput")
xlabel("Time (s)")
ylabel("Distance (pixels")

t = hebi_vel.t;

%% Computing true velocity (red dot)

T = 120;
window_size = 1000;
f = 0.1;

f0 = 0.001;
f1 = 0.1;

for i = 1:length(t)
    fa = f0 + (f1 - f0) * t(i)/T;
    vdx(i) = pi.*fa.*sin(pi.*fa.*t(i));
    vdy(i) = -pi.*fa.*sin(pi.*fa.*t(i));
end
%% Comparing desired and actual velocity


amp = 1000/0.37;

veld1 = hebi_vel.veld1;
veld2 = hebi_vel.veld2;


pos1d = [hebi_vel.pos1];
pos2d = [hebi_vel.pos2];


pos1 = [hebi_vel.pos_in1];
pos2 = [hebi_vel.pos_in2];

vel1 = diff(pos1)./diff(t);
vel2 = -diff(pos2)./diff(t);

vel1 = lowpass(vel1, 0.25)/amp;
vel2 = lowpass(vel2, 0.25)/amp;

vel1d = (diff(pos1d)./diff(t))/amp;
vel2d = -diff(pos2d)./diff(t)/amp;

figure;
plot(t(1:end-1), vel1, 'b', t, veld1, 'b--', t(1:end-1), vel1d, "b.", t(1:end-1),...
    vel2, 'r', t, veld2, 'r--', t(1:end-1), vel2d, "r.")
legend("Actual Velocity 1", "User Desired Velocity 1", "Desired Velocity 1",...
    "Actual Velocity 2", "User Desired Velocity 2", "Desired Velocity 1")



%% RMSE vs time

% RMSE = sqrt((controller.pos1 - controller.pos_in1).^2);
% figure;
% plot(controller.t, RMSE)
% title("Controller Error, x-direction")
% xlabel("Time (s)")
% ylabel("Distance (pixels)")
% 
% RMSE = sqrt((hebi.pos1 - hebi.pos_in1).^2);
% figure;
% plot(hebi.t, RMSE)
% title("HEBI Error, x-direction")
% xlabel("Time (s)")
% ylabel("Distance (pixels)")
% 
% RMSE = sqrt((encoder_uncomp.pos1 - encoder_uncomp.pos_in1).^2);
% figure;
% plot(encoder_uncomp.t, RMSE)
% title("Cables Error, x-direction")
% xlabel("Time (s)")
% ylabel("Distance (pixels)")
% 
% RMSE = sqrt((encoder_comp.pos1 - encoder_comp.pos_in1).^2);
% figure;
% plot(encoder_comp.t, RMSE)
% title("Compensated Error, x-direction")
% xlabel("Time (s)")
% ylabel("Distance (pixels)")

%% Compute errors
% 
% controller_error = sqrt(sum(controller.error.^2)./(length(controller.error)))
% hebi_error = sqrt(sum(hebi.error.^2)./(length(hebi.error)))
% uncomp_error = sqrt(sum(encoder_uncomp.error.^2)./(length(encoder_uncomp.error)))
% comp_error = sqrt(sum(encoder_comp.error.^2)./(length(encoder_comp.error)))
% 
% X = categorical({'Controller','HEBI','Uncompensated','Compensated'});
% X = reordercats(X,{'Controller','HEBI','Uncompensated','Compensated'});
% Y = [controller_error hebi_error uncomp_error comp_error];
% figure;
% bar(X,Y)
% ylabel("RMSE (pixels)")

%% Bode controller

% Ts = 0.01
% L = length(controller.pos1);
% 
% Y = fft(controller.pos1);
% Y = Y(1:L/2);
% U = fft(controller.pos_in1);
% U = U(1:L/2);
% w = 1/Ts*(1:(L/2))/L;
% 
% figure;
% plot(w, U, w, Y)
% 
% G = Y./U;
% 
% mag = sqrt(real(G).^2 + imag(G).^2);
% phase = atan2(imag(G), real(G));
% 
% figure()
% subplot(2,1,1)
% semilogx(w, mag2db(mag), 'ko', 'linewidth', 1.5); hold on; grid on;
% title('Magnitude');
% xlabel('Frequency [Hz]');
% ylabel('Magnitude [dB]');
% xlim([0 0.5])
% subplot(2,1,2)
% semilogx(w, 180/pi*phase, 'ko', 'linewidth', 1.5); hold on; grid on;
% title('Phase');
% xlabel('Frequency [Hz]');
% ylabel('Phase [degree]');
% xlim([0 1])


%% Bode HEBI uncoupled

Ts = 0.01;

L = length(hebi_vel.pos1);

Y = fft(hebi_vel.pos1);
Y = Y(1:L/2);
U = fft(hebi_vel.pos_in1);
U = U(1:L/2);
w = 1/Ts*(1:(L/2))/L;

G = Y./U;

mag = sqrt(real(G).^2 + imag(G).^2);
phase = atan2(imag(G), real(G));

figure()
subplot(2,1,1)
semilogx(w, mag2db(mag), 'ko', 'linewidth', 1.5); hold on; grid on;
title('Magnitude');
xlabel('Frequency [Hz]');
ylabel('Magnitude [dB]');
xlim([0 1])
subplot(2,1,2)
semilogx(w, 180/pi*phase, 'ko', 'linewidth', 1.5); hold on; grid on;
title('Phase');
xlabel('Frequency [Hz]');
ylabel('Phase [degree]');
xlim([0 1])





%% Function to import data

function data = importfilecontrol(filename, dataLines)

%% Input handling

% If dataLines is not specified, define defaults
if nargin < 2
    dataLines = [1, Inf];
end

%% Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 6);

% Specify range and delimiter
opts.DataLines = dataLines;
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["t", "pos_in1", "pos_in2", "pos1", "pos2", "error"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
data = readtable(filename, opts);

end

function data = importfile(filename, dataLines)

%% Input handling

% If dataLines is not specified, define defaults
if nargin < 2
    dataLines = [1, Inf];
end

%% Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 8);

% Specify range and delimiter
opts.DataLines = dataLines;
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["t", "pos_in1", "pos_in2", "pos1", "pos2", "veld1", "veld2", "error"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
data = readtable(filename, opts);

end
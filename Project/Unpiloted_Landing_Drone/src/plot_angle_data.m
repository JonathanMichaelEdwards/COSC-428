%% 3 CartLab - Matlab Code
clc, clear, close all;



%% Reading File(s) - Loading data
data_xyz = load('data/xyz_plotting_data.csv');
data_angle = load('data/angle_plotting_data.csv');


%% Setting up plotting data
% x, y, z data
t_stamp_xyz = data_xyz(:,1);
x = data_xyz(:,2);
y = data_xyz(:,3);
z = data_xyz(:,4);
yaw = data_xyz(:,5);

% angle data
t_stamp_angle = data_angle(:,1);
yaw_angle = data_angle(:,2);
cp_angle = data_angle(:,3);
descent =  data_angle(:,4);   % **Fix** - Writing file in python does not write this data


%% Plotting the (x, y, z) data
% Research:
%   - electronics-08-01532 (3)'
hold on
figure(1)

subplot(4,1,1);
plot(t_stamp_xyz, x)

title("x-axis speed input")
xlabel("Time (s)")
ylabel("Speed (cm/s)")
xlim([15 35])


subplot(4,1,2);
plot(t_stamp_xyz, y)

title("y-axis speed input")
xlabel("Time (s)")
ylabel("Speed (cm/s)")
xlim([15 35])


subplot(4,1,3);
plot(t_stamp_xyz, z)

title("z-axis speed input")
xlabel("Time (s)")
ylabel("Speed (cm/s)")
xlim([15 35])


subplot(4,1,4);
plot(t_stamp_xyz, yaw)

title("yaw speed input")
xlabel("Time (s)")
ylabel("Speed (cm/s)")
xlim([15 35])


hold off




%% Plotting the angle data
hold on
figure(2)


% *** Simulated data ***
% Use splines to interpolate a smoother curve,
% with 10 times as many points,
% that goes exactly through the same data points.
samplingRateIncrease = 10;

spl_yaw = spline(t_stamp_angle, yaw_angle);
spl_cp = spline(t_stamp_angle, cp_angle);
spl_descent = spline(t_stamp_angle, descent);

newXSamplePoints = linspace(min(t_stamp_angle), max(t_stamp_angle), length(t_stamp_angle) * samplingRateIncrease);
xint = linspace(min(t_stamp_angle), max(t_stamp_angle), 15)';

smoothed_yaw = spline(xint, ppval(spl_yaw,xint), newXSamplePoints);
smoothed_cp = spline(xint, ppval(spl_cp,xint), newXSamplePoints);
smoothed_descent = spline(xint, ppval(spl_descent,xint), newXSamplePoints);


% plot(newXSamplePoints, smoothed_yaw, t_stamp_angle, yaw_angle, t_stamp_angle, cp_angle, newXSamplePoints, smoothed_cp)

plot(newXSamplePoints, smoothed_yaw, newXSamplePoints, smoothed_cp)
line(xlim, [0 0], 'Color','black','LineStyle','--')
title("Yaw and Centre Angle")
xlabel("Time (s)")
ylabel("Controlling Angle (deg)")
legend('Yaw Angle', 'Centring point angle')
xlim([15 35])


figure(3)
plot(newXSamplePoints, smoothed_descent, 'g')
line(xlim, [0 0], 'Color','black','LineStyle','--')
title("Descent")
xlabel("Time (s)")
ylabel("Distance (cm)")
xlim([15 35])


% 
% subplot(2,1,1);
% plot(newXSamplePoints, smoothed_yaw)
% line(xlim, [0 0], 'Color','black','LineStyle','--')
% 
% title("Yaw Angle")
% xlabel("Time (s)")
% ylabel("Angle (deg)")
% 
% 
% 
% xlim([min(newXSamplePoints) max(newXSamplePoints)])
% 
% subplot(2,1,2);
% plot(newXSamplePoints, smoothed_cp)
% line(xlim, [0 0], 'Color','black','LineStyle','--')
% 
% title("Centre Point Angle")
% xlabel("Time (s)")
% ylabel("Angle (deg)")


hold off

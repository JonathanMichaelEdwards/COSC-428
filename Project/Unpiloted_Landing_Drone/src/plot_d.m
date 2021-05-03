%% 3 CartLab - Matlab Code
clc, clear, close all;



%% Reading File(s) - Loading data
data_pos = load('data/pos_plotting_data.csv');
data_yaw = load('data/yaw_plotting_data.csv');


%% Setting up plotting data
x_theta = data_pos(:,1);
y_theta = data_pos(:,2);

x_yaw = data_yaw(:,1);
y_yaw = data_yaw(:,2);


%% Plotting the Centre Angle data
hold on
figure(1)

plot(x_theta, y_theta)
xlim([0 50])
ylim([-60 60])
line(xlim, [0 0], 'Color','black','LineStyle','--')
title("Centre Angle control")
xlabel("Time (s)")
ylabel("Centre Angle (deg)")

hold off


%% Plotting the Yaw data
hold on
figure(2)

plot(x_yaw, y_yaw)
xlim([0 50])
ylim([-180 180])
line(xlim, [0 0], 'Color','black','LineStyle','--')
title("Yaw control")
xlabel("Time (s)")
ylabel("Yaw Angle (deg)")

hold off


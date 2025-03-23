% Used for HKUST ELEC 5660

close all;
clc;
clear all;
addpath('./utils','./readonly');
figure(1)
h1 = subplot(3,4,1);
h2 = subplot(3,4,2);
h3 = subplot(3,4,3);
h4 = subplot(3,4,4);
h5 = subplot(3,4,6);
h6 = subplot(3,4,7);
h7 = subplot(3,4,8);
h8 = subplot(3,4,10);
h9 = subplot(3,4,11);
h10 = subplot(3,4,12);
set(gcf, 'Renderer', 'painters');

% Run Trajectory  three trajectories, test one by one
run_trajectory_readonly(h1, h2, h3, h4, h5, h6, h7, h8, h9, h10, @star_trajectory);

calculate_RMSE=@(a,b) sqrt(mean((a(:)-b(:)).^2));
calculate_RMSE_yaw=@(a,b) sqrt(mean(wrapToPi((a(:)-b(:))).^2));

global current_states
global desired_states
rmse_p = calculate_RMSE(current_states(:,1:3),desired_states(:,1:3));
rmse_v = calculate_RMSE(current_states(:,4:6),desired_states(:,4:6));
rmse_yaw = rad2deg(calculate_RMSE_yaw(current_states(:,7),desired_states(:,7)));

disp(['RMSE Position(m):',num2str(rmse_p)])
disp(['RMSE Velocity(m/s):',num2str(rmse_v)])
disp(['RMSE Yaw(deg):',num2str(rmse_yaw)])
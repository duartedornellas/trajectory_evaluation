%% Workspace cleanup

clear all
close all
clc

%% Data input

load('data_000/okvis.csv')
load('data_000/mocap.csv')

%% Dataset trimming

ti = okvis(1,2);
ti_index = 0;
for i=1:size(mocap,1)
    if mocap(i,2) < ti
        ti_index = ti_index + 1;
    else
        mocap(1:ti_index, :) = [];
        break
    end
end
tf = okvis(end,2);
tf_index = 0;
for i=1:size(mocap,1)
    if mocap(i,2) < tf
        tf_index = tf_index + 1;
    else
        mocap(tf_index:end, :) = [];
        break
    end
end

%% Dataset time-alignment

for i = 1:size(okvis,1)
    t_okvis = okvis(i,2);
    [t_mocap, index] = min(abs(mocap(:,2)-t_okvis));
    mocap_aligned(i,:) = mocap(index,:);
end
mocap = mocap_aligned;

%% Dataset geometrical alignment

mocap_p0 = mocap(1,3:5);
mocap_q0 = mocap(1,6:9);

R0 = quat2rotm(mocap_q0);

R = [ 1, 0, 0
      0, 0,-1
      0, 1, 0];

for i=1:size(mocap,1)
    mocap(i,3:5) = (R * R0' * (mocap(i,3:5)-mocap_p0)')';
    mocap(i,6:9) = quatmultiply(mocap(i,6:9), quatconj(mocap_q0));
end

mocap(:,3:4) = -mocap(:,3:4);
okvis(:,3:4) = -okvis(:,3:4);

%% Plots pt. 1

figure;
pause(0.00001);
frame_h = get(handle(gcf),'JavaFrame');
set(frame_h,'Maximized',1);
colors = get(gca,'ColorOrder');
rgb_blue   = colors(1,:);
rgb_red    = colors(2,:);
rgb_yellow = colors(3,:);
rgb_green  = colors(5,:);
rgb_purple = colors(4,:);
rgb_cyan   = colors(6,:);
rgb_dred   = colors(7,:);

subplot(2,3,1)
plot(mocap(1:end,3), mocap(1:end,4))
hold on 
plot(okvis(1:end,3), okvis(1:end,4))
xlim([-0.5,2.5])
ylim([-0.5,2.5])
daspect([1,1,1]);
xlabel('x [m]')
ylabel('y [m]')
legend('Ground-truth', 'OKVIS')
grid minor

error_position = [(mocap(:,3)-okvis(:,3)).^2 + ...
                  (mocap(:,4)-okvis(:,4)).^2 + ...
                  (mocap(:,5)-okvis(:,5)).^2];
subplot(2,3,2)
plot(error_position, 'Color', rgb_dred)
xlabel('Sample counter')
ylabel('Position error [m]')
grid minor

error_quaternion  = quatmultiply(mocap(:,6:9), quatconj(okvis(:,6:9)));
error_axisangle   = quat2axang(error_quaternion);
error_orientation = error_axisangle(:,4) * 180/pi;
subplot(2,3,3)
plot(error_orientation, 'Color', rgb_dred)
xlabel('Sample counter')
ylabel('Orientation error [º]')
grid minor

subplot(2,3,4)
plot(mocap(:,2))
hold on
plot(okvis(:,2))
legend('Ground-truth', 'OKVIS')
grid minor
xlabel('Sample counter')
ylabel('Timestamp')

subplot(2,3,5)
plot(sqrt(mocap(:,3).^2 + mocap(:,4).^2 + mocap(:,5).^2))
hold on
plot(sqrt(okvis(:,3).^2 + okvis(:,4).^2 + okvis(:,5).^2))
xlabel('Sample counter')
ylabel('Position [m]')
legend('Ground-truth', 'OKVIS')
grid minor

okvis_axang = quat2axang(okvis(:,6:9));
mocap_axang = quat2axang(mocap(:,6:9));

subplot(2,3,6)
plot(mocap_axang(:,4) * 180/pi)
hold on
plot(okvis_axang(:,4) * 180/pi)
xlabel('Sample counter')
ylabel('Orientation [º]')
legend('Ground-truth', 'OKVIS')
grid minor

%% Plots pt. 2

figure;
pause(0.00001);
frame_h = get(handle(gcf),'JavaFrame');
set(frame_h,'Maximized',1);
colors = get(gca,'ColorOrder');
rgb_blue   = colors(1,:);
rgb_red    = colors(2,:);
rgb_green  = colors(5,:);

subplot(1,3,1)
plot(mocap(:,3));
hold on
plot(okvis(:,3))
ylim([-0.25,2.5])
xlabel('Sample counter')
ylabel('Position [m]')
legend('MoCap_x', 'OKVIS_x')
grid minor

subplot(1,3,2)
plot(mocap(:,4));
hold on
plot(okvis(:,4))
ylim([-0.25,2.5])
xlabel('Sample counter')
ylabel('Position [m]')
legend('MoCap_y', 'OKVIS_y')
grid minor

subplot(1,3,3)
plot(mocap(:,5));
hold on
plot(okvis(:,5))
ylim([-0.25,2.5])
xlabel('Sample counter')
ylabel('Position [m]')
legend('MoCap_z', 'OKVIS_z')
grid minor

%% Compute error metrics

TPE = rms(error_position);
TOE = rms(error_orientation);

%% Workspace cleanup

clearvars -except mocap okvis TPE TOE


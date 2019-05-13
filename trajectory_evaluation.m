%% Workspace cleanup

close all
clear all
clc

%% Data input

dataset = 'data_000/'

load(strcat(dataset, 'okvis.csv'))
load(strcat(dataset, 'mocap.csv'))

okvis(1:750, :) = [];
mocap(1:750, :) = [];

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

% Mocap to local r.f. rotation matrix
R = [ 1, 0, 0
      0, 0,-1
      0, 1, 0];

% Position/orientation ground-truth data
mocap_p0 = mocap(1,3:5);
mocap_q0 = mocap(1,6:9);

R0 = quat2rotm(mocap_q0);  

% Transform ground-truth to the local r.f.
for i=1:size(mocap,1)
    mocap(i,3:5) = (R * R0' * (mocap(i,3:5)-mocap_p0)')';
    mocap(i,6:9) = quatmultiply(mocap(i,6:9), quatconj(mocap_q0));
end
mocap(:,3:4) = -mocap(:,3:4);
okvis(:,3:4) = -okvis(:,3:4);

% Get orientations as axis/angle vectors
okvis_axang = quat2axang(okvis(:,6:9));
mocap_axang = quat2axang(mocap(:,6:9));

% Enforce positive rotation axis/angle vector
for i=1:size(okvis_axang,1)
    if okvis_axang(i,4) < 0
        okvis_axang(i,:) = -okvis_axang(i,:);
    end
    if mocap_axang(i,4) < 0
        mocap_axang(i,:) = -mocap_axang(i,:);
    end
end

% Rotate axis/angle vector component to the local r.f.
for i=1:size(mocap_axang,1)
    mocap_axang(i,1:3) = (R*mocap_axang(i,1:3)')';
end

% Get orientations as quaternions
okvis_quat = axang2quat(okvis_axang);
mocap_quat = axang2quat(mocap_axang);

%% Error metrics

error_position = [(mocap(:,3)-okvis(:,3)).^2 + ...
                  (mocap(:,4)-okvis(:,4)).^2 + ...
                  (mocap(:,5)-okvis(:,5)).^2];

error_quaternion  = quatmultiply(mocap_quat, quatconj(okvis_quat));
error_axisangle   = quat2axang(error_quaternion);
error_orientation = error_axisangle(:,4) * 180/pi;

TPE = rms(error_position);
TOE = rms(error_orientation);

%% Plots (thesis images)

% 2D Trajectory plot
figure(1);
pause(0.00001);
frame_h = get(handle(1),'JavaFrame');
set(frame_h,'Maximized',1);

plot(mocap(1:end,3), mocap(1:end,4))
hold on 
plot(okvis(1:end,3), okvis(1:end,4))
% xlim([-0.5,2.5])
% ylim([-0.5,2.5])
daspect([1,1,1]);
xlabel('x [m]')
ylabel('y [m]')
legend('Ground-truth', 'OKVIS')
grid minor

saveas(1, strcat(dataset, 'trajectory.epsc'));

% Position error plot
figure(2);
pause(0.00001);
frame_h = get(handle(2),'JavaFrame');
set(frame_h,'Maximized',1);

plot(error_position)
xlabel('Sample counter')
ylabel('Position error [m]')
grid minor
legend(sprintf('TPE = %.3f m', TPE))

saveas(2, strcat(dataset, 'error_pos.epsc'));

% Orientation error plot
figure(3);
pause(0.00001);
frame_h = get(handle(3),'JavaFrame');
set(frame_h,'Maximized',1);

plot(error_orientation)
xlabel('Sample counter')
ylabel('Orientation error [º]')
grid minor
legend(sprintf('TOE = %.3f º', TOE))

saveas(3, strcat(dataset, 'error_ang.epsc'));


%% Plots (general information)

figure(10);
pause(0.00001);
frame_h = get(handle(10),'JavaFrame');
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
% xlim([-0.5,2.5])
% ylim([-0.5,2.5])
daspect([1,1,1]);
xlabel('x [m]')
ylabel('y [m]')
legend('Ground-truth', 'OKVIS')
grid minor

subplot(2,3,2)
plot(error_position, 'Color', rgb_dred)
legend(sprintf('TPE = %.3f m', TPE))
xlabel('Sample counter')
ylabel('Position error [m]')
grid minor

subplot(2,3,3)
plot(error_orientation, 'Color', rgb_dred)
legend(sprintf('TOE = %.3f º', TOE))
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

subplot(2,3,6)
plot(mocap_axang(:,4) * 180/pi)
hold on
plot(okvis_axang(:,4) * 180/pi)
xlabel('Sample counter')
ylabel('Orientation [º]')
legend('Ground-truth', 'OKVIS')
grid minor

saveas(10, strcat(dataset, 'data.jpg'));

%% Plots (DEBUG)
% 
% figure(11);
% pause(0.00001);
% frame_h = get(handle(11),'JavaFrame');
% set(frame_h,'Maximized',1);
% 
% subplot(1,3,1)
% plot(mocap(:,3));
% hold on
% plot(okvis(:,3))
% ylim([-0.25,2.5])
% xlabel('Sample counter')
% ylabel('Position [m]')
% legend('MoCap_x', 'OKVIS_x')
% grid minor
% 
% subplot(1,3,2)
% plot(mocap(:,4));
% hold on
% plot(okvis(:,4))
% ylim([-0.25,2.5])
% xlabel('Sample counter')
% ylabel('Position [m]')
% legend('MoCap_y', 'OKVIS_y')
% grid minor
% 
% subplot(1,3,3)
% plot(mocap(:,5));
% hold on
% plot(okvis(:,5))
% ylim([-0.25,2.5])
% xlabel('Sample counter')
% ylabel('Position [m]')
% legend('MoCap_z', 'OKVIS_z')
% grid minor
% 
% 
% figure(12);
% pause(0.00001);
% frame_h = get(handle(12),'JavaFrame');
% set(frame_h,'Maximized',1);
% 
% subplot(1,4,1)
% plot(mocap_axang(:,1));
% hold on
% plot(okvis_axang(:,1))
% xlabel('Sample counter')
% legend('MoCap_{Wx}', 'OKVIS_{Wx}')
% grid minor
% 
% subplot(1,4,2)
% plot(mocap_axang(:,2));
% hold on
% plot(okvis_axang(:,2))
% xlabel('Sample counter')
% legend('MoCap_{Wy}', 'OKVIS_{Wy}')
% grid minor
% 
% subplot(1,4,3)
% plot(mocap_axang(:,3));
% hold on
% plot(okvis_axang(:,3))
% xlabel('Sample counter')
% legend('MoCap_{Wz}', 'OKVIS_{Wz}')
% grid minor
% 
% subplot(1,4,4)
% plot(mocap_axang(:,4)*180/pi);
% hold on
% plot(okvis_axang(:,4)*180/pi)
% xlabel('Sample counter')
% legend('MoCap_{\theta}', 'OKVIS_{\theta}')
% grid minor
% 
% 
% figure(13);
% pause(0.00001);
% frame_h = get(handle(13),'JavaFrame');
% set(frame_h,'Maximized',1);
% 
% subplot(1,4,1)
% plot(mocap_quat(:,1));
% hold on
% plot(okvis(:,6))
% xlabel('Sample counter')
% legend('MoCap_{qw}', 'OKVIS_{qw}')
% grid minor
% 
% subplot(1,4,2)
% plot(mocap_quat(:,2));
% hold on
% plot(okvis(:,7))
% xlabel('Sample counter')
% legend('MoCap_{qx}', 'OKVIS_{qx}')
% grid minor
% 
% subplot(1,4,3)
% plot(mocap_quat(:,3));
% hold on
% plot(okvis(:,8))
% xlabel('Sample counter')
% legend('MoCap_{qy}', 'OKVIS_{qy}')
% grid minor
% 
% subplot(1,4,4)
% plot(mocap_quat(:,4));
% hold on
% plot(okvis(:,9))
% xlabel('Sample counter')
% legend('MoCap_{qz}', 'OKVIS_{qz}')
% grid minor


%% Workspace cleanup

clearvars -except mocap okvis TPE TOE


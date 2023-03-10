%% Clean environment
clear; clc; close all;

%% Read IMU data from BAG file
% sim('read_imu_data.slx',30)

axang = {};
pt = [0 0 0];

%% Create quaternion
axang = [imu_orientation.W.Data imu_orientation.X.Data imu_orientation.Y.Data imu_orientation.Z.Data];
quat = axang2quat(axang);

%% Convert to RPY format
RPYel = quat2eul(quat);

%% Plot object
for i=1 : length(RPYel)
    quiver3(pt(1),pt(2),pt(3), RPYel(i),RPYel(i),RPYel(i))
    pause(1)
end

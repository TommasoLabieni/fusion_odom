close all
clc
format long     %% evita di troncare i decimali
if isempty(out)
    out = sim('/home/tommaso/Documents/Tesi/Matlab_Examples/Test_Bag_Reader/read_imu_data.slx');
end

imu_data_sz = length(imu_orientation.Orientation.X.Data);

tp = theaterPlot('XLimit',[-1 1],'YLimit',[-1 1],'ZLimit',[-1 1]);
set(gca,'Ydir', 'reverse')
op = orientationPlotter(tp,'DisplayName','Fused Data',...
    'LocalAxesLength',2);

pose = randrot(200,1);

for i=2:imu_data_sz
    qx = imu_orientation.Orientation.X.Data(i);
    qy = imu_orientation.Orientation.Y.Data(i);
    qz = imu_orientation.Orientation.Z.Data(i);
    qw = imu_orientation.Orientation.W.Data(i);

    % All angles are rad !!
    yaw = atan2(2.0*(qy*qz + qw*qx), qw*qw - qx*qx - qy*qy + qz*qz);
    pitch = asin(-2.0*(qx*qz - qw*qy));
    roll = atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz);

    % Get rotation angles in degrees
    xyz = rad2deg(quat2eul([qw qx qy qz], 'XYZ'));

    x = xyz(1);
    y = xyz(2);
    z = xyz(3);

    % Somehow the Z axis is reversed -> you have to use -z
    a = 0:9000;
    plotOrientation(op, x, y, -z)
    %plotOrientation(op, rad2deg(roll), rad2deg(pitch), rad2deg(yaw))
    drawnow
    fprintf('roll: %f pitch: %f yaw: %f\n', roll, pitch, yaw)
end

% for i=1:numel(pose)
%     new_pose = pose(i);
%     plotOrientation(op,pose(i))
%     drawnow
% end
clear all
% close all
clc

%% Read bag
bag = rosbag('/home/tommaso/bags/gps_imu_two_laps_10hz.bag');


gpsTopic = select(bag,'Topic','/gps/position');

gpsData = readMessages(gpsTopic,'DataFormat','struct');

gpsDataSz = gpsTopic.NumMessages;

gps_position = zeros(gpsDataSz,3);

for i=1 : gpsDataSz
    tmp_lat = gpsData{i}.Latitude;
    tmp_lon = gpsData{i}.Longitude;
    tmp_alt = gpsData{i}.Altitude;
    gps_position(i,:) = [tmp_lat tmp_lon tmp_alt];
end

%% Create empty map projection
proj = defaultm("mercator");

color = [".b", ".g"];


%% Extrack x, y coordinates
[x,y] = projfwd(proj,gps_position(:,1),gps_position(:,2));

init_pos = [x(1) y(1)];

figure();

color_index = 1;

for i = 1:gpsDataSz
    act_pos = [x(i) y(i)];
    distance = sqrt((act_pos(1) - init_pos(1)).^2 + (act_pos(2) - init_pos(2)).^2);
    if (i > 50) && (distance < 0.000001)
        fprintf("NEW LAP. DISTANCE: %lf", distance);
        color_index = 2;
    end
    plot(x(i), y(i), color(color_index), 'markersize', 8)
    hold on     %% Senza hold on lui ti plotta SOLO l'ultimo punto
end
grid on
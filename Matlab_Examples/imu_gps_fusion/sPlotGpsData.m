%% Read BAG from simulink model
format long     %% evita di troncare i decimali
load('bag_gps_imu.mat')

gpsDataSz = length(gps_position.Latitude.Data(2:end));
%% Create empty map projection
proj = defaultm("mercator");

color = [".b", ".g"]


%% Extrack x, y coordinates
[x,y] = projfwd(proj,gps_position.Latitude.Data(2:end),gps_position.Longitude.Data(2:end));

figure();

for i = 1:gpsDataSz
    plot(x(i), y(i), color(rem(i,length(color))+1), 'markersize', 8)
    hold on     %% Senza hold on lui ti plotta SOLO l'ultimo punto
end
grid on
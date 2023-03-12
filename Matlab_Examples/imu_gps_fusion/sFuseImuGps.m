close all
clear
clc

imuFs = 100;
gpsFs = 10;

format long     %% evita di troncare i decimali

%% Read bag
bag = rosbag('/home/tommaso/bags/gps_imu_two_laps_10hz.bag');


gpsTopic = select(bag,'Topic','/gps/position');
imuTopic = select(bag,'Topic','/imu/data');

gpsData = readMessages(gpsTopic,'DataFormat','struct');
imuData = readMessages(imuTopic,'DataFormat','struct');

gpsDataSz = gpsTopic.NumMessages;
imuDataSz = imuTopic.NumMessages;

gps_position = zeros(gpsDataSz,3);
imu_orientation.Orientation = zeros(imuDataSz,4);
imu_orientation.LinearAcceleration = zeros(imuDataSz,3);
imu_orientation.AngularVelocity = zeros(imuDataSz,3);

% Fill GPS data
for i=1 : gpsDataSz
    tmp_lat = gpsData{i}.Latitude;
    tmp_lon = gpsData{i}.Longitude;
    tmp_alt = gpsData{i}.Altitude;
    gps_position(i,:) = [tmp_lat tmp_lon tmp_alt];
end

imu_orientation.Orientation = zeros(imuDataSz,4);
imu_orientation.LinearAcceleration = zeros(imuDataSz,3);
imu_orientation.AngularVelocity =zeros(imuDataSz,3);

% Fill IMU Data

for i=1 : imuDataSz
    tmp_orientation = [imuData{i}.Orientation.X, imuData{i}.Orientation.Y, imuData{i}.Orientation.Z, imuData{i}.Orientation.W];
    tmp_acc = [imuData{i}.LinearAcceleration.X, imuData{i}.LinearAcceleration.Y, imuData{i}.LinearAcceleration.Z];
    tmp_gyro = [imuData{i}.AngularVelocity.X, imuData{i}.AngularVelocity.Y, imuData{i}.AngularVelocity.Z];
    imu_orientation.Orientation(i,:) = tmp_orientation;
    imu_orientation.LinearAcceleration(i,:) = tmp_acc;
    imu_orientation.AngularVelocity(i,:) = tmp_gyro;
end


%% EKF State initialization

qx_init = imu_orientation.Orientation(1,1);
qy_init = imu_orientation.Orientation(1,2);
qz_init = imu_orientation.Orientation(1,3);
qw_init = imu_orientation.Orientation(1,4);

latitude_init = gps_position(1,1);
longitude_init = gps_position(1,2);
altitude_init = gps_position(1,3);

% Define where on the Earth this simulated scenario takes place using the
% latitude, longitude and altitude.
localOrigin = [latitude_init longitude_init altitude_init];

gndFusion = insfilterNonholonomic('ReferenceFrame', 'NED', ...
    'IMUSampleRate', imuFs, ...
    'ReferenceLocation', localOrigin, ...
    'DecimationFactor', 1);

% Initialize the states of the filter
gndFusion.State(1:4) = compact(quaternion(qw_init, qx_init, qy_init, qz_init));
gndFusion.State(5:7) = [0.000265955156806132 0.000118613066929290 -0.000865860620781665];
gndFusion.State(8:10) = 0;
gndFusion.State(11:13) = 0;
gndFusion.State(14:16) = [9.42558485122576e-05 -2.35931658257626e-05 0.000160709506499922];

% Measurement noises
Rvel = 0;
Rpos = 0.005;

% The dynamic model of the ground vehicle for this filter assumes there is
% no side slip or skid during movement. This means that the velocity is 
% constrained to only the forward body axis. The other two velocity axis 
% readings are corrected with a zero measurement weighted by the 
% |ZeroVelocityConstraintNoise| parameter.
gndFusion.ZeroVelocityConstraintNoise = 1e-3; %e-2

% Process noises
gndFusion.GyroscopeNoise = 4.8e-2;    %4e-6
gndFusion.GyroscopeBiasNoise = 4e-14; %e-14
gndFusion.AccelerometerNoise = 4.8e-2; %e-5
gndFusion.AccelerometerBiasNoise = 4e-14; %e-14

% Initial error covariance
gndFusion.StateCovariance = 1e-9*eye(16);


useErrScope = false; % Turn on the streaming error plot
usePoseView = true;  % Turn on the 3D pose viewer
use2DView = ~usePoseView;

if useErrScope
    errscope = HelperScrollingPlotter( ...
            'NumInputs', 4, ...
            'TimeSpan', 10, ...
            'SampleRate', imuFs, ...
            'YLabel', {'degrees', ...
            'meters', ...
            'meters', ...
            'meters'}, ...
            'Title', {'Quaternion Distance', ...
            'Position X Error', ...
            'Position Y Error', ...
            'Position Z Error'}, ...
            'YLimits', ...
            [-1, 1
             -1, 1
             -1, 1
             -1, 1]);
end

if usePoseView
    viewer = HelperPoseViewer( ...
        'XPositionLimits', [-5, 25], ...
        'YPositionLimits', [-15, 15], ...
        'ZPositionLimits', [-2, 2], ...
        'ReferenceFrame', 'NED');
end


totalSimTime = round(bag.EndTime - bag.StartTime) - 1; % seconds

% Log data for final metric computation.
numsamples = floor(totalSimTime * gpsFs);
truePosition = zeros(numsamples,3);
trueOrientation = quaternion.zeros(numsamples,1);
estPosition = zeros(numsamples,3);
estOrientation = quaternion.zeros(numsamples,1);

idx = 0;
imuSamplesPerGPS = imuFs / gpsFs;

index_gps = 1;

if use2DView
    figure();
end

for sampleIdx = 1:numsamples
    % Predict loop at IMU update frequency.
    for i = 1:imuSamplesPerGPS
        idx = idx + 1;
        
        accel_x = imu_orientation.LinearAcceleration(i*sampleIdx,1);
        if accel_x == 0
            fprintf('SKIPPED IMU\n')
            continue
        end
        accel_y = imu_orientation.LinearAcceleration(i*sampleIdx,2);
        accel_z = imu_orientation.LinearAcceleration(i*sampleIdx,3);

        gyro_x = imu_orientation.AngularVelocity(i*sampleIdx,1);
        gyro_y = imu_orientation.AngularVelocity(i*sampleIdx,2);
        gyro_z = imu_orientation.AngularVelocity(i*sampleIdx,3);

        accelData = [accel_x accel_y accel_z];

        gyroData = [gyro_x gyro_y gyro_z];
        
        % Use the predict method to estimate the filter state based
        % on the accelData and gyroData arrays.
        predict(gndFusion, accelData, gyroData);
        
        % Log the estimated orientation and position.
        [estPosition(idx,:), estOrientation(idx,:)] = pose(gndFusion);
        
        % Compute the errors and plot.
        if useErrScope
            orientErr = rad2deg( ...
                dist(estOrientation(idx,:), trueOrientation(idx,:)));
            posErr = estPosition(idx,:) - truePosition(idx,:);
            errscope(orientErr, posErr(1), posErr(2), posErr(3));
        end

        % Update the pose viewer.
        if usePoseView
            viewer(estPosition(idx,:), estOrientation(idx,:), ...
                estPosition(idx,:), estOrientation(idx,:));
        else
            plot(estPosition(idx,2), estPosition(idx,1),'.k' ,'Markersize', 10);
            xlim([-30 30])
            ylim([-30 30])
            drawnow limitrate
            hold on;
            grid on;
        end
    end
    
    latitude = gps_position(sampleIdx,1);
    if latitude == 0
        fprintf("SKIPPED GPS")
        continue
    end
    longitude = gps_position(sampleIdx,2);
    altitude = gps_position(sampleIdx,3);
%     altitude = altitude_init;

    lla = [latitude longitude altitude];
    if index_gps > 1
        lat_prev = gps_position(sampleIdx-1,1);
        lng_prev = gps_position(sampleIdx-1,2);
%         alt_prev = gps_position(sampleIdx-1,3);
        alt_prev = altitude; %assuming no altitude deviation
        rv = measure_distance(latitude, longitude, altitude, lat_prev, lng_prev, alt_prev, (1 / gpsFs));
    else
        % during 1st set dummy speed to avoid error in pose correction
        rv = [0.1, 0.1, 0]; %measure_distance(1, 0, 0, 0, 0, 0, (1 / gpsFs));
    end
    vx = rv(1);
    vy = rv(2);
    vz = 0;
    gpsVel = [vx vy vz];

    % Update the filter states based on the GPS data.
    fusegps(gndFusion, lla, Rpos, gpsVel, Rvel);
    index_gps = index_gps + 1;
end











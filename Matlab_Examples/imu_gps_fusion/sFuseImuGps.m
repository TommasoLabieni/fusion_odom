close all
clear all
clc

imuFs = 100;
gpsFs = 1;

format long     %% evita di troncare i decimali
load('bag_gps_imu.mat')
load('sample_init_state.mat')

qx_init = imu_orientation.Orientation.X.Data(2);
qy_init = imu_orientation.Orientation.Y.Data(2);
qz_init = imu_orientation.Orientation.Z.Data(2);
qw_init = imu_orientation.Orientation.W.Data(2);

latitude_init = gps_position.Latitude.Data(2);
longitude_init = gps_position.Longitude.Data(2);
altitude_init = gps_position.Altitude.Data(2);

% Define where on the Earth this simulated scenario takes place using the
% latitude, longitude and altitude.
localOrigin = [latitude_init longitude_init altitude_init];

gndFusion = insfilterNonholonomic('ReferenceFrame', 'NED', ...
    'IMUSampleRate', imuFs, ...
    'ReferenceLocation', localOrigin, ...
    'DecimationFactor', 2);

% Initialize the states of the filter
gndFusion.State(1:4) = compact(quaternion(qw_init, qx_init, qy_init, qz_init));
gndFusion.State(5:7) = 0;
gndFusion.State(8:10) = 0;
gndFusion.State(11:13) = 0;
gndFusion.State(14:16) = 0;

% Measurement noises
Rvel = 0;
Rpos = 0.01;

% The dynamic model of the ground vehicle for this filter assumes there is
% no side slip or skid during movement. This means that the velocity is 
% constrained to only the forward body axis. The other two velocity axis 
% readings are corrected with a zero measurement weighted by the 
% |ZeroVelocityConstraintNoise| parameter.
gndFusion.ZeroVelocityConstraintNoise = 1e-2;

% Process noises
gndFusion.GyroscopeNoise = 4e-3;    %e-6
gndFusion.GyroscopeBiasNoise = 4e-10; %e-14
gndFusion.AccelerometerNoise = 4.8e-5;
gndFusion.AccelerometerBiasNoise = 4e-14;

% Initial error covariance
gndFusion.StateCovariance = 1e-9*eye(16);


useErrScope = false; % Turn on the streaming error plot
usePoseView = true;  % Turn on the 3D pose viewer

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
        'XPositionLimits', [-40, 40], ...
        'YPositionLimits', [-40, 40], ...
        'ZPositionLimits', [-2, 2], ...
        'ReferenceFrame', 'NED');
end


totalSimTime = 90; % seconds

% Log data for final metric computation.
numsamples = floor(totalSimTime * gpsFs);
truePosition = zeros(numsamples,3);
trueOrientation = quaternion.zeros(numsamples,1);
estPosition = zeros(numsamples,3);
estOrientation = quaternion.zeros(numsamples,1);

idx = 0;
imuSamplesPerGPS = imuFs / gpsFs;

index_gps = 2;

for sampleIdx = 1:numsamples
    % Predict loop at IMU update frequency.
    for i = 2:imuSamplesPerGPS
        idx = idx + 1;
        
        accel_x = imu_orientation.LinearAcceleration.X.Data(i*sampleIdx);
        accel_y = imu_orientation.LinearAcceleration.Y.Data(i*sampleIdx);
        accel_z = imu_orientation.LinearAcceleration.Z.Data(i*sampleIdx);

        gyro_x = imu_orientation.AngularVelocity.X.Data(i*sampleIdx);
        gyro_y = imu_orientation.AngularVelocity.Y.Data(i*sampleIdx);
        gyro_z = imu_orientation.AngularVelocity.Z.Data(i*sampleIdx);

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
        end
    end
    
    latitude = gps_position.Latitude.Data(index_gps);
    longitude = gps_position.Longitude.Data(index_gps);
    %altitude = gps_position.Altitude.Data(index_gps);
    altitude = altitude_init;
    %altitude_v(sampleIdx-1,:) = altitude

    lla = [latitude longitude altitude];
    if index_gps > 2
        lat_prev = gps_position.Latitude.Data(index_gps-1);
        lng_prev = gps_position.Longitude.Data(index_gps-1);
        alt_prev = gps_position.Altitude.Data(index_gps-1);
        rv = measure_distance(latitude, longitude, altitude, lat_prev, lng_prev, alt_prev);
    else
        % during 1st iteration speed = 0
        rv = measure_distance(1, 0, 0, 0, 0, 0);
    end
    vx = rv(1);
    vy = rv(2);
    vz = 0;
    gpsVel = [vx vy vz];

    % Update the filter states based on the GPS data.
    fusegps(gndFusion, lla, Rpos, gpsVel, Rvel);
    index_gps = index_gps + 1;
end











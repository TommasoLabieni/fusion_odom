%%clear
close
close all
clc
%example documentation: https://it.mathworks.com/help/nav/ug/estimate-position-and-orientation-of-a-ground-vehicle.html
%example video: https://it.mathworks.com/support/search.html/videos/sensor-fusion-part-3-fusing-a-gps-and-an-imu-to-estimate-pose-1569911082630.html?fq%5B%5D=asset_type_name:video&fq%5B%5D=category:fusion/index&page=1
%% Read BAG from simulink model
%%out = [];
format long     %% evita di troncare i decimali
if isempty(out)
    out = sim('/home/tommaso/Documents/Tesi/Matlab_Examples/Test_Bag_Reader/read_imu_data.slx');
end

imuDataSz = length(imu_orientation.Orientation.X.Data(2:end));
gpsDataSz = length(gps_position.Latitude.Data(2:end));
imuSamplesPerGPS = (imuDataSz/gpsDataSz);

% Estimate Position and Orientation of a Ground Vehicle
% This example shows how to estimate the position and orientation of ground vehicles by fusing data from an inertial measurement unit (IMU) and a global positioning system (GPS) receiver. 
% Simulation Setup
% Set the sampling rates. In a typical system, the accelerometer and gyroscope in the IMU run at relatively high sample rates. The complexity of processing data from those sensors in the fusion algorithm is relatively low. Conversely, the GPS runs at a relatively low sample rate and the complexity associated with processing it is high. In this fusion algorithm the GPS samples are processed at a low rate, and the accelerometer and gyroscope samples are processed together at the same high rate.
% To simulate this configuration, the IMU (accelerometer and gyroscope) is sampled at 100 Hz, and the GPS is sampled at 10 Hz.
imuFs = 100;
gpsFs = 1;

% Define where on the Earth this simulation takes place using latitude, 
% longitude, and altitude (LLA) coordinates.
localOrigin = [44.6291196880000 10.9501188220000 35.3170000000000];


%% Create Filter
% Fusion Filter
% Create the filter to fuse IMU + GPS measurements. The fusion filter uses an extended Kalman filter to track orientation (as a quaternion), position, velocity, and sensor biases.
% The insfilterNonholonomic object has two main methods: predict and fusegps. The predict method takes the accelerometer and gyroscope samples from the IMU as input. Call the predict method each time the accelerometer and gyroscope are sampled. This method predicts the states forward one time step based on the accelerometer and gyroscope. The error covariance of the extended Kalman filter is updated in this step.
% The fusegps method takes the GPS samples as input. This method updates the filter states based on the GPS sample by computing a Kalman gain that weights the various sensor inputs according to their uncertainty. An error covariance is also updated in this step, this time using the Kalman gain as well.
% The insfilterNonholonomic object has two main properties: IMUSampleRate and DecimationFactor. The ground vehicle has two velocity constraints that assume it does not bounce off the ground or slide on the ground. These constraints are applied using the extended Kalman filter update equations. These updates are applied to the filter states at a rate of IMUSampleRate/DecimationFactor Hz.
gndFusion = insfilterNonholonomic('ReferenceFrame', 'ENU', ...
    'IMUSampleRate', imuFs, ...
    'ReferenceLocation', localOrigin, ...
    'DecimationFactor', 10);

% Initialize the States of the insfilterNonholonomic
% The states are:
% States                            Units    Index
% Orientation (quaternion parts)             1:4  
% Gyroscope Bias (XYZ)              rad/s    5:7  
% Position (NED)                    m        8:10 
% Velocity (NED)                    m/s      11:13
% Accelerometer Bias (XYZ)          m/s^2    14:16

idx = 2;
% Initialize the states of the filter
gndFusion.State(1:4) = [imu_orientation.Orientation.X.Data(idx), imu_orientation.Orientation.Y.Data(idx), imu_orientation.Orientation.Z.Data(idx), imu_orientation.Orientation.W.Data(idx)];
gndFusion.State(5:7) = 0;
gndFusion.State(8:10) = [gps_position.Latitude.Data(idx), gps_position.Longitude.Data(idx), 0];
gndFusion.State(11:13) = 0;
gndFusion.State(14:16) = 0;

% Initialize the Variances of the insfilterNonholonomic
% The measurement noises describe how much noise is corrupting the GPS reading based on the gpsSensor parameters and how much uncertainty is in the vehicle dynamic model.
% The process noises describe how well the filter equations describe the state evolution. Process noises are determined empirically using parameter sweeping to jointly optimize position and orientation estimates from the filter. 
% Measurement noises

Rvel = 0.1.^2;
Rpos = 1.0.^2;

% The dynamic model of the ground vehicle for this filter assumes there is
% no side slip or skid during movement. This means that the velocity is 
% constrained to only the forward body axis. The other two velocity axis 
% readings are corrected with a zero measurement weighted by the 
% |ZeroVelocityConstraintNoise| parameter.
gndFusion.ZeroVelocityConstraintNoise = 1e-2;

% Process noises
gndFusion.GyroscopeNoise = 4e-6;
gndFusion.GyroscopeBiasNoise = 4e-14;
gndFusion.AccelerometerNoise = 4.8e-5;
gndFusion.AccelerometerBiasNoise = 4e-14;

% Initial error covariance
gndFusion.StateCovariance = 1e-9*eye(16);

% Initialize Scopes
% The HelperScrollingPlotter scope enables plotting of variables over time. It is used here to track errors in pose. The HelperPoseViewer scope allows 3-D visualization of the filter estimate and ground truth pose. The scopes can slow the simulation. To disable a scope, set the corresponding logical variable to false.
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
        'XPositionLimits', [-10000, 10000], ...
        'YPositionLimits', [-10000, 10000], ...
        'ZPositionLimits', [-10000, 10000], ...
        'ReferenceFrame', 'ENU');
end

% Simulation Loop
% The main simulation loop is a while loop with a nested for loop. The while loop executes at the gpsFs, which is the GPS measurement rate. The nested for loop executes at the imuFs, which is the IMU sample rate. The scopes are updated at the IMU sample rate.
totalSimTime = 90; % seconds

% Log data for final metric computation.
numsamples = floor(totalSimTime) * gpsFs;
truePosition = zeros(numsamples,3);
trueOrientation = quaternion.zeros(numsamples,1);
estPosition = zeros(numsamples,3);
estOrientation = quaternion.zeros(numsamples,1);

idx = 2;

for sampleIdx = 1:numsamples
    % Predict loop at IMU update frequency.
    for i = 2:imuSamplesPerGPS
        % Get IMU accelerations along X, Y and Z axes
        accelData = [imu_orientation.LinearAcceleration.X.Data(i*sampleIdx), imu_orientation.LinearAcceleration.Y.Data(i*sampleIdx), -9.8];
        gyroData = [imu_orientation.Orientation.X.Data(i*sampleIdx), imu_orientation.Orientation.Y.Data(i*sampleIdx), imu_orientation.Orientation.Z.Data(i*sampleIdx)];

        % Use the predict method to estimate the filter state based
        % on the accelData and gyroData arrays.
        predict(gndFusion, accelData, gyroData);
        
        % Log the estimated orientation and position.
        [estPosition(i,:), estOrientation(i,:)] = pose(gndFusion);
        truePosition(sampleIdx,:) = estPosition(i,:);
        truePosition(sampleIdx,:)
        % Compute the errors and plot.
        %if useErrScope
        %    orientErr = rad2deg( ...
        %        dist(estOrientation(idx,:), trueOrientation(idx,:)));
        %    posErr = estPosition(idx,:) - truePosition(idx,:);
        %    errscope(orientErr, posErr(1), posErr(2), posErr(3));
        %end
        % Update the pose viewer.
        if usePoseView
            viewer(estPosition(i,:), estOrientation(i,:), ...
                truePosition(sampleIdx,:), estOrientation(i,:));
        end
    end
    
    latitude = gps_position.Latitude.Data(idx);   
    longitude = gps_position.Longitude.Data(idx);
    altitude = gps_position.Altitude.Data(idx);

    lla = [latitude, longitude, altitude];
    % This next step happens at the GPS sample rate.
    % Simulate the GPS output based on the current pose.
    % Update the filter states based on the GPS data.
    fusegps(gndFusion, lla, Rpos, [5, 0,0], Rvel);
    idx = idx + 1;
end

% Error Metric Computation
% Position and orientation were logged throughout the simulation. Now compute an end-to-end root mean squared error for both position and orientation.
%posd = estPosition - truePosition;

% For orientation, quaternion distance is a much better alternative to
% subtracting Euler angles, which have discontinuities. The quaternion
% distance can be computed with the |dist| function, which gives the
% angular difference in orientation in radians. Convert to degrees for
% display in the command window.

% quatd = rad2deg(dist(estOrientation, trueOrientation));

% Display RMS errors in the command window.
%fprintf('\n\nEnd-to-End Simulation Position RMS Error\n');
%msep = sqrt(mean(posd.^2));
%fprintf('\tX: %.2f , Y: %.2f, Z: %.2f   (meters)\n\n', msep(1), ...
%    msep(2), msep(3));

%fprintf('End-to-End Quaternion Distance RMS Error (degrees) \n');
%fprintf('\t%.2f (degrees)\n\n', sqrt(mean(quatd.^2)));

function [fusedX, fusedY, fusedVX, fusedVY, time] = multiSensorFusionVelocityForCollisionAvoidance_v5()
    % Generate multi-sensor data from the driving scenario
    [allData, scenario, sensors] = sonnastrasseDatageneration();  % Data generation function

    % Initialize Kalman Filters for each sensor type
    kfRadar = configureKalmanFilterForRadar();
    kfLidar = configureKalmanFilterForLidar();
    kfVision = configureKalmanFilterForVision();

    % Initialize arrays to store results
    fusedFilteredX = [];
    fusedFilteredY = [];
    fusedRawX = [];
    fusedRawY = [];
    time = [];

    % Process multi-sensor data
    for i = 1:numel(allData)
        objectDetections = allData(i).ObjectDetections;
        radarEstimate = [];
        lidarEstimate = [];
        visionEstimate = [];

        for j = 1:numel(objectDetections)
            detection = objectDetections{j};
            measurement = detection.Measurement(1:2);  % Extract x, y position
            sensorIndex = detection.SensorIndex;

            switch sensorIndex
                case 1  % Radar
                    kfRadar.correct(measurement);
                    radarEstimate = kfRadar.State(1:2);                
                case 2  % Lidar
                    kfLidar.correct(measurement);
                    lidarEstimate = kfLidar.State(1:2);
                case 3  % Vision
                    kfVision.correct(measurement);
                    visionEstimate = kfVision.State(1:2);
            end
        end

        % Fusion Logic: Combine estimates from different sensors
        estimates = [radarEstimate, lidarEstimate, visionEstimate];  % 2xN matrix

        % Select columns where both X and Y are non-NaN
        validIdx = all(~isnan(estimates), 1);  % Logical index for valid columns
        validEstimates = estimates(:, validIdx);  % Select valid estimates

        if ~isempty(validEstimates)
            % Average of the valid estimates
            fusedEstimate = mean(validEstimates, 2);  % 2x1 vector (mean of valid X and Y)
            fusedFilteredX(end+1) = fusedEstimate(1);
            fusedFilteredY(end+1) = fusedEstimate(2);
            time(end+1) = allData(i).Time;
        end
    end

    % Compute velocities
    numSteps = numel(fusedFilteredX); % Assuming data size is known
    fusedVX = zeros(1, numSteps);
    fusedVY = zeros(1, numSteps);

    if numSteps > 1
        % Compute time differences between steps
        timeDiff = diff(time);
        disp('Time differences:');
        disp(timeDiff);

        % Valid time steps should not be zero or NaN
        validIdx = timeDiff > 0;
        disp('Valid time indices:');
        disp(validIdx);

        if any(validIdx)
            % Velocity calculations
            fusedVX(2:end) = diff(fusedFilteredX) ./ timeDiff; % Velocity in X direction
            fusedVY(2:end) = diff(fusedFilteredY) ./ timeDiff; % Velocity in Y direction
        else
            disp('No valid time steps for velocity calculation.');
        end
    else
        fusedVX = NaN;
        fusedVY = NaN;
    end

    % Plot the fused data for visualization
    figure;
    subplot(2, 1, 1);
    plot(fusedFilteredX, fusedFilteredY, 'bo-', 'LineWidth', 1.5, 'DisplayName', 'Fused Position Data');
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    title('Fused Position Data from Multi-Sensor Fusion');
    legend;
    grid on;
    
    subplot(2, 1, 2);
    plot(time, fusedVX, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Velocity in X');
    hold on;
    plot(time, fusedVY, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Velocity in Y');
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title('Velocity Estimates from Fused Data');
    legend;
    grid on;
end

% Configure the Kalman Filter for radar sensor
function kf = configureKalmanFilterForRadar()
    kf = trackingKF('MotionModel', '2D Constant Velocity', 'MeasurementNoise', 0.5);
end

% Configure the Kalman Filter for lidar sensor
function kf = configureKalmanFilterForLidar()
    kf = trackingKF('MotionModel', '2D Constant Velocity', 'MeasurementNoise', 0.3);
end

% Configure the Kalman Filter for vision sensor
function kf = configureKalmanFilterForVision()
    kf = trackingKF('MotionModel', '2D Constant Velocity', 'MeasurementNoise', 0.1);
end

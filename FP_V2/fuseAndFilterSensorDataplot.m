function [fusedData, scenario, sensors] = fuseAndFilterSensorDataplot()
    % Generate multi-sensor data from the driving scenario
    [allData, scenario, sensors] = generateSensorData(); 

    % Initialize Kalman Filters for fusion process
    fusionKF = configureKalmanFilterForFusion();

    % Initialize the fused and filtered data structure, similar to input format
    fusedData = struct('Time', {}, 'ActorPoses', {}, 'ObjectDetections', {}, 'LaneDetections', {}, 'PointClouds', {});

    % Initialize arrays to store raw sensor data for plotting
    rawRadarX = [];
    rawRadarY = [];
    rawLidarX = [];
    rawLidarY = [];
    rawVisionX = [];
    rawVisionY = [];
    
    % Initialize arrays for fused/filtered data
    fusedFilteredX = [];
    fusedFilteredY = [];

    % Process multi-sensor data
    for i = 1:numel(allData)
        objectDetections = allData(i).ObjectDetections;
        laneDetections = allData(i).LaneDetections;
        ptClouds = allData(i).PointClouds;
        time = allData(i).Time;

        % Initialize sensor estimates
        radarEstimate = [];
        lidarEstimate = [];
        visionEstimate = [];

        % Process each object detection (object-based fusion)
        for j = 1:numel(objectDetections)
            detection = objectDetections{j};
            measurement = detection.Measurement(1:2);  % Extract x, y position
            sensorIndex = detection.SensorIndex;

            switch sensorIndex
                case 1  % Radar
                    radarEstimate = measurement;
                    rawRadarX = [rawRadarX, measurement(1)]; 
                    rawRadarY = [rawRadarY, measurement(2)]; 
                case 2  % Lidar
                    lidarEstimate = measurement;
                    rawLidarX = [rawLidarX, measurement(1)]; 
                    rawLidarY = [rawLidarY, measurement(2)]; 
                case 3  % Vision
                    visionEstimate = measurement;
                    rawVisionX = [rawVisionX, measurement(1)]; 
                    rawVisionY = [rawVisionY, measurement(2)]; 
            end
        end

        % Fusion Logic: Combine estimates from different sensors
        estimates = [radarEstimate, lidarEstimate, visionEstimate];  % 2xN matrix

        % Select valid estimates (non-NaN)
        validIdx = all(~isnan(estimates),1);  % Logical index for valid columns
        validEstimates = estimates(:, validIdx);  % Select valid estimates
        % validIdy = all(~isnan(estimates), 1);
        % validEstimates = estimates(:, validIdy); % changed but didnt work
        disp('Valid Estimates:');
        disp(validEstimates);

        disp(['Filtered X: ', num2str(fusionKF.State(1)), ', Filtered Y: ', num2str(fusionKF.State(2))]);
        if ~isempty(validEstimates)
            % Average of the valid estimates
            fusedEstimate = mean(validEstimates, 2);  % 2x1 vector (mean of valid X and Y)
            % Display the raw fused X and Y data before filtering
            disp(['Raw Fused X: ', num2str(fusedEstimate(1)), ', Raw Fused Y: ', num2str(fusedEstimate(2))]);
            % Apply filtering on the fused estimate using the Kalman Filter
            % Predict and correct the Kalman Filter state
            disp(['Kalman Filter State Before Correction: ', num2str(fusionKF.State')]);

            
            fusionKF.correct(fusedEstimate);
            filteredFusedEstimate = fusionKF.State(1:2);  % Get the filtered x, y position
            
            


            % Store the fused/filtered data
            fusedFilteredX = [fusedFilteredX, filteredFusedEstimate(1)]; 
            fusedFilteredY = [fusedFilteredY, filteredFusedEstimate(2)]; 

            % Create fused object detection using the first sensor index as a base
            fusedObjectDetection = objectDetections{1}; % Using first detection as a base
            fusedObjectDetection.Measurement(1:2) = filteredFusedEstimate; % Replace with filtered fused estimate

            % Add the fused object detection to the fused data
            fusedObjectDetections = {fusedObjectDetection};
        else
            fusedObjectDetections = objectDetections; % Keep original detections if no fusion
        end
        % Print after Kalman filter update
        disp(['Filtered Fused Estimate: X = ', num2str(filteredFusedEstimate(1)), ', Y = ', num2str(filteredFusedEstimate(2))]);
        % Store the fused and filtered data in the output structure
        fusedData(i).Time = time;
        fusedData(i).ActorPoses = allData(i).ActorPoses;
        fusedData(i).ObjectDetections = fusedObjectDetections;  % Fused and filtered object detections
        fusedData(i).LaneDetections = laneDetections;           % Unchanged lane detections
        fusedData(i).PointClouds = ptClouds;                    % Unchanged point clouds
    end

    % Plot the results
    plotSensorDataComparison(rawRadarX, rawRadarY, rawLidarX, rawLidarY, rawVisionX, rawVisionY, fusedFilteredX, fusedFilteredY);
end

% Helper function for configuring Kalman Filter for fusion
function kf = configureKalmanFilterForFusion()
    % Create a Kalman Filter for fusing estimates
    % 'MotionModel' can be 2D constant velocity or acceleration model
    kf = trackingKF('MotionModel', '2D Constant Velocity', 'MeasurementNoise', 0.1);
end

% Helper function to plot raw and filtered/fused data
function plotSensorDataComparison(radarX, radarY, lidarX, lidarY, visionX, visionY, fusedX, fusedY)
    figure;

    % Plot raw sensor data
    subplot(2, 1, 1);
    hold on;
    plot(radarX, radarY, 'r.', 'DisplayName', 'Radar');
    plot(lidarX, lidarY, 'g.', 'DisplayName', 'Lidar');
    plot(visionX, visionY, 'b.', 'DisplayName', 'Vision');
    title('Raw Sensor Data');
    xlabel('X Position');
    ylabel('Y Position');
    legend;
    hold off;

    % Plot fused and filtered data
    subplot(2, 1, 2);
    hold on;
    plot(fusedX, fusedY, 'k-o', 'LineWidth', 1.5, 'DisplayName', 'Fused & Filtered');
    title('Fused and Filtered Data');
    xlabel('X Position');
    ylabel('Y Position');
    legend;
    hold off;
end

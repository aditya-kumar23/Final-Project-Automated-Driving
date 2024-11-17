function fusedData = multisensorFilter_fusionStaticWeight()
    % Generate multi-sensor data from the driving scenario
    [allData, ~, ~] = check_checkv2();
    
    % Get the ego vehicle's initial position
    initialEgoData = allData(1).ActorPoses;
    egoActor = initialEgoData([initialEgoData.ActorID] == 1);
    initialPosition = egoActor.Position(1:2)'; % Extract X, Y position

    % Initialize Kalman Filters for each sensor
    kfRadar = configureKalmanFilterForRadar(initialPosition);
    kfVision1 = configureKalmanFilterForVision(initialPosition);
    kfVision2 = configureKalmanFilterForVision(initialPosition);

    % Initialize output structure
    fusedData = struct('Time', {}, 'ActorPoses', {}, 'ObjectDetections', {}, ...
                       'LaneDetections', {}, 'PointClouds', {}, 'FusedEstimate', {});
    
    % Loop through each time step 
    for i = 1:numel(allData)
        
        fusedData(i).Time = allData(i).Time;
        fusedData(i).ActorPoses = allData(i).ActorPoses;
        fusedData(i).LaneDetections = allData(i).LaneDetections;
        fusedData(i).PointClouds = allData(i).PointClouds;

        % Initialize variables for sensor fusion
        objectDetections = allData(i).ObjectDetections;
        fusedObjectDetections = {};
        weightedFilteredSumX = 0; 
        weightedFilteredSumY = 0;
        totalFilteredWeight = 0;

        % Process each sensor's detection and apply Kalman filtering
        for j = 1:numel(objectDetections)
            detection = objectDetections{j};
            sensorIndex = detection.SensorIndex;
            measurement = detection.Measurement(1:2);
            filteredMeasurement = [];

            % Apply Kalman filter for each sensor
            switch sensorIndex
                case 1  % Radar sensor
                    predict(kfRadar);
                    correct(kfRadar, measurement);
                    filteredMeasurement = kfRadar.State(1:2);
                    sensorWeight = 0.5;  
                    
                case 2  % Vision sensor 1
                    predict(kfVision1);
                    correct(kfVision1, measurement);
                    filteredMeasurement = kfVision1.State(1:2);
                    sensorWeight = 0.3;  
                    
                case 3  % Vision sensor 2
                    predict(kfVision2);
                    correct(kfVision2, measurement);
                    filteredMeasurement = kfVision2.State(1:2);
                    sensorWeight = 0.3;  
            end

            % weighted sums for filtered measurements
            weightedFilteredSumX = weightedFilteredSumX + sensorWeight * filteredMeasurement(1);
            weightedFilteredSumY = weightedFilteredSumY + sensorWeight * filteredMeasurement(2);
            totalFilteredWeight = totalFilteredWeight + sensorWeight;

            % Store filtered measurements in fusedObjectDetections
            fusedObjectDetections{end+1} = struct('SensorIndex', sensorIndex, ...
                                                  'FilteredMeasurement', filteredMeasurement, ...
                                                  'Weight', sensorWeight);
        end
        
        % Calculate weighted fusion of filtered estimates
        if totalFilteredWeight > 0
            filteredFusedEstimate = [weightedFilteredSumX / totalFilteredWeight; weightedFilteredSumY / totalFilteredWeight];
        else
            filteredFusedEstimate = [0; 0];
        end

        % Store fused estimate in the output structure
        fusedData(i).ObjectDetections = fusedObjectDetections;
        fusedData(i).FusedEstimate = struct('Filtered', filteredFusedEstimate);
    end
end

% Kalman Filter Configuration
function kf = configureKalmanFilterForRadar(initialPos)
    initialState = [initialPos(1); initialPos(2)]; 
    initialErrorCovariance = [0.001, 0.001];  % Initial error covariance 
    processNoise = [.3, .3];  % Process noise 
    measurementNoise = 0.0005; 
    kf = configureKalmanFilter('ConstantVelocity', initialState, initialErrorCovariance, processNoise, measurementNoise);
end

function kf = configureKalmanFilterForVision(initialPos)
    initialState = [initialPos(1); initialPos(2)];  
    initialErrorCovariance = [0.001, 0.001];  % Initial error covariance 
    processNoise = [.3, .3];  % Process noise
    measurementNoise = 0.0005; 
    kf = configureKalmanFilter('ConstantVelocity', initialState, initialErrorCovariance, processNoise, measurementNoise);
end

function fusedData = multisensorFilter_fusion_Adaptiv_weight()
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
            residual = [];

            % Apply the appropriate Kalman filter 
            switch sensorIndex
                case 1  % Radar sensor
                    predict(kfRadar);
                    correctedState = correct(kfRadar, measurement);
                    filteredMeasurement = correctedState(1:2);
                    residual = norm(measurement - filteredMeasurement); % Calculate residual
                    
                case 2  % Vision sensor 1
                    predict(kfVision1);
                    correctedState = correct(kfVision1, measurement);
                    filteredMeasurement = correctedState(1:2);
                    residual = norm(measurement - filteredMeasurement); % Calculate residual
                    
                case 3  % Vision sensor 2
                    predict(kfVision2);
                    correctedState = correct(kfVision2, measurement);
                    filteredMeasurement = correctedState(1:2);
                    residual = norm(measurement - filteredMeasurement); % Calculate residual
            end

            % Calculate adaptive weight based on inverse of residuals (add epsilon to avoid division by zero)
            epsilon = 1e-3;
            sensorWeight = 1 / (residual + epsilon);

            %  weighted sums for filtered measurements
            weightedFilteredSumX = weightedFilteredSumX + sensorWeight * filteredMeasurement(1);
            weightedFilteredSumY = weightedFilteredSumY + sensorWeight * filteredMeasurement(2);
            totalFilteredWeight = totalFilteredWeight + sensorWeight;

            % Store filtered measurements and weights in fusedObjectDetections
            fusedObjectDetections{end+1} = struct('SensorIndex', sensorIndex, ...
                                                  'FilteredMeasurement', filteredMeasurement, ...
                                                  'Residual', residual, ...
                                                  'Weight', sensorWeight);
        end
        
        % Calculate weighted fusion of filtered estimates
        if totalFilteredWeight > 0
            filteredFusedEstimate = [weightedFilteredSumX / totalFilteredWeight; ...
                                     weightedFilteredSumY / totalFilteredWeight];
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
    initialErrorCovariance = [.001, .001];  % Initial error covariance 
    processNoise = [.3, .3];  % Process noise 
    measurementNoise = .0005; 
    kf = configureKalmanFilter('ConstantVelocity', initialState, initialErrorCovariance, processNoise, measurementNoise);
end

function kf = configureKalmanFilterForVision(initialPos)
    initialState = [initialPos(1); initialPos(2)];  
    initialErrorCovariance = [.001,.001];  % Initial error covariance 
    processNoise = [.3, .3];  % Process noise 
    measurementNoise = .0005; 
    kf = configureKalmanFilter('ConstantVelocity', initialState, initialErrorCovariance, processNoise, measurementNoise);
end

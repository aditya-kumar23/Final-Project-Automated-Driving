function ACC_CAS_System_RawData()
    % Generate sensor data and ACC input from the defined scenario
    [allData, ~, ~] = check_checkv2();
    
    % Parameters for ACC and CAS control
    maxSpeed = 35;             % Maximum speed allowed for the ego vehicle (m/s)
    minSpeed = 2;              % Minimum speed allowed for the ego vehicle (m/s)
    timeGap = 1.5;             % Time gap in seconds for ACC
    safeDistance = 25;         % Safe following distance for ACC (m)
    criticalDistance = 5;      % Critical distance for CAS (m)
    decelerationRate = 2;    % Deceleration rate for ACC (m/s^2)
    emergencyBrakeRate = 7.0;  % Deceleration rate for CAS (m/s^2)
    accelerationRate = 1.2;    % Acceleration rate (m/s^2)
    
    % Retrieve initial ego speed from the driving scenario data
    initialEgoData = allData(1).ActorPoses; % Array of all actor poses
    egoActor = initialEgoData([initialEgoData.ActorID] == 1); % Find the actor with ID = 1

    % Extract longitudinal and lateral speeds
    longitudinalSpeed = egoActor.Velocity(1); % Longitudinal velocity (x-direction)
    lateralSpeed = egoActor.Velocity(2);      % Lateral velocity (y-direction)
    egoSpeed = sqrt(longitudinalSpeed^2 + lateralSpeed^2); % Initial overall speed

    % Initialize simulation time and data storage
    numSteps = length(allData); % Number of simulation steps
    speedHistory = zeros(numSteps, 1); % Preallocate storage for speed over time
    
    for t = 1:numSteps
        % Retrieve sensor data for the current time step
        currentData = allData(t);
        objectDetections = currentData.ObjectDetections;
        
        % Initialize new speed as max speed
        newSpeed = maxSpeed;
        
        % Track if emergency braking is needed
        emergencyBrake = false;

        % Process detected objects
        if ~isempty(objectDetections)
            for i = 1:numel(objectDetections)
                detection = objectDetections{i};
                
                % Check if object is ahead of the ego vehicle
                if detection.Measurement(1) > 0  % Positive x distance means object ahead
                    distanceToObj = norm(detection.Measurement(1:2));  % Distance to object
                    objLongitudinalVelocity = detection.Measurement(3); % Object's longitudinal velocity
                    objLateralVelocity = detection.Measurement(4);      % Object's lateral velocity
                    objOverallVelocity = sqrt(objLongitudinalVelocity^2 + objLateralVelocity^2); % Overall speed of the object
                    
                    % Calculate the safe following distance based on ACC time gap
                    accSafeDistance = max(safeDistance, timeGap * egoSpeed);
                    
                    % Check for CAS (emergency braking) if object is within critical distance
                    if distanceToObj < criticalDistance
                        emergencyBrake = true;
                        newSpeed = minSpeed; % Set speed to minimum for emergency brake
                        break; % Prioritize CAS over ACC, exit loop
                        
                    % Otherwise, if within ACC safe following distance, adjust speed
                    elseif distanceToObj < accSafeDistance
                        % Slow down to the object's overall velocity or minimum speed
                        newSpeed = max(min(objOverallVelocity, maxSpeed), minSpeed);
                    end
                end
            end
        end
        
        % Adjust ego speed based on newSpeed and braking need
        if emergencyBrake
            % Emergency braking for CAS
            egoSpeed = max(egoSpeed - emergencyBrakeRate, minSpeed);
            fprintf('Emergency Brake Activated! Ego Speed: %.2f m/s\n', egoSpeed);
        elseif newSpeed < egoSpeed
            % Decelerate for ACC if needed
            egoSpeed = max(egoSpeed - decelerationRate, newSpeed);
            fprintf('Collision risk detected within safety distance. Decelerating to %.2f m/s\n', egoSpeed);
        else
            % Accelerate if safe to do so
            egoSpeed = min(egoSpeed + accelerationRate, newSpeed);
        end
        
        % Store speed for this time step
        speedHistory(t) = egoSpeed;
        
        % Visualize the speed adjustment
        fprintf('Time: %.2f s, Ego Speed: %.2f m/s\n', currentData.Time, egoSpeed);
    end
    
    % Plot the speed of the ego vehicle over time
    figure;
    plot([allData.Time], speedHistory, 'o-g');
    xlabel('Time (s)');
    ylabel('Ego Vehicle Speed (m/s)');
    title('ACC and CAS Speed Control');
    grid on;
end

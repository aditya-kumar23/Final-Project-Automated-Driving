function ACC_CAS_System(fusedData, maxSpeed, minSpeed, safetyDistance, criticalDistance, decelerationRate, emergencyBrakeRate, accelerationRate)
    % Assign default values if inputs are missing or empty
    if isempty(maxSpeed), maxSpeed = 35; end            % Maximum speed allowed (m/s)
    if isempty(minSpeed), minSpeed = 2; end             % Minimum speed allowed (m/s)
    if isempty(safetyDistance), safetyDistance = 25; end % Safety distance for ACC (m)
    if isempty(criticalDistance), criticalDistance = 5; end % Critical distance for CAS (m)
    if isempty(decelerationRate), decelerationRate = 2; end   % Deceleration rate for ACC (m/s^2)
    if isempty(emergencyBrakeRate), emergencyBrakeRate = 7; end % Deceleration rate for CAS (m/s^2)
    if isempty(accelerationRate), accelerationRate = 1.2; end   % Acceleration rate (m/s^2)
    
    % Estimate initial ego speed if not directly provided in fusedData
    initialEgoSpeed = 30; % Default initial speed (m/s)
    if isfield(fusedData(1), 'ActorPoses')
        egoActorPose = fusedData(1).ActorPoses([fusedData(1).ActorPoses.ActorID] == 1);
        initialEgoVelocity = egoActorPose.Velocity;
        initialEgoSpeed = norm(initialEgoVelocity(1:2)); % Use the velocity vector magnitude as initial speed
    else
        disp('Ego velocity data not available in fusedData. Using default speed of 30 m/s.');
    end
    egoSpeed = initialEgoSpeed;

    % Initialize time steps and speed history
    numSteps = length(fusedData);
    speedHistory = zeros(numSteps, 1);
    
    for t = 1:numSteps
        % Retrieve fused data for the current time step
        currentData = fusedData(t);
        fusedDetections = currentData.ObjectDetections;
        
        % Default to current ego speed as the new speed
        newSpeed = egoSpeed;
        
        % Initialize collision risk flags
        collisionRiskDetected = false;
        emergencyBrakeRequired = false;
        
        % Process fused detections to evaluate potential collision risk
        if ~isempty(fusedDetections)
            for i = 1:numel(fusedDetections)
                detection = fusedDetections{i};
                
                % Retrieve distance and velocity of detected object
                objectPos = detection.FilteredMeasurement;   % Object position (x, y)
                
                % Check if velocity data is available
                if isfield(detection, 'Velocity')
                    objectVelocity = detection.Velocity;  % Object velocity (vx, vy)
                    relativeSpeed = egoSpeed - norm(objectVelocity); % Relative speed between ego and object
                else
                    objectVelocity = [0; 0];  % Assume stationary object if velocity is unavailable
                    relativeSpeed = egoSpeed;
                end
                
                % Calculate distance to object
                distanceToObject = norm(objectPos); % Euclidean distance to the object
                
                % Determine braking requirements based on distance to object
                if objectPos(1) > 0 % Object is ahead
                    if distanceToObject < criticalDistance
                        % Emergency brake if within critical distance
                        newSpeed = max(egoSpeed - emergencyBrakeRate, minSpeed);
                        emergencyBrakeRequired = true;
                        collisionRiskDetected = true;
                        fprintf('Emergency braking activated! New Speed: %.2f m/s\n', newSpeed);
                        break;
                    elseif distanceToObject < safetyDistance
                        % Normal deceleration for ACC if within safety distance
                        newSpeed = max(egoSpeed - decelerationRate, minSpeed);
                        collisionRiskDetected = true;
                        fprintf('Collision risk detected within safety distance. Decelerating to %.2f m/s\n', newSpeed);
                        break;
                    end
                end
            end
        end
        
        % Adjust ego speed: apply braking if risk detected, else accelerate if safe
        if emergencyBrakeRequired
            egoSpeed = newSpeed; % Apply emergency braking
        elseif collisionRiskDetected
            egoSpeed = newSpeed; % Apply normal deceleration for ACC
        else
            egoSpeed = min(egoSpeed + accelerationRate, maxSpeed); % Accelerate if clear
        end
        
        % Store the speed at this time step
        speedHistory(t) = egoSpeed;
        
        % Print speed information for debugging
        fprintf('Time: %.2f s, Ego Speed: %.2f m/s\n', currentData.Time, egoSpeed);
    end
    
    % Plot speed of ego vehicle over time to visualize CAS & ACC behavior
    figure;
    plot([fusedData.Time], speedHistory, 'o-r', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Ego Vehicle Speed (m/s)');
    title('ACC & CAS Speed Control Using Fused Data');
    grid on;
end

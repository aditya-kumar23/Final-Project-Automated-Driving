function collisionAvoidanceSystem()
    % Run the multi-sensor fusion tracking function to get the data
    [fusedX, fusedY, fusedVX, fusedVY, time] = multiSensorFusionVelocityForCollisionAvoidance_v5();
    
    % Define collision parameters
    collisionThreshold = 50; % meters, distance at which collision is detected
    safetyTime = 2; % seconds, safety time to react
    
    % Initialize arrays for collision detection
    numPoints = numel(fusedX);
    collisionWarnings = false(1, numPoints);
    
    % Compute collision times and check for collisions
    for i = 1:numPoints
        if i < numPoints
            % Predict the future position based on current velocity
            futureX = fusedX(i) + fusedVX(i) * safetyTime;
            futureY = fusedY(i) + fusedVY(i) * safetyTime;
            
            % Check distance to the predicted position
            distance = norm([futureX - fusedX(i), futureY - fusedY(i)]);
            
            % Check if the distance is below the collision threshold
            if distance < collisionThreshold
                collisionWarnings(i) = true;
            end
        end
    end
    
    % Display collision warnings
    for i = 1:numPoints
        if collisionWarnings(i)
            disp(['Collision warning at time ', num2str(time(i)), 's.']);
            % Implement your collision avoidance logic here (e.g., change trajectory)
        end
    end
    
    % Plot collision warnings
    figure;
    plot(fusedX, fusedY, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Fused Position Data');
    hold on;
    plot(fusedX(collisionWarnings), fusedY(collisionWarnings), 'ro', 'MarkerSize', 8, 'DisplayName', 'Collision Warnings');
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    title('Collision Avoidance System');
    legend;
    grid on;
end

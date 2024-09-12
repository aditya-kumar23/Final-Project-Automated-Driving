
% Call the project function to get the data
[allData, scenario, sensors] = simulationEnvironment();

% Extract the radar sensor data from allData
radarDetections = {};  % Store all radar detections
for i = 1:length(allData)
    % Extract radar detections from ObjectDetections
    detections = allData(i).ObjectDetections;
    for j = 1:length(detections)
        detection = detections{j};
        if isa(sensors{1}, 'drivingRadarDataGenerator') % Check if it's a radar detection
            radarDetections = [radarDetections; detection.Measurement]; %#ok<AGROW>
        end
    end
end

% Kalman filter setup
dt = 0.1;  % Time step (you can adjust it according to your simulation)
F = [1 0 dt 0;  % State transition model (constant velocity in X and Y)
     0 1 0 dt;
     0 0 1 0;
     0 0 0 1];  % 4x4 state transition matrix for position and velocity
H = [1 0 0 0;  % Measurement model (we measure only position)
     0 1 0 0];  % 2x4 measurement matrix
Q = 0.2* eye(4);  % Process noise covariance
R = 0.5 * eye(2);  % Measurement noise covariance
P = eye(4);  % Initial state covariance
x = [-10; -15; 0; 0];  % Initial state: [posX; posY; velX; velY]

% Create an occupancy grid map
gridResolution = 1;  % 1 meter per cell
gridSize = [100, 100];  % Size of the grid
occupancyGrid = zeros(gridSize);  % Initialize grid as empty

% Loop through radar detections and apply Kalman filter for tracking
figure;
hold on;
grid on;
title('Radar Detections with Kalman Filter');
xlabel('X (meters)');
ylabel('Y (meters)');
set(gca, 'YDir', 'reverse');  % Reverse Y-axis directio

for i = 1:length(radarDetections)
    detection = radarDetections{i};
    
    % Assuming radar detections contain [X, Y] coordinates
    z = detection(1:2);  % Measurement is the detected [X, Y] position

    % Kalman filter prediction step
    x = F * x;  % Predict the next state
    P = F * P * F' + Q;  % Predict the covariance

    % Kalman filter update step
    y = z - H * x;  % Measurement residual (difference between predicted and actual)
    S = H * P * H' + R;  % Residual covariance
    K = P * H' / S;  % Kalman gain
    x = x + K * y;  % Update the state estimate with the measurement
    P = (eye(4) - K * H) * P;  % Update the covariance estimate
    
    % Extract the updated position from the Kalman filter state
    posX = x(1);
    posY = x(2);

    % Update the occupancy grid with the tracked position
    gridX = round(posX / gridResolution) + gridSize(1) / 2;  % Convert position to grid index
    gridY = round(posY / gridResolution) + gridSize(2) / 2;
    
    % Ensure the indices are within grid bounds
    if gridX > 0 && gridX <= gridSize(1) && gridY > 0 && gridY <= gridSize(2)
       occupancyGrid(gridX, gridY) = 1;  % Mark the cell as occupied
    end

    % Plot the radar detection (as a red dot)
    plot(z(1), z(2), 'ro');  % Measurement (from radar)

    % Plot the Kalman filter tracked position (as a blue dot)
    plot(posX, posY, 'bo');  % Estimated position (from Kalman filter)

    pause(0.1);  % Pause to visualize the updates step by step
end

% Visualize the occupancy grid map
figure;
imagesc(occupancyGrid);
axis equal;
title('Occupancy Grid Map with Kalman Filtered Radar Data');
xlabel('X (grid)');
ylabel('Y (grid)');
colormap(gray);

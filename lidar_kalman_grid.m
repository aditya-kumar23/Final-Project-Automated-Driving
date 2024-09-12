% Call the project function to get the data
[allData, scenario, sensors] = simulationEnvironment();

% Extract time and point cloud data from allData
time = [allData.Time];
ptClouds = {allData.PointClouds};

% Parameters for Kalman filter
dt = 0.1;  % Time step (seconds)
F = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];  % State transition matrix
H = [1 0 0 0; 0 1 0 0];  % Measurement matrix
Q = 0.1 * eye(4);  % Process noise covariance
R = 5* eye(2);  % Measurement noise covariance
P = eye(4);  % Initial estimation covariance
x = [-10; -15; 0; 0];  % Initial state [x; y; vx; vy]

% Initialize the manual occupancy grid
gridSize = [100 100];  % Grid size (in cells)
gridResolution = 1;  % Grid resolution (meters per cell)
grid = zeros(gridSize);  % Initialize grid to zeros (unoccupied)

% Set figure for both raw data and filtered data
figure('Color', 'white');
hold on;
%grid on;
title('LiDAR Measurements with Kalman Filter Estimates');
xlabel('X (meters)');
ylabel('Y (meters)');
set(gca, 'YDir', 'reverse');  % Reverse Y-axis direction
%legend();

% Loop through point clouds and apply Kalman filter
for i = 1:numel(ptClouds)
    ptCloud = ptClouds{i};
    
    % Check if the point cloud data exists and is not empty
    if ~isempty(ptCloud)
        locations = ptCloud.Location;  % Get the locations
        indices = locations(:, 3) < 0.7;  % Find indices where z-coordinate is less than 0.7
        locations(indices, 3) = 0;  % Set z-coordinate to zero for those indices
        
        % Extract X and Y coordinates from point cloud
        xData = locations(:,1);
        yData = locations(:,2);
        
        % Apply Kalman filter to each point in the point cloud
        for j = 1:size(locations, 1)
            % Get the measured position (LiDAR)
            z = [xData(j); yData(j)];
            
            % Prediction step
            x = F * x;
            P = F * P * F' + Q;
            
            % Update step
            y_kalman = z - H * x;  % Measurement residual
            S = H * P * H' + R;  % Residual covariance
            K = P * H' / S;  % Kalman gain
            x = x + K * y_kalman;  % Updated state estimate
            P = (eye(4) - K * H) * P;  % Updated covariance estimate
            
            % Convert estimated position to grid indices
            gridX = round(x(1) / gridResolution) + gridSize(1) / 2;  % Convert position to grid index
            gridY = round(x(2) / gridResolution) + gridSize(2) / 2;
            
            % Ensure indices are within grid bounds
            if gridX > 0 && gridX <= gridSize(1) && gridY > 0 && gridY <= gridSize(2)
               grid(gridX, gridY) = grid(gridX, gridY) + 1;  % Increment occupancy value
            end
            
            % Plot measured data (LiDAR point)
            scatter(z(1), z(2), 50, 'ro');  % Raw LiDAR data (red)
            
            % Plot Kalman filter estimate
            scatter(x(1), x(2), 50, 'bo');  % Estimated position (blue)
        end
        
        pause(0.1);  % Pause to observe each time step
    else
        warning('Empty point cloud at time step %d', i);
    end
end

% Plot the manual occupancy grid map
figure;
imagesc(grid);  % Display the grid as an image
colormap(flipud(gray));  % Set the color map to gray (occupied cells are dark)
colorbar;  % Show color bar for occupancy values
title('Manual Occupancy Grid Map with Kalman Filtered LiDAR Data');
xlabel('X (grid)');
ylabel('Y (grid)');
set(gca, 'YDir', 'reverse');  % Reverse Y-axis direction

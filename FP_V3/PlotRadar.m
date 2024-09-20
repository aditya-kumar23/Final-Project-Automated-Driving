% Instantiate utility_functions object
functions = utility_functions;

% Call the simulation environment function to get the data
[allData, ~, ~] = Sonnenstrasse_sim();

% Define the actor_id for which you want to get aggregated measures (e.g., ego vehicle with ID = 2)
actor_id = 2;
ego_id = 1;
% Call the get_trajectory function to get the trajectory
trajectory = functions.get_trajectory(allData, actor_id);

% Call the get_aggregated_measures function to get aggregated positions
aggregatedMeasures = functions.get_aggregated_measures(allData, ego_id);
aggregatedMeasures= aggregatedMeasures(~any(isnan(aggregatedMeasures), 2), :);

figure;
hold on;

% Plot the ego vehicle's trajectory in the world coordinate system
for i = 1:length(allData)
    % Extract object detections at the current time step
    objectDetections = allData(i).ObjectDetections;
    
    % Get the ego vehicle's world position and yaw (orientation)
    ego_x_world = allData(i).ActorPoses(ego_id).Position(1);  % X position
    ego_y_world = allData(i).ActorPoses(ego_id).Position(2);  % Y position
    ego_yaw = allData(i).ActorPoses(ego_id).Yaw;              % Yaw (heading) angle in degrees

    % Convert yaw angle to radians for the transformation
    ego_yaw = deg2rad(ego_yaw);
    
    % Plot the ego vehicle's world position
    plot(ego_x_world, ego_y_world, 'bo', 'DisplayName', 'Ego Vehicle');
    
    % Plot detected objects after transforming their coordinates to world frame
    if ~isempty(objectDetections)
        for j = 1:length(objectDetections)
            % Object detection in ego-centric coordinates
            x_ego = objectDetections{j}.Measurement(1);
            y_ego = objectDetections{j}.Measurement(2);

            % Convert ego-centric to world coordinates
            x_world_relative = cos(ego_yaw) * x_ego - sin(ego_yaw) * y_ego;
            y_world_relative = sin(ego_yaw) * x_ego + cos(ego_yaw) * y_ego;

            % Apply translation by ego vehicle's global position
            x_world = x_world_relative + ego_x_world;
            y_world = y_world_relative + ego_y_world;

            % Plot the object detection in the world coordinate system
            plot(x_world, y_world, 'rx', 'DisplayName', 'Detected Object');
            
            % Debug print: World coordinates of the detected object
            disp(['Time = ', num2str(allData(i).Time), ', Object ', num2str(j), ' World X: ', num2str(x_world), ', Y: ', num2str(y_world)]);
        end
    end
     %Plot aggregated positions
    if i <= size(aggregatedMeasures, 1)
        plot(aggregatedMeasures(i, 1), aggregatedMeasures(i, 2), 'g*', 'DisplayName', 'Aggregated Measures');
    end

    % Pause to visualize each time step (optional)
    pause(0.1);
end

% Set plot properties
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Object Detections in World Coordinates');
legend show;
hold off;
function filtered_measures = detect_outliers_zscore(measures, threshold)
    % Calculate the mean and standard deviation
    mean_meas = mean(measures);
    std_meas = std(measures);

    % Initialize filtered measures
    filtered_measures = measures;

    % Loop through each measurement
    for i = 1:size(measures, 1)
        % Calculate Z-score for x and y measurements
        z_score_x = (measures(i, 1) - mean_meas(1)) / std_meas(1);
        z_score_y = (measures(i, 2) - mean_meas(2)) / std_meas(2);

        % Check if Z-score exceeds the threshold
        if abs(z_score_x) > threshold
            filtered_measures(i, 1) = mean_meas(1); % Replace with mean
        end
        if abs(z_score_y) > threshold
            filtered_measures(i, 2) = mean_meas(2); % Replace with mean
        end
    end
end

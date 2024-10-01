% clear;
% % Instantiate utility_functions object
% functions = utility_functions;
% 
% % Call the simulation environment function to get the data
% [allData, ~, ~] = simulationEnvironmentV2();
% 
% % Define the actor_id for which you want to get aggregated measures (e.g., ego vehicle with ID = 2)
% actor_id = 1;
% ego_id = 2;
% % Call the get_trajectory function to get the trajectory
% trajectory = functions.get_trajectory(allData, actor_id);
% 
% % Call the get_aggregated_measures function to get aggregated positions
% aggregatedMeasures = functions.get_aggregated_measures(allData, ego_id);
% aggregatedMeasures= aggregatedMeasures(~any(isnan(aggregatedMeasures), 2), :);
% threshold = 2; % Number of standard deviations for outlier detection
% aggregatedMeasures = detect_outliers_zscore(aggregatedMeasures, threshold);
% 
% figure;
% hold on;
% RADAR_INDEX = 1;
% CAMERA_INDEX = 2;
% radarDetections = [];
% cameraDetections = [];
% 
% % Extract radar and camera measurements from allData
% for i = 1:length(allData)
%     for j = 1:length(allData(i).ObjectDetections)
%         detection = allData(i).ObjectDetections{j};
%         if detection.SensorIndex == RADAR_INDEX % Radar detection
%             radarDetections = [radarDetections; detection.Measurement']; % Append radar measurements as row vectors
%         elseif detection.SensorIndex == CAMERA_INDEX % Camera detection
%             cameraDetections = [cameraDetections; detection.Measurement']; % Append camera measurements as row vectors
%         end
%     end
% end
% % Plot the ego vehicle's trajectory in the world coordinate system
% for i = 1:length(allData)
%     % Extract object detections at the current time step
%     objectDetections = allData(i).ObjectDetections;
% 
%     % Get the ego vehicle's world position and yaw (orientation)
%     ego_x_world = allData(i).ActorPoses(ego_id).Position(1);  % X position
%     ego_y_world = allData(i).ActorPoses(ego_id).Position(2);  % Y position
%     ego_yaw = allData(i).ActorPoses(ego_id).Yaw;              % Yaw (heading) angle in degrees
% 
%     % Convert yaw angle to radians for the transformation
%     ego_yaw = deg2rad(ego_yaw);
% 
%     % Plot the ego vehicle's world position
%     plot(ego_x_world, ego_y_world, 'bo', 'DisplayName', 'Ego Vehicle');
% 
%     % Plot detected objects after transforming their coordinates to world frame
%     if ~isempty(objectDetections)
%         for j = 1:length(objectDetections)
%             % Object detection in ego-centric coordinates
%             x_ego = objectDetections{j}.Measurement(1);
%             y_ego = objectDetections{j}.Measurement(2);
% 
%             % Convert ego-centric to world coordinates
%             x_world_relative = cos(ego_yaw) * x_ego - sin(ego_yaw) * y_ego;
%             y_world_relative = sin(ego_yaw) * x_ego + cos(ego_yaw) * y_ego;
% 
%             % Apply translation by ego vehicle's global position
%             x_world = x_world_relative + ego_x_world;
%             y_world = y_world_relative + ego_y_world;
% 
%             % Plot the object detection in the world coordinate system
%             plot(x_world, y_world, 'rx', 'DisplayName', 'Detected Object');
% 
%             % Debug print: World coordinates of the detected object
%             disp(['Time = ', num2str(allData(i).Time), ', Object ', num2str(j), ' World X: ', num2str(x_world), ', Y: ', num2str(y_world)]);
%         end
%     end
%     % % Plot detected objects after transforming their coordinates to world frame
%     % if ~isempty(radarDetections)
%     %     for j = 1:length(radarDetections)
%     %         % Object detection in ego-centric coordinates
%     %         x_ego = radarDetections(:,1);
%     %         y_ego = radarDetections(:,2);
%     % 
%     %         % Convert ego-centric to world coordinates
%     %         x_world_relative = cos(ego_yaw) * x_ego - sin(ego_yaw) * y_ego;
%     %         y_world_relative = sin(ego_yaw) * x_ego + cos(ego_yaw) * y_ego;
%     % 
%     %         % Apply translation by ego vehicle's global position
%     %         x_world = x_world_relative + ego_x_world;
%     %         y_world = y_world_relative + ego_y_world;
%     % 
%     %         % Plot the object detection in the world coordinate system
%     %         plot(x_world, y_world, '*', 'DisplayName', 'Detected Object');
%     % 
%     %         % % Debug print: World coordinates of the detected object
%     %         % disp(['Time = ', num2str(allData(i).Time), ', Object ', num2str(j), ' World X: ', num2str(x_world), ', Y: ', num2str(y_world)]);
%     %     end
%     % end
%      %Plot aggregated positions
%     if i <= size(aggregatedMeasures, 1)
%         plot(aggregatedMeasures(i, 1), aggregatedMeasures(i, 2), 'g*', 'DisplayName', 'Aggregated Measures');
%     end
% 
%     % Pause to visualize each time step (optional)
%     pause(0.1);
% end
% 
% % Set plot properties
% xlabel('X Position (m)');
% ylabel('Y Position (m)');
% title('Object Detections in World Coordinates');
% legend show;
% hold off;
% 
% 
% function filtered_measures = detect_outliers_mad(measures, threshold)
%     % Calculate the median and MAD
%     median_meas = median(measures);
%     mad_meas = mad(measures, 1); % MAD with scaling factor of 1
% 
%     % Initialize filtered measures
%     filtered_measures = measures;
% 
%     % Loop through each measurement
%     for i = 1:size(measures, 1)
%         % Check if the measurement is an outlier
%         if abs(measures(i, 1) - median_meas(1)) > threshold * mad_meas(1)
%             filtered_measures(i, 1) = median_meas(1); % Replace with median
%         end
%         if abs(measures(i, 2) - median_meas(2)) > threshold * mad_meas(2)
%             filtered_measures(i, 2) = median_meas(2); % Replace with median
%         end
%     end
% end
% 
% function filtered_measures = detect_outliers_zscore(measures, threshold)
%     % Calculate the mean and standard deviation
%     mean_meas = mean(measures);
%     std_meas = std(measures);
% 
%     % Initialize filtered measures
%     filtered_measures = measures;
% 
%     % Loop through each measurement
%     for i = 1:size(measures, 1)
%         % Calculate Z-score for x and y measurements
%         z_score_x = (measures(i, 1) - mean_meas(1)) / std_meas(1);
%         z_score_y = (measures(i, 2) - mean_meas(2)) / std_meas(2);
% 
%         % Check if Z-score exceeds the threshold
%         if abs(z_score_x) > threshold
%             filtered_measures(i, 1) = mean_meas(1); % Replace with mean
%         end
%         if abs(z_score_y) > threshold
%             filtered_measures(i, 2) = mean_meas(2); % Replace with mean
%         end
%     end
% end

clear;
% Instantiate utility_functions object
functions = utility_functions;

% Call the simulation environment function to get the data
[allData, ~, ~] = simulationEnvironmentV2();

% Define the actor_id for which you want to get aggregated measures (e.g., ego vehicle with ID = 2)
actor_id = 1;
ego_id = 2;

% Call the get_trajectory function to get the trajectory
trajectory = functions.get_trajectory(allData, actor_id);

% Call the get_aggregated_measures function to get aggregated positions
aggregatedMeasures = functions.get_aggregated_measures(allData, ego_id);
aggregatedMeasures= aggregatedMeasures(~any(isnan(aggregatedMeasures), 2), :);
threshold = 2; % Number of standard deviations for outlier detection
aggregatedMeasures = detect_outliers_zscore(aggregatedMeasures, threshold);

figure;
hold on;
RADAR_INDEX = 1;
CAMERA_INDEX = 2;
radarDetections = [];
cameraDetections = [];

% Extract radar and camera measurements from allData
for i = 1:length(allData)
    for j = 1:length(allData(i).ObjectDetections)
        detection = allData(i).ObjectDetections{j};
        if detection.SensorIndex == RADAR_INDEX % Radar detection
            radarDetections = [radarDetections; detection.Measurement']; % Append radar measurements as row vectors
        elseif detection.SensorIndex == CAMERA_INDEX % Camera detection
            cameraDetections = [cameraDetections; detection.Measurement']; % Append camera measurements as row vectors
        end
    end
end

% Plot the ego vehicle's trajectory in the world coordinate system
h_ego = plot(nan, nan, 'bo', 'DisplayName', 'Ego Vehicle'); % Placeholder for ego vehicle plot
h_object = plot(nan, nan, 'rx', 'DisplayName', 'Detected Object'); % Placeholder for detected object plot
h_agg = plot(nan, nan, 'g*', 'DisplayName', 'Aggregated Measures'); % Placeholder for aggregated measures plot

% Loop through each time step in allData to update plot
for i = 1:length(allData)
    % Extract object detections at the current time step
    objectDetections = allData(i).ObjectDetections;
    
    % Get the ego vehicle's world position and yaw (orientation)
    ego_x_world = allData(i).ActorPoses(ego_id).Position(1);  % X position
    ego_y_world = allData(i).ActorPoses(ego_id).Position(2);  % Y position
    ego_yaw = allData(i).ActorPoses(ego_id).Yaw;              % Yaw (heading) angle in degrees

    % Convert yaw angle to radians for the transformation
    ego_yaw = deg2rad(ego_yaw);
    
    % Update the ego vehicle's position plot
    set(h_ego, 'XData', [get(h_ego, 'XData') ego_x_world], 'YData', [get(h_ego, 'YData') ego_y_world]);
    
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

            % Update detected object plot
            set(h_object, 'XData', [get(h_object, 'XData') x_world], 'YData', [get(h_object, 'YData') y_world]);
            
            % Debug print: World coordinates of the detected object
            disp(['Time = ', num2str(allData(i).Time), ', Object ', num2str(j), ' World X: ', num2str(x_world), ', Y: ', num2str(y_world)]);
        end
    end
    
    % Plot aggregated positions
    if i <= size(aggregatedMeasures, 1)
        set(h_agg, 'XData', [get(h_agg, 'XData') aggregatedMeasures(i, 1)], 'YData', [get(h_agg, 'YData') aggregatedMeasures(i, 2)]);
    end

    % Pause to visualize each time step (optional)
    pause(0.1);
end

% Set plot properties
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Object Detections in World Coordinates');
legend('show'); % Show legend only once, after plotting
hold off;

% Function definitions for outlier detection
function filtered_measures = detect_outliers_mad(measures, threshold)
    % Calculate the median and MAD
    median_meas = median(measures);
    mad_meas = mad(measures, 1); % MAD with scaling factor of 1

    % Initialize filtered measures
    filtered_measures = measures;

    % Loop through each measurement
    for i = 1:size(measures, 1)
        % Check if the measurement is an outlier
        if abs(measures(i, 1) - median_meas(1)) > threshold * mad_meas(1)
            filtered_measures(i, 1) = median_meas(1); % Replace with median
        end
        if abs(measures(i, 2) - median_meas(2)) > threshold * mad_meas(2)
            filtered_measures(i, 2) = median_meas(2); % Replace with median
        end
    end
end

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

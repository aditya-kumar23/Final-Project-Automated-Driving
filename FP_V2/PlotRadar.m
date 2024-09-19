% Instantiate utility_functions object
functions = utility_functions;

% Call the simulation environment function to get the data
[allData, ~, ~] = simulationEnvironment();

% Define the actor_id for which you want to get aggregated measures (e.g., ego vehicle with ID = 2)
actor_id = 1;
ego_id = 2;
% Call the get_trajectory function to get the trajectory
trajectory = functions.get_trajectory(allData, actor_id);

% Call the get_aggregated_measures function to get aggregated positions
aggregatedMeasures = functions.get_aggregated_measures(allData, ego_id);
aggregatedMeasures= aggregatedMeasures(~any(isnan(aggregatedMeasures), 2), :);
% % Plotting code
% figure;
% hold on;
% 
% for i = 1:length(allData)
%     % Extract object detections at the current time step
%     objectDetections = allData(i).ObjectDetections;
% 
%     % Find the ego vehicle (ID = 2) position
%     egoPose = [];
%     for actorIndex = 1:length(allData(i).ActorPoses)
%         if allData(i).ActorPoses(actorIndex).ActorID == ego_id
%             egoPose = allData(i).ActorPoses(actorIndex).Position;
%             break;
%         end
%     end
% 
%     % Plot ego vehicle position
%     if ~isempty(egoPose)
%         plot(egoPose(1), egoPose(2), 'bo', 'DisplayName', 'Ego Vehicle');
%     end
% 
%     % Plot detected objects
%     if ~isempty(objectDetections)
%         for j = 1:length(objectDetections)
%             detectedObj = objectDetections{j};
%             plot(detectedObj.Measurement(1), detectedObj.Measurement(2), 'rx', 'DisplayName', 'Detected Object');
%         end
%     end
% 
%     % Plot aggregated positions
%     if i <= size(aggregatedMeasures, 1)
%         plot(aggregatedMeasures(i, 1), aggregatedMeasures(i, 2), 'g*', 'DisplayName', 'Aggregated Measures');
%     end
% 
% 
%     % Pause to visualize each time step (optional)
%     pause(0.1);
% end
%  % Set plot properties
%     xlabel('X Position (m)');
%     ylabel('Y Position (m)');
%     title(['Object Detections at Time = ' num2str(allData(i).Time)]);
%     legend show;
% 
% hold off;
% Initialize a figure for the plot
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


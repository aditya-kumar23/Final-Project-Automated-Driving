classdef utility_functions
    methods
%         function measures = get_aggregated_measures(obj,allData, actor_id)
%             actorPoses = allData(1).ActorPoses;
%             for j = 1:numel(actorPoses)
%                 if actorPoses(j).ActorID == actor_id
%                     fprintf('Actor ID: %d, Position: (%f, %f, %f)\n', actorPoses(j).ActorID, actorPoses(j).Position(1), actorPoses(j).Position(2), actorPoses(j).Position(3));
%                     x_ego = actorPoses(j).Position(1);
%                     y_ego = actorPoses(j).Position(2);
%                     z_ego = actorPoses(j).Position(3);
%                     break
%                 end
% 
%             end
% 
%             for i = 1:length(allData)
%                 % Extract object detections for current time step
%                 objectDetections = allData(i).ObjectDetections;
%                 temp_x = 0;
%                 temp_y = 0;
%                 temp_z = 0;
%                 for j = 1:length(objectDetections)
%                     % Extract object measurement (assuming it contains position information)
%                     measurement = objectDetections{j}.Measurement;
%                     temp_x = temp_x  + measurement(1);
%                     temp_y = temp_y + measurement(2);
%                     temp_z = temp_z + measurement(3);
%                 end
% 
%                 %fprintf(length(objectDetections));
%                 avgPositions(i,1) = x_ego + temp_x/length(objectDetections);
%                 avgPositions(i,2) = y_ego + temp_y/length(objectDetections);
%                 avgPositions(i,3) = z_ego + temp_z/length(objectDetections);
% 
%             end
%             measures = avgPositions;
%         end
% function measures = get_aggregated_measures(obj, allData, actor_id)
%     % Initialize variables for ego vehicle position and orientation
%     actorPoses = allData(1).ActorPoses;
%     x_ego = 0; y_ego = 0; z_ego = 0; yaw_ego = 0; % Default initialization
% 
%     % Find the ego vehicle's position and yaw angle (heading) in world coordinates
%     for j = 1:numel(actorPoses)
%         if actorPoses(j).ActorID == actor_id
%             % Ego vehicle's position (x, y, z) and yaw angle
%             x_ego = actorPoses(j).Position(1);
%             y_ego = actorPoses(j).Position(2);
%             z_ego = actorPoses(j).Position(3);
%             yaw_ego = actorPoses(j).Yaw;  % Extract yaw from the ActorPoses struct
%             break;
%         end
%     end
% 
%     % Initialize variable to store average positions for all time steps
%     avgPositions = zeros(length(allData), 3);
% 
%     % Loop over all time steps
%     for i = 1:length(allData)
%         % Extract object detections for the current time step
%         objectDetections = allData(i).ObjectDetections;
% 
%         if isempty(objectDetections)
%             continue;  % Skip if no detections
%         end
% 
%         temp_x = 0;
%         temp_y = 0;
%         temp_z = 0;
% 
%         % Loop through each detected object
%         for j = 1:length(objectDetections)
%             % Extract object measurement (assuming it contains position information in local coordinates)
%             measurement = objectDetections{j}.Measurement;
%             local_x = measurement(1); % X in ego-vehicle frame
%             local_y = measurement(2); % Y in ego-vehicle frame
%             local_z = measurement(3); % Z in ego-vehicle frame (if needed)
% 
%             % Apply 2D rotation to convert local coordinates to world coordinates
%             % Rotation matrix for 2D (considering yaw angle around Z-axis)
%             world_x = cos(yaw_ego) * local_x - sin(yaw_ego) * local_y;
%             world_y = sin(yaw_ego) * local_x + cos(yaw_ego) * local_y;
% 
%             % Translate to world position by adding the ego vehicle's position in world coordinates
%             world_x = world_x + x_ego;
%             world_y = world_y + y_ego;
% 
%             % Accumulate positions for averaging
%             temp_x = temp_x + world_x;
%             temp_y = temp_y + world_y;
%             temp_z = temp_z + local_z + z_ego; % If Z transformation is necessary
%         end
% 
%         % Calculate average world positions of detected objects
%         avg_x_world = temp_x / length(objectDetections);
%         avg_y_world = temp_y / length(objectDetections);
%         avg_z_world = temp_z / length(objectDetections);
% 
%         % Convert the average positions from world coordinates to ego-vehicle coordinates
%         % Apply inverse rotation
%         avg_x_ego = cos(-yaw_ego) * (avg_x_world - x_ego) - sin(-yaw_ego) * (avg_y_world - y_ego);
%         avg_y_ego = sin(-yaw_ego) * (avg_x_world - x_ego) + cos(-yaw_ego) * (avg_y_world - y_ego);
%         avg_z_ego = avg_z_world - z_ego; % If Z transformation is necessary
% 
%         % Store in avgPositions
%         avgPositions(i, 1) = avg_x_ego; % Relative x-coordinate
%         avgPositions(i, 2) = avg_y_ego; % Relative y-coordinate
%         avgPositions(i, 3) = avg_z_ego; % Relative z-coordinate (if needed)
%     end
% 
%     % Return aggregated positions in ego vehicle coordinate system
%     measures = avgPositions;
% end
function measures = get_aggregated_measures(obj,allData, ego_id)
            for i = 1:length(allData)
                % if actorPoses(j).ActorID == actor_id
                % fprintf('Actor ID: %d, Position: (%f, %f, %f)\n', actorPoses(j).ActorID, actorPoses(j).Position(1), actorPoses(j).Position(2), actorPoses(j).Position(3));
                    x_ego_world = allData(i).ActorPoses(ego_id).Position(1);
                    y_ego_world = allData(i).ActorPoses(ego_id).Position(2);
                    ego_yaw = allData(i).ActorPoses(ego_id).Yaw; 
                    % Convert yaw angle to radians for the transformation
                    ego_yaw = deg2rad(ego_yaw);
                    
                % Extract object detections for current time step
                objectDetections = allData(i).ObjectDetections;
                temp_x = 0;
                temp_y = 0;
           
                for j = 1:length(objectDetections)
                % Object detection in ego-centric coordinates
                x_ego = objectDetections{j}.Measurement(1);
                y_ego = objectDetections{j}.Measurement(2);
                % Convert ego-centric to world coordinates
                x_world_relative = cos(ego_yaw) * x_ego - sin(ego_yaw) * y_ego;
                y_world_relative = sin(ego_yaw) * x_ego + cos(ego_yaw) * y_ego; 

                temp_x = temp_x + x_world_relative;
                temp_y = temp_y + y_world_relative;
                   
                end


                avgPositions(i,1) = x_ego_world + temp_x/length(objectDetections);
                avgPositions(i,2) = y_ego_world + temp_y/length(objectDetections);
               
            end
            measures = avgPositions;
        end

        function trajectory = get_trajectory(obj, allData, actor_id)
            carTrajectory = [];
            for i = 1:numel(allData)
                actorPoses = allData(i).ActorPoses;
                for j = 1:numel(actorPoses)
                    if actorPoses(j).ActorID == actor_id 
                        carTrajectory = [carTrajectory; actorPoses(j).Position];

                    end
                end
            end
            trajectory = carTrajectory;
        end
    end

end
classdef utility_functions
    methods
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

    function actor_velocity = compute_actual_velocity(obj, allData, actor_id, time)
    numSteps = numel(allData) - 1;  % Number of time steps
    actor_velocity = zeros(numSteps, 2);  % Preallocate velocity array
    
    for i = 1:numSteps
        % Get the actor poses at times t and t+1
        actorPoses_t = allData(i).ActorPoses;
        actorPoses_t1 = allData(i+1).ActorPoses;
        
        % Compute the time difference dynamically for this step
        dt = time(i+1) - time(i);
        
        % Find the actor with the specified actor_id at both time steps
        foundActor = false;
        for j = 1:numel(actorPoses_t)
            if actorPoses_t(j).ActorID == actor_id
                % Extract the velocity directly from the data
                v_x = actorPoses_t(j).Velocity(1);
                v_y = actorPoses_t(j).Velocity(2);

                actor_velocity(i, :) = [v_x, v_y];
                foundActor = true;
                break;  % Exit once the actor is found
            end
        end

        % Handle case when actor is not found (could happen with tracking failures)
        if ~foundActor
            actor_velocity(i, :) = [NaN, NaN];  % Assign NaN if actor not found
        end
    end
end

    end
end
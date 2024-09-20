classdef utility_functions
    methods
        function measures = get_aggregated_measures(obj, allData, ego_id)
    numSteps = length(allData);  % Preallocate space for all timesteps
    avgPositions = zeros(numSteps, 2);  % Preallocate for the aggregated positions

    for i = 1:numSteps
        % Get ego position and yaw
        x_ego_world = allData(i).ActorPoses(ego_id).Position(1);
        y_ego_world = allData(i).ActorPoses(ego_id).Position(2);
        ego_yaw = deg2rad(allData(i).ActorPoses(ego_id).Yaw);  % Convert yaw to radians

        % Extract object detections for current time step
        objectDetections = allData(i).ObjectDetections;

        if isempty(objectDetections)
            % No detections, keep ego position as the measurement
            avgPositions(i, 1) = x_ego_world;
            avgPositions(i, 2) = y_ego_world;
        else
            % Sum up relative positions in world coordinates
            temp_x = 0;
            temp_y = 0;

            for j = 1:length(objectDetections)
                % Ego-centric to world coordinates transformation
                x_ego = objectDetections{j}.Measurement(1);
                y_ego = objectDetections{j}.Measurement(2);
                x_world_relative = cos(ego_yaw) * x_ego - sin(ego_yaw) * y_ego;
                y_world_relative = sin(ego_yaw) * x_ego + cos(ego_yaw) * y_ego;

                temp_x = temp_x + x_world_relative;
                temp_y = temp_y + y_world_relative;
            end

            % Average positions in world coordinates
            avgPositions(i, 1) = x_ego_world + temp_x / length(objectDetections);
            avgPositions(i, 2) = y_ego_world + temp_y / length(objectDetections);
        end
    end

    measures = avgPositions;  % Return aggregated measures
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

    function actor_velocity = compute_actual_velocity(obj, allData, actor_id, dt)
    numSteps = numel(allData) - 1;  % Number of time steps
    actor_velocity = zeros(numSteps, 2);  % Preallocate velocity array
    
    for i = 1:numSteps
        % Get the actor poses at times t and t+1
        actorPoses_t = allData(i).ActorPoses;
        actorPoses_t1 = allData(i+1).ActorPoses;
        
        % Find the actor with the specified actor_id at both time steps
        foundActor = false;
        for j = 1:numel(actorPoses_t)
            if actorPoses_t(j).ActorID == actor_id
                % Check if actor exists in both frames
                x_t = actorPoses_t(j).Position(1);
                y_t = actorPoses_t(j).Position(2);
                x_t1 = actorPoses_t1(j).Position(1);
                y_t1 = actorPoses_t1(j).Position(2);

                % Compute velocity in x and y directions
                v_x = (x_t1 - x_t) / dt;
                v_y = (y_t1 - y_t) / dt;

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
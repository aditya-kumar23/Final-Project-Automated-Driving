classdef utility_functions
    methods
        function measures = get_aggregated_measures(obj,allData, actor_id)
            actorPoses = allData(1).ActorPoses;
            for j = 1:numel(actorPoses)
                if actorPoses(j).ActorID == actor_id
                    fprintf('Actor ID: %d, Position: (%f, %f, %f)\n', actorPoses(j).ActorID, actorPoses(j).Position(1), actorPoses(j).Position(2), actorPoses(j).Position(3));
                    x_ego = actorPoses(j).Position(1);
                    y_ego = actorPoses(j).Position(2);
                    z_ego = actorPoses(j).Position(3);
                    break
                end
                
            end

            for i = 1:length(allData)
                % Extract object detections for current time step
                objectDetections = allData(i).ObjectDetections;
                temp_x = 0;
                temp_y = 0;
                temp_z = 0;
                for j = 1:length(objectDetections)
                    % Extract object measurement (assuming it contains position information)
                    measurement = objectDetections{j}.Measurement;
                    temp_x = temp_x  + measurement(1);
                    temp_y = temp_y + measurement(2);
                    temp_z = temp_z + measurement(3);
                end
        
        
                avgPositions(i,1) = x_ego + temp_x/length(objectDetections);
                avgPositions(i,2) = y_ego + temp_y/length(objectDetections);
                avgPositions(i,3) = z_ego + temp_z/length(objectDetections);

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
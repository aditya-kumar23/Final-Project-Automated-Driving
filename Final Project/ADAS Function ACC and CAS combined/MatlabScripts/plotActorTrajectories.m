function plotActorTrajectories(allData)
    % Check if allData is not empty
    if isempty(allData)
        error('The input allData is empty. Ensure check_checkv2() provides valid data.');
    end

    % Extract unique Actor IDs by iterating over all ActorPoses in allData
    actorIds = [];
    for i = 1:numel(allData)
        actorPoses = allData(i).ActorPoses;
        actorIds = [actorIds, [actorPoses.ActorID]];
    end
    actorIds = unique(actorIds);  % Ensure actorIds are unique

    % Initialize a figure for plotting
    figure;
    hold on;
    grid on;
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    title('Actor Trajectories');

    % Plot trajectory for each actor
    for id = actorIds
        xTrajectory = [];
        yTrajectory = [];

        % Gather trajectory points for the actor with the current id
        for i = 1:numel(allData)
            poses = allData(i).ActorPoses;
            for j = 1:numel(poses)
                if poses(j).ActorID == id
                    xTrajectory = [xTrajectory; poses(j).Position(1)];
                    yTrajectory = [yTrajectory; poses(j).Position(2)];
                end
            end
        end

        % Plot the actor's trajectory if any points were collected
        if ~isempty(xTrajectory) && ~isempty(yTrajectory)
            plot(xTrajectory, yTrajectory, '-o', 'DisplayName', ['Actor ' num2str(id)]);
        end
    end

    legend show;
    hold off;
end

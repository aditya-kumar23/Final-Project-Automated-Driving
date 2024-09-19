% Load the generated data from your function
[allData, ~, ~] = simulationEnvironment(); % replace with the actual name of your function

% Initialize carTrajectory
carTrajectory = [];

% Extract the trajectory of the car
for i = 1:numel(allData)
    actorPoses = allData(i).ActorPoses;
    for j = 1:numel(actorPoses)
        if actorPoses(j).ActorID == 1 % Change here with the actual car's actor ID
            carTrajectory = [carTrajectory; actorPoses(j).Position];
        end
    end
end

% Ensure that the carTrajectory is not empty
if ~isempty(carTrajectory)
    % Plot the trajectory
    figure;
    plot(carTrajectory(:,1), carTrajectory(:,2), '-o');
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Trajectory of the Car');
    grid on;
else
    disp('No trajectory data found for the specified car.');
end




% % Load the generated data from your function
% [allData, ~, ~] = Sonnenstrasse_sim(); % replace with name of the scenario you created
% 
% % Extract the trajectory of the car
% carTrajectory = [];
% for i = 1:numel(allData)
%     actorPoses = allData(i).ActorPoses;
%     for j = 1:numel(actorPoses)
%         if actorPoses(j).ActorID == 1 % Change here with the actor ID
%             carTrajectory = [carTrajectory; actorPoses(j).Position];
%         end
%     end
% end
% 
% % Plot the trajectory
% figure;
% plot(carTrajectory(:,1), carTrajectory(:,2), '-o');
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('Trajectory of the Car');
% grid on;

% % Extract the trajectory of the truck
% truckTrajectory = [];
% for i = 1:numel(allData)
%     actorPoses = allData(i).ActorPoses;
%     for j = 1:numel(actorPoses)
%         if actorPoses(j).ActorID == 1 % Change here with the actor ID
%             truckTrajectory = [truckTrajectory; actorPoses(j).Position];
%         end
%     end
% end
% 
% % Plot the trajectory
% figure;
% plot(truckTrajectory(:,1), truckTrajectory(:,2), '-o');
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('Trajectory of the Truck');
% grid on;
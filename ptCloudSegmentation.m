%% Create 3D clusters
clc
clearvars

ptCloud = pcread('ptCloudRaw.ply');

% Visualize the point cloud
% figure; hold on; grid on;
% pcshow(ptCloud, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
%     'MarkerSize', 45);
% 
% % Rotate and zoom the plot
% camorbit(0, -30);
% camzoom(1.5);
% 
% % Detect the globe
% globe = pcfitsphere(ptCloud, 0.1);
% 
% % Display the surface of the globe
% plot(globe);
% title('Estimated Location and Size of the Globe');
% hold off
% 
% % Label the axes
% xlabel('x-axis');
% ylabel('y-axis');
% zlabel('z-axis')
% 
% title('Up to Scale Reconstruction of the Scene');
[ptCloud1, invalidIndices] = removeInvalidPoints(ptCloud);
% figure
% pcshow(ptCloud1)
% hold on

% Isolate Points on globe
maxDistance = 0.01;
roi = [-inf, 0.5, 0.2, 0.4, 0.1, inf];
sampleIndices = findPointsInROI(ptCloud, roi);
[model, inlierIndices] = pcfitsphere(ptCloud1,maxDistance, 'SampleIndices', sampleIndices);
globe = select(ptCloud, inlierIndices);

% Get Nearest Neighbors
point = [mean(globe.Location(:,1)), mean(globe.Location(:,2)), mean(globe.Location(:,3))];
point2 = [point(1), point(2)-.25, point(3)];
K = 200;
[kIndices1, dists] = findNearestNeighbors(ptCloud1, point, K);
[kIndices2, dist2] = findNearestNeighbors(ptCloud1, point2, K);

% Plot Nearest Neighbors 
% plot3(point(1), point(2), point(3),'y*')
% plot3(ptCloud1.Location(kIndices1,1),ptCloud1.Location(kIndices1,2),ptCloud1.Location(kIndices1,3),'*')
% plot3(point2(1), point2(2), point2(3),'r*')
% plot3(ptCloud1.Location(kIndices2,1),ptCloud1.Location(kIndices2,2),ptCloud1.Location(kIndices2,3),'*')
ptCloud_nearest = pcmerge(select(ptCloud1, kIndices1), select(ptCloud1,kIndices2),1);


% figure
% pcshow(ptCloud_nearest)
% locations = ptCloud_nearest.Location;
% [x,y,z] = meshgrid(locations(:,1), locations(:,2), locations(:,3));
% x = double(x(:));
% y = double(y(:));
% z = double(z(:));
% [k1, av1] = convhull(x, y, z);
% figure
% trisurf = convhull(x,y,z);
% axis equal
ptCloud2 = pcdenoise(ptCloud_nearest,'Threshold', 0.3);

% pcshow(ptCloud2)











% figure
% pcshow(globe)
% title('actuator')
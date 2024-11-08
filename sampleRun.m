clearvars;
% Define the inputs for the UI
launch_point = [39.59 -87.75];
boundary_coords = [39.6, -87.75; 39.58, -87.75; 39.58, -87.725; 39.6, -87.725];
num_UAVs = 3;
max_speed = 15; % Speed in meters per second
uav_height = 25;
sensor_width = 75;
disp("Hello")
% Call your function to calculate the regions, paths, etc.
% (Assume 'radialsplitting' is a function you've implemented for this purpose)
[sub_polygon_vertices, optimal_paths, path_lengths, sub_areas] = radialsplitting(launch_point, boundary_coords, num_UAVs, max_speed, uav_height, sensor_width);
path_lengths
% Now, call the createUAVUI function with max_speed
createUAVUI(launch_point, boundary_coords, sub_polygon_vertices, optimal_paths, num_UAVs, max_speed);
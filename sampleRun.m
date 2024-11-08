function uavSimulation(launch_point, boundary_coords, num_UAVs, max_speed, uav_height, sensor_width)
% uavSimulation Simulates UAV deployment and visualization.
%
%   uavSimulation(launch_point, boundary_coords, num_UAVs, max_speed, uav_height, sensor_width)
%   calculates the optimal regions and paths for UAVs and creates a user interface
%   to visualize the deployment.
%
%   Inputs:
%       launch_point   - 1x2 vector [latitude, longitude] of the UAV launch point
%       boundary_coords - Nx2 matrix of [latitude, longitude] defining the boundary
%       num_UAVs       - Number of UAVs to deploy
%       max_speed      - Maximum speed of UAVs in meters per second
%       uav_height     - Altitude of UAVs in meters
%       sensor_width   - Width of the sensor coverage in meters
%
%   Example:
%       launch_point = [39.59, -87.75];
%       boundary_coords = [39.6, -87.75; 39.58, -87.75; 39.58, -87.725; 39.6, -87.725];
%       uavSimulation(launch_point, boundary_coords, 3, 15, 25, 75);

    %% Validate Inputs
    narginchk(6,6); % Ensure exactly six input arguments are provided
    
    % Display a welcome message
    disp("Initializing UAV Simulation...");

    %% Calculate Regions and Paths
    try
        [sub_polygon_vertices, optimal_paths, path_lengths, sub_areas] = radialsplitting(...
            launch_point, boundary_coords, num_UAVs, max_speed, uav_height, sensor_width);
    catch ME
        error("Error in radialsplitting function: %s", ME.message);
    end

    %% Display Path Lengths
    disp("Calculated Path Lengths (meters):");
    disp(path_lengths);

    %% Create UAV User Interface
    try
        createUAVUI(launch_point, boundary_coords, sub_polygon_vertices, optimal_paths, num_UAVs, max_speed);
    catch ME
        error("Error in createUAVUI function: %s", ME.message);
    end

    disp("UAV Simulation Completed Successfully.");
end

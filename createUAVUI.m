function createUAVUI(launch_point, boundary_coords, sub_polygon_vertices, optimal_paths, num_UAVs, max_speed)
    % Create a UIFigure
    fig = uifigure('Name', 'UAV Path Visualization', 'AutoResizeChildren', 'off');
    fig.Position = [100, 100, 800, 600];

    % Define UAV models and their battery strengths (in minutes)
    uavModels = { 'UAV Bat', 'RQ-4 Global Hawk', 'MQ-4C Triton'};
    batteryStrengths = containers.Map({'UAV Bat', 'RQ-4 Global Hawk', 'MQ-4C Triton'}, [888000, 2092000, 13705000]); % Example values


    % Create a geographic axis for the main map plotting
    main_geoax = geoaxes(fig, 'Position', [0.1, 0.25, 0.8, 0.7]);
    geolimits(main_geoax, [min(boundary_coords(:, 1))-0.01, max(boundary_coords(:, 1))+0.01], ...
        [min(boundary_coords(:, 2))-0.01, max(boundary_coords(:, 2))+0.01]);
    geobasemap(main_geoax, 'satellite');
    hold(main_geoax, 'on');

    % Create a minimap in the bottom-left corner
    minimap_width = 0.26; % Approximately 33% of main_geoax width (0.8 * 0.33)
    minimap_height = 0.231; % Approximately 33% of main_geoax height (0.7 * 0.33)
    minimap_left = 0.1; % Same as main_geoax left position
    minimap_bottom = 0.25; % Same as main_geoax bottom position

    % Adjust so that minimap is in bottom-left corner of the main_geoax
    minimap = geoaxes(fig, 'Position', [minimap_left, minimap_bottom, minimap_width, minimap_height]);

    % Obtain combined limits of the boundary_coords and launch_point
    all_latitudes = [boundary_coords(:, 1); launch_point(1)];
    all_longitudes = [boundary_coords(:, 2); launch_point(2)];
    
    % Set limits in the minimap to include both boundary_coords and launch_point
    geolimits(minimap, [min(all_latitudes)-0.005, max(all_latitudes)+0.005], ...
        [min(all_longitudes)-0.005, max(all_longitudes)+0.005]);

    geobasemap(minimap, 'satellite');
    hold(minimap, 'on');

    minimap.LongitudeLabel.Visible = 'off';
    minimap.LatitudeLabel.Visible = 'off';

    % Hide the grid lines (which serve as tick marks)
    minimap.GridAlpha = 0;
    
    % (Optional) Hide minor grid lines if they exist
    if isprop(minimap, 'MinorGridVisible')
        minimap.MinorGridAlpha = 0;
    end


    % Plot the boundary on both main map and minimap
    boundary_coords_closed = [boundary_coords; boundary_coords(1,:)];
    geoplot(main_geoax, boundary_coords_closed(:,1), boundary_coords_closed(:,2), ...
        'w--', 'LineWidth', 1.5);
    geoplot(minimap, boundary_coords_closed(:,1), boundary_coords_closed(:,2), ...
        'w--', 'LineWidth', 1.5);

    % Plot the launch point on both main map and minimap
    geoscatter(main_geoax, launch_point(1), launch_point(2), 100, 'r', 'filled', 'DisplayName', 'Launch Point');
    geoscatter(minimap, launch_point(1), launch_point(2), 100, 'r', 'filled');


    
    % Input for Max Speed
    max_speed_label = uilabel(fig, 'Text', 'Max Speed (m/s):', 'Position', [20, 80, 100, 20]);
    max_speed_input = uieditfield(fig, 'numeric', 'Value', max_speed, 'Position', [130, 80, 50, 25], 'Limits', [0, Inf]);

    % Speed Multiplier Input
    speedLabel = uilabel(fig, 'Text', 'Speed Multiplier:', 'Position', [20, 50, 100, 20]);
    speedInput = uieditfield(fig, 'numeric', 'Value', 1, 'Position', [130, 50, 50, 25], 'Limits', [0.1, 1000], 'RoundFractionalValues', false);

    % Time Elapsed Label
    timeLabel = uilabel(fig, 'Text', 'Time Elapsed: 0 s / 0 s', 'Position', [200, 80, 200, 20]);

    % UAV Model Dropdown
    uavModelLabel = uilabel(fig, 'Text', 'Select UAV Model:', 'Position', [20, 110, 120, 20]);
    uavModelDropdown = uidropdown(fig, ...
        'Items', uavModels, ...
        'Value', uavModels{1}, ... % Default selection
        'Position', [150, 110, 100, 25]);

    % Display Battery Strength
    batteryLabel = uilabel(fig, 'Text', sprintf('Max Distance: %d km', batteryStrengths(uavModels{1})/1000), ...
        'Position', [270, 110, 150, 20]);

    % Set up callback for UAV Model selection
    uavModelDropdown.ValueChangedFcn = @(src, event) updateUAVModel(src.Value);

    % Initialize battery strength variable
    batteryStrength = batteryStrengths(uavModelDropdown.Value); % in minutes

    % Create a Play button
    playButton = uibutton(fig, 'push', 'Text', 'Play', 'Position', [20, 15, 70, 30]);

    % Set up slider
    slider = uislider(fig, 'Position', [120, 30, 650, 3], 'Limits', [0, 1], 'Value', 0);
    slider.UserData.speedMultiplier = 1; % Default speed multiplier

    % Configure callbacks
    speedInput.ValueChangedFcn = @(src, event) updateSpeedMultiplier();
    max_speed_input.ValueChangedFcn = @(src, event) recomputePaths();
    slider.ValueChangedFcn = @(sld, event) updateUAVPositions(sld.Value);
    playButton.ButtonPushedFcn = @(btn,event) togglePlay();

    % Initialize variables
    t = timer('ExecutionMode', 'fixedRate', 'Period', 0.1);
    t.TimerFcn = @(~,~) incrementSlider();
    colors = lines(num_UAVs);
    uav_positions_main = gobjects(num_UAVs, 1);
    uav_trails_main = gobjects(num_UAVs, 1);
    uav_positions_mini = gobjects(num_UAVs, 1);
    uav_trails_mini = gobjects(num_UAVs, 1);
    uav_paths = [];
    total_path_length = [];
    durations = [];
    max_duration = 0;

    % Initialize battery bars as rectangles above UAV positions on main map
    battery_bars_main = gobjects(num_UAVs,1);
    battery_bars_mini = gobjects(num_UAVs,1);
    
    
    % Initial plotting for UAV paths and positions
    for k = 1:num_UAVs

        % Define initial battery level
        battery_percentage = 1; % 100%
        
        % Define position offsets
        lat_offset = 0.0007; % Slightly above the UAV
        lon_offset = 0;
        
        % Plot battery bar on main_geoax
        battery_bars_main(k) = geoplot(main_geoax, ...
            [optimal_paths{k}(1,1) + lat_offset, optimal_paths{k}(1,1) + lat_offset], ...
            [optimal_paths{k}(1,2) - 0.0002, optimal_paths{k}(1,2) + 0.0002], ...
            'g-', 'LineWidth', 2);
        
        % Plot battery bar on minimap
        battery_bars_mini(k) = geoplot(minimap, ...
            [optimal_paths{k}(1,1) + lat_offset, optimal_paths{k}(1,1) + lat_offset], ...
            [optimal_paths{k}(1,2) - 0.0002, optimal_paths{k}(1,2) + 0.0002], ...
            'g-', 'LineWidth', 2);
        % Plot initial UAV position along the path on main map
        uav_positions_main(k) = geoscatter(main_geoax, optimal_paths{k}(1,1), optimal_paths{k}(1,2), 100, ...
            'MarkerFaceColor', colors(k, :), 'DisplayName', sprintf('UAV %d', k));
        % Create line for UAV path trace on main map
        uav_trails_main(k) = geoplot(main_geoax, NaN, NaN, 'Color', colors(k, :), 'LineWidth', 1.5);

        % Plot initial UAV position along the path on minimap
        uav_positions_mini(k) = geoscatter(minimap, optimal_paths{k}(1,1), optimal_paths{k}(1,2), 100, ...
            'MarkerFaceColor', colors(k, :));
        % Create line for UAV path trace on minimap
        uav_trails_mini(k) = geoplot(minimap, NaN, NaN, 'Color', colors(k, :), 'LineWidth', 1.5);

           % Initialize battery labels
        battery_labels_main(k) = text(main_geoax, optimal_paths{k}(1,1) + lat_offset, optimal_paths{k}(1,2), ...
           sprintf('%d%%', round(100 * battery_percentage)), 'Color', 'w', 'HorizontalAlignment', 'center');
        battery_labels_mini(k) = text(minimap, optimal_paths{k}(1,1) + lat_offset, optimal_paths{k}(1,2), ...
           sprintf('%d%%', round(100 * battery_percentage)), 'Color', 'w', 'HorizontalAlignment', 'center');
    end

    % Recompute paths with initial max_speed
    recomputePaths();

    % Update title and add legend to the main map
    title(main_geoax, 'UAV Paths Over Time');
    hold(main_geoax, 'off');

    % Nested functions
    function recomputePaths()
        max_speed = max_speed_input.Value; % Get the current max speed

        % Recompute cumulative distances and times for each UAV path
        uav_paths = cell(num_UAVs, 1);
        total_path_length = zeros(num_UAVs, 1);
        durations = zeros(num_UAVs, 1);
        for k = 1:num_UAVs
            lats = optimal_paths{k}(:,1);
            lons = optimal_paths{k}(:,2);
            distances = computeDistances(lats, lons);
            times = distances / max_speed; % Times in seconds
            uav_paths{k} = struct('path', [lats, lons], 'distances', distances, 'times', times);
            total_path_length(k) = distances(end);
            durations(k) = times(end);
        end
        % Update slider limits based on new durations
        max_duration = max(durations); % In seconds
        slider.Limits = [0, max_duration];
        slider.Value = 0; % Reset slider

        % Reset UAV trails and positions on both maps
        for k = 1:num_UAVs
            set(uav_trails_main(k), 'LatitudeData', NaN, 'LongitudeData', NaN);
            set(uav_positions_main(k), 'LatitudeData', optimal_paths{k}(1,1), 'LongitudeData', optimal_paths{k}(1,2));

            set(uav_trails_mini(k), 'LatitudeData', NaN, 'LongitudeData', NaN);
            set(uav_positions_mini(k), 'LatitudeData', optimal_paths{k}(1,1), 'LongitudeData', optimal_paths{k}(1,2));
        end
        % Update time label
        timeLabel.Text = sprintf('Time Elapsed: 0 s / %.1f s', max_duration);
    end

    % Nested function to handle UAV model changes
    function updateUAVModel(selectedModel)
        % Update battery strength based on selected model
        batteryStrength = batteryStrengths(selectedModel);
        
        % Update the battery strength label
        batteryLabel.Text = sprintf('Max Distance: %d km', batteryStrength/1000);
        
        % Optionally, recompute UAV paths or adjust flight parameters
        recomputePaths(); % Example: Adjust paths based on new battery strength
        
        % You can also implement additional logic here, such as limiting flight duration
        % or modifying UAV behaviors based on batteryStrength
    end

    function updateSpeedMultiplier()
        speedMultiplier = speedInput.Value;
        slider.UserData.speedMultiplier = speedMultiplier;
    end

    function togglePlay()
        % Toggle play/pause for the timer
        if strcmp(t.Running, 'off')
            start(t);
            playButton.Text = 'Pause';
        else
            stop(t);
            playButton.Text = 'Play';
        end
    end

    function incrementSlider()
        % Get the speed multiplier
        speedMultiplier = slider.UserData.speedMultiplier;
        % Increment the slider value
        increment = t.Period;
        new_value = slider.Value + speedMultiplier * increment;
        if new_value <= slider.Limits(2)
            slider.Value = new_value;
            updateUAVPositions(slider.Value);
        else
            slider.Value = slider.Limits(2); % Stop at max
            updateUAVPositions(slider.Value);
            stop(t); % Stop the timer when the end is reached
            playButton.Text = 'Play';
        end
    end

    function updateUAVPositions(current_time)
        % Update time label
        timeLabel.Text = sprintf('Time Elapsed: %.1f s / %.1f s', current_time, max_duration);
        
        % Update the position of each UAV based on current time and trace path
        for k = 1:numel(uav_paths)
            path_info = uav_paths{k};
            times = path_info.times;
            distances = path_info.distances;
            
            % Determine UAV position based on current time
            if current_time >= times(end)
                pos = path_info.path(end, :);
                trail_lats = path_info.path(:,1);
                trail_lons = path_info.path(:,2);
                distance_traveled = distances(end);
            else
                segment_idx = find(times <= current_time, 1, 'last');
                if segment_idx < numel(times)
                    t1 = times(segment_idx);
                    t2 = times(segment_idx + 1);
                    lat1 = path_info.path(segment_idx,1);
                    lon1 = path_info.path(segment_idx,2);
                    lat2 = path_info.path(segment_idx +1,1);
                    lon2 = path_info.path(segment_idx +1,2);
                    fraction = (current_time - t1) / (t2 - t1);
                    pos_lat = lat1 + fraction * (lat2 - lat1);
                    pos_lon = lon1 + fraction * (lon2 - lon1);
                    pos = [pos_lat, pos_lon];
                    trail_lats = [path_info.path(1:segment_idx,1); pos_lat];
                    trail_lons = [path_info.path(1:segment_idx,2); pos_lon];
                    distance_traveled = distances(segment_idx) + fraction * (distances(segment_idx +1) - distances(segment_idx));
                else
                    pos = path_info.path(end, :);
                    trail_lats = path_info.path(:,1);
                    trail_lons = path_info.path(:,2);
                    distance_traveled = distances(end);
                end
            end
    
            % Update UAV marker position on main map
            uav_positions_main(k).LatitudeData = pos(1);
            uav_positions_main(k).LongitudeData = pos(2);

            % Update the UAV trail on main map
            set(uav_trails_main(k), 'LatitudeData', trail_lats, 'LongitudeData', trail_lons);
    
            % Update UAV marker position on minimap
            uav_positions_mini(k).LatitudeData = pos(1);
            uav_positions_mini(k).LongitudeData = pos(2);
            
            % Update the UAV trail on minimap
            set(uav_trails_mini(k), 'LatitudeData', trail_lats, 'LongitudeData', trail_lons);
    
            % Calculate remaining battery based on distance traveled
            battery_total_distance = batteryStrength; % meters            
            distance_remaining = max(battery_total_distance - distance_traveled, 0);
            battery_percentage = distance_remaining / battery_total_distance;
            battery_percentage = max(min(battery_percentage, 1), 0);
    
            % Update battery bars
            max_bar_length_meters = 50; % Maximum length of the battery bar in meters
            bar_length_meters = battery_percentage * max_bar_length_meters;
            bar_length = bar_length_meters / 111000; % Convert meters to degrees latitude
    
            % Update battery bar on main_geoax
            set(battery_bars_main(k), 'LatitudeData', [pos(1) + lat_offset, pos(1) + lat_offset + bar_length]);
            set(battery_bars_main(k), 'LongitudeData', [pos(2), pos(2)]);
           
            % Update battery bar color
            if battery_percentage > 0.5
                bar_color = 'g';
            elseif battery_percentage > 0.2
                bar_color = 'y';
            else
                bar_color = 'r';
            end
            set(battery_bars_main(k), 'Color', bar_color);

            % Update battery labels
            set(battery_labels_main(k), 'Position', [pos(1) + lat_offset, pos(2)], 'String', sprintf('%d%%', round(100 * battery_percentage)), 'Color', bar_color);
            set(battery_labels_mini(k), 'Position', [pos(1) + lat_offset, pos(2)], 'String', sprintf('%d%%', round(100 * battery_percentage)), 'Color', bar_color);
           
            % Update battery bar on minimap
            set(battery_bars_mini(k), 'LatitudeData', [pos(1) + lat_offset, pos(1) + lat_offset + bar_length]);
            set(battery_bars_mini(k), 'LongitudeData', [pos(2), pos(2)]);
            set(battery_bars_mini(k), 'Color', bar_color);
    
            % Optionally update battery labels if you have them
            % battery_labels_main(k).String = sprintf('%d%%', round(battery_percentage * 100));
            % battery_labels_main(k).Color = label_color;
            % battery_labels_mini(k).String = sprintf('%d%%', round(battery_percentage * 100));
            % battery_labels_mini(k).Color = label_color;
        end
    end

    function distances = computeDistances(latitudes, longitudes)
        % Calculate distances in meters between successive lat/lon points
        % latitudes and longitudes are column vectors or arrays
        % distances is a column vector of cumulative distances

        % Number of points
        n = length(latitudes);

        % Initialize distances
        segment_distances = zeros(n-1,1);

        % Earth radius in meters
        R = 6371000;

        % Convert degrees to radians
        latitudes_rad = deg2rad(latitudes);
        longitudes_rad = deg2rad(longitudes);

        for i=1:n-1
            % Compute distance between points i and i+1
            delta_lat = latitudes_rad(i+1) - latitudes_rad(i);
            delta_lon = longitudes_rad(i+1) - longitudes_rad(i);
            a = sin(delta_lat/2)^2 + cos(latitudes_rad(i)) * cos(latitudes_rad(i+1)) * sin(delta_lon/2)^2;
            c = 2 * atan2(sqrt(a), sqrt(1 - a));
            d = R * c; % Distance in meters
            segment_distances(i) = d;
        end
        % Cumulative distances
        distances = [0; cumsum(segment_distances)];
    end
end
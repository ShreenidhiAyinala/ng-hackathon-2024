function [sub_polygon_vertices, optimal_paths, path_lengths, sub_areas] = radialsplitting(launch_point, boundary_coords, num_UAVs, ~, uav_height, sensor_width)

    % Extract the launch point coordinates
    x0 = launch_point(1);
    y0 = launch_point(2);

    % Extract boundary coordinates for the polygon vertices
    xv = boundary_coords(:, 1)';
    yv = boundary_coords(:, 2)';

    % Set the number of UAVs
    n = num_UAVs;

    launch_x = x0;
    launch_y = y0;

    % Before scaling, check the internal angles at each vertex
    n_vertices = length(xv);
    angles = zeros(n_vertices,1);

    angle_sum = 0;

    for i = 1:n_vertices
        % Indices of previous and next vertices (with wrapping)
        iprev = mod(i - 2, n_vertices) + 1;
        inext = mod(i, n_vertices) + 1;
    
        % Vectors from current vertex to adjacent vertices
        vec_prev = [xv(iprev) - xv(i), yv(iprev) - yv(i)];
        vec_next = [xv(inext) - xv(i), yv(inext) - yv(i)];
    
        % Normalize the vectors
        vec_prev = vec_prev / norm(vec_prev);
        vec_next = vec_next / norm(vec_next);
    
        % Compute the angle between vec_prev and vec_next
        angle = atan2(vec_prev(1)*vec_next(2) - vec_prev(2)*vec_next(1), dot(vec_prev, vec_next));
        if angle < 0
            angle = angle + 2*pi;
        end
    
        % Accumulate the angle sum
        angle_sum = angle_sum + angle;
    
        % Store the angle
        angles(i) = angle;
    end
    
    % Expected sum of internal angles for a simple polygon
    expected_sum = (n_vertices - 2) * pi;
    
    % Check if the sum exceeds the expected sum
    if angle_sum > expected_sum + 1e-6 % Adding a small tolerance
        % Adjust angles by subtracting from 2*pi
        angles = 2*pi - angles;
        disp('Angles adjusted by subtracting from 2*pi.');
    end
    
    % Now, detect concave angles (angles > pi)
    idx = find(angles > pi);
    
    % Display the angles and indices of concave vertices
    disp('Adjusted Angles at each vertex (in radians):');
    disp(angles);
    if ~isempty(idx)
        disp('Indices of concave vertices:');
        disp(idx);
        % Update the launch point if needed
        x0 = xv(mod(idx + 1, 4) + 1);
        y0 = yv(mod(idx + 1, 4) + 1);
    else
        disp('No concave vertices detected.');
        % Find the closest point on each edge of the polygon to the launch point
        min_dist = Inf;
        closest_point = [x0, y0]; % Initialize to current launch point in case no closer point is found
        
        for i = 1:n_vertices
            % Define current edge endpoints
            p1 = [xv(i), yv(i)];
            p2 = [xv(mod(i, n_vertices) + 1), yv(mod(i, n_vertices) + 1)];
            
            % Calculate the closest point on this edge to the launch point
            t = dot([x0 - p1(1), y0 - p1(2)], p2 - p1) / norm(p2 - p1)^2;
            t = max(0, min(1, t));  % Clamp t to [0, 1] to ensure the point lies on the segment
            
            % Compute the closest point
            closest_on_edge = p1 + t * (p2 - p1);
            
            % Calculate the distance from launch point to this closest point
            dist = norm([x0, y0] - closest_on_edge);
            
            % Update if this is the closest point found so far
            if dist < min_dist
                min_dist = dist;
                closest_point = closest_on_edge;
            end
        end
        
        % Set the new launch point to the closest point on the polygon boundary
        x0 = closest_point(1);
        y0 = closest_point(2);
        
        disp('Updated launch point to closest point on polygon boundary.');
        disp(['New launch point: (', num2str(x0), ', ', num2str(y0), ')'])
    end
    scale_factor = 1000;

    % Number of steps for the sweeping ray
    num_steps = 1000;

    x0_scaled = x0 * scale_factor;
    y0_scaled = y0 * scale_factor;

    xv_scaled = xv * scale_factor;
    yv_scaled = yv * scale_factor;
    
    % Create the polyshape for the polygon
    pgon = polyshape(xv_scaled, yv_scaled);
    
    % Compute total area of the polygon
    total_area = vpa(area(pgon));
    
    % Check if the point (x0, y0) is inside the polygon
    [inside, onEdge] = inpolygon(x0_scaled, y0_scaled, pgon.Vertices(:,1), pgon.Vertices(:,2));
    if inside
        theta_min = vpa(0);
        theta_max = vpa(2*pi);
    else
        angles = atan2(yv_scaled(1:end) - y0_scaled, xv_scaled(1:end) - x0_scaled);
        angles_unwrapped = unwrap(angles);
        theta_min = vpa(min(angles_unwrapped));
        theta_max = vpa(max(angles_unwrapped));
    end
    
    theta_i = vpa(linspace(theta_min, theta_max, num_steps));
    
    % Define the symbolic variables
    R = vpa(1e3);
    xi_closest = sym(zeros(1, num_steps));
    yi_closest = sym(zeros(1, num_steps));
    
    num_vertices = length(xv_scaled);
    
    for i = 1:num_steps
        theta = theta_i(i);
    
        cos_theta = cos(theta);
        sin_theta = sin(theta);
    
        intersections_x = [];
        intersections_y = [];
        distances = [];
    
        for j = 1:num_vertices
            % Get edge endpoints
            x1 = xv_scaled(j);
            y1 = yv_scaled(j);
            if j < num_vertices
                x2 = xv_scaled(j+1);
                y2 = yv_scaled(j+1);
            else
                x2 = xv_scaled(1); % Wrap around to the first vertex
                y2 = yv_scaled(1);
            end
    
            % Construct matrices to solve for t and s
            A = [cos_theta, -(x2 - x1); sin_theta, -(y2 - y1)];
            b = [x1 - x0_scaled; y1 - y0_scaled];
    
            % Check if the determinant is zero (lines are parallel)
            if abs(det(A)) < 1e-10
                continue; % No intersection if lines are parallel
            end
    
            % Solve for t and s
            u = A \ b;
            t = u(1);
            s = u(2);
    
            % Check if the intersection point lies on the ray and edge segment
            if t >= 0 && s >= 0 && s <= 1
                x_int = x0_scaled + t * cos_theta;
                y_int = y0_scaled + t * sin_theta;
    
                intersections_x(end+1) = x_int;
                intersections_y(end+1) = y_int;
                distances(end+1) = hypot(double(x_int - x0_scaled), double(y_int - y0_scaled));
            end
        end
    
        if isempty(intersections_x)
            xi_closest(i) = NaN;
            yi_closest(i) = NaN;
        else
            % Find the closest intersection point
            [~, idx_min] = min(distances);
            xi_closest(i) = vpa(intersections_x(idx_min));
            yi_closest(i) = vpa(intersections_y(idx_min));
        end
    end
    
    % Filter NaN values and keep valid intersections
    xi_closest
    valid_idx = ~isnan(xi_closest);
    xi_valid = xi_closest(valid_idx);
    yi_valid = yi_closest(valid_idx);
    theta_valid = theta_i(valid_idx);
    
    % Calculate areas of wedges with increased precision
    area_wedges = sym(zeros(1, length(theta_valid) - 1));
    theta_valid
    for i = 1:length(theta_valid) - 1
        % Define the sector boundary using symbolic precision
        x_sector = [x0_scaled, x0_scaled + R * cos(theta_valid(i)), x0_scaled + R * cos(theta_valid(i+1)), x0_scaled];
        y_sector = [y0_scaled, y0_scaled + R * sin(theta_valid(i)), y0_scaled + R * sin(theta_valid(i+1)), y0_scaled];
        
        % Construct the sector and intersection shapes with high precision
        sector_shape = polyshape(double(x_sector), double(y_sector));
        intersection_shape = intersect(sector_shape, pgon); % Intersection with high resolution
        
        % Calculate and store the area with higher precision
        area_wedges(i) = vpa(area(intersection_shape), 32); % Higher precision (32 decimal places)
    end
    
    A_cum = vpa(cumsum(area_wedges));
    A_cum = A_cum * (total_area / A_cum(end));
    total_area
    n

    theta_k = vpa(zeros(1, n + 1));
    theta_k(1) = min(theta_valid);
    theta_k(end) = max(theta_valid);
    for k = 1:n - 1
        target_area = k * (total_area / n);
        idx = find(A_cum >= target_area, 1);
        idx
        theta_k(k + 1) = theta_valid(idx);
    end

    theta_k
    x0
    y0


    disp('Values of theta_k:');
    disp(theta_k);
    
    % Initialize outputs
    sub_polygon_vertices = cell(1, n);
    optimal_paths = cell(1, n);
    path_lengths = vpa(zeros(1, n));
    sub_areas = vpa(zeros(1, n));


    pgon = polyshape(xv, yv);
    
    % For each region
    for k = 1:n
        num_points = 10; % Adjust as needed

        % Create a vector of theta values between theta_k(k) and theta_k(k+1)
        theta_values = linspace(theta_k(k), theta_k(k+1), num_points);
        
        % Compute the x and y coordinates with variable precision arithmetic
        x_points = vpa(x0 + R * cos(theta_values));
        y_points = vpa(y0 + R * sin(theta_values));
        
        % Construct x_sector and y_sector arrays with additional points
        x_sector = vpa([x0, x_points, x0]);
        y_sector = vpa([y0, y_points, y0]);
        sector_shape = polyshape(double(x_sector), double(y_sector));
        intersection_shape = intersect(sector_shape, pgon);

        intersection_shape.Vertices
    
        if ~isempty(intersection_shape.Vertices) && area(intersection_shape) > 0
            % Store the vertices of the intersection_shape
            sub_polygon_vertices{k} = intersection_shape.Vertices;
            
            % Calculate the sub-area for each region
            sub_areas(k) = vpa(area(intersection_shape));
            
            % Extract the vertices
            vertices = intersection_shape.Vertices;
            
            % Number of vertices
            num_vertices = size(vertices, 1);
            
            % Initialize an array to store internal angles
            internal_angles = zeros(num_vertices, 1);

            % Calculate internal angles at each vertex
            for i = 1:num_vertices
                % Get the previous, current, and next vertices (with wrap-around)
                prev_vertex = vertices(mod(i - 2, num_vertices) + 1, :);
                curr_vertex = vertices(i, :);
                next_vertex = vertices(mod(i, num_vertices) + 1, :);
            
                % Vectors forming the angle at the current vertex
                vec1 = prev_vertex - curr_vertex;
                vec2 = next_vertex - curr_vertex;
            
                % Compute the cross product (z-component)
                cross_z = vec1(1) * vec2(2) - vec1(2) * vec2(1);
            
                % Compute the dot product
                dot_product = dot(vec1, vec2);
            
                % Calculate the angle in degrees using atan2d
                angle = atan2d(cross_z, dot_product);
            
                % Normalize the angle to be between 0 and 360 degrees
                if angle < 0
                    angle = angle + 360;
                end
            
                % Store the internal angle
                internal_angles(i) = angle;
            end
            
            % Identify the concave vertices (angles greater than 180 degrees)
            concave_idx = find(internal_angles > 180);
            
            % If a concave vertex is found, split the polygon
            if ~isempty(concave_idx)
                % Index of the vertex two positions away (with wrap-around)
                split_idx = mod(concave_idx + 1, num_vertices) + 1;
                
                % Define the first convex polygon
                if concave_idx < split_idx
                    poly1_indices = concave_idx:split_idx;
                else
                    poly1_indices = [concave_idx:num_vertices, 1:split_idx];
                end
                poly1_vertices = vertices(poly1_indices, :);
                
                % Define the second convex polygon
                if split_idx < concave_idx
                    poly2_indices = split_idx:concave_idx;
                else
                    poly2_indices = [split_idx:num_vertices, 1:concave_idx];
                end
                poly2_vertices = vertices(poly2_indices, :);
                
                % Do not explicitly close the polygons
                % if ~isequal(poly1_vertices(1, :), poly1_vertices(end, :))
                %     poly1_vertices(end + 1, :) = poly1_vertices(1, :);
                % end
                % if ~isequal(poly2_vertices(1, :), poly2_vertices(end, :))
                %     poly2_vertices(end + 1, :) = poly2_vertices(1, :);
                % end
                
                % Create the coverage space with the convex polygons
                space = uavCoverageSpace("Polygons", {poly1_vertices, poly2_vertices}, ...
                    "ReferenceHeight", uav_height, "UseLocalCoordinates", false);
            else
                % If the polygon is already convex, use it directly
                space = uavCoverageSpace("Polygons", {vertices}, ...
                    "ReferenceHeight", uav_height, "UseLocalCoordinates", false);
            end
            
            % Set unit dimensions
            space.UnitWidth = sensor_width;
            space.UnitLength = 0.001;
            
            % Define the range of angles to sweep through
            angles = 0:5:180;
            
            % Initialize variables to store optimal values
            min_length = Inf;
            optimal_wp = [];
            optimal_angle = 0;
            
            % Loop through each angle
            for idx_angle = 1:length(angles)
                angle = angles(idx_angle);
                
                % Set the coverage pattern with the current sweep angle
                setCoveragePattern(space, 1, "SweepAngle", angle);
                
                % Create the coverage planner and compute the path
                planner = uavCoveragePlanner(space, "Solver", "MinTraversal");
                [wp, ~] = plan(planner, [launch_x, launch_y, 0], [launch_x, launch_y, 0]);
                
                % Calculate traversal path length for the current angle
                path_diffs = diff(wp(:, 1:2));
                traversal_length = sum(sqrt(sum(path_diffs.^2, 2)));
                
                % Check if this is the minimal length so far
                if traversal_length < min_length
                    min_length = traversal_length;
                    optimal_wp = wp;
                    optimal_angle = angle;
                end
            end
            
            % Store the optimal path and path length
            optimal_paths{k} = optimal_wp;
            path_lengths(k) = min_length;

            
        else
            warning('Region %d is invalid and will not be processed.', k);
            % Set empty outputs for this region
            sub_polygon_vertices{k} = [];
            optimal_paths{k} = [];
            path_lengths(k) = NaN;
            sub_areas(k) = NaN;
        end
    end
end

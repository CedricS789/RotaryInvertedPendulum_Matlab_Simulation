function animate_pendulum_v2(t, phi, theta, l, r)
    %% Settings and Options (Adjustable)
    %max_frames = 1000;       % Maximum number of frames for the animation
    axis_margin = 0.05;      % Margin added to axis limits (meters)
    view_angle = [70, 8];   % Viewing angle for the 3D plot [azimuth, elevation]
    line_width = 3;          % Line width for arm and pendulum
    line_base_width = 2*7;
    bob_size = 8;            % Marker size for the pendulum bob
    font_size = 12;          % Font size for labels and title
    motor_height = l + 0.5;  % Height of the motor (pivot point) above the floor

    %% Ensure 'theta', 'phi', and 't' are column vectors of the same length
    theta = theta(:);
    phi = phi(:);
    t = t(:);
    if length(theta) ~= length(phi) || length(theta) ~= length(t)
        error('Theta, Phi, and t must be vectors of the same length.');
    end

    % %% Downsample data if needed to speed up the animation
    % total_frames = length(t);
    % if total_frames > max_frames
    %     idx = round(linspace(1, total_frames, max_frames));
    %     theta = theta(idx);
    %     phi = phi(idx);
    %     t = t(idx);
    % end

    %% Compute Positions
    % Base (motor) position
    x_base = 0;
    y_base = 0;
    z_base = motor_height;  % Lift the motor position

    % Pivot point positions (end of the arm)
    x_pivot = x_base + r * cos(theta);
    y_pivot = y_base + r * sin(theta);
    z_pivot = z_base * ones(size(theta));

    % Unit vectors along the arm direction
    arm_dir_x = cos(theta);
    arm_dir_y = sin(theta);
    arm_dir_z = zeros(size(theta));

    % Unit vectors perpendicular to the arm in horizontal plane
    perp_dir_x = -sin(theta);
    perp_dir_y = cos(theta);
    perp_dir_z = zeros(size(theta));

    % Pendulum bob positions
    x_bob = x_pivot + l * sin(phi) .* perp_dir_x;
    y_bob = y_pivot + l * sin(phi) .* perp_dir_y;
    % Corrected z_bob calculation
    z_bob = z_pivot + l * cos(phi);

    %% Animation Setup
    fig = figure('Color', 'w', 'Name', 'Rotary Inverted Pendulum Animation', 'NumberTitle', 'off');
    clf;
    hold on;
    axis equal;

    % Adjust axis limits for closer view
    max_range = l + r + axis_margin;
    axis([-max_range, max_range, -max_range, max_range, 0, motor_height + l + axis_margin]);
    grid on;

    % Plot the base point
    plot3(x_base, y_base, z_base, 'ko', 'MarkerSize', bob_size*3, 'MarkerFaceColor', 'k');
    
    % Plot the arm from the floor to the motor position
    plot3([x_base, x_base], [y_base, y_base], [0, z_base], 'k-', 'LineWidth', line_base_width);

    % Initialize the arm and pendulum
    arm = plot3([x_base, x_pivot(1)], [y_base, y_pivot(1)], [z_base, z_pivot(1)], 'b-', 'LineWidth', line_width);
    pendulum = plot3([x_pivot(1), x_bob(1)], [y_pivot(1), y_bob(1)], [z_pivot(1), z_bob(1)], 'r-', 'LineWidth', line_width);
    bob = plot3(x_bob(1), y_bob(1), z_bob(1), 'ro', 'MarkerSize', (bob_size/1e10), 'MarkerFaceColor', 'r');

    % Set labels and title
    xlabel('X Position (m)', 'FontSize', font_size);
    ylabel('Y Position (m)', 'FontSize', font_size);
    zlabel('Z Position (m)', 'FontSize', font_size);
    title('Rotary Inverted Pendulum Animation', 'FontSize', font_size + 2);
    view(view_angle);

    % Optimize rendering
    set(gca, 'XLimMode', 'manual', 'YLimMode', 'manual', 'ZLimMode', 'manual');

    %% Animation Loop
    % Initialize timer
    tic;
    start_time = t(1);
    for i = 1:length(theta)
        % Update the arm position
        set(arm, 'XData', [x_base, x_pivot(i)], 'YData', [y_base, y_pivot(i)], 'ZData', [z_base, z_pivot(i)]);

        % Update the pendulum rod position
        set(pendulum, 'XData', [x_pivot(i), x_bob(i)], 'YData', [y_pivot(i), y_bob(i)], 'ZData', [z_pivot(i), z_bob(i)]);

        % Update the pendulum bob position
        set(bob, 'XData', x_bob(i), 'YData', y_bob(i), 'ZData', z_bob(i));

        % Update the title with current simulation time
        t_current = t(i);
        title(sprintf('Rotary Inverted Pendulum Animation (t = %.2f s)', t_current), 'FontSize', font_size + 2);

        % Render the updates efficiently
        drawnow limitrate;

        % Control the animation speed based on simulation time
        if i < length(theta)
            % Calculate time to pause until next frame
            elapsed_time = toc;
            pause_time = (t(i+1) - start_time) - elapsed_time;
            if pause_time > 0
                pause(pause_time);
            end
        end
        % Reset timer for next frame
        tic;
        start_time = t(i);
    end
end

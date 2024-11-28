classdef wheelMobileRobot
    properties
        wheel_radius
        length
        prop_gain
        amplitude
        omega
    end
    
    methods
        % Constructor
        function obj = wheelMobileRobot(wheel_radius, length, prop_gain)
            obj.wheel_radius = wheel_radius;
            obj.length = length;
            obj.prop_gain = prop_gain;
            obj.amplitude = 5;
            obj.omega = 0.2;
        end

        % Position error calculation
        function [error_x, error_y, error_theta] = position_error(obj, xd, yd, theta_d, x, y, theta)
            error_x = xd - x;
            error_y = yd - y;
            theta_d = atan2(error_y, error_x);
            theta_d = atan2(sin(theta_d), cos(theta_d));
            error_theta = theta_d - theta;
            error_theta = atan2(sin(error_theta), cos(error_theta));
        end

        % Correction of error
        function [theta_dot, velocity] = correction_error(obj, error_x, error_y, error_theta)
            mse = sqrt(error_x^2 + error_y^2);
            theta_dot = obj.prop_gain * error_theta;
            velocity = obj.prop_gain * mse;
        end

        % Wheel speeds calculation
        function [phidotl, phidotr] = wheel_speeds(obj, velocity, theta_dot)
            velocity_vector = [velocity; theta_dot];
            wheel_radius_matrix = [obj.wheel_radius / 2, obj.wheel_radius / 2; -obj.wheel_radius / obj.length, obj.wheel_radius / obj.length];
            inverse_wheel_radius = inv(wheel_radius_matrix);
            wheel_speeds = inverse_wheel_radius * velocity_vector;
            phidotl = wheel_speeds(1);
            phidotr = wheel_speeds(2);
        end

        % Differential motion model
        function [x_dot, y_dot, theta_dot] = diffmodel(obj, phidotl, phidotr, theta)
            x_dot = (obj.wheel_radius / 2) * (phidotl + phidotr) * cos(theta);
            y_dot = (obj.wheel_radius / 2) * (phidotl + phidotr) * sin(theta);
            theta_dot = (obj.wheel_radius / obj.length) * (phidotr - phidotl);
        end

        % Update current position
        function [x, y, theta] = position_actuel(obj, x, y, theta, x_dot, y_dot, theta_dot, dt)
            x = x + x_dot * dt;
            y = y + y_dot * dt;
            theta = theta + theta_dot * dt;
            theta = atan2(sin(theta), cos(theta));
        end
    end
end

%% Main Function
function main()
    % Create robot object
    robot = wheelMobileRobot(0.5, 2, 1.25);
    
    % Initialize variables
    x = 0.0; y = 0.0; theta = 0.0;
    amplitude = 5; half_amplitude = amplitude / 2; omega = 0.2;
    positions = [x, y]; erreur_distance = []; erreur_orientation = [];
    dt = 0.1; t_final = 60; time_steps = 0:dt:t_final;

    % User input for trajectory type
    disp("Input the name of the trajectory you want the robot to describe: infinity, circular, target");
    input_trajectory = input("Chosen trajectory:", 's');

    if strcmp(input_trajectory, "infinity")
        x_theorique = amplitude * sin(omega * time_steps);
        y_theorique = half_amplitude * sin(2 * omega * time_steps);

        for t = time_steps
            x_target = amplitude * sin(t * omega);
            y_target = half_amplitude * sin(2 * t * omega);

            [err_x, err_y, err_theta] = robot.position_error(x_target, y_target, theta, x, y, theta);
            [theta_dot, vb] = robot.correction_error(err_x, err_y, err_theta);
            [phidotl, phidotr] = robot.wheel_speeds(vb, theta_dot);
            [x_dot, y_dot, theta_dot] = robot.diffmodel(phidotl, phidotr, theta);
            [x, y, theta] = robot.position_actuel(x, y, theta, x_dot, y_dot, theta_dot, dt);

            positions = [positions; x, y];
            erreur_distance = [erreur_distance, sqrt((x - x_target)^2 + (y - y_target)^2)];
            error_orientation = atan2(y_target - y, x_target - x) - theta;
            erreur_orientation = [erreur_orientation, atan2(sin(error_orientation), cos(error_orientation))];
        end

    elseif strcmp(input_trajectory, "target")
        x_target = input("Please input the coordinates on the x-axis:");
        y_target = input("Please input the coordinates on the y-axis:");

        x_theorique = [x_target]; y_theorique = [y_target];

        for t = time_steps
            [err_x, err_y, err_theta] = robot.position_error(x_target, y_target, theta, x, y, theta);
            [theta_dot, vb] = robot.correction_error(err_x, err_y, err_theta);
            [phidotl, phidotr] = robot.wheel_speeds(vb, theta_dot);
            [x_dot, y_dot, theta_dot] = robot.diffmodel(phidotl, phidotr, theta);
            [x, y, theta] = robot.position_actuel(x, y, theta, x_dot, y_dot, theta_dot, dt);

            positions = [positions; x, y];
            erreur_distance = [erreur_distance, sqrt((x - x_target)^2 + (y - y_target)^2)];
            error_orientation = atan2(y_target - y, x_target - x) - theta;
            erreur_orientation = [erreur_orientation, atan2(sin(error_orientation), cos(error_orientation))];
        end

    elseif strcmp(input_trajectory, "circular")
        x_theorique = amplitude * sin(omega * time_steps);
        y_theorique = amplitude * cos(omega * time_steps);

        for t = time_steps
            x_target = amplitude * sin(t * omega);
            y_target = amplitude * cos(t * omega);

            [err_x, err_y, err_theta] = robot.position_error(x_target, y_target, theta, x, y, theta);
            [theta_dot, vb] = robot.correction_error(err_x, err_y, err_theta);
            [phidotl, phidotr] = robot.wheel_speeds(vb, theta_dot);
            [x_dot, y_dot, theta_dot] = robot.diffmodel(phidotl, phidotr, theta);
            [x, y, theta] = robot.position_actuel(x, y, theta, x_dot, y_dot, theta_dot, dt);

            positions = [positions; x, y];
            erreur_distance = [erreur_distance, sqrt((x - x_target)^2 + (y - y_target)^2)];
            error_orientation = atan2(y_target - y, x_target - x) - theta;
            erreur_orientation = [erreur_orientation, atan2(sin(error_orientation), cos(error_orientation))];
        end

    else
        disp("The given trajectory name is not known");
    end

    % Plot results
    positions = positions';
    x_vals = positions(1, :); y_vals = positions(2, :);

    if length(x_vals) >= 2
        figure;

        % Trajectory plot
        subplot(1, 2, 1);
        plot(x_vals, y_vals, 'b', 'DisplayName', 'Robot Trajectory'); hold on;
        plot(x_theorique, y_theorique, 'y--', 'DisplayName', 'Theoretical Trajectory');
        scatter(x_vals(1), y_vals(1), 'r', 'DisplayName', 'Initial Position');
        scatter(x_theorique(end), y_theorique(end), 'g', 'DisplayName', 'Target Position');
        xlabel('x (m)'); ylabel('y (m)');
        legend; grid on; axis equal;
        title('Robot Trajectory with Sliding Mode Control');

        % Error plot
        subplot(1, 2, 2);
        plot(time_steps, erreur_distance, 'b', 'DisplayName', 'Distance Error'); hold on;
        plot(time_steps, erreur_orientation, 'r', 'DisplayName', 'Orientation Error');

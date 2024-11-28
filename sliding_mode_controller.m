classdef wheelMobileRobot
    properties
        wheel_radius
        length
        prop_gain
        amplitude
        omega
    end
    
    methods
        function obj = wheelMobileRobot(wheel_radius, length, prop_gain)
            obj.wheel_radius = wheel_radius;
            obj.length = length;
            obj.prop_gain = prop_gain;
            obj.amplitude = 5;
            obj.omega = 0.2;
        end
        
        function [error_x, error_y, error_theta] = position_error(~, xd, yd, theta_d, x, y, theta)
            error_x = xd - x;
            error_y = yd - y;
            theta_d = atan2(error_y, error_x);
            theta_d = atan2(sin(theta_d), cos(theta_d));
            error_theta = theta_d - theta;
            error_theta = atan2(sin(error_theta), cos(error_theta));
        end
        
        function [theta_dot, velocity] = sliding_mode(obj, error_x, error_y, error_theta)
            rho = sqrt(error_x^2 + error_y^2);
            
            % Sliding surface
            s = rho * sin(error_theta);
            
            % Linear and angular velocity commands
            theta_dot = -1 * obj.prop_gain * s;
            velocity = obj.prop_gain * rho * cos(error_theta);
        end
        
        function [phidotl, phidotr] = wheel_speeds(obj, velocity, theta_dot)
            velocity_vector = [velocity; theta_dot];
            wheel_radius_matrix = [obj.wheel_radius/2, obj.wheel_radius/2; ...
                                   -obj.wheel_radius/obj.length, obj.wheel_radius/obj.length];
            inverse_wheel_radius = inv(wheel_radius_matrix);
            wheel_speeds = inverse_wheel_radius * velocity_vector;
            phidotl = wheel_speeds(1);
            phidotr = wheel_speeds(2);
        end
        
        function [x_dot, y_dot, theta_dot] = diffmodel(obj, phidotl, phidotr, theta)
            x_dot = (obj.wheel_radius / 2) * (phidotl + phidotr) * cos(theta);
            y_dot = (obj.wheel_radius / 2) * (phidotl + phidotr) * sin(theta);
            theta_dot = (obj.wheel_radius / obj.length) * (phidotr - phidotl);
        end
        
        function [x, y, theta] = position_actuel(~, x, y, theta, x_dot, y_dot, theta_dot, dt)
            x = x + x_dot * dt;
            y = y + y_dot * dt;
            theta = theta + theta_dot * dt;
            theta = atan2(sin(theta), cos(theta)); % Normalization
        end
    end
end

% Main script
clear; clc;

% Create object
robot = wheelMobileRobot(0.5, 2, 1.25);

% Initialize variables
x = 0.0; y = 0.0; theta = 0.0;
amplitude = 5;
half_amplitude = amplitude / 2;
omega = 0.2;
positions = [x, y];
erreur_distance = [];
erreur_orientation = [];
dt = 0.1;
t_final = 60;
time_steps = 0:dt:t_final;

disp('Input the name of the trajectory you want the robot to describe: infinity, circular, target');
input_trajectory = input('Chosen trajectory:', 's');

if strcmp(input_trajectory, 'infinity')
    x_theorique = amplitude * sin(omega * time_steps);
    y_theorique = half_amplitude * sin(2 * omega * time_steps);
    
    for t = time_steps
        x_target = amplitude * sin(t * omega);
        y_target = half_amplitude * sin(2 * t * omega);
        
        % Position error calculation
        [err_x, err_y, err_theta] = robot.position_error(x_target, y_target, theta, x, y, theta);
        
        % Velocity and theta_dot
        [theta_dot, vb] = robot.sliding_mode(err_x, err_y, err_theta);
        [phidotl, phidotr] = robot.wheel_speeds(vb, theta_dot);
        
        % Using the motion equations
        [x_dot, y_dot, theta_dot] = robot.diffmodel(phidotl, phidotr, theta);
        
        % Actual position and orientation
        [x, y, theta] = robot.position_actuel(x, y, theta, x_dot, y_dot, theta_dot, dt);
        
        % Saving present position
        positions = [positions; x, y];
        
        % Errors
        error_distance = sqrt((x - x_target)^2 + (y - y_target)^2);
        erreur_distance = [erreur_distance, error_distance];
        
        error_orientation = atan2(y_target - y, x_target - x) - theta;
        error_orientation = atan2(sin(error_orientation), cos(error_orientation));
        erreur_orientation = [erreur_orientation, error_orientation];
    end

elseif strcmp(input_trajectory, 'target')
    x_target = input('Please input the coordinates on the x-axis:');
    y_target = input('Please input the coordinates on the y-axis:');
    theta_target = input('Please input the coordinates of theta:');
    
    x_theorique = [x_target];
    y_theorique = [y_target];
    
    for t = time_steps
        [err_x, err_y, err_theta] = robot.position_error(x_target, y_target, theta, x, y, theta);
        
        [theta_dot, vb] = robot.sliding_mode(err_x, err_y, err_theta);
        [phidotl, phidotr] = robot.wheel_speeds(vb, theta_dot);
        
        [x_dot, y_dot, theta_dot] = robot.diffmodel(phidotl, phidotr, theta);
        [x, y, theta] = robot.position_actuel(x, y, theta, x_dot, y_dot, theta_dot, dt);
        
        positions = [positions; x, y];
        
        error_distance = sqrt((x - x_target)^2 + (y - y_target)^2);
        erreur_distance = [erreur_distance, error_distance];
        
        error_orientation = atan2(y_target - y, x_target - x) - theta;
        error_orientation = atan2(sin(error_orientation), cos(error_orientation));
        erreur_orientation = [erreur_orientation, error_orientation];
    end

elseif strcmp(input_trajectory, 'circular')
    x_theorique = amplitude * sin(omega * time_steps);
    y_theorique = amplitude * cos(omega * time_steps);
    
    for t = time_steps
        x_target = amplitude * sin(t * omega);
        y_target = amplitude * cos(t * omega);
        
        [err_x, err_y, err_theta] = robot.position_error(x_target, y_target, theta, x, y, theta);
        
        [theta_dot, vb] = robot.sliding_mode(err_x, err_y, err_theta);
        [phidotl, phidotr] = robot.wheel_speeds(vb, theta_dot);
        
        [x_dot, y_dot, theta_dot] = robot.diffmodel(phidotl, phidotr, theta);
        [x, y, theta] = robot.position_actuel(x, y, theta, x_dot, y_dot, theta_dot, dt);
        
        positions = [positions; x, y];
        
        error_distance = sqrt((x - x_target)^2 + (y - y_target)^2);
        erreur_distance = [erreur_distance, error_distance];
        
        error_orientation = atan2(y_target - y, x_target - x) - theta;
        error_orientation = atan2(sin(error_orientation), cos(error_orientation));
        erreur_orientation = [erreur_orientation, error_orientation];
    end

else
    disp('The given trajectory name is not known');
end

positions = positions';
x_vals = positions(1, :);
y_vals = positions(2, :);

if length(x_vals) >= 2
    figure;
    
    % Trajectory plot
    subplot(1, 2, 1);
    plot(x_vals, y_vals, 'b-', 'DisplayName', 'Robot Trajectory');
    hold on;
    plot(x_theorique, y_theorique, 'y--', 'DisplayName', 'Theoretical Trajectory');
    scatter(x_vals(1), y_vals(1), 'r', 'DisplayName', 'Initial Position');
    scatter(x_theorique(end), y_theorique(end), 'g', 'DisplayName', 'Target Position');
    xlabel('Position x (m)');
    ylabel('Position y (m)');
    legend;
    title('Trajectory of Differential Robot with Sliding Mode Control');
    grid on;
    axis equal;
    
    % Error plot
    subplot(1, 2, 2);
    plot(time_steps, erreur_distance, 'b-', 'DisplayName', 'Distance Error');
    hold on;
    plot(time_steps, erreur_orientation, 'r-', 'DisplayName', 'Orientation Error');
    xlabel('Time (s)');
    ylabel('Error');
    legend;
    title('Error Evolution Over Time');
    grid on;
end

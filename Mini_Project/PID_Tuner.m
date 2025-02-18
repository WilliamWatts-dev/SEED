%% Run simulation with feedback control, retrieve data, and analyze system

% Load the Simulink model
model = 'PID_Motor_Control.slx';  % Replace with your actual model name
open_system(model);

% Run the simulation
out = sim(model);

% Extract simulation results from timeseries objects
time = out.tout;  % Time vector
desired_pos = out.DesiredPosition.Data;  % Desired position (replace with actual variable)
actual_pos = out.Position.Data;  % Actual position (replace with actual variable)

% Plot Step Response
figure;
subplot(2,1,1);
plot(time, desired_pos, 'r--', 'LineWidth', 2); hold on;
plot(time, actual_pos, 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Position');
legend('Desired Position', 'Actual Position');
title('System Step Response');

% Compute step response characteristics
step_info = stepinfo(actual_pos, time, 'SettlingTimeThreshold', 0.02);

% Extract key metrics
T_rise = step_info.RiseTime;
T_settle = step_info.SettlingTime;
Overshoot = step_info.Overshoot;
SteadyStateError = abs(desired_pos(end) - actual_pos(end));

% Estimate system parameters
Kp_sys = (actual_pos(end) - actual_pos(1)) / (desired_pos(end) - desired_pos(1));
Tau = T_settle / 4;

% Compute PI Controller Gains using Ziegler-Nichols method
Ku = (1.2 / Kp_sys);
Tu = 2 * Tau;

Kp = 0.45 * Ku;
Ki = 1.2 * Ku / Tu;

% Display results
fprintf('Estimated System Parameters:\n');
fprintf(' Process Gain (Kp_sys): %.4f\n', Kp_sys);
fprintf(' Time Constant (Tau): %.4f sec\n', Tau);
fprintf(' Overshoot: %.2f%%\n', Overshoot);
fprintf(' Steady-State Error: %.4f\n', SteadyStateError);

fprintf('\nTuned PI Controller Gains:\n');
fprintf(' Kp: %.4f\n', Kp);
fprintf(' Ki: %.4f\n', Ki);

% Control Simulation: Apply the PI controller to the system (example)
% Assuming a simple system, you would create a controller like this:
% Note: Adjust this example for your specific system dynamics

% Initialize the controller and simulation parameters
dt = time(2) - time(1);  % Sample time (assumed constant)
num_samples = length(time);
control_signal = zeros(1, num_samples);
integral_error = 0;

% Initialize the controller gains (Kp and Ki) from the Ziegler-Nichols method
for i = 2:num_samples
    error = desired_pos(i) - actual_pos(i-1);  % Position error
    integral_error = integral_error + error * dt;  % Integral term
    control_signal(i) = Kp * error + Ki * integral_error;  % PI control signal
    
    % Apply control signal to system (for simulation, adjust actual position)
    actual_pos(i) = actual_pos(i-1) + control_signal(i) * dt;  % Update position (simplified model)
end

% Plot the response with the PI controller applied
subplot(2,1,2);
plot(time, desired_pos, 'r--', 'LineWidth', 2); hold on;
plot(time, actual_pos, 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Position');
legend('Desired Position', 'Actual Position with PI Control');
title('System Response with PI Control');

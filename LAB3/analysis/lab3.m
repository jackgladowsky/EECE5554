% Load data from CSV file
data = readtable('output_imu_data.csv');

% Extract accelerometer and gyroscope data
accel_x = data.Accel_X;
accel_y = data.Accel_Y;
accel_z = data.Accel_Z;
gyro_x = data.Gyro_X;
gyro_y = data.Gyro_Y;
gyro_z = data.Gyro_Z;

% Set sampling frequency (adjust this to match your data)
Fs = 40; % Hz
t0 = 1/Fs;

% Define tau values
maxNumM = 100;
L = length(accel_x);
maxM = 2.^floor(log2(L/2));
m = logspace(log10(1), log10(maxM), maxNumM).';
m = ceil(m); % m must be an integer.
m = unique(m); % Remove duplicates.
tau = m*t0;

% Calculate Allan variance for each sensor axis
[avar_accel_x, tau] = allanvar(accel_x, m, Fs);
[avar_accel_y, ~] = allanvar(accel_y, m, Fs);
[avar_accel_z, ~] = allanvar(accel_z, m, Fs);
[avar_gyro_x, ~] = allanvar(gyro_x, m, Fs);
[avar_gyro_y, ~] = allanvar(gyro_y, m, Fs);
[avar_gyro_z, ~] = allanvar(gyro_z, m, Fs);

% Calculate Allan deviation
adev_accel_x = sqrt(avar_accel_x);
adev_accel_y = sqrt(avar_accel_y);
adev_accel_z = sqrt(avar_accel_z);
adev_gyro_x = sqrt(avar_gyro_x);
adev_gyro_y = sqrt(avar_gyro_y);
adev_gyro_z = sqrt(avar_gyro_z);

% Function to calculate noise parameters
function [N, K, B, tauB, lineN, lineK, lineB] = calculate_noise_params(tau, adev)
    % Angle Random Walk
    slope = -0.5;
    logtau = log10(tau);
    logadev = log10(adev);
    dlogadev = diff(logadev) ./ diff(logtau);
    [~, i] = min(abs(dlogadev - slope));
    b = logadev(i) - slope*logtau(i);
    logN = slope*log10(1) + b;
    N = 10^logN;
    lineN = N ./ sqrt(tau);
    
    % Rate Random Walk
    slope = 0.5;
    [~, i] = min(abs(dlogadev - slope));
    b = logadev(i) - slope*logtau(i);
    logK = slope*log10(3) + b;
    K = 10^logK;
    lineK = K .* sqrt(tau/3);
    
    % Bias Instability
    slope = 0;
    [~, i] = min(abs(dlogadev - slope));
    b = logadev(i) - slope*logtau(i);
    scfB = sqrt(2*log(2)/pi);
    logB = b - log10(scfB);
    B = 10^logB;
    tauB = tau(i);
    lineB = B * scfB * ones(size(tau));
end

% Calculate noise parameters for each axis
[N_accel_x, K_accel_x, B_accel_x, tauB_accel_x, lineN_accel_x, lineK_accel_x, lineB_accel_x] = calculate_noise_params(tau, adev_accel_x);
[N_accel_y, K_accel_y, B_accel_y, tauB_accel_y, lineN_accel_y, lineK_accel_y, lineB_accel_y] = calculate_noise_params(tau, adev_accel_y);
[N_accel_z, K_accel_z, B_accel_z, tauB_accel_z, lineN_accel_z, lineK_accel_z, lineB_accel_z] = calculate_noise_params(tau, adev_accel_z);
[N_gyro_x, K_gyro_x, B_gyro_x, tauB_gyro_x, lineN_gyro_x, lineK_gyro_x, lineB_gyro_x] = calculate_noise_params(tau, adev_gyro_x);
[N_gyro_y, K_gyro_y, B_gyro_y, tauB_gyro_y, lineN_gyro_y, lineK_gyro_y, lineB_gyro_y] = calculate_noise_params(tau, adev_gyro_y);
[N_gyro_z, K_gyro_z, B_gyro_z, tauB_gyro_z, lineN_gyro_z, lineK_gyro_z, lineB_gyro_z] = calculate_noise_params(tau, adev_gyro_z);

% Function to plot Allan deviation with noise parameters
function plot_allan_deviation(tau, adev, lineN, lineK, lineB, tauB, N, K, B, title_str, y_label)
    figure;
    loglog(tau, adev, 'b', ...
           tau, lineN, 'r--', ...
           tau, lineK, 'g--', ...
           tau, lineB, 'm--', ...
           1, N, 'ro', ...
           3, K, 'go', ...
           tauB, 0.664*B, 'mo');
    title(title_str);
    xlabel('\tau (s)');
    ylabel(y_label);
    legend('\sigma', '\sigma_N', '\sigma_K', '\sigma_B', 'Location', 'Best');
    text(1, N, 'N', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left');
    text(3, K, 'K', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left');
    text(tauB, 0.664*B, '0.664B', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left');
    grid on;
    axis equal;
end

% Plot Allan deviation with noise parameters for each axis
plot_allan_deviation(tau, adev_accel_x, lineN_accel_x, lineK_accel_x, lineB_accel_x, tauB_accel_x, N_accel_x, K_accel_x, B_accel_x, 'Accelerometer X-axis Allan Deviation', '\sigma(\tau) (m/s^2)');
plot_allan_deviation(tau, adev_accel_y, lineN_accel_y, lineK_accel_y, lineB_accel_y, tauB_accel_y, N_accel_y, K_accel_y, B_accel_y, 'Accelerometer Y-axis Allan Deviation', '\sigma(\tau) (m/s^2)');
plot_allan_deviation(tau, adev_accel_z, lineN_accel_z, lineK_accel_z, lineB_accel_z, tauB_accel_z, N_accel_z, K_accel_z, B_accel_z, 'Accelerometer Z-axis Allan Deviation', '\sigma(\tau) (m/s^2)');
plot_allan_deviation(tau, adev_gyro_x, lineN_gyro_x, lineK_gyro_x, lineB_gyro_x, tauB_gyro_x, N_gyro_x, K_gyro_x, B_gyro_x, 'Gyroscope X-axis Allan Deviation', '\sigma(\tau) (rad/s)');
plot_allan_deviation(tau, adev_gyro_y, lineN_gyro_y, lineK_gyro_y, lineB_gyro_y, tauB_gyro_y, N_gyro_y, K_gyro_y, B_gyro_y, 'Gyroscope Y-axis Allan Deviation', '\sigma(\tau) (rad/s)');
plot_allan_deviation(tau, adev_gyro_z, lineN_gyro_z, lineK_gyro_z, lineB_gyro_z, tauB_gyro_z, N_gyro_z, K_gyro_z, B_gyro_z, 'Gyroscope Z-axis Allan Deviation', '\sigma(\tau) (rad/s)');

% Display results
disp('Accelerometer Noise Parameters:');
disp(['X-axis: ARW = ', num2str(N_accel_x), ' m/s/√Hz, RRW = ', num2str(K_accel_x), ' m/s^2/√Hz, BI = ', num2str(B_accel_x), ' m/s^2 at tau = ', num2str(tauB_accel_x), ' s']);
disp(['Y-axis: ARW = ', num2str(N_accel_y), ' m/s/√Hz, RRW = ', num2str(K_accel_y), ' m/s^2/√Hz, BI = ', num2str(B_accel_y), ' m/s^2 at tau = ', num2str(tauB_accel_y), ' s']);
disp(['Z-axis: ARW = ', num2str(N_accel_z), ' m/s/√Hz, RRW = ', num2str(K_accel_z), ' m/s^2/√Hz, BI = ', num2str(B_accel_z), ' m/s^2 at tau = ', num2str(tauB_accel_z), ' s']);

disp('Gyroscope Noise Parameters:');
disp(['X-axis: ARW = ', num2str(N_gyro_x), ' rad/√Hz, RRW = ', num2str(K_gyro_x), ' rad/s/√Hz, BI = ', num2str(B_gyro_x), ' rad/s at tau = ', num2str(tauB_gyro_x), ' s']);
disp(['Y-axis: ARW = ', num2str(N_gyro_y), ' rad/√Hz, RRW = ', num2str(K_gyro_y), ' rad/s/√Hz, BI = ', num2str(B_gyro_y), ' rad/s at tau = ', num2str(tauB_gyro_y), ' s']);
disp(['Z-axis: ARW = ', num2str(N_gyro_z), ' rad/√Hz, RRW = ', num2str(K_gyro_z), ' rad/s/√Hz, BI = ', num2str(B_gyro_z), ' rad/s at tau = ', num2str(tauB_gyro_z), ' s']);
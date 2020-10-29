clear all; close all; clc;

% read configuration
cfgname = 'data_cfg.m';
run(cfgname);

% read raw data
[t, f, w, T, data] = load_data(cfg.input_file, cfg.freq, cfg.delim, cfg.headerlines);

% verify if there is missed samples, differentiating time
diff_t = diff(t);
figure(1)
plot(t, [0;diff_t]);
title('time differences');
xlabel('time [sec]');
ylabel('dt/dt');

% plot accelerometer data
figure(2)
plot(t,f);
title('accelerometers');
legend('f_x', 'f_y', 'f_z');
xlabel('time [s]');
ylabel('specific force [m/sec^2]');

% plot gyroscope data
figure(3)
plot(t,w);
title('gyroscopes');
legend('\omega_x', '\omega_y', '\omega_z');
xlabel('time [s]');
ylabel('angular rates [\circ/sec]');

% plot temperature data
figure(4)
plot(t,T);
title('temperature');
xlabel('time [sec]');
ylabel('temperature [\circC]');

% save processed data
if cfg.save_data
    save_data(f, w, cfg.output_file);
end
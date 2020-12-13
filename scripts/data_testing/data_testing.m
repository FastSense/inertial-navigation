% script for testing data from IMU

close all;

global cfg
% input file settings
cfg.input_file = '..\..\data\ADIS16505-1\2020_11_27_MSU_rot_table_temp\raw\raw_burst_16bit_2000Hz_y_rot_table.csv';
cfg.freq        = 2000;
cfg.delim       = ';';
cfg.headerlines = 1;

% output file settings (save/don't save converted data)
cfg.output_file = '..\..\data\ADIS16505-1\2020_11_27_MSU_rot_table_temp\conv\conv_burst_16bit_2000Hz_y_rot_table.csv';
cfg.save_data   = false;
cfg.make_offset = false;
cfg.offset = 5;   % [s]  cut first seconds of sensors output

% test statistics counting settings
cfg.t_start=1;      % [s]
cfg.t_stop=100;     % [s]
cfg.window_len_t=1; % [s]
cfg.av_t_min=1;     % [s] minimal Allan variance window length
cfg.av_t_max=1000;  % [s] maximal Allan variance window length
cfg.av_t_incr=1;    % [s] time increment for Allan variance calculation

% test statistics control
cfg.simple_stat = false;
cfg.noise_hist = false;
cfg.accumulated_moments = false;
cfg.psd_estim = false;
cfg.autocorr_estim = false;
cfg.Allan_variance = false;

% visualization settings
cfg.plot_offset=2; % [s]

%% load data 
% note: different functions should be used depending on file structure

% read raw data
[t, w, f, T] = load_sensors_data();

% calculate arrays length
N=length(t);

%% plot sensors data
% accs data
figure(1)
set(gcf,'Name','ACC');
plot(t,f);
title('accelerometers')
legend('f_x','f_y','f_z')
xlabel('time [s]');
ylabel('specific force [m/sec^2]');

% gyros data
figure(2)
set(gcf,'Name','GYRO');
plot(t,w);
title('gyroscopes')
legend('\omega_x','\omega_y','\omega_z')
xlabel('time [s]');
ylabel('angular rates [deg/sec]');

% temperature
figure(3)
set(gcf,'Name','TEMP');
plot(t,T);
title('Temperature')
xlabel('time [sec]');
ylabel('temperature [\circ C]');

%% save sensors data in txt files
if cfg.save_data
	save_sensors_data(w,f);
end

%% calculate simple statistics
if cfg.simple_stat
    % calculate mean over exact interval [t_start,t_stop]
    n_start=cfg.t_start*cfg.freq;
    n_stop=cfg.t_stop*cfg.freq;

    mean_f=mean(f(n_start:n_stop,:));      % [m/s^2]
    mean_w=mean(w(n_start:n_stop,:))*3600; % [deg/h]
    mean_w_norm=norm(mean_w);

    ARW=cumsum(detrend(w(n_start:n_stop,:), 'constant'))/(2*cfg.freq);

    figure(101)
    set(gcf,'Name','ARW');
    plot(t(n_start:n_stop),ARW);
    title('gyros integration')
    legend('\int \omega_x','\int \omega_y','\int \omega_z')
    xlabel('time [s]');
    ylabel('angle[deg]');
end

% calculate offset for counting test statistics
n_offset=cfg.offset*cfg.freq;
% calculate offset for plotting results (to exclude "transition process")
n_plot=cfg.plot_offset*cfg.freq;

%% calculate noise distribution histograms
if cfg.noise_hist
    figure(4)
    set(gcf,'Name','ACC fx hist');
    histogram(detrend(f(n_offset:end,1),'constant'));
    title('f_x noise')
    xlabel('[m/s^2]')

    figure(5)
    set(gcf,'Name','ACC fy hist');
    histogram(detrend(f(n_offset:end,2),'constant'));
    title('f_y noise')
    xlabel('[m/s^2]')

    figure(6)
    set(gcf,'Name','ACC fz hist');
    histogram(detrend(f(n_offset:end,1),'constant'));
    title('f_z noise')
    xlabel('[m/s^2]')

    figure(7)
    set(gcf,'Name','GYRO wx hist');
    histogram(detrend(w(:,1).*3600,'constant'));
    title('\omega_x noise')
    xlabel('[deg/h]')

    figure(8)
    set(gcf,'Name','GYRO wy hist');
    histogram(detrend(w(:,2).*3600,'constant'));
    title('\omega_y noise')
    xlabel('[deg/h]')

    figure(9)
    set(gcf,'Name','GYRO wz hist');
    histogram(detrend(w(:,3).*3600,'constant'));
    title('\omega_z noise')
    xlabel('[deg/h]')
end

%% accumulate mean and standard deviation
if cfg.accumulated_moments
    w=w*3600;  % [deg/s] to [deg/h]

    ind_1_N=[1:N-n_offset+1;1:N-n_offset+1;1:N-n_offset+1]';
    ind_2_N=[2,2:N-n_offset+1;2,2:N-n_offset+1;2,2:N-n_offset+1]'-1;

    mean_f_acc=cumsum(f(n_offset:N,:))./ind_1_N;
    mean_w_acc=cumsum(w(n_offset:N,:))./ind_1_N;

    std_f_acc=(cumsum(f(n_offset:N,:).^2)-ind_1_N.*(mean_f_acc.^2))./ind_2_N;
    std_w_acc=(cumsum(w(n_offset:N,:).^2)-ind_1_N.*(mean_w_acc.^2))./ind_2_N;

    figure(10)
    clf
    hold on
    set(gcf,'Name','ACC mean');
    subplot(3,1,1)
    plot(t(n_offset+n_plot:end),mean_f_acc(n_plot+1:end,1));
    title('accelerometers accumulated mean')
    legend('<f_x>')
    xlabel('time [s]');
    subplot(3,1,2)
    plot(t(n_offset+n_plot:end),mean_f_acc(n_plot+1:end,2));
    legend('<f_y>')
    xlabel('time [s]');
    ylabel('specific force [m/sec^2]');
    subplot(3,1,3)
    plot(t(n_offset+n_plot:end),mean_f_acc(n_plot+1:end,3));
    legend('<f_z>')
    xlabel('time [s]');

    figure(11)
    clf
    hold on
    set(gcf,'Name','GYRO mean');
    subplot(3,1,1)
    plot(t(n_offset+n_plot:end),mean_w_acc(n_plot+1:end,1));
    title('gyroscopes accumulated mean')
    legend('<\omega_x>')
    xlabel('time [s]');
    subplot(3,1,2)
    plot(t(n_offset+n_plot:end),mean_w_acc(n_plot+1:end,2));
    legend('<\omega_y>')
    xlabel('time [s]');
    ylabel('angular rates [deg/h]');
    subplot(3,1,3)
    plot(t(n_offset+n_plot:end),mean_w_acc(n_plot+1:end,3));
    legend('<\omega_z>')
    xlabel('time [s]');

    figure(12)
    clf
    hold on
    set(gcf,'Name','ACC std');
    subplot(3,1,1)
    plot(t(n_offset+n_plot:end),sqrt(std_f_acc(n_plot+1:end,1)));
    title('accelerometers accumulated std')
    legend('\sigma_{f_x}')
    xlabel('time [s]');
    subplot(3,1,2)
    plot(t(n_offset+n_plot:end),sqrt(std_f_acc(n_plot+1:end,2)));
    legend('\sigma_{f_y}')
    xlabel('time [s]');
    ylabel('specific force [m/sec^2]');
    subplot(3,1,3)
    plot(t(n_offset+n_plot:end),sqrt(std_f_acc(n_plot+1:end,3)));
    legend('\sigma_{f_z}')
    xlabel('time [s]');

    figure(13)
    clf
    hold on
    set(gcf,'Name','GYRO std');
    subplot(3,1,1)
    plot(t(n_offset+n_plot:end),sqrt(std_w_acc(n_plot+1:end,1)));
    title('gyroscopes accumulated std')
    legend('\sigma_{\omega_x}')
    xlabel('time [s]');
    subplot(3,1,2)
    plot(t(n_offset+n_plot:end),sqrt(std_w_acc(n_plot+1:end,2)));
    legend('\sigma_{\omega_y}')
    xlabel('time [s]');
    ylabel('angular rates [deg/h]');
    subplot(3,1,3)
    plot(t(n_offset+n_plot:end),sqrt(std_w_acc(n_plot+1:end,3)));
    legend('\sigma{\omega_z}')
    xlabel('time [s]');
end

% calculate mean calculated in a sliding window
n_window=cfg.window_len_t*cfg.freq;

mean_f_window=movsum(f(n_offset:end,:),n_window);
mean_w_window=movsum(w(n_offset:end,:),n_window);

%% calculate power spectral density
if cfg.psd_estim
    [freq_fx, PSD2_fx, RMS_fx, RND_fx] = process_noise(f(:,1),cfg.freq);
    [freqs_fx, PSDs_fx] = smooth_psd(freq_fx, PSD2_fx, 128);
    [freq_fy, PSD2_fy, RMS_fy, RND_fy] = process_noise(f(:,2),cfg.freq);
    [freqs_fy, PSDs_fy] = smooth_psd(freq_fy, PSD2_fy, 128);
    [freq_fz, PSD2_fz, RMS_fz, RND_fz] = process_noise(f(:,3),cfg.freq);
    [freqs_fz, PSDs_fz] = smooth_psd(freq_fz, PSD2_fz, 128);

    [freq_wx, PSD2_wx, RMS_wx, RND_wx] = process_noise(w(:,1),cfg.freq);
    [freqs_wx, PSDs_wx] = smooth_psd(freq_wx, PSD2_wx, 128);
    [freq_wy, PSD2_wy, RMS_wy, RND_wy] = process_noise(w(:,2),cfg.freq);
    [freqs_wy, PSDs_wy] = smooth_psd(freq_wy, PSD2_wy, 128);
    [freq_wz, PSD2_wz, RMS_wz, RND_wz] = process_noise(w(:,3),cfg.freq);
    [freqs_wz, PSDs_wz] = smooth_psd(freq_wz, PSD2_wz, 128);

    figure(14)
    clf
    hold on
    set(gcf,'Name','ACC psd');
    loglog(freqs_fx, PSDs_fx)
    loglog(freqs_fy, PSDs_fy)
    loglog(freqs_fz, PSDs_fz)
    set(gca,'yscale','log')
    grid on;
    xlabel('Frequency [Hz]');
    ylabel('Noise Density [m/s^2 per sqrt(Hz)]');
    title('Accelerometers PSD');
    legend('psd f_x','psd f_y','psd f_z');

    figure(15)
    clf
    hold on
    set(gcf,'Name','GYRO psd');
    loglog(freqs_wx, PSDs_wx);
    loglog(freqs_wy, PSDs_wy);
    loglog(freqs_wz, PSDs_wz);
    set(gca,'yscale','log')
    grid on;
    xlabel('Frequency, [Hz]');
    ylabel('Noise Density [deg/h per sqrt(Hz)]');
    title('Gyroscopes PSD');
    legend('psd \omega_x','psd \omega_y','psd \omega_z');
end

%% calculate autocorrelation function
if cfg.autocorr_estim
    [autocorr_f,lag_f]=xcorr(detrend(f,'constant'),'coeff');
    [autocorr_w,lag_w]=xcorr(detrend(w,'constant'),'coeff');

    figure(16)
    clf
    hold on
    set(gcf,'Name','ACC autocorr');
    plot(lag_f,autocorr_f(:,[1,5,9]));
    xlabel('lag, [s]');
    ylabel('correlation [(m/s^2)^2]');
    title('Accelerometers autocorrelation');
    legend('R_{f_x f_x}','R_{f_y f_y}','R_{f_z f_z}');

    figure(17)
    clf
    hold on
    set(gcf,'Name','GYRO autocorr');
    plot(lag_w,autocorr_w(:,[1,5,9]));
    xlabel('lag, [s]');
    ylabel('[(d/h)^2]');
    title('Gyroscopes autocorrelation');
    legend('R_{\omega_x \omega_x}','R_{\omega_y \omega_y}','R_{\omega_z \omega_z}');
end

%% calculate Allan deviation
if cfg.Allan_variance
    n_min=cfg.freq*cfg.av_t_min;
    n_max=cfg.freq*cfg.av_t_max;
    dn=cfg.freq*cfg.av_t_incr;

    GyrosAllanVar  =  zeros( floor((n_max - n_min)/dn + 1), 3 );
    wx  =  w(1:N, 1)*3600;    % [deg/h]
    wy  =  w(1:N, 2)*3600;    % [deg/h]
    wz  =  w(1:N, 3)*3600;    % [deg/h]
    for n  =  n_min : dn : n_max
        GyrosAllanVar((n - n_min)/dn + 1, 1)  =  Allan_variance(wx,n);    
        GyrosAllanVar((n - n_min)/dn + 1, 2)  =  Allan_variance(wy,n);    
        GyrosAllanVar((n - n_min)/dn + 1, 3)  =  Allan_variance(wz,n);    
    end
    n_w=size(GyrosAllanVar,1);

    AccelsAllanVar  =  zeros( floor((n_max - n_min)/dn + 1), 3 );
    fx  =  ( f(1:N, 1) )*1e6;  % [micro m/s^2] 
    fy  =  ( f(1:N, 2) )*1e6;  % [micro m/s^2]
    fz  =  ( f(1:N, 3) )*1e6;  % [micro m/s^2]
    for n  =  n_min : dn : n_max
        AccelsAllanVar((n - n_min)/dn + 1, 1)  =  Allan_variance(fx,n);    
        AccelsAllanVar((n - n_min)/dn + 1, 2)  =  Allan_variance(fy,n);    
        AccelsAllanVar((n - n_min)/dn + 1, 3)  =  Allan_variance(fz,n);    
    end
    n_f=size(AccelsAllanVar,1);

    % in-run bias stability
    acc_bias_stability=[min(AccelsAllanVar(:,1)),min(AccelsAllanVar(:,2)),min(AccelsAllanVar(:,3))]*1e-6; % [m/s^2]
    gyro_bias_stability=[min(GyrosAllanVar(:,1)),min(GyrosAllanVar(:,2)),min(GyrosAllanVar(:,3))];   % [deg/h]

    % ARW and VRW ( here AV(t=1) = AV(1) because t_min=1s )
    VRW=[AccelsAllanVar(1,1),AccelsAllanVar(1,2),AccelsAllanVar(1,3)]*1e-6*60;
    ARW=[GyrosAllanVar(1,1),GyrosAllanVar(1,2),GyrosAllanVar(1,3)];

    figure(18)
    clf
    hold on
    set(gcf,'Name','GYRO Allan');
    loglog([n_min:dn:n_max]./cfg.freq,GyrosAllanVar);
    loglog([n_min:dn:n_max]./cfg.freq,gyro_bias_stability(ones(n_w,1),:));
    grid on;
    set(gca,'xscale','log')
    set(gca,'yscale','log')
    title('Allan deviation for gyros');
    xlabel('window length, s');
    ylabel('Allan devition, [deg/h]');
    legend('\omega_x','\omega_y','\omega_z');

    figure(19)
    clf
    hold on
    set(gcf,'Name','ACC Allan');
    loglog([n_min:dn:n_max]./cfg.freq,AccelsAllanVar);
    loglog([n_min:dn:n_max]./cfg.freq,acc_bias_stability(ones(n_f,1),:)*1e6);
    grid on;
    set(gca,'xscale','log')
    set(gca,'yscale','log')
    title('Allan deviation for accelerometers');
    xlabel('window length, [s]');
    ylabel('Allan devition, [\mu m/s^2]');
    legend('f_x','f_y','f_z');
end
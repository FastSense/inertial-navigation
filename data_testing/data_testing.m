function data_testing
	clear all; close all;

	fsz = 15; % plot font size

	% read configuration
	cfgname = 'data_cfg.m';
	run(cfgname);

	% read raw data
	[t, w, f, T] = load_data(cfg.input_file, cfg.freq,...
		cfg.delim, cfg.headerlines);

	% verify if there is missed samples, differentiating time
	diff_t = diff(t);
	create_fig(fsz);
	plot(t, [diff_t(1); diff_t]);
	title('time differences'); xlabel('time [sec]'); ylabel('[sec]');

	% plot sensors
	create_fig(fsz);
	plot(t,f);
	title('accelerometers'); xlabel('time [s]'); ylabel('[m/sec^2]');
	legend({'f_x', 'f_y', 'f_z'});

	create_fig(fsz);
	plot(t,w);
	title('gyroscopes'); xlabel('time [s]'); ylabel('[deg/sec]');
	legend({'\omega_x', '\omega_y', '\omega_z'});

	create_fig(fsz);
	plot(t,T);
	title('temperature'); xlabel('time [sec]');	ylabel('[deg]');

	% save processed data
	if cfg.save_data
		save_data(w,f,cfg.output_file);
	end
end

% read raw ADIS16505-1 16-bit data
% input:
%     input_file  --- input file path
%     freq        --- input data frequency
%     delim       --- delimiter in the input file
%     headerlines --- number of header lines
% output:
%     t    --- time [sec]
%     w    --- angular velocity [deg/sec]
%     f    --- specific force [m/sec^2]
%     T    --- temperature [celsius]
function [t, w, f, T] = load_data(input_file, freq, delim, headerlines)
	% scale factors
	w_scale = 0.00625;
	f_scale = 0.002447;
	T_scale = 0.1;

	% read file
	file  = importdata(input_file, delim, headerlines);

	w_raw = file.data(:,2:4); % angular rate
	f_raw = file.data(:,5:7); % specific force
	T     = file.data(:,8);   % temperature
	ts    = file.data(:,9);   % timestamp
	
	N = size(ts,1);
	t = [1:N]'/freq;
	
	% convert uint16 to int16
	w = zeros(N,3);
	f = zeros(N,3);
	for i = 1:3
		w(:,i) = typecast(uint16(w_raw(:,i)), 'int16');
		f(:,i) = typecast(uint16(f_raw(:,i)), 'int16');
	end

	% get scaled data
	f = f_scale*f;
	w = w_scale*w;
	T = T_scale*T;
end

% create basic figure
% input:
%     fsz --- plot font size
% output:
%     fig --- figure
function [fig] = create_fig(fsz)
	fig = figure('defaultaxesfontsize',fsz);
	set(gcf,'PaperPositionMode','auto', 'Units', 'Normalized',...
	'OuterPosition', [0, 0.06, 1, 0.92]);
	hold on; grid on;
end

% save processed data
% input:
%     w           --- angular velocity [deg/sec]
%     f           --- specific force [m/sec^2]
%     output_file --- output file path
function save_data(w,f, output_file)
	fid = fopen(output_file, 'w');
	if ~fid, error('Can not open output file!\n'); end
	header = '   w1[deg/s]  w2[deg/s]  w3[deg/s]  f1[m/s^2]  f2[m/s^2]  f3[m/s^2]';
	fprintf(fid, '%s\n', header);
	fprintf(fid, '%10.5f %10.5f %10.5f %10.5f %10.5f %10.5f\n', [w'; f']);
	fclose(fid);
end
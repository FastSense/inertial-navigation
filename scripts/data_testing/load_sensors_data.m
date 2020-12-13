% read raw ADIS16505-1 16-bit data
% input:
%     input_file      --- input file path
%     cfg.freq        --- input data frequency (global variable)
%     cfg.delim       --- delimiter in the input file (global variable)
%     cfg.headerlines --- number of header lines (global variable)
% output:
%     t    --- time [sec]
%     w    --- angular velocity [deg/sec]
%     f    --- specific force [m/sec^2]
%     T    --- temperature [celsius]
function [t, w, f, T] = load_sensors_data()

global cfg

	% scale factors
	w_scale = 0.00625;
	f_scale = 0.002447;
	T_scale = 0.1;

	% read file
	file  = importdata(cfg.input_file, cfg.delim, cfg.headerlines);

	w_raw = file.data(:,2:4); % angular rate
	f_raw = file.data(:,5:7); % specific force
	T     = file.data(:,8);   % temperature
	ts    = file.data(:,9);   % timestamp
	
	N = size(ts,1);
	t = [1:N]'/cfg.freq;
	
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

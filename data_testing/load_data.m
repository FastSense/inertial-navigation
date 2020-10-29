% read raw ADIS16505-1 16-bit data
% input:
%     input_file  --- input file path
%     freq        --- inpur data frequency
%     delim       --- delimiter in the input file
%     headerlines --- number of header lines
% output:
%     t    --- time[sec]
%     f    --- specific force [m/sec^2]
%     w    --- angular velocity [deg/sec]
%     T    --- temperature [celsius]
%     data --- array with all file data
function [t, f, w, T, data] = load_data(input_file, freq, delim, headerlines)

% define scales
w_scale = 0.00625;
f_scale = 0.002447;
T_scale = 0.1;

% read file
file  = importdata(input_file, delim, headerlines);
data  = file.data;
ts    = data(:,9);   % timestamp
w_raw = data(:,2:4); % angular rate
f_raw = data(:,5:7); % specific force
T     = data(:,8);   % temperature

% convert uint16 to int16
N = length(ts);
w = zeros(N,3);
f = zeros(N,3);
for i = 1:3
    w(:,i) = typecast(uint16(w_raw(:,i)), 'int16');
    f(:,i) = typecast(uint16(f_raw(:,i)), 'int16');
end

% get scaled data
f = f * f_scale;
w = w * w_scale;
T = T * T_scale;

% get time from timestamps
dt = 1 / freq;  % [sec]
t  = ts / freq; % [sec]

% get rid of restarting moments
for i = 1:N
    t(i) = i*dt;
end
end
% save processed data
% input:
%     w           --- angular velocity [deg/sec]
%     f           --- specific force [m/sec^2]
%     output_file --- output file path
function save_sensors_data(w,f)
    global cfg
	fid = fopen(cfg.output_file, 'w');
	if ~fid, error('Can not open output file!\n'); end
   
    % Axes: 1-forward, 2-left, 3-up
    header = '   w1[deg/s]  w2[deg/s]  w3[deg/s]  f1[m/s^2]  f2[m/s^2]  f3[m/s^2]';
    fprintf(fid, '%s\n', header);
    fprintf(fid, '%10.5f %10.5f %10.5f %10.5f %10.5f %10.5f\n', [w'; f']);
        
    fclose(fid);
end
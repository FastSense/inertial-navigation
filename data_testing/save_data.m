% save processed data
% input:
%     f           --- specific force [m/sec^2]
%     w           --- angular velocity [deg/sec]
%     input_file  --- input file path
function save_data(f, w, output_file)

header = '   w1[deg/s]  w2[deg/s]  w3[deg/s]  f1[m/s^2]  f2[m/s^2]  f3[m/s^2]';

% write header into the file
fid = fopen(output_file, 'w');
if fid >= 0
    fprintf(fid, '%s\n', header);
    fclose(fid);
else
    disp("can't open output file\n")
end

% write array into the file
dlmwrite(output_file, [w,f], '-append', 'delimiter', ' ', 'precision', '%10.5f')
end
%% configuration script for data testing

% input settings
cfg.input_file  = 'raw_burst_16bit_2000Hz_x_rot_table.csv';
cfg.freq        = 2000;
cfg.delim       = ';';
cfg.headerlines = 1;

% output settings
cfg.output_file = 'raw_burst_16bit_2000Hz_x_rot_table.txt';
cfg.save_data   = true;
%% configuration script for data testing

% input settings
cfg.input_file  = 'Datalog_16bit_RAW_500Hz_no_mov.csv';
cfg.freq        = 500;
cfg.delim       = ',';
cfg.headerlines = 1;

% output settings
cfg.output_file = 'Datalog_16bit_500Hz_no_mov.txt';
cfg.save_data   = true;
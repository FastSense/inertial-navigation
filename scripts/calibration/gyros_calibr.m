%% MAIN SCRIPT
close all;

% data parameters
freq = 2000;    % [Hz]
Az = 45;        % [deg]
phi = 55.69925; % [deg]

% thresholds
st_th = 0.5;  % static threshold,   [sec]
rot_th = 0.5; % rotation threshold, [sec]

% plot settings
fsz = 20;
lw = 2;

% decimal precision of k estimate
dec_prec = 7;

% labels
axs = 'XYZ';
drifts = {'nu01', 'nu02', 'nu03'}; 
scales = {'Th11', 'Th22', 'Th33'};

% flags
WRITE_COEF = false; % write tempurature model coefficients to a file

%% load converted files
if ~exist('xrt','var') || ~exist('yrt','var') || ~exist('zrt','var')	    
    % x rotation files
    xrt{1} = load('../../data/ADIS16505-1/2020_11_14_MSU_rot_table_temp/conv/conv_burst_16bit_2000Hz_x_rot_table.txt');
    xrt{2} = load('../../data/ADIS16505-1/2020_11_24_MSU_rot_table_temp/conv/conv_burst_16bit_2000Hz_x_rot_table.txt');
    xrt{3} = load('../../data/ADIS16505-1/2020_12_02_MSU_rot_table/conv/conv_burst_16bit_2000Hz_split_x_rot_table.txt');
    
    % y rotation files
    yrt{1} = load('../../data/ADIS16505-1/2020_11_14_MSU_rot_table_temp/conv/conv_burst_16bit_2000Hz_y_rot_table.txt');
    yrt{2} = load('../../data/ADIS16505-1/2020_11_24_MSU_rot_table_temp/conv/conv_burst_16bit_2000Hz_y_rot_table.txt');
    yrt{3} = load('../../data/ADIS16505-1/2020_11_27_MSU_rot_table_temp/conv/conv_burst_16bit_2000Hz_y_rot_table.txt');
    yrt{4} = load('../../data/ADIS16505-1/2020_12_02_MSU_rot_table/conv/conv_burst_16bit_2000Hz_split_y_rot_table.txt');
	
    % z rotation files
    zrt{1} = load('../../data/ADIS16505-1/2020_11_14_MSU_rot_table_temp/conv/conv_burst_16bit_2000Hz_z_rot_table.txt');
    zrt{2} = load('../../data/ADIS16505-1/2020_11_24_MSU_rot_table_temp/conv/conv_burst_16bit_2000Hz_z_rot_table.txt');
    zrt{3} = load('../../data/ADIS16505-1/2020_12_02_MSU_rot_table/conv/conv_burst_16bit_2000Hz_split_z_rot_table.txt');
end

flist = {
 	xrt,...
	yrt,...
	zrt,...
};

%% load chunks
run('gyros_chunks.m');
chlist = {
    chx,...
    chy,...
    chz,...
};
chlist_excl = {
    chx_excl,...
    chy_excl,...
    chz_excl,...
};

%% clear lists for storing data
clear T_st_list T_rot_list T0_list nu0_list Th_list

%% loop though axes and files
% axes loop
for axn = 1:3
    % files loop
    for fn = 1:numel(flist{axn})
        ftitle = ['------ file #', num2str(fn), ' (', axs(axn), ') ------'];
        disp(ftitle);
        
        T_st_arr  = []; % mean temperature, static chunks
        T_rot_arr = []; % mean temperature, rotation chunks
        T0_arr    = []; % mean temperature, static + rotation chunks
        nu0_arr   = []; % drift approximation, static chunks
        Th_arr    = []; % scale approximation, static + rotation chunks
        
        % chunks loop
        chn_excl = 1;
        excl_ind = [];
        for chn = 1:numel(chlist{axn}{fn}(:,1))
            chtitle = ['chunk ', num2str(chn)];
            disp(chtitle);
            
            %% calculate time indexes
            st_ind_ = uint64((chlist{axn}{fn}(chn,1)+st_th)*freq):uint64((chlist{axn}{fn}(chn,2)-st_th)*freq);
            
            % exclude indexes
            if chn_excl <= size(chlist_excl{axn}{fn}, 1)
                if chlist_excl{axn}{fn}(chn_excl,1) >= chlist{axn}{fn}(chn,1)
                    excl_ind = uint64(chlist_excl{axn}{fn}(chn_excl,1)*freq):uint64(chlist_excl{axn}{fn}(chn_excl,2)*freq);
                    chn_excl = chn_excl + 1;
                else
                    excl_ind = [];
                end
            end
            st_ind = st_ind_(~ismember(st_ind_, excl_ind));
            
            if chn ~= numel(chlist{axn}{fn}(:,1))
                rot_ind = uint64((chlist{axn}{fn}(chn,2)+rot_th)*freq):uint64((chlist{axn}{fn}(chn+1,1)-rot_th)*freq);
            end

            %% estimate drift    
            w = flist{axn}{fn}(st_ind,1:3);
            [nu0, w0] = estimate_gyro_drift(w, Az, phi);
            disp(['nu0 = [',...
                sprintf('%.9f', nu0(axn)*180/pi*3600), '] deg/h']); % [deg/h]

            %% estimate scale
            if chn ~= numel(chlist{axn}{fn}(:,1))
                w = flist{axn}{fn}(rot_ind,1:3); % w in rotation chunk, [deg/s]
                f = flist{axn}{fn}(rot_ind,4:6); % f in rotation chunk, [m/s^2]

                w_ = zeros(size(w));
                for k = 1:3, w_(:,k) = (w(:,k) - w0(k))*pi/180; end 

                w1 = w_(:,axn);
                perm = find(1:3 ~= axn); % two other axes indices
                f23 = f(:,perm);
                [Th, J] = estimate_gyro_scale(w1, f23, freq);

                disp(['J(k), Th = [' ...
                    sprintf(['%.3f %.' num2str(dec_prec+1) 'f'],[J,Th]) ']']);
            end
            
            %% calculate temperature mean
            T_st  = mean(flist{axn}{fn}(st_ind ,7));
            if chn ~= numel(chlist{axn}{fn}(:,1))
                T_rot = mean(flist{axn}{fn}(rot_ind,7));
            end

            %% store values
            T_st_arr  = [T_st_arr, T_st];
            nu0_arr   = [nu0_arr, nu0(axn)*180/pi*3600]; % [deg/h]
            if chn ~= numel(chlist{axn}{fn}(:,1))
                T_rot_arr = [T_rot_arr, T_rot];
                T0_arr    = [T0_arr, mean([T_st;T_rot])]; 
                Th_arr    = [Th_arr, Th];
            end
            
            disp('------')
        end
        
        T_st_list {axn}{fn} = T_st_arr; 
        T_rot_list{axn}{fn} = T_rot_arr; 
        T0_list   {axn}{fn} = T0_arr; 
        nu0_list  {axn}{fn} = nu0_arr; 
        Th_list   {axn}{fn} = Th_arr;
        
        disp(' ');
    end
end

%% calculate approximation
disp('------ temperature polynomials a2*T^2+a1*T+a0 -> [mean(a0) mean(a1) mean(a2)] ------');
if WRITE_COEF
    fileID = fopen('temp_coef.txt','w');
end
% axes loop
for axn = 1:3
    % files loop
    for fn = 1:numel(flist{axn})
        % approximate drift
        A = [ones(1,numel(T_st_list{axn}{fn})); T_st_list{axn}{fn}; T_st_list{axn}{fn}.^2]';
        nu0_appr{axn}{fn} = (pinv(A)*nu0_list{axn}{fn}')';
        
        % approximate scale
        A = [ones(1,numel(T0_list{axn}{fn})); T0_list{axn}{fn}; T0_list{axn}{fn}.^2]';
        Th_appr{axn}{fn} = (pinv(A)*Th_list{axn}{fn}')';
    end
    
    % display
    nu0_appr_mean = mean(cat(1, nu0_appr{axn}{:}), 1);
    Th_appr_mean  = mean(cat(1, Th_appr{axn} {:}), 1);
    T_min = min(cat(2, T_st_list{axn}{:}, T0_list{axn}{:}));
    T_max = max(cat(2, T_st_list{axn}{:}, T0_list{axn}{:}));
    
    disp([drifts{axn}, ' coeffs. = [', num2str(nu0_appr_mean(1:3),'%+12.6f'), ']']);
    disp([scales{axn}, ' coeffs. = [', num2str(Th_appr_mean (1:3),'%+16.10f'), ']']);
    disp(['temp limits  = [', num2str(T_min,'%.1f'), ', ', num2str(T_max,'%.1f'), ']']);
    disp(' ')
    
    % write to file
    if WRITE_COEF
        fprintf(fileID, 'nu0%1d_a1 = %+.6f\nnu0%1d_a2 = %+.6f\n', axn, nu0_appr_mean(2), axn, nu0_appr_mean(3));
        fprintf(fileID, 'nu0%1d_T0 = %.1f\nnu0%1d_T1 = %.1f\n', axn, T_min, axn, T_max);
    end
end

%% set plots settings
set(groot, 'defaulttextinterpreter', 'latex');  
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');  
set(groot, 'defaultLegendInterpreter', 'latex');

% set drifts 1 & 3 plot settings
f1 = figure('defaultaxesfontsize',fsz,'defaultlinelinewidth',lw);
set(f1,'PaperPositionMode','auto', 'Units', 'Normalized',...
    'OuterPosition', [0.04, 0.04, 0.96, 0.92]);
xlabel('$T$, deg'); ylabel('$\widetilde{\nu}^0$, deg/h');

% set drift 2 plot settings
f2 = figure('defaultaxesfontsize',fsz,'defaultlinelinewidth',lw);
set(f2,'PaperPositionMode','auto', 'Units', 'Normalized',...
    'OuterPosition', [0.04, 0.04, 0.96, 0.92]);
xlabel('$T$, deg'); ylabel('$\widetilde{\nu}^0$, deg/h');

% set scales plot settings
f3 = figure('defaultaxesfontsize',fsz,'defaultlinelinewidth',lw);
set(f3,'PaperPositionMode','auto', 'Units', 'Normalized',...
    'OuterPosition', [0.04, 0.04, 0.96, 0.92]);
xlabel('$T$, deg'); ylabel('$\widetilde{\Theta}$');

% set temperature plot settings
f4 = figure('defaultaxesfontsize',fsz,'defaultlinelinewidth',lw);
set(f4,'PaperPositionMode','auto', 'Units', 'Normalized',...
    'OuterPosition', [0.04, 0.04, 0.96, 0.92]);
xlabel('$t$, sec'); ylabel('$T$, deg');


%% plot
% drifts 1 & 3
figure(f1);
hold on; grid on;
for fn = 1:numel(flist{1})    
    p_nu01 = plot(T_st_list{1}{fn}, nu0_list{1}{fn}, ['-b','o']);
    p_nu01_app = fplot(@(x) nu0_appr{1}{fn}(1) + nu0_appr{1}{fn}(2)*x + nu0_appr{1}{fn}(3)*x^2,...
                       [min(T_st_list{1}{fn}) max(T_st_list{1}{fn})],...
                       '--b', 'LineWidth', 2);
end

for fn = 1:numel(flist{3})
    p_nu03 = plot(T_st_list{3}{fn}, nu0_list{3}{fn}, ['-r','o']);
    p_nu03_app = fplot(@(x) nu0_appr{3}{fn}(1) + nu0_appr{3}{fn}(2)*x + nu0_appr{3}{fn}(3)*x^2,...
                       [min(T_st_list{3}{fn}) max(T_st_list{3}{fn})],...
                       '--r', 'LineWidth', 2);
end

legend([p_nu01, p_nu01_app, p_nu03, p_nu03_app],...
       {'$\widetilde{\nu}^0_X$', '$\widetilde{\nu}^{0\,\texttt{app}}_X$',...
        '$\widetilde{\nu}^0_Z$', '$\widetilde{\nu}^{0\,\texttt{app}}_Z$'},...
        'location', 'southwest');

% drift 2
figure(f2);
hold on; grid on;

for fn = 1:numel(flist{2})
    p_nu02 = plot(T_st_list{2}{fn}, nu0_list{2}{fn}, ['-g','o']);
    p_nu02_app = fplot(@(x) nu0_appr{2}{fn}(1) + nu0_appr{2}{fn}(2)*x + nu0_appr{2}{fn}(3)*x^2,...
                       [min(T_st_list{2}{fn}) max(T_st_list{2}{fn})],...
                       '--g', 'LineWidth', 2);
end

legend([p_nu02, p_nu02_app],...
       {'$\widetilde{\nu}^0_Y$', '$\widetilde{\nu}^{0\,\texttt{app}}_Y$'},...
        'location', 'southwest');
      
% scales
figure(f3);
hold on; grid on;

for fn = 1:numel(flist{1})
    p_Th1 = plot(T0_list{1}{fn}, Th_list{1}{fn}, ['-b','o']);
    p_Th1_app = fplot(@(x) Th_appr{1}{fn}(1) + Th_appr{1}{fn}(2)*x + Th_appr{1}{fn}(3)*x^2,...
                      [min(T0_list{1}{fn}) max(T0_list{1}{fn})],...
                      '--b', 'LineWidth', 2);
end

for fn = 1:numel(flist{2})
    p_Th2 = plot(T0_list{2}{fn}, Th_list{2}{fn}, ['-g','o']);
    p_Th2_app = fplot(@(x) Th_appr{2}{fn}(1) + Th_appr{2}{fn}(2)*x + Th_appr{2}{fn}(3)*x^2,...
                         [min(T0_list{2}{fn}) max(T0_list{2}{fn})],...
                         '--g', 'LineWidth', 2);
end

for fn = 1:numel(flist{3})
    p_Th3 = plot(T0_list{3}{fn}, Th_list{3}{fn}, ['-r','o']);
    p_Th3_app = fplot(@(x) Th_appr{3}{fn}(1) + Th_appr{3}{fn}(2)*x + Th_appr{3}{fn}(3)*x^2,...
                      [min(T0_list{3}{fn}) max(T0_list{3}{fn})],...
                      '--r', 'LineWidth', 2);
end

legend([p_Th1, p_Th1_app, p_Th2, p_Th2_app, p_Th3, p_Th3_app],...
       {'$\widetilde{\Theta}_X$','$\widetilde{\Theta}_X^{\texttt{app}}$',...
        '$\widetilde{\Theta}_Y$','$\widetilde{\Theta}_Y^{\texttt{app}}$',...
        '$\widetilde{\Theta}_Z$','$\widetilde{\Theta}_Z^{\texttt{app}}$'},...
        'location','southwest');

% temperature
figure(f4);
hold on; grid on;

for axn = 1:3
    for fn = 1:numel(flist{axn})
        plot([0:(numel(flist{axn}{fn}(:,1))-1)]/freq, flist{axn}{fn}(:,7));
    end
end





%% FUNCTIONS
% estimate gyro drift
% input:
%     w   --- gyro readings for one axis
%     Az  --- azimuth
%     phi --- latitude
% output:
%     nu0 --- drift estimate
%     w0  --- mean of gyro readings
function [nu0, w0] = estimate_gyro_drift(w, Az, phi)
    u = 7.2921158553e-5; % [rad/s]

    w0 = mean(w);
    nu0 = w0'*pi/180 - u*cosd(phi)*cosd(Az);
end

% estimate gyro scale
% input:
%     w1   --- gyro readings for one axis
%     f23  --- accels readings for other two axes
%     freq --- data frequency
% output:
%     Th --- scale estimate
%     J  --- functional value at the calculated minimum point 
function [Th, J] = estimate_gyro_scale(w1, f23, freq)
    dec_prec   = 7;  % decimal precision of k estimate
    delta_freq = 48; % input data frequency correction
    
    J_arr = [];	k_arr = [];
	range = 1-.1:0.01:1+.1; % initial range of k
    
    for s = 0.1.^(1:dec_prec)
		x = zeros(6,1);
		f_tilde = zeros(size(f23));
		ind = 0;
		J = zeros(1,numel(range));
		for k = range
			ind = ind + 1;
 			a = k.*cumsum(w1)/(freq+delta_freq);
			H = [sin(a) cos(a), ones(numel(a),1)]; % | xrt | yrt | zrt |
			x(1:3,:) = pinv(H)*f23(:,2);		   % | f_Z | f_Z | f_Y |
			x(4:6,:) = pinv(H)*f23(:,1);		   % | f_Y | f_X | f_X |
			f_tilde(:,2) = H*x(1:3,:); 
			f_tilde(:,1) = H*x(4:6,:); 

			J(ind) = sum(sum((f23 - f_tilde).^2));
		end
		[J_, ind_] = min(J);
		J_arr = [J_arr, J_];
		k_arr = [k_arr, range(ind_)];
		
		range = [range(ind_)-s:s/10:range(ind_)+s];
    end

    Th = 1 - k_arr(end);
    J = J_;
end
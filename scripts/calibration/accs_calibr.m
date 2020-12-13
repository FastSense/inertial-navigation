%% user parameters
freq = 2000; % [Hz]
phi = 55.782111; % latitude, [deg]
h = 220; % height, [m]
SCND_ORDER_TEST = 0; % experimental

fsz = 20;
lw = 2;
Tlbl = 'temperature, deg';

g = 9.780318*(1 + 0.005302*sind(phi)*sind(phi) -...
	0.000006*sind(2*phi)*sind(2*phi)) - 0.000003086*h; % [m/s^2]

%% converted files loading
if	~exist('ud_1','var') || ~exist('ud_2','var')
	ud_1 = load('../../data/ADIS16505-1/2020_11_11_MSU_static_temp/conv/conv_burst_16bit_2000Hz_one_take.txt');
	ud_2 = load('../../data/ADIS16505-1/2020_11_14_MSU_static_temp/conv/conv_burst_16bit_2000Hz_one_take.txt');
end

flist = {
	ud_1,...
	ud_2,...
};

run('accs_chunks.m');
t_ch_1(1,1) = 1/freq;
t_ch_1 = t_ch_1*freq;
t_ch_2(1,1) = 1/freq;
t_ch_2 = t_ch_2*freq;

f1 = figure('defaultaxesfontsize',fsz,'defaultlinelinewidth',lw);
set(f1,'PaperPositionMode','auto', 'Units', 'Normalized',...
	'OuterPosition', [0.04, 0.04, 0.96, 0.92]);
xlabel(Tlbl); ylabel('b_f^0, m/sec^2');

f2 = figure('defaultaxesfontsize',fsz,'defaultlinelinewidth',lw);
set(f2,'PaperPositionMode','auto', 'Units', 'Normalized',...
	'OuterPosition', [0.04, 0.04, 0.96, 0.92]);
xlabel(Tlbl); ylabel('S_f^0');

bf1T = [];
bf2T = [];
bf3T = [];
Sf1T = [];
Sf2T = [];
Sf3T = [];

for k = 1:numel(flist)
	f = flist{k}(:,4:6);		% [m/s^2]
	T = flist{k}(:,7);			% [deg]
	t = [0:numel(T)-1]/freq;	% [sec]
	
	for axis = 3:-1:1 % Z,Y,X order in experiments
		x0 = [];
		T0 = [];
		H2 = [];
		if SCND_ORDER_TEST, z2 = []; T2 = []; x2 = []; end
		for j = [1:6:36]+2*(3-axis) % number of chunk when axis directed upwards
			if k == 1
				f_u = mean(f(t_ch_1(j,1):t_ch_1(j,2),axis));
				T_u = mean(T(t_ch_1(j,1):t_ch_1(j,2)),1);
				f_d = mean(f(t_ch_1(j+1,1):t_ch_1(j+1,2),axis));
				T_d = mean(T(t_ch_1(j+1,1):t_ch_1(j+1,2),1));
				if j == 33 && 1 % Y down (the 34th chunk)
					cut = [t_ch_1(j+1,1):1028*freq 1033*freq:t_ch_1(j+1,2)];
					f_d = mean(f(cut,axis));
					T_d = mean(T(cut));
				end
				mrkr = 'o';
			elseif k == 2
				f_u = mean(f(t_ch_2(j,1):t_ch_2(j,2),axis));
				T_u = mean(T(t_ch_2(j,1):t_ch_2(j,2)));
				f_d = mean(f(t_ch_2(j+1,1):t_ch_2(j+1,2),axis));
				T_d = mean(T(t_ch_2(j+1,1):t_ch_2(j+1,2)));
				mrkr = 's';
			end

			T0 = [T0; mean([T_u;T_d])];
			
			% simplified calibration
			z0 = [f_u; f_d]; 
			H0 = [1,  g; 1, -g];
			x0 = [x0; pinv(H0)*z0];
			% 2nd order model
			if SCND_ORDER_TEST
				H2 = [H2; 1 T_u T_u^2, g g*T_u g*T_u^2;...
						  1 T_d T_d^2, g g*T_d g*T_d^2];
				z2 = [z2; f_u; f_d];
				T2 = [T2; T_u; T_d];
			end
		end
		
		if SCND_ORDER_TEST, x2 = [x2; pinv(H2)*z2]; end
		if axis == 1
			figure(f1);
			hold on; grid on; 
			pbf1 = plot(T0, x0(1:2:end),['-b',mrkr]);
			bf1T = [bf1T; T0, x0(1:2:end)];
% 			plot(T2, H2(:,1:3)*x2(1:3),['-c',mrkr]);			
			figure(f2);
			hold on; grid on; 
			pSf1 = plot(T0, x0(2:2:end),['-b',mrkr]);
			Sf1T = [Sf1T; T0, x0(2:2:end)];
		elseif axis == 2
			figure(f1);
			hold on; grid on; 
			pbf2 = plot(T0, x0(1:2:end),['-g',mrkr]);
			bf2T = [bf2T; T0, x0(1:2:end)];
			
			figure(f2);
			hold on; grid on; 
			pSf2 = plot(T0, x0(2:2:end),['-g',mrkr]);
			Sf2T = [Sf2T; T0, x0(2:2:end)];
		elseif axis == 3
			figure(f1);
			hold on; grid on; 
			pbf3 = plot(T0, x0(1:2:end),['-r',mrkr]);
			bf3T = [bf3T; T0, x0(1:2:end)];
			
			figure(f2);
			hold on; grid on; 
			pSf3 = plot(T0, x0(2:2:end),['-r',mrkr]);
			Sf3T = [Sf3T; T0, x0(2:2:end)];
		end
	end
end

%% bf approximation plot
figure(f1); hold on; 
H = [1+0*bf1T(:,1) bf1T(:,1) bf1T(:,1).^2];
bf1c = pinv(H)*bf1T(:,2);

T1x = [min(bf1T(:,1)):0.2:max(bf1T(:,1))]';
bf1y = [1+0*T1x(:,1) T1x(:,1) T1x(:,1).^2]*bf1c;
pbf1a = plot(T1x,bf1y,'b--');

H = [1+0*bf2T(:,1) bf2T(:,1) bf2T(:,1).^2];
bf2c = pinv(H)*bf2T(:,2);

T2x = [min(bf2T(:,1)):0.2:max(bf2T(:,1))]';
bf2y = [1+0*T2x(:,1) T2x(:,1) T2x(:,1).^2]*bf2c;
pbf2a = plot(T2x,bf2y,'g--');

H = [1+0*bf3T(:,1) bf3T(:,1) bf3T(:,1).^2];
bf3c = pinv(H)*bf3T(:,2);

T3x = [min(bf3T(:,1)):0.2:max(bf3T(:,1))]';
bf3y = [1+0*T3x(:,1) T3x(:,1) T3x(:,1).^2]*bf3c;
pbf3a = plot(T3x,bf3y,'r--');

legend([pbf1 pbf2 pbf3 pbf1a pbf2a pbf3a],...
	{'b_{f_X}^{0}','b_{f_Y}^{0}','b_{f_Z}^{0}',...
	'b_{f_X}^{0 app}','b_{f_Y}^{0 app}','b_{f_Z}^{0 app}'},...
	'location','southwest');

%% Sf approximation plot
figure(f2); hold on; 
H = [1+0*Sf1T(:,1) Sf1T(:,1) Sf1T(:,1).^2];
Sf1c = pinv(H)*Sf1T(:,2);

T1x = [min(Sf1T(:,1)):0.2:max(Sf1T(:,1))]';
Sf1y = [1+0*T1x(:,1) T1x(:,1) T1x(:,1).^2]*Sf1c;
pSf1a = plot(T1x,Sf1y,'b--');

H = [1+0*Sf2T(:,1) Sf2T(:,1) Sf2T(:,1).^2];
Sf2c = pinv(H)*Sf2T(:,2);

T2x = [min(Sf2T(:,1)):0.2:max(Sf2T(:,1))]';
Sf2y = [1+0*T2x(:,1) T2x(:,1) T2x(:,1).^2]*Sf2c;
pSf2a = plot(T2x,Sf2y,'g--');

H = [1+0*Sf3T(:,1) Sf3T(:,1) Sf3T(:,1).^2];
Sf3c = pinv(H)*Sf3T(:,2);

T3x = [min(Sf3T(:,1)):0.2:max(Sf3T(:,1))]';
Sf3y = [1+0*T3x(:,1) T3x(:,1) T3x(:,1).^2]*Sf3c;
pSf3a = plot(T3x,Sf3y,'r--');

legend([pSf1 pSf2 pSf3 pSf1a pSf2a pSf3a],...
	{'S_{f_X}^{0}','S_{f_Y}^{0}','S_{f_Z}^{0}',...
	'S_{f_X}^{0 app}','S_{f_Y}^{0 app}','S_{f_Z}^{0 app}'},...
	'location','southwest');

%% approximation model
disp('temperature polynomials a2*T^2+a1*T+a0 -> [a0 a1 a2]:');
disp(['T1 = [' num2str(T1x(1),'%.1f') ' ' num2str(T1x(end),'%.1f') ']']);
disp(['bf1 coeffs. = [' num2str(bf1c','%+.6f	') ']']);
disp(['Sf1 coeffs. = [' num2str(Sf1c','%+.6f	') ']']);
disp(['T2 = [' num2str(T2x(1),'%.1f') ' ' num2str(T2x(end),'%.1f') ']']);
disp(['bf2 coeffs. = [' num2str(bf2c','%+.6f	') ']']);
disp(['Sf2 coeffs. = [' num2str(Sf2c','%+.6f	') ']']);
disp(['T3 = [' num2str(T3x(1),'%.1f') ' ' num2str(T3x(end),'%.1f') ']']);
disp(['bf3 coeffs. = [' num2str(bf3c','%+.6f	') ']']);
disp(['Sf3 coeffs. = [' num2str(Sf3c','%+.6f	') ']']);
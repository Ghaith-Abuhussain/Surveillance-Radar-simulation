clear all ; clc ; close all ;

% Radar Parameters
% Radar charactaristic parameters
pd = 0.9;                              % Probability of detection
pfa = 1e-6;                            % Probability of false alarm
max_range = 10000;                     % Maximum unambiguous range
range_res = 100;                        % Required range resolution
tgt_rcs = 10;                          % Required target radar cross section


prop_speed = physconst('LightSpeed');   % Propagation speed
pulse_bw = prop_speed./(2*range_res);               % Pulse bandwidth
pulse_width = 10/pulse_bw;               % Pulse width
prf = prop_speed./(2*[max_range]);        % Pulse repetition frequency
%prf = prop_speed/(2*(max_range));      % Pulse repetition frequency
fs = 2*pulse_bw;                        % Sampling rate
num_pulse_int = 10;                     % Number of integrated pulses
fc = 10e9;                              % Operating frequency
lambda = prop_speed/fc;                 % The wavelength
tx_gain = 35;
sensorheight = 40;

snr_min = albersheim(pd, pfa, num_pulse_int);
peak_power = 400;%radareqpow(lambda,max_range,snr_min,pulse_width,...
    %'RCS',tgt_rcs,'Gain',tx_gain);

% Create the grids
fast_time_grid = unigrid(0,1/fs,1/max(prf),'[)');
slow_time_grid = (0:num_pulse_int-1)/max(prf);


% Modeling The Basics Of Radar Components
waveform = phased.RectangularWaveform(...
    'PulseWidth',pulse_width,...
    'PRF',prf,...
    'SampleRate',fs);

receiver = phased.ReceiverPreamp(...
    'Gain',tx_gain,...
    'NoiseFigure',0,...
    'SampleRate',fs,...
    'EnableInputPort',true,'PhaseNoiseInputPort',true,...
    'SeedSource','Property','Seed',2010);

transmitter = phased.Transmitter(...
    'Gain',tx_gain,...
    'PeakPower',peak_power,...
    'InUseOutputPort',true,'CoherentOnTransmit',false,...
    'PhaseNoiseOutputPort',true);

azang = -180:180;
elang = -90:90;
gain = 0;
sArray = CreateAntenna1(azang,elang,gain,fc);

% The Radiator and collector objects
radiator = phased.Radiator(...
    'Sensor',sArray,...
'OperatingFrequency',fc);
collector = phased.Collector(...
    'Sensor',sArray,...
    'OperatingFrequency',fc);

% Create FreeSpace Channel Response
channel = phased.FreeSpace(...
    'SampleRate',fs,...
    'TwoWayPropagation',true,...
    'OperatingFrequency',fc);
%%
trgamma = 35;
clutter = phased.ConstantGammaClutter('Sensor',sArray,...
    'PropagationSpeed',radiator.PropagationSpeed,'EarthModel','Curved',...
    'OperatingFrequency',radiator.OperatingFrequency,...
    'SampleRate',waveform.SampleRate,...
    'PRF',waveform.PRF,'Gamma',trgamma,'PlatformHeight',sensorheight,...
    'PlatformSpeed',90,'PlatformDirection',[0;0],...
    'BroadsideDepressionAngle',0,'MaximumRange',max_range,...
    'AzimuthCoverage',360,'PatchAzimuthWidth',5,...
    'SeedSource','Property','Seed',0);

% Simulation Loop
range_gates = prop_speed*fast_time_grid/2;
r = [];
%%
release(sArray);
sArray.ElementNormal = [1;10];

pulse = waveform();
[pulse,txstatus,phsnoise] = transmitter(pulse);
tic;
csig = clutter();
toc;

rxsig = receiver(csig,~(txstatus>0),phsnoise);
r = [r csig];
%%
figure;
plot(pow2db(abs(r(:,1) - r(:,2))));
%%
ang = [180:-1:-180];
X = range_gates'*cosd(ang); Y = range_gates'*sind(ang);
numscans = length(ang);
clutter_sig = zeros(length(range_gates),numscans);
tic;
for j = 1:361
    release(clutter);
	sArray.ElementNormal = [ang(j);10];
	clutter.PlatformDirection = [ang(j);10];
	clutter.PlatformSpeed = 90;
 	for m = 1:num_pulse_int
        clutter_sig(:,j) = clutter();
	end
       
end

toc;
%%
clutter_sig_wind = clutter_sig;
save("clutter_signal203","clutter_sig_wind");
%%
s = abs(fft(r(:,3:10),8,2));
fd = -prf/2:prf/8:prf/2 -prf/8;
plot(fd,pow2db(fftshift(s(1,:))));



clear all ; clc ; close all ;

% Radar Parameters
% Radar charactaristic parameters
pd = 0.9;            % Probability of detection
pfa = 1e-6;          % Probability of false alarm
max_range = 10000;    % Maximum unambiguous range
range_res = 100;      % Required range resolution
tgt_rcs = 1;         % Required target radar cross section


prop_speed = physconst('LightSpeed');   % Propagation speed
pulse_bw = prop_speed/(2*range_res);    % Pulse bandwidth
pulse_width = 5.3/pulse_bw;               % Pulse width
prf = prop_speed./(2*max_range);         % Pulse repetition frequency
fs = 4*pulse_bw;                        % Sampling rate
num_pulse_int = 10;                     % Number of integrated pulses
fc = 10e9;                              % Operating frequency
lambda = prop_speed/fc;                 % The wavelength
tx_gain = 20;
sensorheight = 10;

snr_min = shnidman(pd,pfa,num_pulse_int,2);
peak_power = radareqpow(lambda,max_range,snr_min,pulse_width,...
    'RCS',tgt_rcs,'Gain',tx_gain);


% Create the grids
fast_time_grid = unigrid(0,1/fs,1/max(prf),'[)');
slow_time_grid = (0:num_pulse_int-1)/max(prf);

% Modeling WaveForm
waveform = phased.LinearFMWaveform(...
    'SweepBandwidth',pulse_bw,...
    'PulseWidth',pulse_width,...
    'PRF',prf,...
    'SampleRate',fs);
%%
% Plot WaveForm
lfmwav = waveform();
nsamp = size(lfmwav,1);
t = [0:(nsamp-1)]/fs;
plot(t*1000,real(lfmwav))
xlabel('Time (millisec)')
ylabel('Amplitude')
grid
%%
% Modeling The Basics Of Radar Components
receiver = phased.ReceiverPreamp(...
    'Gain',20,...
    'NoiseFigure',0,...
    'SampleRate',fs,...
    'EnableInputPort',true,'PhaseNoiseInputPort',true,...
    'SeedSource','Property','Seed',2007);

transmitter = phased.Transmitter(...
    'Gain',tx_gain,...
    'PeakPower',peak_power,...
    'InUseOutputPort',true,'CoherentOnTransmit',false,...
    'PhaseNoiseOutputPort',true);

azang = -180:180;
elang = -90:90;
gain = 3;
sArray = CreateAntenna(azang,elang,gain,fc);

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
figure(1);
patternAzimuth(sArray,fc,[0],'Type','powerdb');
%%
% PlatForms
sensormotion = phased.Platform(...
    'InitialPosition',[0; 0; sensorheight],...
    'Velocity',[0; 0; 0]);

% redefine Targets
tgtpos = [[-1600 0 400]',[4900 0 0]',[7000 0 0]'];
tgtvel = [[100 100 0]',[-100 0 0]',[200 0 0]'];
tgtmotion = phased.Platform('InitialPosition',tgtpos,'Velocity',tgtvel);
tgtrcs = [2.2 1.1 1.05];
target = phased.RadarTarget('MeanRCS',tgtrcs,'Model','Swerling2',...
    'OperatingFrequency',fc);
target.SeedSource = 'Property';
target.Seed = 2007;
%%
% Modeling Clutter

% blind speeed
prf = waveform.PRF;
wavelength = prop_speed/fc;
blindspd = dop2speed(prf,wavelength)/2; % half to compensate round trip
trgamma = 15;

clutter = phased.ConstantGammaClutter('Sensor',sArray,...
    'PropagationSpeed',radiator.PropagationSpeed,'EarthModel','Curved',...
    'OperatingFrequency',radiator.OperatingFrequency,...
    'SampleRate',waveform.SampleRate,'TransmitSignalInputPort',true,...
    'PRF',waveform.PRF,'Gamma',trgamma,'PlatformHeight',sensorheight,...
    'PlatformSpeed',sqrt(sum(sensormotion.Velocity.^2)),'PlatformDirection',[0;0],...
    'BroadsideDepressionAngle',0,'MaximumRange',max_range,...
    'AzimuthCoverage',360,'PatchAzimuthWidth',5,...
    'SeedSource','Property','Seed',2011);
%%
% CFAR Detector
Ntraining = 100;
Nguard = 2;
Pfa_goal = pfa;
detector = phased.CFARDetector('Method','GOCA',...
    'NumTrainingCells',Ntraining,'NumGuardCells',Nguard,...
    'ProbabilityFalseAlarm',Pfa_goal,'ThresholdOutputPort',true);

%%
% Simulation Loop
range_gates = prop_speed*fast_time_grid/2;

% Time Variant gain
tvg = phased.TimeVaryingGain(...
    'RangeLoss',2*fspl(range_gates,lambda),...
    'ReferenceLoss',2*fspl(max(range_gates),lambda));

%Matched Filter
matchingcoeff = getMatchedFilter(waveform);
matchedfilter = phased.MatchedFilter('Coefficients',matchingcoeff,...
    'CoefficientsSource','Property');

%%
% Scan Angles
ang = [0];
num_360_rotations = 1;
numscans = numel(ang);
est_range = [];
est_angle = [];

int_pulses = zeros(numel(fast_time_grid),numscans);
screens = zeros(numel(fast_time_grid),numscans,num_360_rotations);
screens1 = zeros(numel(fast_time_grid),numscans,num_360_rotations);

%%
% Scinario View
BeamSteering = [ang(1);0];
r_update = 20;

sceneview = phased.ScenarioViewer('BeamRange',5.0e3,'BeamWidth',[2;60],'UpdateRate',r_update,...
    'PlatformNames',{'Ground Radar','Vehicle1','Vehicle2','Vehicle3'},'ShowPosition',true,...
    'ShowSpeed',true,'ShowAltitude',true,'ShowLegend',true,'ShowRange',true,...
    'Title','Multiplatform Scenario','BeamSteering',BeamSteering);

%%
%reset(sceneview);
X = range_gates'*cosd(ang); Y = range_gates'*sind(ang);
tic;
for i = 1:num_360_rotations
    
    for j = 1:numscans
        
        release(radiator);
        release(collector);
        release(channel);
        release(sArray);
        release(clutter);
        sArray.ElementNormal = [ang(j);0];
        clutter.PlatformDirection = [ang(j);0];
        clutter.PlatformSpeed = 0;%sqrt(sum(sensormotion.Velocity.^2));
        for m = 1:num_pulse_int

            
            pulse = waveform();
            % Update sensor and target positions
            steptime = length(pulse)/fs;
            [sensorpos,sensorvel] = sensormotion(steptime);
            [tgtpos,tgtvel] = tgtmotion(steptime);

            % Calculate the target angles as seen by the sensor
            [~,tgtang] = rangeangle(tgtpos,sensorpos);

            % Simulate propagation of pulse in direction of targets
            [pulse,txstatus,phsnoise] = transmitter(pulse);
            txsig = radiator(pulse,tgtang);
            release(channel)
            txsig = channel(txsig,sensorpos,tgtpos,sensorvel,tgtvel);

            % Reflect pulse off of targets
            tgtsig = target(txsig,true);
            
            %csig = clutter(pulse(txstatus>0));
            %csig = [csig(length(pulse)*(prf*pulse_width)+1:end);zeros(length(pulse)*(prf*pulse_width),1)];
            csig = rainClutter1(pulse,pulse_width,...
                transmitter,receiver,radiator,collector,ang(j),0,...
                range_res,[0;0;0],[20;20;-100],[0;0;0],channel,10,...
                1.5,500e-6,fc,2,60);
            %csig = [csig(length(pulse)*(prf*pulse_width)+1:end);zeros(length(pulse)*(prf*pulse_width),1)];
            %csig = rainClutter(max_range,length(txsig),Lz,...
                    %fc,Rain_Rate,20,0);
            %if(ang(j) <= 5 && ang(j) >= -5)
                % Collect clutter signal at sensor
                %csig = clutter(pulse(txstatus>0));
            %else
                %csig = zeros(length(txsig),1);
            %end

            % Receive target returns and clutter at sensor
            rxsig = collector(tgtsig,tgtang);

            rxsig = receiver(rxsig+csig,~(txstatus>0),phsnoise);

            % For staggered PRF waveforms, the pulses will be different lengths,
            % here we truncate to the length of the shortest pulse in the PRF
            % stagger
            rxpulses(:,m) = rxsig(1:length(fast_time_grid));
            Pulse_Map(:,j,m) = rxsig(1:length(fast_time_grid));
        end
        rxPulse = rxpulses;
        mfiltOut = matchedfilter(rxPulse);

        matchingdelay = size(matchingcoeff,1)-1;
        mfiltOut = buffer(mfiltOut(matchingdelay+1:end),size(mfiltOut,1));
        mfiltOut = tvg(mfiltOut);
        h = [1 -2 1];
        mtiseq = filter(h,1,mfiltOut,[],2);
        
        mtiseq = pulsint(mtiseq(:,3:end));
        mfilt_without_MTI = pulsint(mfiltOut(:,1:end));
        x = detector(mtiseq,1:length(mtiseq));
        int_pulses(:,j) = x;
        screens1(:,j,i) = mtiseq;
        
        % display scene
        sceneview.BeamSteering = [ang(j);0];
        sceneview(sensorpos,sensorvel,tgtpos,tgtvel);

        % perform next detection when next update is needed
        sensormotion(1/r_update);
        tgtmotion(1/r_update);
        
        
    end
    screens(:,:,i) = int_pulses;
    int_pulses1 = int_pulses(length(pulse)*(prf*pulse_width)+1:end-1,:);
    [~,peakInd] = findpeaks(int_pulses1(1:end),'MinPeakHeight',0.5);
    [rngInd,angInd] = ind2sub(size(int_pulses1),peakInd);
    est_range = [est_range {range_gates(rngInd)+length(pulse)*(prf*pulse_width)*(range_res/2)}]; % Estimated range
    est_angle = [est_angle {ang(angInd)}];    % Estimated direction
end
toc;

%%
[x,TH] = detector(screens1(:,1,1),1:length(screens1(:,1,1)));
%%

% Plot the received pulse energy again range
plot(range_gates,pow2db(abs(screens1(:,1,1)).^2),'b-'...
    ,range_gates,pow2db(abs(TH).^2)); grid on;
title('Fast Time Sequences Using a Uniform PRF');
xlabel('Range (m)'); ylabel('Power (dB)');
legend('After MTI filter');
%%

% Plot the received pulse energy again range
plot(range_gates,abs(screens(:,1,1)).^2,'b-'); grid on;
title('Fast Time Sequences Using a Uniform PRF');
xlabel('Range (m)'); ylabel('Power (dB)');
legend('After MTI filter');
%%
nn = zeros(400,10);
nn(:,:) = Pulse_Map(:,1,:);
plot(range_gates,pow2db(abs(Pulse_Map(:,1,6)).^2));
%%

plot(fftshift(abs(fft(nn))));
%%
glo = ones(200,73);
X = range_gates'*cosd(ang); Y = range_gates'*sind(ang);
for ii = 1:num_360_rotations
    for m = 2:length(ang)
        x = X(:,1:m); y = Y(:,1:m);
        figure(1);
        pcolor(x,y,pow2db(abs(screens1(:,1:m,ii)).^2));
        axis equal tight; 
        shading interp;
        axis off;
        axis([-10000 10000 -10000 10000]);
        title('Radar PPI Display');

    end
end

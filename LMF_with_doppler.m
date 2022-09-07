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
prf = prop_speed./(2*max_range);        % Pulse repetition frequency
%prf = prop_speed/(2*(max_range));      % Pulse repetition frequency
fs = 2*pulse_bw;                        % Sampling rate
num_pulse_int = 20;                     % Number of integrated pulses
fc = 10e9;                              % Operating frequency
lambda = prop_speed/fc;                 % The wavelength
tx_gain = 35;
sensorheight = 40;

snr_min = albersheim(pd, pfa, num_pulse_int);
peak_power = 1000;%radareqpow(lambda,max_range,snr_min,pulse_width,...
    %'RCS',tgt_rcs,'Gain',tx_gain);

% Create the grids
fast_time_grid = unigrid(0,1/fs,1/max(prf),'[)');
slow_time_grid = (0:num_pulse_int-1)/max(prf);


% Modeling WaveForm
waveform = phased.LinearFMWaveform(...
    'SweepBandwidth',pulse_bw,...
    'PulseWidth',pulse_width,...
    'PRF',prf,...
    'SampleRate',fs);

% Plot WaveForm
lfmwav = waveform();
nsamp = size(lfmwav,1);
t = [0:(nsamp-1)]/fs;
plot(t*1000,real(lfmwav))
xlabel('Time (millisec)')
ylabel('Amplitude')
grid
%%

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
figure(1);
patternAzimuth(sArray,fc,[0],'Type','powerdb');


%%
% PlatForms
sensormotion = phased.Platform(...
    'InitialPosition',[0; 0; sensorheight],...
    'Velocity',[0; 0; 0]);

% redefine Targets
tgtpos = [[-5000 0 500]',[5000 0 300]',[-4300 -4000 300]'];
Acceleration = [[0 0 0]',[0 0 0]',[0 0 0]'];


tgtvel = [[50 0 0]',[-90 0 0]',[-45 55 0]'];
tgtmotion = phased.Platform('MotionModel','Acceleration','InitialPosition',...
    tgtpos,'InitialVelocity',tgtvel,'Acceleration',Acceleration);
tgtrcs = [1 2.2 1.1 ];
target = phased.RadarTarget('MeanRCS',tgtrcs,'Model','Swerling2',...
    'OperatingFrequency',fc);
target.SeedSource = 'Property';
target.Seed = 2007;


% Modeling Clutter

% blind speeed
prf = waveform.PRF;
wavelength = prop_speed/fc;
blindspd = dop2speed(prf,wavelength)/2; % half to compensate round trip
trgamma = 15;

clutter = phased.ConstantGammaClutter('Sensor',sArray,...
    'PropagationSpeed',radiator.PropagationSpeed,'EarthModel','Curved',...
    'OperatingFrequency',radiator.OperatingFrequency,...
    'SampleRate',waveform.SampleRate,...
    'PRF',waveform.PRF,'Gamma',trgamma,'PlatformHeight',sensorheight,...
    'PlatformSpeed',0,'PlatformDirection',[0;0],...
    'BroadsideDepressionAngle',0,'MaximumRange',max_range,...
    'AzimuthCoverage',360,'PatchAzimuthWidth',5,...
    'SeedSource','Property','Seed',2011);
%%
% CFAR Detector
N_train = 10;
N_guard = 5;
cfar2D = phased.CFARDetector2D('Method','SOCA','GuardBandSize',N_guard,'TrainingBandSize',[N_train N_train],...
  'ProbabilityFalseAlarm',pfa,'ThresholdOutputPort',true,'ThresholdFactor','custom',...
  'CustomThresholdFactor',10);


% Simulation Loop
range_gates = prop_speed*fast_time_grid/2;

% Time Variant gain
tvg = phased.TimeVaryingGain(...
    'RangeLoss',2*fspl(range_gates,lambda),...
    'ReferenceLoss',2*fspl(max(range_gates),lambda));

%Matched Filter
matchingcoeff = getMatchedFilter(waveform);
matchedfilter = phased.MatchedFilter('Coefficients',matchingcoeff);


% Scan Angles
ang = [-180:180];
num_360_rotations = 5;
numscans = numel(ang);
est_range = [];
est_angle = [];
estm_range = []; % Estimated range
estm_doppler = [];% Estimated range

int_pulses = zeros(numel(fast_time_grid),numscans);
screens = [];
screens1 = zeros(numel(fast_time_grid),numscans,20);
CMap = zeros(numel(fast_time_grid),num_pulse_int,numscans);

% Scinario View
BeamSteering = [ang(1);0];
r_update = 1/4;


%%

% Configure your tracker here:
ratio = 10;
numTracks = 50;
gate = 800/ratio;
vol = 1e9;
beta = 1e-14;
pd = 0.9;
far = 1e-6;
ang1 = [0:360];
max_rng = 10000;
FontSize = 7;
dt = 4;
positionSelector = [1 0 0 0 0 0 ;0 0 1 0 0 0; 0 0 0 0 0 0]; % [x, y, 0]
velocitySelector = [0 1 0 0 0 0 ; 0 0 0 1 0 0 ;0 0 0 0 0 0]; % [vx, vy, 0]
tracker = trackerJPDA(...
    'FilterInitializationFcn',@initekfimm,...
    'MaxNumTracks', numTracks, ...
    'MaxNumSensors', 1, ...
    'AssignmentThreshold',gate, ...
    'DeathRate',0.01,...
    'TrackLogic','Integrated',...
    'DetectionProbability', pd, ...
    'ConfirmationThreshold',0.95,...
    'DeletionThreshold',0.15,...
    'ClutterDensity', far/vol, ...
    'NewTargetDensity', beta,...
    'TimeTolerance',10);

% Add a trackPlotter here:
tp = theaterPlot('XLim',[-max_range max_range],'YLim',[-max_range max_range]);
trackp = trackPlotter(tp,'DisplayName','Tracks','HistoryDepth',50,'LabelOffset',[0 0 0],'ConnectHistory','on','ColorizeHistory','on' );
hold on;
    h1 = plot(0.2*max_rng*cosd(ang1),0.2*max_rng*sind(ang1),0.4*max_rng*cosd(ang1),0.4*max_rng*sind(ang1),...
    0.6*max_rng*cosd(ang1),0.6*max_rng*sind(ang1),...
    0.8*max_rng*cosd(ang1),0.8*max_rng*sind(ang1),...
    1*max_rng*cosd(ang1),1*max_rng*sind(ang1),[0 max_rng*cosd(0)],[0 max_rng*sind(0)],[0 max_rng*cosd(30)],[0 max_rng*sind(30)],...
    [0 max_rng*cosd(60)],[0 max_rng*sind(60)],...
    [0 max_rng*cosd(90)],[0 max_rng*sind(90)],...
    [0 max_rng*cosd(120)],[0 max_rng*sind(120)],[0 max_rng*cosd(150)],[0 max_rng*sind(150)],...
    [0 max_rng*cosd(180)],[0 max_rng*sind(180)],[0 max_rng*cosd(210)],[0 max_rng*sind(210)],...
    [0 max_rng*cosd(240)],[0 max_rng*sind(240)],[0 max_rng*cosd(270)],[0 max_rng*sind(270)],...
    [0 max_rng*cosd(300)],[0 max_rng*sind(300)],[0 max_rng*cosd(330)],[0 max_rng*sind(330)]);

    set( h1, 'Color', 'r' );
    text(max_rng/sqrt(2)+0.05,max_rng/sqrt(2)+0.05,num2str(max_rng),'FontSize',FontSize);
    text(0.8*max_rng*(1/sqrt(2)+0.05),0.8*max_rng*(1/sqrt(2)+0.05),num2str(max_rng*4/5),'FontSize',FontSize);
    text(0.6*max_rng*(1/sqrt(2)+0.05),0.6*max_rng*(1/sqrt(2)+0.05),num2str(max_rng*3/5),'FontSize',FontSize);
    text(0.4*max_rng*(1/sqrt(2)+0.05),0.4*max_rng*(1/sqrt(2)+0.05),num2str(max_rng*2/5),'FontSize',FontSize);
    text(0.2*max_rng*(1/sqrt(2)+0.05),0.2*max_rng*(1/sqrt(2)+0.05),num2str(max_rng*1/5),'FontSize',FontSize);

    text(max_rng+0.05,0,'0','FontSize',FontSize);
    text(max_rng*sqrt(3)/2+0.05,max_rng*0.5+0.05,'30','FontSize',FontSize);
    text(max_rng*0.5+0.05,max_rng*sqrt(3)/2+0.05,'60','FontSize',FontSize);
    text(0,max_rng+0.05,'90','FontSize',FontSize);
    text(-max_rng*sqrt(3)/2-0.1,max_rng*0.5+0.05,'150','FontSize',FontSize);
    text(-max_rng*0.5-0.1,max_rng*sqrt(3)/2+0.05,'120','FontSize',FontSize);
    text(-max_rng-0.1,0,'180','FontSize',FontSize);
    text(-max_rng*sqrt(3)/2-0.1,-max_rng*0.5-0.05,'210','FontSize',FontSize);
    text(-max_rng*0.5-0.05,-max_rng*sqrt(3)/2-0.05,'240','FontSize',FontSize);
    text(0-0.05,-max_rng-0.05,'270','FontSize',FontSize);
    text(max_rng*sqrt(3)/2+0.05,-max_rng*0.5-0.05,'330','FontSize',FontSize);
    text(max_rng*0.5+0.05,-max_rng*sqrt(3)/2-0.05,'300','FontSize',FontSize);
   axis off;

%%
X = range_gates'*cosd(ang); Y = range_gates'*sind(ang);
fft_size = 64;
pulse = waveform();
csig = clutter();
csig = [csig(length(pulse)*(prf*pulse_width)+1:end);zeros(length(pulse)*(prf*pulse_width),1)];
tic;
for i = 1:num_360_rotations
    estm_range = []; % Estimated range
    estm_doppler = [];% Estimated range
    est_angle = [];
    det_pos = [];
    dop = [];
    for j = 1:numscans
        
        release(radiator);
        release(collector);
        release(channel);
        release(sArray);
        release(clutter);
        sArray.ElementNormal = [ang(j);10];
        clutter.PlatformDirection = [ang(j);10];
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
            
            txsig = channel(txsig,sensorpos,tgtpos,sensorvel,tgtvel);

            % Reflect pulse off of targets
            if(i == 1)
                tgtsig = txsig;
            else
                tgtsig = target(txsig,true);
            end
            
            
            % Receive target returns and clutter at sensor
            rxsig = collector(tgtsig,tgtang);

            rxsig = receiver(rxsig,~(txstatus>0),phsnoise);

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
        if(i >= 2)
            mfiltOut = mfiltOut - CMap(:,:,j);
            screens1(:,j,:) = mfiltOut;
            rng_dpp_map = abs(fftshift(fft(mfiltOut,fft_size,2),2));
            x = range_gates;
            y = -prf/2:prf/size(rng_dpp_map,2):prf/2-prf/size(rng_dpp_map,2);
            rangeIndx = 1+(N_train + N_guard):length(range_gates)-(N_train + N_guard);
            dopplerIndx = 1+(N_train + N_guard):fft_size -(N_train + N_guard);
            [rowInds,columnInds] = meshgrid(rangeIndx,...
              dopplerIndx);
            CUTIdx = [rowInds(:) columnInds(:)]';
            [~,CF_TH] = cfar2D(rng_dpp_map,CUTIdx);
            %CF_data_map = reshape(CF_data,length(dopplerIndx),length(rangeIndx));
            CF_TH_map = reshape(CF_TH,length(dopplerIndx),length(rangeIndx));
            CF_Data = max(rng_dpp_map(rangeIndx,dopplerIndx) - CF_TH_map',0);
            dop_grid_cf = y(dopplerIndx);
            rng_grid_cf = x(rangeIndx);
            [~,ind_mat] = find_peaks(CF_Data);
            [rngInd,dopInd] = ind2sub(size(CF_Data),ind_mat);
            if(length(rng_grid_cf(rngInd)) ~= 0)
                estm_range = [estm_range {rng_grid_cf(rngInd)}]; % Estimated range
                estm_doppler = [estm_doppler {2*dop_grid_cf(dopInd)*lambda}];    % Estimated direction
                est_angle = [est_angle ang(j)];
            end
        else
            CMap(:,:,j) = mfiltOut;
        end
   
    end
    
    if(i>=2)
        sensormotion(1/r_update);
        tgtmotion(1/r_update);
        for k = 1 : length(est_angle)
            for tt = 1:length(estm_range{k})
                pos = [estm_range{k}(tt)*cosd(est_angle(k));estm_range{k}(tt)*sind(est_angle(k));0];
                det_pos = [det_pos pos];
                dop = [dop estm_doppler{k}(tt)];
            end
        end
        [det_pos,dop] = merging(range_res,det_pos,dop);
        time = i*dt;
        if(size(det_pos,2) ~=0)
            
               for k = 1:size(det_pos,2)
                    detection(k) = objectDetection(time,det_pos(:,k)/ratio);
               end
                det = detection(1:size(det_pos,2));
                tracks = tracker(det,time);

                %State = tracks.State;
                Len = length(det_pos);
                position = getTrackPositions(tracks,positionSelector);
                vel = getTrackVelocities(tracks,velocitySelector);
                position = position*ratio;
                vel = vel*ratio;
                labels = string([tracks.TrackID]);
                figure(1);
                trackp.plotTrack(position,labels);grid on;
        end
    end
end
toc;
%%
plot(pow2db(abs(screens1(:,181,2,1))));
%%
H1(:,:) = screens{1}(:,181,:);
F = fftshift(fft(tvg(H1),64,2),2);
%CM = fftshift(fft(cluter_map,64,2),2);
%F = F - CM;
x = range_gates;
y = -prf/2:prf/size(F,2):prf/2-prf/size(F,2);
pcolor(y,x,abs(F));shading interp;
%%
% CFAR Detector
N_train = 10;
N_guard = 5;
cfar2D = phased.CFARDetector2D('Method','GOCA','GuardBandSize',N_guard,'TrainingBandSize',[N_train N_train],...
  'ProbabilityFalseAlarm',pfa,'ThresholdOutputPort',true,'ThresholdFactor','custom',...
  'CustomThresholdFactor',15);

[~,ind_mat] = find_peaks(abs(F));
A = abs(F);
rangeIndx = 16:184;
dopplerIndx = 16:48;
[rowInds,columnInds] = meshgrid(rangeIndx,...
  dopplerIndx);
CUTIdx = [rowInds(:) columnInds(:)]';
[CF_data,CF_TH] = cfar2D(A,CUTIdx);

CF_data_map = reshape(CF_data,length(dopplerIndx),length(rangeIndx));
CF_TH_map = reshape(CF_TH,length(dopplerIndx),length(rangeIndx));
CF_Data = max(A(rangeIndx,dopplerIndx) - CF_TH_map',0);
A(rangeIndx,dopplerIndx) = CF_Data;
pcolor(y(dopplerIndx),x(rangeIndx),max(CF_Data,0));shading interp;

dop_grid_cf = y(dopplerIndx);
rng_grid_cf = x(rangeIndx);
[~,ind_mat] = find_peaks(CF_Data);
[rngInd,dopInd] = ind2sub(size(CF_Data),ind_mat);

estm_range = rng_grid_cf(rngInd); % Estimated range
estm_doppler = dop_grid_cf(dopInd);    % Estimated direction

%%
[Max_dop_data,ind_max_dop] = max(CF_Data,[],2);

[~,ind_peaks_rng] = findpeaks(Max_dop_data);
dop_ind = ind_max_dop(ind_peaks_rng);
estm_range = rng_grid_cf(ind_peaks_rng); % Estimated range
estm_doppler = dop_grid_cf(dop_ind);    % Estimated direction



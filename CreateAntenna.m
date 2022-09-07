function antenna_array = CreateAntenna(azang,elang,gain,fc)

    lambda = physconst('LightSpeed')/fc;
    magpattern = zeros(numel(elang),numel(azang));
    for i = 1:numel(elang)
        for j = 1:numel(azang)
            if(azang(j) >= -90 && azang(j) <=90)
                magpattern(i,j) = 10*abs((cosd(azang(j))^10)*(cosd(elang(i))^16));
            end
        end
    end
    magpattern = mag2db(magpattern);                % Transform The Magnitude Pattern into dB's
    phasepattern = zeros(size(magpattern));
    antenna = phased.CustomAntennaElement('AzimuthAngles',azang, ...
        'ElevationAngles',elang,'MagnitudePattern',magpattern, ...
        'PhasePattern',phasepattern);
    ula = phased.ULA('Element',antenna,...          % uniform rectangular array
        'NumElements',75,'ElementSpacing',lambda,'ArrayAxis','y');

    % multiply the magnitude pattern with blackman window to reduce beamwidth
    PAT = patternAzimuth(ula,fc,[-90:90],'Type','powerdb');
    nbar = 5; 
    sll = -30;
    sltaper = zeros(361,181);
    a = blackman(91);
    b = [zeros(1,135) a' zeros(1,135)];
    for i=1:181
        sltaper(:,i) = b;
    end
    c = mag2db(sltaper);
    PAT = PAT + mag2db(sltaper) - max(PAT) + gain;

    % The Antenna That is used
    antenna1 = phased.CustomAntennaElement('AzimuthAngles',azang, ...
        'ElevationAngles',elang,'MagnitudePattern',PAT', ...
        'PhasePattern',phasepattern);

    sArray = phased.ConformalArray('Element',antenna1,...
        'ElementPosition',[0;0;0],...
        'ElementNormal',[0;0]);
    
    antenna_array = sArray;
end
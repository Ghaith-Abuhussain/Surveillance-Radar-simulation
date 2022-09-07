function signal = rain_clutter(density,fft_size,rngBinsNum,rain_speed,...
    distortion,radar_steering,rain_steering,speed_axis,Noise_level)
    x = -fft_size/2:fft_size/2-1;
    kk = [];
    cos_theta = sum([cosd(radar_steering);sind(radar_steering)].*[cosd(rain_steering);sind(rain_steering)]);
    real_speed = rain_speed*cos_theta;
    sp_diff = abs(speed_axis - real_speed);
    [~,ind] = min(sp_diff);
    est_speed_ind = ind-fft_size/2+1;
    
    for i = 1:rngBinsNum
        r = randi([est_speed_ind-floor(distortion/2) est_speed_ind+floor(distortion/2)]);
        y = abs((sin(x-r)./(pi*(x-r))));
        y(fft_size/2+1+r) = y(fft_size/2+r);
        y = y*density;
        y = awgn(y,Noise_level,'measured');
        kk = [kk;y];
    end
    signal = ifft(fftshift(kk,2),128,2);


end
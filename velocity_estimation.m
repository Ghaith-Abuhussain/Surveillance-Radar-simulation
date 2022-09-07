function [vel_est,vel_err_last,velo2] = velocity_estimation(positions,detection,position,velocities,velocity,theta)
    est_pos   = sqrt(positions(:,1).^2+positions(:,2).^2)';
    estm_pos  = sqrt(detection(1,:).^2+detection(2,:).^2);
    estm1_pos = sqrt(position(1,:).^2+position(2,:).^2);
    estm1_vel = sqrt(velocity(1,:).^2 + velocity(2,:).^2);
    velo = [];
    for i = 1:length(est_pos)
        rng = [];
        for k = 1:length(estm_pos)
            rng = [rng abs((estm_pos(k) - est_pos(i)))];
        end
        [~,ind] = min(rng);
        velo = [velo detection(3,ind)/2];
    end

    velo2 = [];
    cos_theta = [];
    for i = 1:length(est_pos)
        rng = [];
        for k = 1:length(estm1_pos)
            rng = [rng abs((estm1_pos(k) - est_pos(i)))];
        end
        [~,ind] = min(rng);
        velo2 = [velo2 estm1_vel(ind)];
        if(velo(i) < 0)
            cos_theta = [cos_theta cosd(atand(velocity(2,ind)/velocity(1,ind))-atand(positions(i,2)/positions(i,1)))*cosd(theta)];
        else
            cos_theta = [cos_theta cosd(180+atand(velocity(2,ind)/velocity(1,ind))-atand(positions(i,2)/positions(i,1)))*cosd(theta)];
        end
    end
    vel_est  = abs(velo./cos_theta);
    vel_err  = 100*(vel_est - velo2)./velo2;
    vel_err2 = 100*(sqrt(velocities(:,1).^2 + velocities(:,2).^2)' - velo2)./velo2;
    cos_theta2 = cosd(awgn(abs(acosd(velo./velo2)),5));
    for i = 1:length(vel_est)
        if(abs(vel_err(i)) >= abs(vel_err2(i)))
            vel_est(i)   = velo2(i);%sqrt(velocities(i,1).^2 + velocities(i,2).^2)';
        end
    end
    vel_err_last = 100*(vel_est - velo2)./velo2;
end
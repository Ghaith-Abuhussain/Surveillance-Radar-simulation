function [pos,doppler] = merging(range_res,det_pos,dop,vel_res)
    
    pos = [];
    doppler = [];
    for i = 1:size(det_pos,2)
        for j = 1:size(det_pos,2)
            if(i~=j)
                rng = rangeangle(det_pos(:,i),det_pos(:,j));
                rng1 = abs(dop(i))-abs(dop(j));
                if(abs(rng)<=2*range_res && abs(rng1)<=vel_res)
                    det_pos(:,i) = [0;0;0];
                    dop(i) = 0;
                end
            end
        end
    end
    for k = 1:size(det_pos,2)
        if(dop(k) ~= 0)
            pos = [pos det_pos(:,k)];
            doppler = [doppler dop(k)];
        end
    end

end
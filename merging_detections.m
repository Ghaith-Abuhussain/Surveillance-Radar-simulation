function [detection] = merging_detections(all_detections)
    
    det = [];
   
    for i = 1:size(all_detections,2)
        for j = 1:size(all_detections,2)
            if(i~=j)
                rng = abs(all_detections(1,i)-all_detections(1,j));
                rng1 = abs(all_detections(2,i))-abs(all_detections(2,j));
                rng2 = abs(all_detections(3,i))-abs(all_detections(3,j));
                if(abs(rng)<=2 && abs(rng1)<=2 && abs(rng2)<=2)
                    all_detections(1,i) = 0;
                    all_detections(2,i) = 0;
                    all_detections(3,i) = 0;
                end
            end
        end
    end
    for k = 1:size(all_detections,2)
        if(all_detections(2,k) ~= 0)
            det = [det all_detections(:,k)];
        end
    end
    detection = det;
end
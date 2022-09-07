function [peaks,ind_mat] = find_peaks(a)
out_p = [];
ind_op1 = [];
ind_op2 = [];
for i = 1:size(a,1)
    for j = 1:size(a,2)
        v = 0;
        if(j+1 <= size(a,2) && j-1 > 0 && i+1 <= size(a,1) && i-1 > 0)
          if(a(i,j) > a(i-1,j-1) &&  a(i,j) > a(i-1,j) && ...
                  a(i,j) > a(i-1,j+1) && a(i,j) > a(i,j-1) &&...
                  a(i,j) > a(i,j+1) && a(i,j) > a(i+1,j-1) && ...
                  a(i,j) > a(i+1,j) && a(i,j) > a(i+1,j+1) )
              out_p = [out_p a(i,j)];
              ind_op1 = [ind_op1 i];
              ind_op2 = [ind_op2 j];
          end
        elseif(i == 1 || i == size(a,1))
            if(j+1<=size(a,2) && j-1>0)
                if(i == 1)
                    if(a(i,j) > a(i,j-1) && a(i,j) > a(i,j+1) &&...
                            a(i,j) > a(i+1,j-1) && a(i,j) > a(i+1,j) &&...
                            a(i,j) > a(i+1,j+1) )
                        out_p = [out_p a(i,j)];
                        ind_op1 = [ind_op1 i];
                        ind_op2 = [ind_op2 j];
                    end
                end
                if(i == size(a,1))
                    if(a(i,j) > a(i-1,j-1) &&  a(i,j) > a(i-1,j) && ...
                      a(i,j) > a(i-1,j+1) && a(i,j) > a(i,j-1) &&...
                      a(i,j) > a(i,j+1) )
                        out_p = [out_p a(i,j)];
                        ind_op1 = [ind_op1 i];
                        ind_op2 = [ind_op2 j];
                    end
                end
            end
        elseif(j == 1 || j == size(a,2))
            if(i+1<=size(a,1) && i-1>0)
                if(j == 1)
                    if(a(i,j) > a(i-1,j) && a(i,j) > a(i-1,j+1) &&...
                            a(i,j) > a(i,j+1) && a(i,j) > a(i+1,j) &&...
                            a(i,j) > a(i+1,j+1) )
                        out_p = [out_p a(i,j)];
                        ind_op1 = [ind_op1 i];
                        ind_op2 = [ind_op2 j];
                    end
                end
                if(j == size(a,2))
                    if(a(i,j) > a(i-1,j) &&  a(i,j) > a(i+1,j) && ...
                      a(i,j) > a(i-1,j-1) && a(i,j) > a(i,j-1) &&...
                      a(i,j) > a(i+1,j-1) )
                        out_p = [out_p a(i,j)];
                        ind_op1 = [ind_op1 i];
                        ind_op2 = [ind_op2 j];
                    end
                end
            end
        end
           
       end
    end
    peaks = out_p;
    ind_mat = [];
    for i=1:length(peaks)
        ind = find(a == peaks(i));
        ind_mat = [ind_mat ind];
    end
    
end
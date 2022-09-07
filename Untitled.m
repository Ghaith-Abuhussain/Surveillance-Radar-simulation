clear all ; clc; close all;
len = 1;
D_raw_Rng = {};
D_raw_Azm = {};
col = cellstr(num2str([1:len]'))';
fig = figure('Position',[0 300 560 100]);
uit  = uitable(fig,'ColumnName',col,'ColumnWidth','auto','RowName',{'Range';'Azimuth'},'Data',[D_raw_Rng;D_raw_Azm]);
uit.Position = [0 0 560 100];
%uit1 = uitable(fig,'RowName',{'Name1';'Number1'},'Data',{'Bob1';30});
%uit1.Position = [0 350-100 560 70];
%%
len = 4;
D_raw_Rng = {5000,6020,7000,9000};
D_raw_Azm = {5,60,8,-11};
col = cellstr(num2str([1:len]'))';
uit.ColumnName = col;
uit.Data = [D_raw_Rng;D_raw_Azm];
%%
f = figure;
uit = uitable(f);

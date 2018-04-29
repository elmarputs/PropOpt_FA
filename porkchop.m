%% Output Reader
clc
clear all
close all

deltaV = dlmread('SimulationOutput/porkchopLeoGeoTransfer.dat');
x = dlmread('SimulationOutput/porkchopLeoGeoTransfer_x_data.dat');
y = dlmread('SimulationOutput/porkchopLeoGeoTransfer_y_data.dat');

figure
contourf(x,y,deltaV);
xlabel('Thrust [N]')
ylabel('Specific impulse [s]')
zlabel('Delta V [m/s]')
title('Delta V [m/s]')
colorbar()
%% Output Reader
clc
clear all
close all

Cartresults = dlmread('SimulationOutput/FA_output/prop_output.dat');

t = Cartresults(:,1);
x = Cartresults(:,2);
y = Cartresults(:,3);
z = Cartresults(:,4);

figure
plot3(x,y,z);
grid on
xlabel('x Position [m]')
ylabel('y Position [m]')
zlabel('z Position [m]')
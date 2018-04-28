
% clear all; clc; close all; 
addpath('C:\tudatBundle.git\tudatApplications\PropOpt_FA\SimulationOutput')



nrIndividuals = 1000;
nrGen = 30;

fitness = [];
input = [];
for i = 0:(nrGen-1)
    stringfitness = ['fitness_leoGeoTransfer_0_' num2str(i) '_nominal_seed160_ihs.dat'];
    stringfitness = join(stringfitness);
    stringInput = ['population_leoGeoTransfer_0_' num2str(i) '_nominal_seed160_ihs.dat'];
    stringInput = join(stringInput);
    
    fitnessGen = dlmread(stringfitness);
    inputGen = dlmread(stringInput);
    fitness = [fitness; fitnessGen];
    input = [input; inputGen];
end
    

%%
figure
for i = [1, 5, 10, 20, 30]
    start = (i-1)*nrIndividuals+1;
    end1 = i*nrIndividuals;
    deltaV = sort(fitness(start:end1,1));
    flightTime = sort(fitness(start:end1,2)/3600, 'descend');
%     deltaV = fitness(start:end1,1);
%     flightTime = fitness(start:end1,2)/3600;


    plot(deltaV, flightTime)
    xlabel("\Delta V [m/s]")
    ylabel("Time of flight [hours]")
%     set(gca,'yscale','log')
    hold on
end
ylim([0, 600])
legend("1st gen", '5th gen', '10th gen', '20th gen', '30th gen')
title("Results seed 160, thrust 0.01-5.0 N, optimizer: IHS")

figure
for i = [1, 5, 10, 20, 30]
    start = (i-1)*nrIndividuals+1;
    end1 = i*nrIndividuals;
    deltaV = sort(fitness(start:end1,1));
    flightTime = sort(fitness(start:end1,2)/3600, 'descend');
%     deltaV = fitness(start:end1,1);
%     flightTime = fitness(start:end1,2)/3600;


    plot(deltaV, flightTime)
    xlabel("\Delta V [m/s]")
    ylabel("Time of flight [hours]")
%     set(gca,'yscale','log')
    hold on
end
ylim([50, 200])
xlim([4550, 4660])
legend("1st gen", '5th gen', '10th gen', '20th gen', '30th gen')
title("Results seed 160, thrust 0.01-5.0 N, optimizer: IHS")

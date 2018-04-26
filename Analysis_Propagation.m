addpath('C:\tudatBundle.git\tudatApplications\PropOpt_FA\SimulationOutput')

nrIndividuals = 100;
nrGen = 30;

fitness = [];
input = [];
for i = 0:(nrGen-1)
    stringfitness = ['fitness_leoGeoTransfer_0_' num2str(i) '.dat'];
    stringfitness = join(stringfitness);
    stringInput = ['population_leoGeoTransfer_0_' num2str(i) '.dat'];
    stringInput = join(stringInput);
    
    fitnessGen = dlmread(stringfitness);
    inputGen = dlmread(stringInput);
    fitness = [fitness; fitnessGen];
    input = [input; inputGen];
end
figure
for i = [1, 5, 10, 15, 20, 25, 30]
    start = i*nrGen+1;
    end1 = (i+1)*nrGen;
deltaV = sort(fitness(start:end1,1));
flightTime = sort(fitness(start:end1,2)/3600, 'descend');



plot(deltaV, flightTime)
xlabel("\Delta V [m/s]")
ylabel("Time of flight [hours]")
% set(gca,'yscale','log')
hold on
end



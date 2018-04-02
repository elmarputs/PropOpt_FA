#include <iostream>
#include <Tudat/SimulationSetup/tudatSimulationHeader.h>

using namespace tudat::simulation_setup;

int main()
{
    // Setup Spice
    tudat::spice_interface::loadStandardSpiceKernels();

    // Specify bodies to propagate
    std::vector<std::string> bodiesToPropagate;
    bodiesToPropagate.push_back("Earth");
    bodiesToPropagate.push_back("Moon");
    bodiesToPropagate.push_back("Sun");

    std::map<std::string, boost::shared_ptr<BodySettings>> bodySettings = getDefaultBodySettings(bodiesToPropagate);
    // Check if default settings are ok

    NamedBodyMap bodyMap = createBodies(bodySettings);
    bodyMap["Vehicle"] = boost::make_shared<Body>();
    bodyMap["Vehicle"]->setConstantBodyMass(5.0E3);



    std::cout << "Hello world!\n";
    return 0;
}

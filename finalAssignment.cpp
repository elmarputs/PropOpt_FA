#include <iostream>
#include <Tudat/SimulationSetup/tudatSimulationHeader.h>

using namespace tudat;
using namespace tudat::simulation_setup;

const double thrustMagnitude = 100.0; // N
const double specificImpulse = 1500.0; // sec

int main()
{
    //-----------------------
    //--SETUP ENVIRONMENT----

    // Setup Spice
    tudat::spice_interface::loadStandardSpiceKernels();

    // Specify all simulated bodies
    std::vector<std::string> bodies;
    bodies.push_back("Earth");
    //bodies.push_back("Moon");
    //bodies.push_back("Sun");

    std::map<std::string, boost::shared_ptr<BodySettings>> bodySettings = getDefaultBodySettings(bodies);
    // Check if default settings are ok

    NamedBodyMap bodyMap = createBodies(bodySettings);
    bodyMap["Vehicle"] = boost::make_shared<Body>();
    bodyMap["Vehicle"]->setConstantBodyMass(5.0E3);

    setGlobalFrameBodyEphemerides(bodyMap, "SSB", "J2000");

    //-----------------------------
    //--SETUP ACCELERATION MODEL---
    SelectedAccelerationMap accMap;
    std::vector<std::string> bodiesToProp;
    std::vector<std::string> centralBodies;

    bodiesToProp.push_back("Vehicle");
    centralBodies.push_back("Earth");

    // Specify accelerations acting on propagated vehicle
    std::map<std::string, std::vector<boost::shared_ptr<AccelerationSettings>>> accOnVehicle;
    boost::shared_ptr<ThrustEngineSettings> thrustMag = boost::make_shared<ConstantThrustEngineSettings>(thrustMagnitude, specificImpulse);
    boost::shared_ptr<ThrustDirectionGuidanceSettings> thrustDir = boost::make_shared<ThrustDirectionFromStateGuidanceSettings>("Earth", true, false);

    accOnVehicle["Earth"].push_back(boost::make_shared<AccelerationSettings>(basic_astrodynamics::central_gravity));
    accOnVehicle["Vehicle"].push_back(boost::make_shared<ThrustAccelerationSettings>(thrustDir, thrustMag));
    accMap["Vehicle"] = accOnVehicle;

    basic_astrodynamics::AccelerationMap accModelMap= createAccelerationModelsMap(bodyMap, accMap, bodiesToProp, centralBodies);

    // Set initial state
    Eigen::Vector6d systemInitialState = Eigen::Vector6d::Zero();
    systemInitialState(0) = 8.0E6;
    systemInitialState(4) = 7.5E3;

    // Define propagation termination conditions (stop after 5 days)
    boost::shared_ptr<PropagationTimeTerminationSettings> terminationSettings =
            boost::make_shared<propagators::PropagationTimeTerminationSettings>(5.0 * physical_constants::JULIAN_DAY);

    // Settings for translational propagation
    boost::shared_ptr< TranslationalStatePropagatorSettings<double>> transPropSettings =
            boost::make_shared<TranslationalStatePropagatorSettings<double>>
            (centralBodies, accModelMap, bodiesToProp, systemInitialState, terminationSettings,
              cowell);


    // Create list of propagation settings
    std::vector< boost::shared_ptr< SingleArcPropagatorSettings<double>>> propagatorSettingsVector;
    propagatorSettingsVector.push_back(transPropSettings);

    boost::shared_ptr< PropagatorSettings<double>> propagatorSettings =
            boost::make_shared< MultiTypePropagatorSettings<double>>(propagatorSettingsVector, terminationSettings);

    // Set integrator settings
    boost::shared_ptr<IntegratorSettings<>> integratorSettings =
            boost::make_shared<IntegratorSettings<>>
            (rungeKutta4, 0.0, 30.0);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////             PROPAGATE ORBIT            ////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    // Create simulation object and propagate dynamics.
    SingleArcDynamicsSimulator<> dynamicsSimulator(
                bodyMap, integratorSettings, propagatorSettings, true, false, false );


    // Output propagation data
//    std::map<double, Eigen::Matrix<double, Eigen::Dynamic, 1 >> numericalSolution =
//            dynamicsSimulator.getEquationsOfMotionNumericalSolution();

//    std::string outputSubFolder = "FA_output/";

//    // Write satellite propagation history to file.
//    input_output::writeDataMapToTextFile( numericalSolution,
//                                          "prop_output.dat",
//                                          tudat_applications::getOutputPath() + outputSubFolder,
//                                          "",
//                                          std::numeric_limits< double >::digits10,
//                                          std::numeric_limits< double >::digits10,
//                                          "," );

    return 0;
}

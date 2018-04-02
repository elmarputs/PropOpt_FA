#include "leogeotransfer.h"
#include <Tudat/SimulationSetup/tudatSimulationHeader.h>
#include "applicationOutput.h"

using namespace tudat;
using namespace tudat::simulation_setup;

namespace final_assignment
{
    LeoGeoTransfer::LeoGeoTransfer(double thrustMag, double spImp)
    {
        this->thrustMagnitude = thrustMag;
        this->specificImpulse = spImp;
        spice_interface::loadStandardSpiceKernels();
    }

    void LeoGeoTransfer::Propagate()
    {
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

        setGlobalFrameBodyEphemerides(bodyMap, "SSB", "ECLIPJ2000");

        //-----------------------------
        //--SETUP ACCELERATION MODEL---
        SelectedAccelerationMap accMap;
        std::vector<std::string> bodiesToProp;
        std::vector<std::string> centralBodies;

        bodiesToProp.push_back("Vehicle");
        centralBodies.push_back("Earth");

        // Specify accelerations acting on propagated vehicle
        std::map<std::string, std::vector<boost::shared_ptr<AccelerationSettings>>> accOnVehicle;
        boost::shared_ptr<ThrustEngineSettings> thrustMag = boost::make_shared<ConstantThrustEngineSettings>(this->thrustMagnitude, this->specificImpulse);
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
        boost::shared_ptr<propagators::PropagationTimeTerminationSettings> terminationSettings =
                boost::make_shared<propagators::PropagationTimeTerminationSettings>(5.0 * physical_constants::JULIAN_DAY);

        // Settings for translational propagation
        boost::shared_ptr<propagators::TranslationalStatePropagatorSettings<double>> transPropSettings =
                boost::make_shared<propagators::TranslationalStatePropagatorSettings<double>>
                (centralBodies, accModelMap, bodiesToProp, systemInitialState, terminationSettings,
                  propagators::cowell);


        // Create list of propagation settings
        std::vector<boost::shared_ptr<propagators::SingleArcPropagatorSettings<double>>> propagatorSettingsVector;
        propagatorSettingsVector.push_back(transPropSettings);

        boost::shared_ptr<propagators::PropagatorSettings<double>> propagatorSettings =
                boost::make_shared<propagators::MultiTypePropagatorSettings<double>>(propagatorSettingsVector, terminationSettings);

        // Set integrator settings
        boost::shared_ptr<numerical_integrators::IntegratorSettings<>> integratorSettings =
                boost::make_shared<numerical_integrators::IntegratorSettings<>>
                (numerical_integrators::rungeKutta4, 0.0, 30.0);


        // Create simulation object and propagate dynamics.
        propagators::SingleArcDynamicsSimulator<> dynamicsSimulator(
                    bodyMap, integratorSettings, propagatorSettings, true, false, false );


        // Output propagation data
        std::map<double, Eigen::Matrix<double, Eigen::Dynamic, 1 >> numericalSolution =
                dynamicsSimulator.getEquationsOfMotionNumericalSolution();

        std::string outputSubFolder = "FA_output/";

        // Write satellite propagation history to file.
        input_output::writeDataMapToTextFile( numericalSolution,
                                              "prop_output.dat",
                                              tudat_applications::getOutputPath() + outputSubFolder,
                                              "",
                                              std::numeric_limits< double >::digits10,
                                              std::numeric_limits< double >::digits10,
                                              "," );
    }
}

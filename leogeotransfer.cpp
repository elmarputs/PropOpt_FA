#include <iostream>

#include <Tudat/SimulationSetup/tudatSimulationHeader.h>
#include "applicationOutput.h"
#include "leogeotransfer.h"

using namespace tudat;
using namespace tudat::simulation_setup;
using namespace tudat::propagators;

namespace final_assignment
{
//    LeoGeoTransfer::LeoGeoTransfer(double thrustMag, double spImp)
//    {
//        this->thrustMagnitude = thrustMag;
//        this->specificImpulse = spImp;
//        spice_interface::loadStandardSpiceKernels();
//        //std::cout << "Constructor called!\n";
//    }

    // LeoGeoTransfer problem contructor
    LeoGeoTransfer::LeoGeoTransfer(const std::vector<std::vector<double> > &bounds)
    {
        problemBounds = bounds;
        spice_interface::loadStandardSpiceKernels();
    }

    std::pair<std::vector<double>, std::vector<double>> LeoGeoTransfer::get_bounds() const
    {
        return {problemBounds.at( 0 ), problemBounds.at( 1 ) };
    }

    // Function calculating fitness: returns delta V and trip time (called by Pagmo)
    std::vector<double> LeoGeoTransfer::fitness(const std::vector<double> &xVec) const
    {
        // Create return (i.e. fitness) vector
        std::vector<double> fitnessVector;

        // Specify all simulated bodies
        std::vector<std::string> bodies;
        bodies.push_back("Earth");
        //bodies.push_back("Moon");
        //bodies.push_back("Sun");

        std::map<std::string, boost::shared_ptr<BodySettings>> bodySettings = getDefaultBodySettings(bodies);
        // Check if default settings are ok

        NamedBodyMap bodyMap = createBodies(bodySettings);
        bodyMap["Vehicle"] = boost::make_shared<Body>();
        bodyMap["Vehicle"]->setConstantBodyMass(500);

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
//        boost::shared_ptr<MeeCostateBasedThrustDirectionSettings> meeThrustDir = boost::make_shared<MeeCostateBasedThrustDirectionSettings>(
//                    "Vehicle",
//                    "Earth",
//                    costateFunction);

        accOnVehicle["Earth"].push_back(boost::make_shared<AccelerationSettings>(basic_astrodynamics::central_gravity));
        accOnVehicle["Vehicle"].push_back(boost::make_shared<ThrustAccelerationSettings>(thrustDir, thrustMag));
        accMap["Vehicle"] = accOnVehicle;

        basic_astrodynamics::AccelerationMap accModelMap= createAccelerationModelsMap(bodyMap, accMap, bodiesToProp, centralBodies);

        // Set initial state
        Eigen::Vector6d systemInitialState = Eigen::Vector6d::Zero();
        systemInitialState(0) = 6.871E6;
        systemInitialState(4) = 7.616556E3;

        // Create dependent variable for use in stop condition
        boost::shared_ptr<SingleDependentVariableSaveSettings> depVariable =
                boost::make_shared<SingleDependentVariableSaveSettings>(relative_distance_dependent_variable, "Vehicle", "Earth");
        // Set propagation stop condition (alt > geo alt)
        boost::shared_ptr<PropagationDependentVariableTerminationSettings> terminationSettings =
                boost::make_shared<PropagationDependentVariableTerminationSettings>(depVariable, geoR, false, false);

        // Settings for translational propagation
        boost::shared_ptr<TranslationalStatePropagatorSettings<double>> transPropSettings =
                boost::make_shared<TranslationalStatePropagatorSettings<double>>
                (centralBodies, accModelMap, bodiesToProp, systemInitialState, terminationSettings,
                  propagators::cowell);


        // Create list of propagation settings
        std::vector<boost::shared_ptr<propagators::SingleArcPropagatorSettings<double>>> propagatorSettingsVector;
        propagatorSettingsVector.push_back(transPropSettings);
        //propagatorSettingsVector.push_back(massPropSettings); // ENABLE IF MASS IS TO BE PROPAGATED USING THRUST MODEL

        boost::shared_ptr<PropagatorSettings<double>> propagatorSettings =
                boost::make_shared<MultiTypePropagatorSettings<double>>(propagatorSettingsVector, terminationSettings);

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

        std::map<double, Eigen::Matrix<double, Eigen::Dynamic, 1>>::const_iterator iter;
        iter = numericalSolution.end();
        std::cout << iter->first;

//        std::string outputSubFolder = "FA_output/";

        // Write satellite propagation history to file.
//        input_output::writeDataMapToTextFile( numericalSolution,
//                                              "prop_output.dat",
//                                              tudat_applications::getOutputPath() + outputSubFolder,
//                                              "",
//                                              std::numeric_limits< double >::digits10,
//                                              std::numeric_limits< double >::digits10,
//                                              "," );
        double deltaV = 0;
        double tripTime = 0;

        fitnessVector.push_back(deltaV);
        fitnessVector.push_back(tripTime);

        return fitnessVector;
    }
}

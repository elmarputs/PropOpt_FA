
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include "pagmo/pagmo.hpp"
#include "pagmo/algorithms/pso.hpp"

#include "leogeotransfer.h"
#include "applicationOutput.h"
//#include "getAlgorithm.h"

using namespace final_assignment;
using namespace tudat_applications;
using namespace pagmo;

int main( )
{
    std::cout << "Initializing program...\n";

    //Set seed for reproducible results
    pagmo::random_device::set_seed( 123 );

    // We have two decision variables each with a lower and upper bound, create a vector of vectors that will contain these.
    std::vector< std::vector< double > > bounds( 2, std::vector< double >( 2, 0.0 ) );

    // Define bounds: Search between thrust magnitude of 1 and 10 N and specific impulse between 3000 and 4000
    bounds[ 0 ][ 0 ] = 20; // Thrust
    bounds[ 1 ][ 0 ] = 100;
    bounds[ 0 ][ 1 ] = 3500; // Isp
    bounds[ 1 ][ 1 ] = 4000;

    std::cout << "Creating problem...\n";

    // Create object to compute the problem fitness
    problem prob{LeoGeoTransfer( bounds )};

    // Perform grid search
    std::cout << "Performing grid search...\n";
    createGridSearch( prob, bounds, { 10, 10 }, "porkchopEarthMars" );

    // Perform optimization with 1 different optimizers
    for( int j = 0; j < 1; j++ )
    {
        // Retrieve algorothm
        algorithm algo{moead()};

        // Create an island with 1024 individuals
        island isl{algo, prob, 1024};

        // Evolve for 100 generations
        for( int i = 0 ; i < 100; i++ )
        {
            isl.evolve();
            while( isl.status()!=pagmo::evolve_status::idle )
                std::cout << "Waiting...\n";
                isl.wait();

            // Write current iteration results to file
            //printPopulationToFile( isl.get_population( ).get_x( ), "earthMarsLambert_" + std::to_string( j ) + "_" + std::to_string( i ) , false );
            //printPopulationToFile( isl.get_population( ).get_f( ), "earthMarsLambert_" + std::to_string( j ) + "_" + std::to_string( i ) , true );

            std::cout<<i<<" "<<j<<std::endl;
        }
    }


    return 0;
}

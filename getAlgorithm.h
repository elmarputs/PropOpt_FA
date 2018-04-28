#include <iostream>
#include "pagmo/algorithms/nsga2.hpp"
#include "pagmo/algorithms/moead.hpp"
#include "pagmo/algorithms/ihs.hpp"

pagmo::algorithm getMultiObjectiveAlgorithm( const int index )
{
    switch( index )
    {
    case 0:
    {
        pagmo::algorithm algo{ pagmo::nsga2( ) };
        return algo;
        break;
    }
    case 1:
    {
        pagmo::algorithm algo{ pagmo::moead( ) };
        return algo;
        break;
    }
    case 2:
    {
        pagmo::algorithm algo{ pagmo::ihs( ) };
        return algo;
        break;
    }
    default:
    {
        throw std::runtime_error( "Error, multi-objective pagmo algorithm " + std::to_string( index ) + " was not found." );
    }
    }
}

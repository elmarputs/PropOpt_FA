#ifndef LEOGEOTRANSFER_H
#define LEOGEOTRANSFER_H

#include <vector>

namespace final_assignment
{
    class LeoGeoTransfer
    {
        public:
            const double geoAlt = 35786.0e3; // m
            LeoGeoTransfer(); // Default constructor for Pagmo compatibility
            LeoGeoTransfer(const std::vector<std::vector<double>> &bounds);
            //LeoGeoTransfer(double thrustMag, double spImp);
            //void Propagate();
            std::vector<double> fitness(const std::vector<double> &x) const;

        private:
            double thrustMagnitude;
            double specificImpulse;
    };
}

#endif // LEOGEOTRANSFER_H

#ifndef LEOGEOTRANSFER_H
#define LEOGEOTRANSFER_H

#include <vector>

namespace final_assignment
{
    class LeoGeoTransfer
    {
        public:
            const double geoR = 42164.0e3; // m
            LeoGeoTransfer(); // Default constructor for Pagmo compatibility
            LeoGeoTransfer(const std::vector<std::vector<double>> &bounds);
            //LeoGeoTransfer(double thrustMag, double spImp);
            //void Propagate();
            std::vector<double> fitness(const std::vector<double> &x) const;
            std::pair<std::vector<double>, std::vector<double>> get_bounds() const;

        private:
            std::vector<std::vector<double>> problemBounds;
            double thrustMagnitude;
            double specificImpulse;
    };
}

#endif // LEOGEOTRANSFER_H

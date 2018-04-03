#ifndef LEOGEOTRANSFER_H
#define LEOGEOTRANSFER_H

namespace final_assignment
{
    class LeoGeoTransfer
    {
        public:
            const double geoAlt = 35786.0e3; // m
            LeoGeoTransfer(double thrustMag, double spImp);
            ~LeoGeoTransfer();
            void Propagate();
        private:
            double thrustMagnitude;
            double specificImpulse;
    };
}

#endif // LEOGEOTRANSFER_H

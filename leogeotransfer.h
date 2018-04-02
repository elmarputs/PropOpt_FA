#ifndef LEOGEOTRANSFER_H
#define LEOGEOTRANSFER_H

namespace final_assignment
{
    class LeoGeoTransfer
    {
        public:
            LeoGeoTransfer(double thrustMag, double spImp);
            void Propagate();
        private:
            double thrustMagnitude;
            double specificImpulse;
    };
}

#endif // LEOGEOTRANSFER_H

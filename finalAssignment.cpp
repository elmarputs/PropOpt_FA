#include "leogeotransfer.h"

int main()
{
    final_assignment::LeoGeoTransfer transfer = final_assignment::LeoGeoTransfer(10.0, 1500.0);
    transfer.Propagate();

    return 0;
}

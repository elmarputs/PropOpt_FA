#include "leogeotransfer.h"
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

int main()
{
    //final_assignment::LeoGeoTransfer transfer = final_assignment::LeoGeoTransfer(10.0, 1500.0);
    //transfer.Propagate();

    //final_assignment::LeoGeoTransfer* pTransfer = new final_assignment::LeoGeoTransfer(10.0, 1500.0);
    //delete pTransfer;

    boost::shared_ptr<final_assignment::LeoGeoTransfer> pTransfer = boost::make_shared<final_assignment::LeoGeoTransfer>(10.0, 1500.0);
    pTransfer->Propagate();

    return 0;
}

package slaw.simulations;

import slaw.contract.ISlawTripManager;
import slaw.contract.ISpeedManager;
import slaw.contract.IPauseTimeManager;
import slaw.walker.Walker;
import slaw.app.TripObserver;

network SLAWTripObserver
{
    parameters:
        walker.mobilityType = default("SlawMobility");
        pauseTimeModel.pauseTimeModelType = default("BParetoPauseTimeModel");
        speedModel.speedModelType = default("NormalSpeedModel");
        string slawModelType;
        string pauseTimeModelType;
        string speedModelType;
        slaw.numOfWalker = default(1);
        @display("bgb=10000,10000;bgg=1000,10,grey50");
    submodules:
        observer: TripObserver;
        slawtripmanager: <slawModelType> like ISlawTripManager;
        walker: Walker;
        pauseTimeModel: <pauseTimeModelType> like IPauseTimeManager;
        speedModel: <speedModelType> like ISpeedManager;
}

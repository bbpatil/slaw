package slaw.simulations;
import slaw.contract.IObserver;
import slaw.contract.IPauseTimeManager;
import slaw.contract.ISpeedManager;
import slaw.contract.ISlawTripManager;
import slaw.walker.Walker;
import observer.InterContactTimeObserver;
import extra.PauseTimeModel;
import extra.SpeedModel;

network SLAWICTExperiment
{
    parameters:
        @display("bgb=2000,2000;bgg=100,2,grey50");
        int numHosts;
        int sampleSize;
        double width @unit(m); //length of the x-axis
        double height @unit(m); //length of the y-axis
        double coverageRadius @unit(m); //coverage radius
        double minInterContactTime @unit(s);
        string slawModelType = default("SlawMobility");
        string observerType;
        string pauseTimeModelType = default("BParetoPauseTimeModel");
        string speedModelType = default("NormalSpeedModel");
        slaw.numOfWalker = numHosts;
        observer.sampleSize = sampleSize;
        observer.networlSize = hostNum;
        observer.x_length = default(2000) @unit(m);
        observer.y_length = default(2000) @unit(m);
        observer.radius = coverageRadius;
        observer.minICT = minInterContactTime;
    submodules:
        slawtripmanager: <slawModelType> like ISlawTripManager;
        pauseTimeModel: <pauseTimeModelType> like IPauseTimeManager;
        speedModel: <speedModelType> like ISpeedManager;
        walker[numHosts]: Walker {
            mobilityType = "SlawMobility";
        }
        observer: <observerType> like IObserver {
            @display("p=480,400");
        }
}

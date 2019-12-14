package slaw.simulations;

import slaw.contract.IObserver;
import slaw.walker.Walker;
import slaw.tripmanager.TripManager;

network SLAWGlobalConnectivityEvaluation
{
    parameters:
        int numHosts;
        int observations;
        double width @unit(m); //length of the x-axis
        double height @unit(m); //length of the y-axis
        double coverageRadius @unit(m); //coverage radius
        string walkerModelType;
        string pauseTimeModelType;
        string speedModelType;
        string filename;
        tripmanager.walkerModelType = walkerModelType;
        tripmanager.walkermodel.numOfWalker = numHosts;
        tripmanager.pauseTimeModelType = pauseTimeModelType;
        tripmanager.speedModelType = speedModelType;
        @display("bgb=1000,1000;bgg=100,2,grey50");
    submodules:
        tripmanager: TripManager;
        walker[numHosts]: Walker {
            mobilityType = "SlawMobility";
        }
        observer: <"GlobalConnectivityObserver"> like IObserver {
            filename = filename;
            numOfNodes = numHosts;
            observations = observations;
            x_length = width;
            y_length = height;
            radius = coverageRadius;
            @display("p=480,400");
        }
}

package slaw.simulations;

import slaw.contract.IObserver;
import slaw.walker.Walker;
import slaw.tripmanager.TripManager;

network SLAWInterContactEvaluation
{
    parameters:
        int numHosts;
        int observations;
        double width @unit(m); //length of the x-axis
        double height @unit(m); //length of the y-axis
        double coverageRadius @unit(m); //coverage radius
        double minInterContactTime @unit(s);
        double minLifetime @unit(s);
        string walkerModelType;
        string pauseTimeModelType;
        string speedModelType;
        tripmanager.walkerModelType = walkerModelType;
        tripmanager.walkermodel.numOfWalker = numHosts;
        tripmanager.pauseTimeModelType = pauseTimeModelType;
        tripmanager.speedModelType = speedModelType;        
        string observerType = "InterContactTimeObserver";
        @display("bgb=2000,2000;bgg=100,2,grey50");
    submodules:
        tripmanager: TripManager;
        walker[numHosts]: Walker {
            mobilityType = "SlawMobility";
        }
        observer: <observerType> like IObserver {
            node_number = numHosts;
            observations = observations;
            x_length = width;
            y_length = height;
            radius = coverageRadius;
            minICT = minInterContactTime;
            minLLT = minLifetime;
            @display("p=480,400");
        }
}

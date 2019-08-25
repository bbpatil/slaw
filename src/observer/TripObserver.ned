package slaw.app;

import slaw.contract.IObserver;

simple TripObserver like IObserver {
    @display("i=block/cogwheel");
    int sampleSize;
    string observerType = "TripObserver";
    @signal[nextWaypointX](type=double);
    @statistic[nextWaypointX](record=vector);
    @signal[nextWaypointY](type=double);
    @statistic[nextWaypointY](record=vector);
    @signal[tripSize_stat](type=long);
    @statistic[tripSize_stat](record=histogram,vector);
}
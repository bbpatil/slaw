package slaw.app;

simple ContactTimeObserver extends PositionObserver {
    parameters:
        @class(ContactTimeObserver);
        int sampleSize;    //link lifetime observations
        int numOfNode;
        double minLLT @unit(s);
        @signal[linkLifeTime](type=double);
        @statistic[linkLifeTime](record=stats,vector; unit=s);
}
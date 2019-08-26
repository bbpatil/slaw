package slaw.app;

simple InterContactTimeObserver extends PositionObserver {
    parameters:
        @class(InterContactObserver);
        int sampleSize;   //number of intercontact time observations
        double minICT @unit(s);
        int networkSize;
        @signal[interContactTime](type=double);
        @statistic[interContactTime](record=stats,vector; unit=s;);
}
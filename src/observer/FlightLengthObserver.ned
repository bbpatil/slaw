package slaw.app;

simple FlightLengthObserver
{
    int sampleSize;
    bool classifyFlight = default(false);
    @display("i=device/receiverdish;is=n");
    @signal[flight_stat](type=double);
    @statistic[flight_stat](record=histogram,vector; unit=m);
    @signal[intraFlightLength_stat](type=double);
    @statistic[intraFlightLength_stat](record=histogram,vector; unit=m);
    @signal[interFlightLength_stat](type=double);
    @statistic[interFlightLength_stat](record=histogram,vector; unit=m);
    @signal[remoteRandomArea_stat](type=bool);
    @statistic[remoteRandomArea_stat](record=vector);
}

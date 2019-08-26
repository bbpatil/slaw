package slaw.app;

import slaw.contract.IObserver;

simple PositionObserver like Iobserver {
    @display("i=block/cogwheel");
    int sampleSize;
    int networkSize;
    double radius @unit(m);
    double x_length @unit(m);
    double y_length @unit(m);
}
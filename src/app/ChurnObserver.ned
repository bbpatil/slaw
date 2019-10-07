//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 
package slaw.app;

simple ChurnObserver extends PositionObserver {
    parameters:
        @class(ChurnObserver);
        int areaID;
        double windowSize @unit(s) = default(600s);
        @signal[sojournTime](type=double);
        @statistic[sojournTime](record=stats,vector; unit=s);
        @signal[departureCounter](type=int);
        @statistic[departureCounter](record=stats,vector);
        @signal[arrivalCounter](type=int);
        @statistic[arrivalCounter](record=stats,vector);
        @signal[membershipCounter](type=int);
        @statistic[membershipCounter](record=stats, vector);
}

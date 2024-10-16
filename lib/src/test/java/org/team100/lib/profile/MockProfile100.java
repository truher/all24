package org.team100.lib.profile;

import org.team100.lib.state.State100;

public class MockProfile100 implements Profile100 {
    State100 result;
    double eta;
    int count = 0;

    @Override
    public State100 calculate(double dt, State100 initial, State100 goal) {
        count++;
        return result;
    }

    @Override
    public ResultWithETA calculateWithETA(double dt, State100 initial, State100 goal) {
        return new ResultWithETA(result, eta);
    }

}

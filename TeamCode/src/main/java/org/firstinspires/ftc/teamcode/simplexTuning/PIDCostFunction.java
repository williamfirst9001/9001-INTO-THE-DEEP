package org.firstinspires.ftc.teamcode.simplexTuning;

import static org.firstinspires.ftc.teamcode.constants.autoGetPoints.*;

import org.apache.commons.math3.analysis.MultivariateFunction;

public class PIDCostFunction implements MultivariateFunction {
    private double setpoint;
    private double measuredValue;
    private double dt;

    public PIDCostFunction(double setpoint, double measuredValue, double dt) {
        this.setpoint = setpoint;
        this.measuredValue = measuredValue;
        this.dt = dt;
    }

    @Override
    public double value(double[] pidParams) {
        PIDController pid = new PIDController(pidParams[0], pidParams[1], pidParams[2]);
        double cost = 0.0;
        double measuredValue = 0.0;


            double controlSignal = pid.compute(setpoint, measuredValue);
            // Simulate the system's response to the control signal here.
            // For simplicity, assume the process responds directly to the control signal.

            cost += Math.pow(setpoint - controlSignal, 2); // Sum of squared errors


        return cost; // We want to minimize this cost
    }
}

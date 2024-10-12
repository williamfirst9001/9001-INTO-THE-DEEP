package org.firstinspires.ftc.teamcode.simplexTuning;

import static org.firstinspires.ftc.teamcode.constants.autoGetPoints.*;

import org.apache.commons.math3.optim.MaxEval;
import org.apache.commons.math3.optim.PointValuePair;
import org.apache.commons.math3.optim.SimpleBounds;
import org.apache.commons.math3.optim.nonlinear.scalar.GoalType;
import org.apache.commons.math3.optim.nonlinear.scalar.MultivariateFunctionMappingAdapter;
import org.apache.commons.math3.optim.nonlinear.scalar.MultivariateOptimizer;
import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.PowellOptimizer;
import org.firstinspires.ftc.teamcode.storage;


public class PIDController {
    private double Kp, Ki, Kd;
    private double previousError = 0.0;
    private double integral = 0.0;
    private double[] initialGuess;
    private double setPoint =0;

    public PIDController(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        initialGuess = new double[]{Kp,Ki,Kd};
    }

    public double compute(double setpoint, double measuredValue) {
        double dt = .02;
        double error = setpoint - measuredValue;
        integral += error * dt;
        double derivative = (error - previousError) / dt;
        double output = Kp * error + Ki * integral + Kd * derivative;
        previousError = error;
        this.setPoint = setpoint;
        return output;
    }


public void setPID(double[] PID){
        this.Kp = PID[0];
        this.Ki = PID[1];
        this.Kd = PID[2];
}
public double getSetPoint(){
        return this.setPoint;
}



        public double[] getVals(double setpoint,int motorPos) {
            double measuredValues = motorPos; // Simulated response
            double dt = 0.02; // Time step
            this.setPoint = setpoint;

            PIDCostFunction costFunction = new PIDCostFunction(setpoint, measuredValues, dt);
            MultivariateOptimizer optimizer = new PowellOptimizer(1e-4, 1e-4); // Set a max of 1000 evaluations

            MultivariateFunctionMappingAdapter mappingAdapter = new MultivariateFunctionMappingAdapter(
                    costFunction,
                    new double[] {0, 0, 0},
                    new double[] {10, 10, 10}
            );

            PointValuePair result = optimizer.optimize(
                    new org.apache.commons.math3.optim.nonlinear.scalar.ObjectiveFunction(mappingAdapter),
                    GoalType.MINIMIZE,
                    new org.apache.commons.math3.optim.InitialGuess(initialGuess),
                    new MaxEval(Integer.MAX_VALUE)
            );

            storage.elevatorPID = result.getPoint();

            return result.getPoint();
        }
    }



package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.globals;
import org.firstinspires.ftc.teamcode.robotHardware;

import java.util.List;

public class elevator extends SubsystemBase  {

    private PIDController elevatorPID = new PIDController(constants.armConstants.middle.P, constants.armConstants.middle.I, constants.armConstants.middle.D);

    private PIDController pivotPID = new PIDController(constants.pivotConstants.P, constants.pivotConstants.I, constants.pivotConstants.D);
    private robotHardware robot = robotHardware.getInstance();
    private boolean pivotHeld = false;
    private boolean armHeld = false;
    private boolean armTest = false;
    private double lastPivPoint = 0;
    private double lastArmPoint = 0;
    private double elevatorPower;
    private double pivotPower;
    private double elevatorPoint;
    private double pivotPoint;
    private boolean runThread = true;

    private enum armState {
        UPUP,
        UPDOWN,
        DOWNDOWN,
        DOWNUP,
        MANUAL,
        AUTO,
        HOLD
    }
    //arm move state
    armState state = armState.HOLD;
    //arm set state
    public static globals.armVal armVal = globals.armVal.STOW;




    /** check if arm is done**/
    public boolean isDone() {
        return (armDone() && pivotDone());
    }


    /** set slide setpoint **/
    public void setArmPoint(double point) {
        elevatorPID.setSetPoint(point);
    }
    /** set pivot setpoint **/
    public void setPivotPoint(double point) {
        pivotPID.setSetPoint(point);
    }

    /** checks if pivot done **/
    public boolean pivotDone() {
        return Math.abs(robot.pivotMotor.getCurrentPosition() - pivotPoint) < 20&&robot.pivotMotor.getVelocity()<30;
    }
    /** checks if slide is done **/
    public boolean armDone() {
        return Math.abs(robot.eMotors.getPosition() - elevatorPoint) < 30&&robot.eMotors.getVelocity()<30;
    }
/** sets setpoint of arm and pivot with 2 doubles **/
    public void setSetPoint(double e, double p) {
        elevatorPID.setSetPoint(e);
        pivotPID.setSetPoint(p);
    }
/** sets setpoints with list **/
    public void setSetPoint(@NonNull List<Double> vals) {
        elevatorPID.setSetPoint(vals.get(0));
        pivotPID.setSetPoint(vals.get(1));
    }


    public double getPivotSetPoint() {
        return pivotPoint;
    }

    public double getArmSetPoint() {
        return elevatorPoint;
    }

    public void endThread() {
        runThread = false;
    }

    public void startThread() {
        runThread = true;
    }



    public void run() {

       // while (runThread) {
            //get new motor values
            elevatorPower = elevatorPID.calculate(robot.eMotors.getPosition());
            pivotPower = pivotPID.calculate(robot.pivotMotor.getCurrentPosition());
            elevatorPoint = elevatorPID.getSetPoint();
            pivotPoint = pivotPID.getSetPoint();


            //get new pivot states
            // checks if arm points have changed
            if (lastPivPoint != pivotPoint || lastArmPoint != elevatorPoint) {
                //pivot down elevator in
                if (pivotPower < -.05 && elevatorPower < -.05) {
                    state = armState.DOWNDOWN;
                }
                //pivot going up elevator going out
                if (pivotPower > .05 && elevatorPower > .05) {
                    state = armState.UPUP;
                }
                //pivot going up elevator going in
                if (pivotPower > .1 && elevatorPower < -.1) {
                    state = armState.DOWNUP;
                }
                //pivot going down elevator going out
                if (pivotPower < -.1 && elevatorPower > .1) {
                    state = armState.UPDOWN;
                }
                //arm not moving
                if (pivotPower < .1 && pivotPower > -.1
                        && elevatorPower < .1 && elevatorPower > -.1) {
                    state = armState.HOLD;


                }
            }
            switch (state) {
                case UPUP:
                    //move pivot then elevator
                    setPivotPower(pivotPower);
                    if (pivotDone()) {
                        setElevatorPower(elevatorPower);
                    }
                    break;

                    //move elevator then pivot
                case DOWNDOWN:
                    setElevatorPower(elevatorPower);
                    if (armDone()) {

                        setPivotPower(pivotPower);
                    }
                    break;
                    //move elevator then pivot
                case UPDOWN:
                    setElevatorPower(elevatorPower);
                    if (armDone()) {
                        setPivotPower(pivotPower);
                    }
                    break;
                case DOWNUP:
                    setPivotPower(pivotPower);
                    setElevatorPower(elevatorPower);
                    //move both

                    //move both
                case HOLD:
                    setPivotPower(pivotPower);
                    setElevatorPower(elevatorPower);
                    break;

            }
            //checks if elevator setpoint is 0
            if (elevatorPoint == 0) {
                //resets encoder if limit switch is pressed
                if (robot.armSwitch.isPressed() && !armHeld) {
                    robot.eMotors.resetEncoder();
                    robot.eMotors.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    armHeld = true;
                }
                if (!robot.armSwitch.isPressed()) {
                    armHeld = false;
                }
            }
            //checks if pivot point is 0
            if (pivotPoint == 0) {
                //reset if point is 0
                if (robot.pivotLimit.isPressed()) {
                    robot.pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    pivotHeld = true;
                }
                if (!robot.pivotLimit.isPressed()) {
                    pivotHeld = false;
                }
                //disable pivot motor if its stowed
                if (pivotPoint == 0 && robot.pivotLimit.isPressed()) {
                    robot.pivotMotor.setMotorDisable();
                } else {
                    robot.pivotMotor.setMotorEnable();
                }
            }
            if (elevatorPoint == 0 && robot.armSwitch.isPressed()) {
                robot.eMotors.disable();
            } else {
                robot.eMotors.enable();
            }
            //update last points
            lastArmPoint = elevatorPoint;
            lastPivPoint = pivotPoint;
    }

    public armState getState(){
        return state;
    }
    private void setElevatorPower(double p){
        robot.eMotors.setPower(p*12.0/robot.voltageSensor.getVoltage());
    }
    private void setPivotPower(double p){
        robot.pivotMotor.setPower(p*12.0/robot.voltageSensor.getVoltage());
    }


    /**
     * PID vals, setpoints,current pos
     **/
    public void setArmVal(globals.armVal val) {
        armVal = val;
    }

    public globals.armVal getArmVal() {
        return armVal;
    }
/** returns all useful motor values **/
    public double[] getAllVals() {
        return new double[]{
                elevatorPID.calculate(robot.eMotors.getPosition()),
                pivotPID.calculate(robot.pivotMotor.getCurrentPosition()),
                elevatorPID.getSetPoint(),
                pivotPID.getSetPoint(),
                robot.eMotors.getPosition(),
                robot.pivotMotor.getCurrentPosition()
        };
    }

    public double pivEncToDeg(double val) {
        return val * constants.pivotConstants.degPerCount - 30;
    }

    public void setPivotGains(double P, double I, double D) {
        pivotPID.setPID(P, I, D);
    }

}
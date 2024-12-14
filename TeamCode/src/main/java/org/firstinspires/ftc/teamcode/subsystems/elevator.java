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
        UP,
        DOWN,
        HOLD
    }
    private enum pivState {
        UP,
        DOWN,
        HOLD
    }
    //arm move state
    armState elevatorState = armState.UP;
    pivState pivotState = pivState.UP;
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



    public void update() {

       // while (runThread) {
            //get new motor values
            elevatorPower = elevatorPID.calculate(robot.eMotors.getPosition());
            pivotPower = pivotPID.calculate(robot.pivotMotor.getCurrentPosition());
            elevatorPoint = elevatorPID.getSetPoint();
            pivotPoint = pivotPID.getSetPoint();


            //get new pivot states
            // checks if arm points have changed

                //pivot down elevator in
                if(pivotPower >0.1){
                    pivotState = pivState.UP;
                } else if(pivotPower<-.1){
                    pivotState = pivState.DOWN;
                } else {
                    pivotState = pivState.HOLD;
                }
        if(elevatorPower >0.1){
            elevatorState = armState.UP;
        } else if(elevatorPower<-.1){
            elevatorState = armState.DOWN;
        } else {
            elevatorState = armState.HOLD;
        }
        if(elevatorState ==armState.UP && pivotState ==pivState.UP){
            setPivotPower(pivotPower);
            if(pivotDone()){
                setElevatorPower(elevatorPower);
            }
        } else if(elevatorState == armState.UP && pivotState ==pivState.DOWN){
            setPivotPower(pivotPower);
            if(pivotDone()){
                setElevatorPower(elevatorPower);
            }
        } else if(elevatorState == armState.UP && pivotState ==pivState.HOLD){
            setElevatorPower(elevatorPower);
            setPivotPower(pivotPower);
        } else if(elevatorState == armState.DOWN && pivotState ==pivState.UP){
            setElevatorPower(elevatorPower);
            setPivotPower(pivotPower);
        } else if(elevatorState == armState.DOWN && pivotState == pivState.DOWN){
            setElevatorPower(elevatorPower);
            if(armDone()){
                setPivotPower(pivotPower);
            }
        } else if(elevatorState == armState.DOWN && pivotState ==pivState.HOLD){
            setElevatorPower(elevatorPower);
            setPivotPower(pivotPower);
        } else if(elevatorState == armState.HOLD &&pivotState == pivState.UP){
            setElevatorPower(elevatorPower);
            setPivotPower(pivotPower);
        } else if(elevatorState == armState.HOLD && pivotState == pivState.DOWN){
            setElevatorPower(elevatorPower);
            setPivotPower(pivotPower);
        } else if(elevatorState == armState.HOLD && pivotState == pivState.HOLD){
            setElevatorPower(elevatorPower);
            setPivotPower(pivotPower);
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
        return elevatorState;
    }
    private void setElevatorPower(double p){
        robot.eMotors.setPower(Math.abs(p) > .07 ?  p*12.0/robot.voltageSensor.getVoltage() : 0);
    }
    private void setPivotPower(double p){
        robot.pivotMotor.setPower(Math.abs(p) > .07 ? p*12.0/robot.voltageSensor.getVoltage() : 0);
    }
    public double getElevatorPower(){
        return elevatorPID.calculate(robot.eMotors.getPosition());
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
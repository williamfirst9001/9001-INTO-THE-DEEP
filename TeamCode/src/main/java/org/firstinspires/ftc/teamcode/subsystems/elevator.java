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

    private enum armMove {
        UPUP,
        UPDOWN,
        DOWNDOWN,
        DOWNUP,
        HOLD,
        MANUAL
    }

    //arm move state
    armMove state = armMove.UPUP;

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


        //pivot coming down elevator going in
        if (lastPivPoint != pivotPoint || lastArmPoint != elevatorPoint) {
            if (pivotPower < -.1 && elevatorPower < -.1) {
                state = armMove.DOWNDOWN;
            }
            //pivot going up elevator going out
            if (pivotPower > .1 && elevatorPower > .1) {
                state = armMove.UPUP;
            }
            //pivot going up elevator going in
            if (pivotPower > .1 && elevatorPower < -.1) {
                state = armMove.DOWNUP;
            }
            if(pivotPower<.1&&pivotPower>-.1
                    && elevatorPower<.1 && elevatorPower>-.1){
                state = armMove.HOLD;
            }
        }

        if (state == armMove.DOWNDOWN) {
            setElevatorPower(elevatorPower);
            if (armDone()) {

                setPivotPower(pivotPower);
            }
        }
        if (state == armMove.UPUP) {
            setPivotPower(pivotPower);

            if (pivotDone()) {
                setElevatorPower(elevatorPower);
            } else {
                robot.eMotors.setPower(0);
            }
        }
        if(globals.manualArm){
            setElevatorPower(elevatorPower);
            setPivotPower(pivotPower);
        }



        if (robot.armSwitch.isPressed() && !armHeld) {
            robot.eMotors.resetEncoder();
            robot.eMotors.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armHeld = true;
        }
        if (!robot.armSwitch.isPressed()) {
            armHeld = false;
        }
        if (robot.pivotLimit.isPressed() &&!pivotHeld) {
            robot.pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            pivotHeld = true;
        }
        if (!robot.pivotLimit.isPressed()) {
            pivotHeld = false;
        }
        if (pivotPID.getSetPoint() == 0 && robot.pivotLimit.isPressed()) {
            robot.pivotMotor.setMotorDisable();
        } else {
            robot.pivotMotor.setMotorEnable();
        }
        if (elevatorPID.getSetPoint() == 0 && robot.armSwitch.isPressed() && Math.abs(elevatorPID.getSetPoint()-robot.eMotors.getPosition())<50) {
            robot.eMotors.disable();
        } else {
            robot.eMotors.enable();
        }

        if (pivotPID.getSetPoint() >= robot.pivotMotor.getCurrentPosition() && robot.pivotMotor.getVelocity() < 50) {
            pivotPID.setI(0);
        } else {
            pivotPID.setI(constants.pivotConstants.I);
        }
        if (!armDone() && robot.eMotors.getVelocity() < 50) {
            elevatorPID.setI(0);
        } else {
            elevatorPID.setI(constants.armConstants.middle.I);
        }

        lastArmPoint = elevatorPID.getSetPoint();
        lastPivPoint = pivotPID.getSetPoint();
    }

    public armMove getState(){
        return state;
    }
    private void setElevatorPower(double p){
        robot.eMotors.setPower(p*12.0/robot.voltageSensor.getVoltage());
    }
    private void setPivotPower(double p){
        robot.pivotMotor.setPower(p*12.0/robot.voltageSensor.getVoltage());
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
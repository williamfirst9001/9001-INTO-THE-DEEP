package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.globals;
import org.firstinspires.ftc.teamcode.robotHardware;

import java.util.List;

public class elevator extends SubsystemBase {

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

    private enum armMove {
        UPUP,
        UPDOWN,
        DOWNDOWN,
        DOWNUP,
        HOLD,
        BASE
    }

    armMove state = armMove.UPUP;
    public static globals.armVal armVal= globals.armVal.STOW;




    public elevator() {

        elevatorPID.setTolerance(150);
        pivotPID.setTolerance(2);

    }


    public boolean isDone() {
        return (armDone() && pivotDone());
    }

    public double getArmPose() {
        return robot.eMotors.getPosition();
    }
    public void setArmPoint(double point){
        elevatorPID.setSetPoint(point);
    }
    public void setPivotPoint(double point){
        pivotPID.setSetPoint(point);
    }


    public boolean pivotDone() {
        return Math.abs(robot.pivotMotor.getCurrentPosition() - pivotPID.getSetPoint()) < 20 || (pivotPID.getSetPoint()==0 && robot.pivotLimit.isPressed());
    }

    public boolean armDone() {
        return Math.abs(robot.eMotors.getPosition() - elevatorPID.getSetPoint()) < 30 ||(elevatorPID.getSetPoint()==0&&robot.armSwitch.isPressed());
    }

    public void setSetPoint(double e, double p) {
        elevatorPID.setSetPoint(e);
        pivotPID.setSetPoint(p);
    }

    public void setSetPoint(List<Double> vals) {
        elevatorPID.setSetPoint(vals.get(0));
        pivotPID.setSetPoint(vals.get(1));
    }


    public double getPivotSetPoint() {
        return pivotPID.getSetPoint();
    }

    public double getArmSetPoint() {
        return elevatorPID.getSetPoint();
    }
    public void initCommand(){
        elevatorPower = getAllVals()[0];
        pivotPower = getAllVals()[1];
        elevatorPoint = getAllVals()[2];
        pivotPoint = getAllVals()[3];
        //pivot coming down elevator going in

        if (lastPivPoint != pivotPoint || lastArmPoint != elevatorPoint) {
            if (pivotPower < -.1 && elevatorPower < -.1) {
                state = armMove.DOWNDOWN;
            } else
                //pivot going up elevator going out
                if (pivotPower > .1 && elevatorPower > .1) {
                    state = armMove.UPUP;
                } else
                    //pivot going up elevator going in
                    if (pivotPower > .1 && elevatorPower < -.1) {
                        state = armMove.DOWNUP;
                    } else
                    if(pivotPower < -.1 && elevatorPower >.1){
                        state = armMove.UPDOWN;
                    } else
                    if(pivotPower<.1&&pivotPower>-.1
                            && elevatorPower<.1 && elevatorPower>-.1){
                        state = armMove.HOLD;
                    } else{
                        state = armMove.BASE;
                    }

        }
    }

    public void update() {


        switch (state) {
            case DOWNDOWN:
                robot.eMotors.setPower(elevatorPower);
                if (armDone()) {
                    robot.pivotMotor.setPower(pivotPower);
                }
                break;

            case UPUP:
                robot.pivotMotor.setPower(pivotPower);
                if (pivotDone()) {
                    robot.eMotors.setPower(elevatorPower);
                }
                break;
            case UPDOWN:
                robot.pivotMotor.setPower(pivotPower);
                if(pivotDone()){
                    robot.eMotors.setPower(elevatorPower);
                }
                break;
            case DOWNUP:
                robot.eMotors.setPower(elevatorPower);
                robot.pivotMotor.setPower(pivotPower);
                break;
            case HOLD:
                robot.pivotMotor.setPower(pivotPower);
                robot.eMotors.setPower(elevatorPower);
                break;
            case BASE:
                robot.eMotors.setPower(elevatorPower);
                robot.pivotMotor.setPower(pivotPower);
                break;
        }

        if(elevatorPoint==0) {
            if (robot.armSwitch.isPressed() && !armHeld) {
                robot.eMotors.resetEncoder();
                robot.eMotors.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armHeld = true;
            }
            if (!robot.armSwitch.isPressed()) {
                armHeld = false;
            }
        }
        if(pivotPoint ==0) {
            if (robot.pivotLimit.isPressed() && !pivotHeld) {
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
        }
        if (elevatorPID.getSetPoint() == 0 && robot.armSwitch.isPressed() && Math.abs(elevatorPID.getSetPoint()-robot.eMotors.getPosition())<50) {
            robot.eMotors.disable();
        } else {
            robot.eMotors.enable();
        }
/**
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
**/
        lastArmPoint = elevatorPID.getSetPoint();
        lastPivPoint = pivotPID.getSetPoint();

    }


    /**
    PID vals, setpoints,current pos
    **/
    public void setArmVal(globals.armVal val){
        armVal = val;
    }
    public globals.armVal getArmVal(){
        return armVal;
    }
    public double[] getAllVals(){
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


    public int getPivotPos() {
        return robot.pivotMotor.getCurrentPosition();
    }

    public double getPivotPower() {
        return robot.pivotMotor.getPower();
    }

    public void setElevatorGains(double P, double I, double D) {
        elevatorPID.setPID(P, I, D);
    }

    public double getExtension() {
        return 0;
    }

    public void setPivotGains(double P, double I, double D) {
        pivotPID.setPID(P, I, D);
    }

}
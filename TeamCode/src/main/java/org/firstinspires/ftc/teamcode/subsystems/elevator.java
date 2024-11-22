package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.robotHardware;

import java.util.List;
import java.util.Objects;

public class elevator extends SubsystemBase {

    private PIDController elevatorPID = new PIDController(constants.armConstants.middle.P, constants.armConstants.middle.I, constants.armConstants.middle.D);

    private PIDController pivotPID = new PIDController(constants.pivotConstants.P, constants.pivotConstants.I, constants.pivotConstants.D);

    private boolean pivotHeld = false;
    private boolean armHeld = false;
    private boolean armTest = false;
    private double lastPivPoint = 0;
    private double lastArmPoint = 0;

    public enum armState {
        UPUP,
        UPDOWN,
        DOWNDOWN,
        DOWNUP,
        MANUAL,
        AUTO,
        HOLD
    }


    private static armState state = armState.UPUP;



    private robotHardware robot = robotHardware.getInstance();

    public elevator() {

        elevatorPID.setTolerance(150);
        pivotPID.setTolerance(2);

    }


    public boolean isDone() {
        return (armDone() && pivotDone());
    }

    public void setState(elevator.armState State){
        state = State;
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
        return Math.abs(robot.pivotMotor.getCurrentPosition() - pivotPID.getSetPoint()) < 20;
    }

    public boolean armDone() {
        return Math.abs(robot.eMotors.getPosition() - elevatorPID.getSetPoint()) < 30;
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


    public void update() {
        setPivotGains(constants.pivotConstants.P,constants.pivotConstants.I,constants.pivotConstants.D);

        if(pivotPID.getSetPoint()==-10000 && robot.pivotLimit.isPressed()){
            pivotPID.setSetPoint(0);
            robot.pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            pivotPID.reset();
        }
        if(elevatorPID.getSetPoint()==-10000 && robot.armSwitch.isPressed()){
            elevatorPID.setSetPoint(0);
            robot.eMotors.resetEncoder();
        }

        if (state != armState.MANUAL) {
            if (pivotPID.calculate(robot.pivotMotor.getCurrentPosition()) < -.1 && elevatorPID.calculate(robot.eMotors.getPosition()) < -.1) {
                state = armState.DOWNDOWN;
            }
            //pivot going up elevator going out
            if (pivotPID.calculate(robot.pivotMotor.getCurrentPosition()) > .1 && elevatorPID.calculate(robot.eMotors.getPosition()) > .1) {
                state = armState.UPUP;
            }
            //pivot going up elevator going in
            if (pivotPID.calculate(robot.pivotMotor.getCurrentPosition()) > .1 && elevatorPID.calculate(robot.eMotors.getPosition()) < -.1) {
                state = armState.DOWNUP;
            }
            if (pivotPID.calculate(robot.pivotMotor.getCurrentPosition()) < -.1 && elevatorPID.calculate(robot.eMotors.getPosition()) > .1) {
                state = armState.UPDOWN;
            }
            if (pivotPID.calculate(robot.pivotMotor.getCurrentPosition()) < .1 && pivotPID.calculate(robot.pivotMotor.getCurrentPosition()) > -.1
                    && elevatorPID.calculate(robot.eMotors.getPosition()) < .1 && elevatorPID.calculate(robot.eMotors.getPosition()) > -.1) {
                state = armState.HOLD;

            }
        }
            if(state == armState.UPUP) {

                robot.pivotMotor.setPower(pivotPID.calculate(robot.pivotMotor.getCurrentPosition()));
                if (pivotDone()) {
                    robot.eMotors.setPower(elevatorPID.calculate(robot.eMotors.getPosition()));
                }
            }

            if(state==armState.DOWNDOWN) {
                robot.eMotors.setPower(elevatorPID.calculate(robot.eMotors.getPosition()));
                if (armDone()) {

                    robot.pivotMotor.setPower(pivotPID.calculate(robot.pivotMotor.getCurrentPosition()));
                }
            }
            if(state==armState.DOWNUP) {

                robot.eMotors.setPower(elevatorPID.calculate(robot.eMotors.getPosition()));
                robot.pivotMotor.setPower(pivotPID.calculate(robot.pivotMotor.getCurrentPosition()));
            }
            if(state==armState.UPDOWN) {
                robot.eMotors.setPower(elevatorPID.calculate(robot.eMotors.getPosition()));
                if (armDone()) {
                    robot.pivotMotor.setPower(pivotPID.calculate(robot.pivotMotor.getCurrentPosition()));
                }
            }

            if(state==armState.HOLD) {
                robot.pivotMotor.setPower(pivotPID.calculate(robot.pivotMotor.getCurrentPosition()));
                robot.eMotors.setPower(elevatorPID.calculate(robot.eMotors.getPosition()));
            }

                if(state==armState.MANUAL) {
                    robot.pivotMotor.setPower(pivotPID.calculate(robot.pivotMotor.getCurrentPosition()));
                    robot.eMotors.setPower(elevatorPID.calculate(robot.eMotors.getPosition()));
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

    public void zero(){
        pivotPID.setSetPoint(-10000);
        elevatorPID.setSetPoint(-10000);
    }




    public armState getarmState(){
        return state;
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
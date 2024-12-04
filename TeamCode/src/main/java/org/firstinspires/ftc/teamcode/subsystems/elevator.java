package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.robotHardware;

import java.util.List;

public class elevator extends SubsystemBase {

    private PIDController elevatorPID = new PIDController(constants.armConstants.middle.P, constants.armConstants.middle.I, constants.armConstants.middle.D);

    private PIDController pivotPID = new PIDController(constants.pivotConstants.P, constants.pivotConstants.I, constants.pivotConstants.D);

    private boolean pivotHeld = false;
    private boolean armHeld = false;
    private boolean armTest = false;
    private double lastPivPoint = 0;
    private double lastArmPoint = 0;

    private enum armMove {
        UPUP,
        UPDOWN,
        DOWNDOWN,
        DOWNUP,
        HOLD
    }

    armMove state = armMove.UPUP;


    private robotHardware robot = robotHardware.getInstance();

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
    //TODO set arm point and pivot point indiv.
    public void setArmPoint(double point){
        elevatorPID.setSetPoint(point);
    }
    public void setPivotPoint(double point){
        pivotPID.setSetPoint(point);
    }


    public boolean pivotDone() {
        return Math.abs(robot.pivotMotor.getCurrentPosition() - pivotPID.getSetPoint()) < 80;
    }

    public boolean armDone() {
        return Math.abs(robot.eMotors.getPosition() - elevatorPID.getSetPoint()) < 50;
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

        //pivot coming down elevator going in
        if (lastPivPoint != pivotPID.getSetPoint() || lastArmPoint != elevatorPID.getSetPoint()) {
            if (pivotPID.calculate(robot.pivotMotor.getCurrentPosition()) < -.1 && elevatorPID.calculate(robot.eMotors.getPosition()) < -.1) {
                state = armMove.DOWNDOWN;
            }
            //pivot going up elevator going out
            if (pivotPID.calculate(robot.pivotMotor.getCurrentPosition()) > .1 && elevatorPID.calculate(robot.eMotors.getPosition()) > .1) {
                state = armMove.UPUP;
            }
            //pivot going up elevator going in
            if (pivotPID.calculate(robot.pivotMotor.getCurrentPosition()) > .1 && elevatorPID.calculate(robot.eMotors.getPosition()) < -.1) {
                state = armMove.DOWNUP;
            }
            if(pivotPID.calculate(robot.pivotMotor.getCurrentPosition())<.1&&pivotPID.calculate(robot.pivotMotor.getCurrentPosition())>-.1
            && elevatorPID.calculate(robot.eMotors.getPosition())<.1 && elevatorPID.calculate(robot.eMotors.getPosition())>-.1){
                state = armMove.HOLD;
            }


        }

            if (state == armMove.DOWNDOWN) {
                robot.eMotors.setPower(elevatorPID.calculate(robot.eMotors.getPosition()));
                if (armDone()) {
                    robot.pivotMotor.setPower(pivotPID.calculate(robot.pivotMotor.getCurrentPosition()));
                 }
            }
            if (state == armMove.UPUP) {
                robot.pivotMotor.setPower(pivotPID.calculate(robot.pivotMotor.getCurrentPosition()));

                if (pivotDone()) {
                    robot.eMotors.setPower(elevatorPID.calculate(robot.eMotors.getPosition()));
                }
            }
            if(state == armMove.HOLD){
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

    public void periodic() {



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
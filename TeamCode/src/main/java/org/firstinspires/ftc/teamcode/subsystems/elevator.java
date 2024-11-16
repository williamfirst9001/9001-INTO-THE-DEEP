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
        EP,
        PE,
        HOLD
    }

    armMove state = armMove.PE;
    public static globals.armVal armVal = globals.armVal.STOW;


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

    public void setArmPoint(double point) {
        elevatorPID.setSetPoint(point);
    }

    public void setPivotPoint(double point) {
        pivotPID.setSetPoint(point);
    }


    public boolean pivotDone() {
        return Math.abs(robot.pivotMotor.getCurrentPosition() - pivotPID.getSetPoint()) < 20 || (pivotPID.getSetPoint() == 0 && robot.pivotLimit.isPressed());
    }

    public boolean armDone() {
        return Math.abs(robot.eMotors.getPosition() - elevatorPID.getSetPoint()) < 30 || (elevatorPID.getSetPoint() == 0 && robot.armSwitch.isPressed());
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

    public void initCommand() {

    }

    public void update() {
        elevatorPower = getAllVals()[0];
        pivotPower = getAllVals()[1];
        elevatorPoint = getAllVals()[2];
        pivotPoint = getAllVals()[3];
        //pivot coming down elevator going in

        if (lastPivPoint != pivotPoint || lastArmPoint != elevatorPoint) {
            if (pivotPower < -.1 && elevatorPower < -.1) {
                state = armMove.EP;
            } else
                //pivot going up elevator going out
                if (pivotPower > .1 && elevatorPower > .1) {
                    state = armMove.PE;
                } else
                    //pivot going up elevator going in
                    if (pivotPower > .1 && elevatorPower < -.1) {
                        state = armMove.EP;
                    } else if (pivotPower < -.1 && elevatorPower > .1) {
                        state = armMove.PE;
                    } else if (pivotPower < .1 && pivotPower > -.1
                            && elevatorPower < .1 && elevatorPower > -.1) {
                        state = armMove.HOLD;
                    }

        }

        switch (state) {
            case PE:
                robot.pivotMotor.setPower(pivotPower);
                if (pivotDone()) {
                    robot.eMotors.setPower(elevatorPower);
                }
                break;
            case EP:
                robot.eMotors.setPower(elevatorPower);
                if (armDone()) {
                    robot.pivotMotor.setPower(pivotPower);
                }
                break;
            default:
                robot.eMotors.setPower(elevatorPower);
                robot.pivotMotor.setPower(pivotPower);
                break;
        }

        if (elevatorPoint == 0) {
            if (robot.armSwitch.isPressed() && !armHeld) {
                robot.eMotors.resetEncoder();
                robot.eMotors.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armHeld = true;
            }
            if (!robot.armSwitch.isPressed()) {
                armHeld = false;
            }
        }
        if (pivotPoint == 0) {
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
        if (elevatorPID.getSetPoint() == 0 && robot.armSwitch.isPressed()) {
            robot.eMotors.disable();
        } else {
            robot.eMotors.enable();
        }
        lastArmPoint = elevatorPID.getSetPoint();
        lastPivPoint = pivotPID.getSetPoint();

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
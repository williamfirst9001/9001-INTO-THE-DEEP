package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.robotHardware;

import org.firstinspires.ftc.teamcode.storage;

import static org.firstinspires.ftc.teamcode.constants.armConstants.middle.*;

public class elevator extends SubsystemBase {

    private PIDController elevatorPID = new PIDController(constants.armConstants.middle.P, constants.armConstants.middle.I, constants.armConstants.middle.D);

    private PIDController pivotPID = new PIDController(constants.pivotConstants.P, constants.pivotConstants.I, constants.pivotConstants.D);



    private robotHardware robot = robotHardware.getInstance();
    public elevator() {

        elevatorPID.setTolerance(150);
        pivotPID.setTolerance(2);

    }

    public void motorEnable(){
        robot.pivotMotor.setMotorEnable();
        robot.elevatorMotor.setMotorEnable();
    }

    public boolean isDone(){
        return pivotDone() && armDone();
    }
    public void brakePivot(){
        robot.pivotMotor.setPower(0);
    }
    public boolean pivotDone(){
        return Math.abs(robot.pivotMotor.getCurrentPosition()-pivotPID.getSetPoint())<30
                && robot.pivotMotor.getVelocity()<30;
    }
    private boolean armDone(){
        return Math.abs(robot.elevatorMotor.getCurrentPosition()-elevatorPID.getSetPoint())<65;
    }
    public void goToSetpoint(double armPoint,double pivotPoint) {
        elevatorPID.setSetPoint(armPoint);
        pivotPID.setSetPoint(pivotPoint);

        robot.elevatorMotor.setPower(elevatorPID.calculate(robot.elevatorMotor.getCurrentPosition()));

        robot.pivotMotor.setPower(pivotPID.calculate(robot.pivotMotor.getCurrentPosition()));

    }
    public int getPivotPos(){
        return robot.pivotMotor.getCurrentPosition();
    }
    public double getPivotPower(){
        return robot.pivotMotor.getPower();
    }
    public void setElevatorGains(double P, double I,double D){
        elevatorPID.setPID(P,I,D);
    }
    public void setPivotGains(double P, double I,double D){
        pivotPID.setPID(P,I,D);
    }

    }
package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.robotHardware;

import org.firstinspires.ftc.teamcode.storage;

import static org.firstinspires.ftc.teamcode.constants.armConstants.middle.*;

public class elevator extends SubsystemBase {

    private PIDController elevatorPID = new PIDController(constants.armConstants.middle.P, constants.armConstants.middle.I, constants.armConstants.middle.D);

    private PIDController pivotPID = new PIDController(constants.pivotConstants.P, constants.pivotConstants.I, constants.pivotConstants.D);

    private boolean pivotHeld = false;
    private boolean armHeld = false;



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
        return (armDone() && pivotDone());
    }
    public double getArmPose(){
        return robot.elevatorMotor.getCurrentPosition();
    }
    public void brakePivot(){
        robot.pivotMotor.setPower(0);
    }
    public boolean pivotDone(){
        return Math.abs(robot.pivotMotor.getCurrentPosition()-pivotPID.getSetPoint())<15;
    }
    public boolean armDone(){
        return Math.abs(robot.elevatorMotor.getCurrentPosition()-elevatorPID.getSetPoint())<30;
    }
    public void goToPivotPoint(double p){
        pivotPID.setSetPoint(p);
        robot.pivotMotor.setPower(pivotPID.calculate(robot.pivotMotor.getCurrentPosition()));
    }

    public void goToSetpoint(double armPoint,double pivotPoint) {
        elevatorPID.setSetPoint(armPoint);
        pivotPID.setSetPoint(pivotPoint);

        robot.elevatorMotor.setPower(elevatorPID.calculate(robot.elevatorMotor.getCurrentPosition()));

        robot.pivotMotor.setPower(pivotPID.calculate(robot.pivotMotor.getCurrentPosition()));

    }
    public void goToSetpoint(double armPoint) {
        elevatorPID.setSetPoint(armPoint);

        robot.elevatorMotor.setPower(elevatorPID.calculate(robot.elevatorMotor.getCurrentPosition()));

    }
    public double getArmVelo(){
        return robot.elevatorMotor.getVelocity();
    }
    public void periodic(){

        if(robot.armSwitch.isPressed() && !armHeld){
            robot.elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armHeld = true;
        }
        if(!robot.armSwitch.isPressed()){
            armHeld = false;
        }
        if(robot.pivotLimit.isPressed() && !pivotHeld){
            robot.pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            pivotHeld = true;
        }
        if(!robot.pivotLimit.isPressed()) {
            pivotHeld = false;

        }
        if(pivotPID.getSetPoint()==0 && Math.abs(robot.pivotMotor.getCurrentPosition()-pivotPID.getSetPoint())<30){
            robot.pivotMotor.setMotorDisable();
        } else{
            robot.pivotMotor.setMotorEnable();
        }
        }









    public boolean validArmExtension(){
        return !(robot.elevatorMotor.getCurrentPosition()>constants.armLimits.maxExtensionRange && robot.elevatorMotor.getPower()>0);

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
    public double getExtension(){
        return 0;
    }
    public void setPivotGains(double P, double I,double D){
        pivotPID.setPID(P,I,D);
    }

    }
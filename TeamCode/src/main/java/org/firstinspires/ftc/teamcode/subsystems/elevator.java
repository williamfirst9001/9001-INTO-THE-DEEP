package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.PID.armPID;
import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.robotContainer;

public class elevator extends SubsystemBase {

    //private PIDController elevatorPID = new PIDController(constants.armPID.middle.kP, constants.armPID.middle.kI, constants.armPID.middle.kD);

    //private PIDController pivotPID = new PIDController(constants.pivotPID.kP, constants.pivotPID.kI, constants.pivotPID.kD);

    private NanoClock clock = NanoClock.system();;
    private double elevatorProfileStart;
    private double pivotProfileStart;
    private double lastArmPoint = 0;
    private double lastPivotPoint = 0;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private double pivotkV,pivotkA,pivotkStatic;
    private double elevatorkV,elevatorkA,elevatorkStatic;
    private MotionProfile elevatorProfile = generateProfile(0,0,0);
    private MotionProfile pivotProfile = generateProfile(0,0,0);

    private double elevatorTolerance = 50;
    private double pivotTolerance = 25;
    private robotContainer robot = robotContainer.getInstance();
    public elevator() {






        //NanoClock clock = NanoClock.system();
        elevatorProfileStart = clock.seconds();
        pivotProfileStart = clock.seconds();


    }
    private static MotionProfile generateProfile(double setPoint,double x,double v) {
        double armOffset = 1;
        MotionState start = new MotionState(x, v, 0,0);
        MotionState goal = new MotionState(setPoint, 0, 0, 0);
        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal,armPID.MAX_VEL,armPID.MAX_ACCEL);
    }
    private double getPower(MotionState motionState,double kV,double kA,double kStatic){
        return Kinematics.calculateMotorFeedforward(motionState.getV(), motionState.getA(), kV, kA, kStatic);

    }
    public boolean isDone(){
        return elevatorProfile.end().getX()-robot.elevatorMotor.getCurrentPosition()<elevatorTolerance
                && pivotProfile.end().getX()-robot.pivotMotor.getCurrentPosition()<pivotTolerance;
    }


        public void goToSetpoint(double armPoint,double pivotPoint) {
            //NanoClock clock = NanoClock.system();


            double elevatorProfileTime = clock.seconds() - elevatorProfileStart;
            double pivotProfileTime = clock.seconds()-pivotProfileStart;

            elevatorkA=constants.armConstants.middle.kA;
            elevatorkV=constants.armConstants.middle.kV;
            elevatorkStatic=constants.armConstants.middle.kStatic;


            if(lastArmPoint!= armPoint){
                elevatorProfile = generateProfile(armPoint,robot.elevatorMotor.getCurrentPosition(),robot.elevatorMotor.getVelocity());
                elevatorProfileStart = clock.seconds();
            }
            if(lastPivotPoint != pivotPoint){
                pivotProfile = generateProfile(pivotPoint,robot.pivotMotor.getCurrentPosition(),0);
                pivotProfileTime = clock.seconds();
             }
            if(elevatorProfileTime<elevatorProfile.duration()){
                elevatorProfile = generateProfile(armPoint-robot.elevatorMotor.getCurrentPosition(),0,robot.elevatorMotor.getVelocity());
                elevatorProfileStart = clock.seconds();
            }



            //elevatorProfile = generateProfile(armPoint,elevatorMotor.getCurrentPosition(),elevatorMotor.getVelocity(),true);
            MotionState elevatorState = elevatorProfile.get(elevatorProfileTime);
            double elevatorTargetPower = Kinematics.calculateMotorFeedforward(elevatorState.getV(), elevatorState.getA(), elevatorkV, elevatorkA, elevatorkStatic);




            pivotProfile = generateProfile(pivotPoint,robot.pivotMotor.getCurrentPosition(),robot.pivotMotor.getVelocity());
            MotionState pivotState = pivotProfile.get(pivotProfileTime);
            double pivotTargetPower = getPower(pivotState,pivotkV, pivotkA, pivotkStatic);






            robot.elevatorMotor.setPower(elevatorTargetPower);
            robot.pivotMotor.setPower(pivotTargetPower);
            lastArmPoint = armPoint;
            lastPivotPoint = pivotPoint;

            //return elevatorTargetPower;


        }
        public void setElevatorGains(double kV, double kA,double kStatic){
            elevatorkV = kV;
            elevatorkA = kA;
            elevatorkStatic= kStatic;
        }
        public void setPivotGains(double kV, double kA,double kStatic){
        pivotkV = kV;
        pivotkA = kA;
        pivotkStatic= kStatic;
    }


    }


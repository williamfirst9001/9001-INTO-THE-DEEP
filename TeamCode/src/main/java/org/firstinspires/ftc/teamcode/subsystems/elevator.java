package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.PID.armPID;
import org.firstinspires.ftc.teamcode.constants;

public class elevator {
    private DcMotorEx elevatorMotor, pivotMotor;

    //private PIDController elevatorPID = new PIDController(constants.armPID.middle.kP, constants.armPID.middle.kI, constants.armPID.middle.kD);

    //private PIDController pivotPID = new PIDController(constants.pivotPID.kP, constants.pivotPID.kI, constants.pivotPID.kD);

    private NanoClock clock = NanoClock.system();;
    private double elevatorProfileStart;
    private double pivotProfileStart;
    private double lastArmPoint;
    private double lastPivotPoint;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private double pivotkV,pivotkA,pivotkStatic;
    private double elevatorkV,elevatorkA,elevatorkStatic;
    private MotionProfile elevatorProfile = generateProfile(0,0,0,true);
    private MotionProfile pivotProfile = generateProfile(0,0,0,false);

    public void init(HardwareMap hardwareMap) {

            elevatorMotor = hardwareMap.get(DcMotorEx.class, "elevator");
            pivotMotor = hardwareMap.get(DcMotorEx.class, "pivot");
            elevatorMotor.setDirection(DcMotor.Direction.REVERSE);
            elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        //NanoClock clock = NanoClock.system();
        elevatorProfileStart = clock.seconds();
        pivotProfileStart = clock.seconds();


    }
    private static MotionProfile generateProfile(double setPoint,double x,double v,boolean arm) {
        double armOffset = 1;
        MotionState start = new MotionState(x, v, 0, 0);
        MotionState goal = new MotionState(setPoint * armOffset, 0, 0, 0);
        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal,armPID.MAX_VEL,armPID.MAX_ACCEL);
    }
    private double getPower(MotionState motionState,double kV,double kA,double kStatic){
        return Kinematics.calculateMotorFeedforward(motionState.getV(), motionState.getA(), kV, kA, kStatic);

    }
        public double pivotToAngle() {
            return pivotMotor.getCurrentPosition() / constants.armGearRatio.countsPerDegree;
        }
        public double test(){
        return elevatorMotor.getCurrentPosition();
        }

        public void update(int pivotPoint,int armPoint,double voltage) {
            //NanoClock clock = NanoClock.system();

            final double NOMINAL_VOLTAGE = 12.0;
            double elevatorProfileTime = clock.seconds() - elevatorProfileStart;
            double pivotProfileTime = clock.seconds()-pivotProfileStart;

            if(armPoint == constants.elevatorSetpoints.armSetpoints.middle){
                elevatorkA = constants.armConstants.middle.kA;
                elevatorkV = constants.armConstants.middle.kV;
                elevatorkStatic = constants.armConstants.middle.kStatic;

                pivotkA = constants.pivotConstants.middle.kA;
                pivotkV = constants.pivotConstants.middle.kA;
                pivotkStatic = constants.pivotConstants.middle.kA;
            }
            if(pivotPoint == constants.elevatorSetpoints.pivotSetpoints.basket){
                pivotkA = constants.pivotConstants.basket.kA;
                pivotkV = constants.pivotConstants.basket.kV;
                pivotkStatic = constants.pivotConstants.basket.kStatic;
                if(armPoint == constants.elevatorSetpoints.armSetpoints.lowBasket){
                    elevatorkA = constants.armConstants.baskets.kA;
                    elevatorkV = constants.armConstants.baskets.kV;
                    elevatorkStatic = constants.armConstants.baskets.kStatic;
                }
            }

            if(pivotPoint == constants.elevatorSetpoints.pivotSetpoints.chamber){
                pivotkA = constants.pivotConstants.chamber.kA;
                pivotkV = constants.pivotConstants.chamber.kV;
                pivotkStatic = constants.pivotConstants.chamber.kStatic;
                if(armPoint == constants.elevatorSetpoints.armSetpoints.lowBasket){
                    elevatorkA = constants.armConstants.chambers.kA;
                    elevatorkV = constants.armConstants.chambers.kV;
                    elevatorkStatic = constants.armConstants.chambers.kStatic;
                }
            }


/**
            if(armPoint != lastArmPoint || elevatorProfileTime>elevatorProfile.duration()){
                elevatorProfile = generateProfile(armPoint,elevatorMotor.getCurrentPosition(),0,true);
                elevatorProfileTime = clock.seconds();
            }
            if(pivotPoint != lastPivotPoint|| pivotProfileTime>pivotProfile.duration()){
                pivotProfile = generateProfile(pivotPoint,pivotMotor.getCurrentPosition(),0,false);
                pivotProfileTime = clock.seconds();
             }
 **/


            elevatorProfile = generateProfile(armPoint,0,elevatorMotor.getVelocity(),true);
            MotionState elevatorState = elevatorProfile.get(elevatorProfileTime);
            double elevatorTargetPower = getPower(elevatorState,elevatorkV, elevatorkA, elevatorkStatic);


            pivotProfile = generateProfile(pivotPoint,pivotMotor.getCurrentPosition(),pivotMotor.getCurrentPosition(),false);
            MotionState pivotState = pivotProfile.get(pivotProfileTime);
            double pivotTargetPower = getPower(pivotState,pivotkV, pivotkA, pivotkStatic);






            elevatorMotor.setPower(NOMINAL_VOLTAGE / voltage * elevatorTargetPower);
            pivotMotor.setPower(NOMINAL_VOLTAGE / voltage * pivotTargetPower);
            lastArmPoint = armPoint;
            lastPivotPoint = pivotPoint;

            //return elevatorTargetPower;


        }


    }


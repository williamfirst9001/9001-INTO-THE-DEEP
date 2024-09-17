package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.PID.armPID;
import org.firstinspires.ftc.teamcode.PID.pivotPID;
import org.firstinspires.ftc.teamcode.constants;

public class elevator {
    private DcMotorEx elevatorMotor, pivotMotor;

    //private PIDController elevatorPID = new PIDController(constants.armPID.middle.kP, constants.armPID.middle.kI, constants.armPID.middle.kD);

    //private PIDController pivotPID = new PIDController(constants.pivotPID.kP, constants.pivotPID.kI, constants.pivotPID.kD);

    private NanoClock clock;
    private double profileStart;
    public elevator(HardwareMap hardwareMap) {

            elevatorMotor = hardwareMap.get(DcMotorEx.class, "elevator");
            pivotMotor = hardwareMap.get(DcMotorEx.class, "pivot");
            elevatorMotor.setDirection(DcMotor.Direction.REVERSE);
            elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        NanoClock clock = NanoClock.system();
        profileStart = clock.seconds();


    }
    private static MotionProfile generateProfile(double setPoint,double x,double v,boolean arm) {
        MotionState start = new MotionState(x, v, 0, 0);
        MotionState goal = new MotionState(setPoint, 0, 0, 0);
        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal, arm ? armPID.MAX_VEL : pivotPID.MAX_VEL, arm ? armPID.MAX_ACCEL:pivotPID.MAX_ACCEL);
    }
        public double pivotToAngle() {
            return pivotMotor.getCurrentPosition() / constants.armGearRatio.countsPerDegree;
        }

        public void update(int pivotPoint,int armPoint) {
            double profileTime = clock.seconds() - profileStart;
        MotionProfile elevatorProfile = generateProfile(armPoint,elevatorMotor.getCurrentPosition(),elevatorMotor.getVelocity(),true);
            MotionState elevatorState = elevatorProfile.get(profileTime);
            double elevatorTargetPower = Kinematics.calculateMotorFeedforward(elevatorState.getV(), elevatorState.getA(), armPID.kV, armPID.kA, armPID.kStatic);

            MotionProfile pivotProfile = generateProfile(pivotPoint,pivotMotor.getCurrentPosition(),pivotMotor.getVelocity(),false);
            MotionState pivotState = pivotProfile.get(profileTime);
            double pivotTargetPower = Kinematics.calculateMotorFeedforward(pivotState.getV(), pivotState.getA(), pivotPID.kV, pivotPID.kA, pivotPID.kStatic);

            elevatorMotor.setPower(elevatorTargetPower);
            pivotMotor.setPower(pivotTargetPower);

        }


    }


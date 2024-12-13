package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.globals;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.elevator;

import java.util.List;

public class armUpdateCMD extends CommandBase {
    private elevator m_arm;
    private static double armPoint,pivotPoint,wristPoint;
    private boolean end;
    private Wrist m_wrist = null;
    private NanoClock clock = NanoClock.system();
    private boolean tune = false;
    private List<Double> vals;
    globals.armVal m_type;




    public armUpdateCMD(elevator arm){
        m_arm = arm;

        addRequirements(m_arm);


    }




    @Override
    public void initialize() {
    }
    @Override
    public void execute(){

        m_arm.update();


    }







}

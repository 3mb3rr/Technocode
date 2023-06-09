package org.firstinspires.ftc.teamcode.Mech.BaseCommands;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mech.SubConstants;
import org.firstinspires.ftc.teamcode.Mech.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.hSlideSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.vSlideSubsystem;

public class tArmDown extends CommandBase {

    // The subsystem the command runs on
    private final IntakeSubsystem IntakeSub;

    public tArmDown(IntakeSubsystem subsystem) {
        IntakeSub = subsystem;
        addRequirements(IntakeSub);
    }

    @Override
    public void initialize() {
        IntakeSub.armToAngle(-15);
        IntakeSub.grotateToAngle(15);
    }


    @Override
    public boolean isFinished() {
        if(((IntakeSub.getArmVelocity()<1) && (IntakeSub.getArmAngle()<-8.5) && (IntakeSub.getArmAngle()>-11.5))
                || (IntakeSub.getArmVelocity()==0))
        {return true;}
        return false;
    }

}
package org.firstinspires.ftc.teamcode.Mech.BaseCommands;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mech.SubConstants;
import org.firstinspires.ftc.teamcode.Mech.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.hSlideSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.vSlideSubsystem;

public class autoArmDown extends CommandBase {

    // The subsystem the command runs on
    private final IntakeSubsystem IntakeSub;

    public autoArmDown(IntakeSubsystem subsystem) {
        IntakeSub = subsystem;
        addRequirements(IntakeSub);
    }

    @Override
    public void initialize() {
        IntakeSub.armToAngle(SubConstants.armAngle[5-SubConstants.conestackHeight]);
        IntakeSub.grotateLevel();
    }


    @Override
    public boolean isFinished() {
        if((IntakeSub.getArmVelocity()<1) && (IntakeSub.getArmAngle()<(SubConstants.armAngle[5-SubConstants.conestackHeight]+1.5)) && (IntakeSub.getArmAngle()>(SubConstants.armAngle[5-SubConstants.conestackHeight]-1.5)))
        {return true;}
        return false;
    }

}
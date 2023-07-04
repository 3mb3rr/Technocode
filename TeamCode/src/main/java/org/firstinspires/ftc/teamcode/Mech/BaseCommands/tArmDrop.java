package org.firstinspires.ftc.teamcode.Mech.BaseCommands;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mech.SubConstants;
import org.firstinspires.ftc.teamcode.Mech.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.hSlideSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.vSlideSubsystem;

public class tArmDrop extends CommandBase {

    // The subsystem the command runs on
    private final IntakeSubsystem IntakeSub;

    public tArmDrop(IntakeSubsystem subsystem) {
        IntakeSub = subsystem;
        addRequirements(IntakeSub);
    }

    @Override
    public void initialize() {
        IntakeSub.armState = IntakeSubsystem.Arm.dropping;
        SubConstants.armFeedforward = 0.2;
        IntakeSub.armToAngle(87);
        IntakeSub.grotateToAngle(0);
    }

    @Override
    public void end(boolean interrupted) {
        IntakeSub.armState = IntakeSubsystem.Arm.holding;
        SubConstants.armFeedforward = 0.23;
    }
    @Override
    public boolean isFinished() {
        if(((IntakeSub.getArmVelocity()<1) && (IntakeSub.getArmAngle()<91.5 && (IntakeSub.getArmAngle()>88.5)))
                || (IntakeSub.getArmVelocity()==0))
        {return true;}
        return false;
    }

}
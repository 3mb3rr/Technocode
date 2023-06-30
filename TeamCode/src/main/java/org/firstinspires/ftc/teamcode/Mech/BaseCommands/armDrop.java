package org.firstinspires.ftc.teamcode.Mech.BaseCommands;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mech.SubConstants;
import org.firstinspires.ftc.teamcode.Mech.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.hSlideSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.vSlideSubsystem;

public class armDrop extends CommandBase {

    // The subsystem the command runs on
    private final IntakeSubsystem IntakeSub;

    public armDrop(IntakeSubsystem subsystem) {
        IntakeSub = subsystem;
        addRequirements(IntakeSub);
    }
    @Override
    public void initialize() {
        IntakeSub.armState = IntakeSubsystem.Arm.dropping;
        IntakeSub.grotateToAngle(75);
        IntakeSub.armToAngle(85);
    }
    @Override
    public void end(boolean interrupted) {
        IntakeSub.armState = IntakeSubsystem.Arm.holding;
    }
    @Override
    public boolean isFinished() {
        if((IntakeSub.getArmVelocity()<1) && (IntakeSub.getArmAngle()>84))
        {return true;}
        return false;
    }

}
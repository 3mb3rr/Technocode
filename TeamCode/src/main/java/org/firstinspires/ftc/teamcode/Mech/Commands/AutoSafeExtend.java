package org.firstinspires.ftc.teamcode.Mech.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Mech.BaseCommands.autoArmDown;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.grabberOpen;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.hSlideAutoOpen;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.hSlideSafeOpen;
import org.firstinspires.ftc.teamcode.Mech.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.hSlideSubsystem;

public class AutoSafeExtend extends SequentialCommandGroup {

    public AutoSafeExtend(IntakeSubsystem IntakeSub, hSlideSubsystem hSlideSub)
    {
        addCommands (
                new SequentialCommandGroup(
                        new InstantCommand(() -> {IntakeSub.botCommandComplete = false;}),
                        new grabberOpen(IntakeSub),
                        new ParallelCommandGroup(
                                new autoArmDown(IntakeSub),
                                new hSlideSafeOpen(hSlideSub)
                        )
                ));
        addRequirements(IntakeSub);
    }

}

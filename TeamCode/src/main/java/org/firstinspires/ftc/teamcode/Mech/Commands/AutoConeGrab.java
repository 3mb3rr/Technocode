package org.firstinspires.ftc.teamcode.Mech.Commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Mech.BaseCommands.armDrop;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.autoArmDown;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.grabberGrab;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.grabberOpen;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.hSlideAutoOpen;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.hSlideClose;
import org.firstinspires.ftc.teamcode.Mech.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.hSlideSubsystem;

public class AutoConeGrab extends SequentialCommandGroup {

    public AutoConeGrab(IntakeSubsystem IntakeSub, hSlideSubsystem hSlideSub)
    {
        addCommands (
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                            new autoArmDown(IntakeSub),
                        new hSlideAutoOpen(hSlideSub)
                    ),
                    new grabberGrab(IntakeSub),
                    new WaitCommand(250),
                    new ParallelCommandGroup(
                        new armDrop(IntakeSub),
                        new hSlideClose(hSlideSub)
                    ),
                    new grabberOpen(IntakeSub),
                    new WaitCommand(200)
        ));
        addRequirements(IntakeSub);
    }

}

package org.firstinspires.ftc.teamcode.Mech.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Mech.BaseCommands.depositClose;
import org.firstinspires.ftc.teamcode.Mech.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.vSlideSubsystem;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.depositOpen;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.dropperDrop;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.dropperGrab;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.highSlideOpen;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.ttTurnMiddle;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.vSlideClose;

public class tCycleDrop extends SequentialCommandGroup {

    public tCycleDrop(DepositSubsystem DepositSub, vSlideSubsystem vSlideSub)
    {
        addCommands (
                new SequentialCommandGroup(
                        new InstantCommand(() -> {DepositSub.hasCone(true);}),
                        new depositClose(DepositSub),
                        new dropperDrop(DepositSub),
                        new highSlideOpen(vSlideSub),
                        new WaitCommand(150),
                        new depositOpen(DepositSub),
                        new InstantCommand(() -> {DepositSub.hasCone(false);}),
                        new WaitCommand(100),
                        new dropperGrab(DepositSub),
                        new WaitCommand(200),
                        new ParallelCommandGroup(new vSlideClose(vSlideSub), new ttTurnMiddle(DepositSub)))
        );
        addRequirements(DepositSub, vSlideSub);
    }

}

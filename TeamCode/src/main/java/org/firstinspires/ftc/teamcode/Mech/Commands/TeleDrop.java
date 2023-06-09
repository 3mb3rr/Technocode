package org.firstinspires.ftc.teamcode.Mech.Commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Mech.BaseCommands.depositClose;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.dropperMid;
import org.firstinspires.ftc.teamcode.Mech.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.vSlideSubsystem;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.depositOpen;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.dropperDrop;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.dropperGrab;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.highSlideOpen;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.ttTurnMiddle;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.ttTurnRight;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.vSlideClose;

public class TeleDrop extends SequentialCommandGroup {

    public TeleDrop(DepositSubsystem DepositSub, vSlideSubsystem vSlideSub)
    {
        addCommands (
                new SequentialCommandGroup(
                        new dropperDrop(DepositSub),
                        new depositOpen(DepositSub),
                        new WaitCommand(100),
                        new dropperGrab(DepositSub),
                        new WaitCommand(150),
                        new ParallelCommandGroup(
                            new vSlideClose(vSlideSub),
                            new ttTurnMiddle(DepositSub))
                ));
        DepositSub.hasCone(false);
        addRequirements(DepositSub, vSlideSub);
    }

}

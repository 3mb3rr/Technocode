package org.firstinspires.ftc.teamcode.Mech.Commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Mech.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.vSlideSubsystem;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.depositOpen;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.dropperDrop;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.dropperGrab;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.ttTurnMiddle;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.vSlideClose;

public class TeleDrop extends SequentialCommandGroup {

    public TeleDrop(DepositSubsystem DepositSub, vSlideSubsystem vSlideSub)
    {
        addCommands (
                new SequentialCommandGroup(
                        new dropperDrop(DepositSub),
                        new depositOpen(DepositSub),
                        new WaitCommand(300),
                        //change: 100 to 300
                        new dropperGrab(DepositSub),
                        new WaitCommand(200),
                        //change 150 to 200
                            new vSlideClose(vSlideSub)
                ));
        DepositSub.hasCone(false);
        addRequirements(DepositSub, vSlideSub);
    }

}

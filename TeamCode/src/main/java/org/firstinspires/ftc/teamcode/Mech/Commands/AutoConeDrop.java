package org.firstinspires.ftc.teamcode.Mech.Commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Mech.BaseCommands.depositClose;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.dropperMid;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.ttTurnLeft;
import org.firstinspires.ftc.teamcode.Mech.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.vSlideSubsystem;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.depositOpen;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.dropperDrop;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.dropperGrab;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.highSlideOpen;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.ttTurnMiddle;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.ttTurnRight;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.vSlideClose;

import java.util.function.BooleanSupplier;

public class AutoConeDrop extends SequentialCommandGroup {

    public AutoConeDrop(DepositSubsystem DepositSub, vSlideSubsystem vSlideSub, boolean right)
    {
        BooleanSupplier RIGHT = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return right;
            }
        };
        addCommands (
                new SequentialCommandGroup(
                        new InstantCommand(() -> {DepositSub.hasCone(true);}),
                        new depositClose(DepositSub),
                        new dropperMid(DepositSub),
                        new ConditionalCommand(new ttTurnRight(DepositSub), new ttTurnLeft(DepositSub), RIGHT),
                        new highSlideOpen(vSlideSub),
                        new dropperDrop(DepositSub),
                        new WaitCommand(200),
                        new depositOpen(DepositSub),
                        new InstantCommand(() -> {DepositSub.hasCone(false);}),
                        new WaitCommand(200),
                        new dropperGrab(DepositSub),
                        new WaitCommand(200),
                        new ParallelCommandGroup(new vSlideClose(vSlideSub), new ttTurnMiddle(DepositSub))),
                        new WaitCommand(100)
        );
        addRequirements(DepositSub, vSlideSub);
    }

}

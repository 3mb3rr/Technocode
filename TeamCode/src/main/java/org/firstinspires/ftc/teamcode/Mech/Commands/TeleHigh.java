package org.firstinspires.ftc.teamcode.Mech.Commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Mech.BaseCommands.depositClose;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.dropperMid;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.tArmDrop;
import org.firstinspires.ftc.teamcode.Mech.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.vSlideSubsystem;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.highSlideOpen;

import java.util.function.BooleanSupplier;

public class TeleHigh extends SequentialCommandGroup {

    public TeleHigh(DepositSubsystem DepositSub, vSlideSubsystem vSlideSub, IntakeSubsystem IntakeSub)
    {
        BooleanSupplier DepositHasCone = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return DepositSub.hasCone();
            }
        };
        addCommands (new ConditionalCommand(
                new SequentialCommandGroup(
                        new tArmDrop(IntakeSub),
                        new depositClose(DepositSub),
                        new dropperMid(DepositSub),
                        new highSlideOpen(vSlideSub)
        ),
                new SequentialCommandGroup(
                        new transfer(IntakeSub, DepositSub),
                        new WaitCommand(200),
                        new tArmDrop(IntakeSub),
                        new depositClose(DepositSub),
                        new dropperMid(DepositSub),
                        new highSlideOpen(vSlideSub)
                ),
        DepositHasCone


        ));
        addRequirements(DepositSub, vSlideSub, IntakeSub);
    }

}

package org.firstinspires.ftc.teamcode.Mech.Commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Mech.BaseCommands.armDrop;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.autoArmDown;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.chassisToConestack;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.grabberGrab;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.grabberOpen;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.hSlideAutoOpen;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.hSlideClose;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.slideToConestack;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.tArmDrop;
import org.firstinspires.ftc.teamcode.Mech.SubConstants;
import org.firstinspires.ftc.teamcode.Mech.subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.hSlideSubsystem;

import java.util.function.BooleanSupplier;

public class AutoSafeGrab extends SequentialCommandGroup {

    InstantCommand DecreaseStackHeight = new InstantCommand(() ->{
        SubConstants.conestackHeight--;
    }) ;
    IntakeSubsystem IntakeSubs;

    public AutoSafeGrab(IntakeSubsystem IntakeSub, hSlideSubsystem hSlideSub, ChassisSubsystem ChassisSub)
    {

        BooleanSupplier DepConeDropped = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return !IntakeSub.depositCone();
            }
        };
        BooleanSupplier IntakeConeDrop = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return !IntakeSub.hasCone();
            }
        };
        BooleanSupplier SecondDrop = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return (!IntakeSub.hasCone() && IntakeSub.grabfailed);
            }
        };
        addCommands (
                new WaitUntilCommand(DepConeDropped),
                new chassisToConestack(hSlideSub, IntakeSub, ChassisSub),
                new grabberGrab(IntakeSub),
                new WaitCommand(400),
                new ParallelCommandGroup( new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new tArmDrop(IntakeSub),
                        new hSlideClose(hSlideSub),
                        new InstantCommand(() ->{
                            SubConstants.conestackHeight--;
                        })
                ),
                new armDrop(IntakeSub),
                new WaitCommand(200),
                new grabberOpen(IntakeSub),
                new WaitCommand(160)), new chassisSafePole(ChassisSub)
                )
        );

        addRequirements(IntakeSub);
    }

}
/*         addCommands (
                    new WaitUntilCommand(DepConeDropped),
                    new slideToConestack(hSlideSub, IntakeSub),
                    new grabberGrab(IntakeSub),
                    new WaitCommand(400),
                    new ParallelDeadlineGroup(
                        new WaitUntilCommand(IntakeConeDrop),
                        new ParallelCommandGroup(
                            new armDrop(IntakeSub),
                            new hSlideClose(hSlideSub))
                    ),
                    new ConditionalCommand(
                            // sequence if cone is dropped
                            new SequentialCommandGroup(
                                    // if cone dropped twice(only the conditional below)
                                    new ConditionalCommand(new InstantCommand(() ->{SubConstants.conestackHeight--;}), new WaitCommand(1), SecondDrop),
                                    new InstantCommand(() ->{IntakeSub.grabfailed = true;}),
                                    new AutoConeExtend(IntakeSub, hSlideSub),
                                    new AutoConeGrab(IntakeSub, hSlideSub)),
                            new SequentialCommandGroup(
                                    // if cone is not dropped
                                    new InstantCommand(() ->{IntakeSub.grabfailed = false;}),
                                    new InstantCommand(() ->{SubConstants.conestackHeight--;}),
                                    new grabberOpen(IntakeSub), new WaitCommand(200)),
                            IntakeConeDrop)

        );*/
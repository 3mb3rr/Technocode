package org.firstinspires.ftc.teamcode.Mech.subsystems;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Mech.SubConstants;

public class vSlideSubsystem extends SubsystemBase {

    public double vSlideOutput = 0;
    private final DcMotorEx vSlide;
    PIDCoefficients coefficients = new PIDCoefficients(SubConstants.vKp, SubConstants.vKi, SubConstants.vKd);
    BasicPID controller = new BasicPID(coefficients);
    public ElapsedTime slideTime = new ElapsedTime();
    public enum Deposit {
        hasCone, noCone
    }
    Deposit depositState = Deposit.hasCone;
    public vSlideSubsystem(final HardwareMap hMap) {
        vSlide =  hMap.get(DcMotorEx.class, "vslide");
    }

    public void vSlideToPosition(int targetPos){
        vSlideOutput = controller.calculate(targetPos, vSlide.getCurrentPosition());
        vSlide.setPower(vSlideOutput+0.1);
    }
    public double getSlideVelocity() { return vSlide.getVelocity();}
    public double getSlidePosition() { return vSlide.getCurrentPosition();}
    public boolean slideCurrentSpike() { return vSlide.isOverCurrent();}

}
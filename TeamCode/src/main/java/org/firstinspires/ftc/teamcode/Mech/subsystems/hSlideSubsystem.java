package org.firstinspires.ftc.teamcode.Mech.subsystems;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Mech.SubConstants;

public class hSlideSubsystem extends SubsystemBase {

    public double hSlideOutput = 0;
    private final DcMotorEx hSlide;
    PIDCoefficients coefficients = new PIDCoefficients(SubConstants.hKp, SubConstants.hKi, SubConstants.hKd);
    BasicPID controller = new BasicPID(coefficients);
    public ElapsedTime slideTime = new ElapsedTime();


    public hSlideSubsystem(final HardwareMap hMap) {
        hSlide =  hMap.get(DcMotorEx.class, "hslide");
    }

    public void hSlideToPosition(int targetPos){
        hSlideOutput = controller.calculate(targetPos, hSlide.getCurrentPosition());
        hSlide.setPower(hSlideOutput);
    }
    public void hSlideSetPower(double power){
        hSlide.setPower(power);
    }
    public double getSlideVelocity() { return hSlide.getVelocity();}
    public double getSlidePosition() { return hSlide.getCurrentPosition();}
    public boolean slideCurrentSpike() { return hSlide.isOverCurrent();}

}
package org.firstinspires.ftc.teamcode.subsystems;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;

import org.firstinspires.ftc.teamcode.subsystems.SubConstants;
import org.firstinspires.ftc.teamcode.subsystems.MotionProfile;

public class DepositSubsystem extends SubsystemBase {

    private final Servo aligner;
    public double output = 0;
    private final DcMotorEx vSlide;
    PIDCoefficients coefficients = new PIDCoefficients(SubConstants.vKp, SubConstants.vKp, SubConstants.vKp);
    BasicPID controller = new BasicPID(coefficients);
    public ElapsedTime slideTime = new ElapsedTime();
    public DepositSubsystem(final HardwareMap hMap) {
        aligner = hMap.get(Servo.class, "aligner");
        vSlide =  hMap.get(DcMotorEx.class, "vslide");
    }

    public void toPosition(int targetPos){
            output = controller.calculate(targetPos, vSlide.getCurrentPosition());
            vSlide.setPower(output+0.1);
    }
    public void openDeposit(){

    }
}
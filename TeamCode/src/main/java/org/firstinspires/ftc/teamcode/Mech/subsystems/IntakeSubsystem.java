package org.firstinspires.ftc.teamcode.Mech.subsystems;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Mech.SubConstants;

public class IntakeSubsystem extends SubsystemBase {

    private final Servo grotate, grabber;
    private final AnalogInput Pot;
    public double armOutput = 0;
    public double armAngle = 0;
    public double grotatePos = 0;
    private final DcMotorEx arm;
    PIDCoefficients coefficients = new PIDCoefficients(SubConstants.tKp, SubConstants.tKi, SubConstants.tKd);
    BasicPID controller = new BasicPID(coefficients);
    public ElapsedTime slideTime = new ElapsedTime();
    public enum Grabber {
        hasCone, noCone
    }
    Grabber grabberState = Grabber.hasCone;
    public IntakeSubsystem(final HardwareMap hMap) {
        grotate = hMap.get(Servo.class, "grotate");
        grabber = hMap.get(Servo.class, "grabber");
        arm =  hMap.get(DcMotorEx.class, "arm");
        Pot = hMap.get(AnalogInput.class, "armpot");
        armAngle = (Pot.getVoltage()-0.584)/SubConstants.degpervolt;
    }



    public void openGrabber(){
        grabber.setPosition(SubConstants.grabberOpen);
    }
    public void closeGrabber(){
        grabber.setPosition(SubConstants.grabberClose);
    }
    public void grotateToAngle(int angle){
//        grotatePos = SubConstants.grotatePosPerDeg*angle +(whatever the 0 degrees is)
        grotate.setPosition(grotatePos);
    }


    public double getArmVelocity() { return arm.getVelocity();}
    public double getArmAngle() { return armAngle;}
    public void armToAngle(int angle) {
            armOutput = controller.calculate(-15, angle)+(SubConstants.armFeedforward*Math.cos(Math.toRadians(angle)));
            arm.setPower(armOutput);}

    public boolean hasCone() {
        switch (grabberState) {
            case hasCone:
                return true;
        }
        return false;
    }
    public void hasCone(boolean decision){
        if(decision){grabberState = Grabber.hasCone;}
        else {grabberState = Grabber.noCone;}
    }

}
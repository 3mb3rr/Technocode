package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class robotHardware {
    public DcMotorEx vSlide;
    public DcMotorEx hSlide;
    public DcMotorEx arm;
    public Servo grabber;
    public Servo dropper;
    public Servo deposit;
    public Servo gRotate;
    public Servo aligner;
    public DigitalChannel vClose;
    public DigitalChannel hClose;
    public DigitalChannel turntable;
    public AnalogInput armpot;
    HardwareMap hMap;
    public robotHardware(HardwareMap hwMap) {
        hMap = hwMap;
        vSlide = hMap.get(DcMotorEx.class, "turntable");
    }

}

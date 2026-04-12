package org.firstinspires.ftc.teamcode.SUBSYSTEMS;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TransferSubsystem extends SubsystemBase {

    private Servo transfer;

    //test the values and input in the next two places
    private static final double servoClosed = 0.3;
    private static final double servoOpen = 0.52;

    public TransferSubsystem(HardwareMap hardwareMap){
        transfer = hardwareMap.get(Servo.class,"transfer");
    }

    public void Open(){

        transfer.setPosition(servoOpen);
    }

    public void Closed(){

        transfer.setPosition(servoClosed);
    }
}

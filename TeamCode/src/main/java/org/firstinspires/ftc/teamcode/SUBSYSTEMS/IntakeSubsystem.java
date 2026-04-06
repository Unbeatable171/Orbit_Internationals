package org.firstinspires.ftc.teamcode.SUBSYSTEMS;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem extends SubsystemBase {

    private final DcMotorEx intake;
    private final DcMotorEx intake2;

    private static final double intakeSpeed = 0.95;
    private static final double reverseSpeed = -0.8;

    private static final double transferSpeed = 0.8;

    public IntakeSubsystem(HardwareMap hardwareMap){

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake2 = hardwareMap.get(DcMotorEx.class, "intake2");

        configureMotor(intake);
        configureMotor(intake2);
    }
    public void on(){
        setPower(intakeSpeed);
    }
    public void reverse(){
        setPower(reverseSpeed);
    }
    public void off(){
        setPower(0);
    }

    public void transfer(){
        setPower(transferSpeed);
    }

    private void configureMotor(DcMotorEx motor) {
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    private void setPower(double power) {
        intake.setPower(power);
        intake2.setPower(power);
    }
}

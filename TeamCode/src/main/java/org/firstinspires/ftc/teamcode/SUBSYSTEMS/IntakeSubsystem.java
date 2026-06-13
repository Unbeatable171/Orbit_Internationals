package org.firstinspires.ftc.teamcode.SUBSYSTEMS;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class IntakeSubsystem extends SubsystemBase {

    private final DcMotorEx intake;
    private final DcMotorEx transfer;

    private static final double intakeSpeed = 1.0;
    private static final double intakeTransferSpeed = 1;
    private static final double reverseSpeed = -0.8;
    private static final double transferSpeed = 1.0;


    public IntakeSubsystem(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfermotor");

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void runAll() {
        intake.setPower(intakeSpeed);
        transfer.setPower(intakeTransferSpeed);
    }

    public void reverseAll() {
        intake.setPower(reverseSpeed);
        transfer.setPower(reverseSpeed);
    }

    public void off() {
        intake.setPower(0);
        transfer.setPower(0);
    }

    public void transfer() {
        intake.setPower(transferSpeed);
        transfer.setPower(transferSpeed);
    }
}
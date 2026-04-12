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

    private static final double intakeSpeed     = 1.0;
    private static final double reverseSpeed    = -0.8;
    private static final double transferSpeed   = 1.0;
    private static final double spikeThreshold  = 2.0;
    private static final int    SPIKE_CONFIRM   = 2;

    private int     spikeCount  = 0;
    private boolean ballDetected = false;

    private IntakeState state = IntakeState.OFF;

    public enum IntakeState {
        INTAKING,
        BALL_HELD_ONE,
        TRANSFERRING,
        REVERSE_STATE_ALL,
        REVERSE_STATE_INTAKE,
        OFF
    }

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intake   = hardwareMap.get(DcMotorEx.class, "intake");
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
        transfer.setPower(intakeSpeed);
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

//    @Override
//    public void periodic() {
//        // Spike detection only — never sets motor powers
//        if (state == IntakeState.INTAKING) {
//            if (transfer.getCurrent(CurrentUnit.AMPS) >= spikeThreshold) {
//                spikeCount++;
//                if (spikeCount >= SPIKE_CONFIRM) {
//                    ballDetected = true;
//                    spikeCount   = 0;
//                    state        = IntakeState.BALL_HELD_ONE;
//                }
//            } else {
//                spikeCount = 0;
//            }
//        }
//
//        switch (state) {
//            case INTAKING:
//                intake.setPower(intakeSpeed);
//                transfer.setPower(intakeSpeed);
//                break;
//            case BALL_HELD_ONE:
//                intake.setPower(intakeSpeed);
//                transfer.setPower(0);           // ball held — transfer stops
//                break;
//            case TRANSFERRING:
//                intake.setPower(transferSpeed);
//                transfer.setPower(transferSpeed);
//                break;
//            case REVERSE_STATE_ALL:
//                intake.setPower(reverseSpeed);
//                transfer.setPower(reverseSpeed);
//                break;
//            case REVERSE_STATE_INTAKE:
//                intake.setPower(reverseSpeed);
//                transfer.setPower(0);
//                break;
//            case OFF:
//            default:
//                intake.setPower(0);
//                transfer.setPower(0);
//                break;
//        }
//    }
//
//    public IntakeState getState()    { return state; }
//    public boolean isBallDetected()  { return ballDetected; }
//    public double currentTransferCurrent() { return transfer.getCurrent(CurrentUnit.AMPS); }
}
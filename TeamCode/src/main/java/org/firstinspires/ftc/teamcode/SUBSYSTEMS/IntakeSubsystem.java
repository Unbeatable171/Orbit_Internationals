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
    private static final double reverseSpeed = -0.8;
    private static final double transferSpeed = 1.0;
    private static final double spikeThreshold = 5.0;
    private static final int SPIKE_CONFIRM = 3;         // added
    private int spikeCount = 0;                         // added
    private boolean ballDetected = false;
    private IntakeState state = IntakeState.OFF;

    public enum IntakeState{
        INTAKING,
        BALL_HELD_ONE,
        TRANSFERRING,
        REVERSE_STATE_ALL,
        REVERSE_STATE_INTAKE,
        OFF
    }

    public IntakeSubsystem(HardwareMap hardwareMap){

        intake = hardwareMap.get(DcMotorEx.class,"intake");
        transfer = hardwareMap.get(DcMotorEx.class,"transfermotor");

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void runAll(){
        ballDetected = false;
        spikeCount = 0;                                 // reset on new intake cycle
        state = IntakeState.INTAKING;
    }

    public void reverseAll(){
        state = IntakeState.REVERSE_STATE_ALL;
    }

    public void reverseIntakeOnly(){
        state = IntakeState.REVERSE_STATE_INTAKE;
    }

    public void off(){
        ballDetected = false;
        spikeCount = 0;                                 // reset on off
        state = IntakeState.OFF;
    }

    public void transfer(){
        state = IntakeState.TRANSFERRING;
    }

    @Override
    public void periodic() {
        if (state == IntakeState.INTAKING) {
            double current = intake.getCurrent(CurrentUnit.AMPS);
            if (current >= spikeThreshold) {
                spikeCount++;                           // count consecutive spikes
                if (spikeCount >= SPIKE_CONFIRM) {
                    ballDetected = true;
                    state = IntakeState.BALL_HELD_ONE;
                    spikeCount = 0;                     // reset for next cycle
                }
            } else {
                spikeCount = 0;                         // not consecutive, reset
            }
        }

        switch (state){
            case INTAKING:
                intake.setPower(intakeSpeed);
                transfer.setPower(intakeSpeed);
                break;
            case BALL_HELD_ONE:
                intake.setPower(intakeSpeed);
                transfer.setPower(0);
                break;
            case TRANSFERRING:
                intake.setPower(transferSpeed);
                transfer.setPower(transferSpeed);
                break;
            case REVERSE_STATE_ALL:
                intake.setPower(reverseSpeed);
                transfer.setPower(reverseSpeed);
                break;
            case REVERSE_STATE_INTAKE:
                intake.setPower(reverseSpeed);
                transfer.setPower(0);
                break;
            case OFF:
            default:
                intake.setPower(0);
                transfer.setPower(0);
                break;
        }
    }

    public IntakeState getState() {
        return state;
    }

    public boolean isBallDetected() {
        return ballDetected;
    }
}
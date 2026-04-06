package org.firstinspires.ftc.teamcode.ACTIONS;

import org.firstinspires.ftc.teamcode.SUBSYSTEMS.IntakeSubsystem;

public class IntakeAction {

    private final IntakeSubsystem intakeSubsystem;

    public IntakeAction(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }

    public void startIntake() {
        intakeSubsystem.runAll();  // handles BALL_HELD automatically
    }

    public void stop() {
        intakeSubsystem.off();
    }

    public boolean hasBall() {
        return intakeSubsystem.isBallDetected();
    }

    public boolean isHoldingBall() {
        return intakeSubsystem.getState() ==
                IntakeSubsystem.IntakeState.BALL_HELD_ONE;
    }
}
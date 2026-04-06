package org.firstinspires.ftc.teamcode.ACTIONS;

import org.firstinspires.ftc.teamcode.SUBSYSTEMS.IntakeSubsystem;

public class IntakeAction {

    private final IntakeSubsystem intakeSubsystem;

    public IntakeAction(IntakeSubsystem intakeSubsystem) {

        this.intakeSubsystem = intakeSubsystem;
    }

    public void startIntake() {

        intakeSubsystem.on();
    }

    public void reverse() {

        intakeSubsystem.reverse();
    }

    public void stop() {

        intakeSubsystem.off();
    }
}

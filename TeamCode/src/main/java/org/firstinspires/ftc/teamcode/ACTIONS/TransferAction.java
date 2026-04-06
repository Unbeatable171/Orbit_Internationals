package org.firstinspires.ftc.teamcode.ACTIONS;

import org.firstinspires.ftc.teamcode.SUBSYSTEMS.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.SUBSYSTEMS.TransferSubsystem;

public class TransferAction {

    private final TransferSubsystem transferSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    public TransferAction(TransferSubsystem transferSubsystem, IntakeSubsystem intakeSubsystem) {
        this.transferSubsystem = transferSubsystem;
        this.intakeSubsystem = intakeSubsystem;
    }

    public void start() {
        transferSubsystem.Open();
        intakeSubsystem.transfer();
    }
}

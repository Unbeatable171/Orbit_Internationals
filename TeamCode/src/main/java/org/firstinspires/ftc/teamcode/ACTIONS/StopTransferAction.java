package org.firstinspires.ftc.teamcode.ACTIONS;

import org.firstinspires.ftc.teamcode.SUBSYSTEMS.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.SUBSYSTEMS.TransferSubsystem;

public class StopTransferAction {

    private final TransferSubsystem transferSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    public StopTransferAction(TransferSubsystem transferSubsystem, IntakeSubsystem intakeSubsystem) {
        this.transferSubsystem = transferSubsystem;
        this.intakeSubsystem = intakeSubsystem;
    }

    public void run() {
        transferSubsystem.Closed();
        intakeSubsystem.off();
    }
}

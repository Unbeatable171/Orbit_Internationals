package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

//@TeleOp(name = "Hood Servo Control", group = "TeleOp")
public class ServoPositionFinderAryan extends OpMode {

    // Hardware
    private Servo hoodServo;

    // Configuration
    private static final String SERVO_NAME = "hoodServo";

    private static final double HOOD_HOME = 1.0;
    private static final double STEP = 1;

    private static final double SWEEP_STEP = 0.01;
    private static final int SWEEP_DELAY = 20;

    // State
    private double currentPosition = 0.0;

    private boolean isHoming = false;

    private boolean lastAButton = false;
    private boolean lastDpadDown = false;

    @Override
    public void init() {

        hoodServo = hardwareMap.get(Servo.class, SERVO_NAME);

        // IMPORTANT:
        // Do NOT command the servo to 0.0 here.
        // Let it stay wherever it currently is.
        currentPosition = 0.0;

        telemetry.addLine("Hood Servo Control Ready");
        telemetry.addLine("A = Sweep to 1.0");
        telemetry.addLine("DPad Down = -0.02");
        telemetry.update();
    }

    @Override
    public void start() {

        // Start homing sweep immediately when START is pressed
        isHoming = true;
    }

    @Override
    public void loop() {

        // ---------------------------------------------------
        // HOMING SWEEP
        // ---------------------------------------------------
        if (isHoming) {

            currentPosition += SWEEP_STEP;

            if (currentPosition >= HOOD_HOME) {
                currentPosition = HOOD_HOME;
                isHoming = false;
            }

            hoodServo.setPosition(currentPosition);

            sleep(SWEEP_DELAY);

            telemetry.addLine("=== HOMING ===");
            telemetry.addData("Position", "%.2f", currentPosition);
            telemetry.update();

            return;
        }

        // ---------------------------------------------------
        // A BUTTON -> Sweep to 1.0
        // ---------------------------------------------------
        boolean aPressed = gamepad1.a;

        if (aPressed && !lastAButton) {
            isHoming = true;
        }

        lastAButton = aPressed;

        // ---------------------------------------------------
        // DPAD DOWN -> Decrease Position
        // ---------------------------------------------------
        boolean dpadDown = gamepad1.dpad_down;

        if (dpadDown && !lastDpadDown) {
            currentPosition -= STEP;
        }

        lastDpadDown = dpadDown;

        // Clamp range
        currentPosition = Math.max(0.0, Math.min(1.0, currentPosition));

        // Update servo
        hoodServo.setPosition(currentPosition);

        // Telemetry
        telemetry.addLine("=== HOOD SERVO CONTROL ===");
        telemetry.addData("Position", "%.2f", currentPosition);
        telemetry.addData("Angle Estimate", "%.1f°", currentPosition * 270.0);
        telemetry.addLine("");
        telemetry.addLine("A = Sweep to 1.0");
        telemetry.addLine("DPad Down = -0.02");
        telemetry.update();
    }

    private void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
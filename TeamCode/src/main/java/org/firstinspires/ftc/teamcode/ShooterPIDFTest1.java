
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Shooter PIDF Tuning")
public class ShooterPIDFTest1 extends LinearOpMode {

    private DcMotorEx shooter;
    private DcMotorEx intakeMotor;
    private Servo shooterGate = null;

    private double targetVelocity = 1150;

    // Starting PIDF values
    private double P = 5.9;
    private double I = 3.035;
    private double D = 0.017;
    private double F = 1.98;

    private boolean fineMode = false;
    // Shooter Gate Positions
    static final double GATE_OUT = 0.2; // Gate Open
    static final double GATE_IN = 0.0; // Gate Closed

    // Button press detection
    private boolean prevA, prevB;
    private boolean prevUp, prevDown;
    private boolean prevLeft, prevRight;
    private boolean prevX, prevY;
    private boolean prevLB, prevRB;
    private boolean prevStick;

    @Override
    public void runOpMode() {

        shooter     = hardwareMap.get(DcMotorEx.class, "shooter");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        shooterGate = hardwareMap.get(Servo.class, "shooterGate");

        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        // Apply starting PIDF values
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));

        // Set Shooter Gate position to closed
        shooterGate.setPosition(GATE_IN);

        telemetry.addLine("SHOOTER PIDF TUNING");
        telemetry.addLine("");
        telemetry.addData("Starting P", P);
        telemetry.addData("Starting I", I);
        telemetry.addData("Starting D", D);
        telemetry.addData("Starting F", F);
        telemetry.addLine("");
        telemetry.addLine("Press START to begin");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Intake Motor
            intakeMotor.setVelocity(1120);

            // Adjustment step sizes
            double pStep = fineMode ? 0.01 : 0.1;
            double iStep = fineMode ? 0.0001 : 0.001;
            double dStep = fineMode ? 0.0001 : 0.001;
            double fStep = fineMode ? 0.01 : 0.1;

            double velocityStep = fineMode ? 1 : 10;

            // Toggle fine mode
            if (gamepad1.left_stick_button && !prevStick) {
                fineMode = !fineMode;
            }

            // Target velocity
            if (gamepad1.a && !prevA) {
                targetVelocity += velocityStep;
            }

            if (gamepad1.b && !prevB) {
                targetVelocity -= velocityStep;
            }

            targetVelocity = Math.max(0, targetVelocity);

            // P adjustment
            if (gamepad1.dpad_up && !prevUp) {
                P += pStep;
            }

            if (gamepad1.dpad_down && !prevDown) {
                P -= pStep;
            }

            // F adjustment
            if (gamepad1.dpad_right && !prevRight) {
                F += fStep;
            }

            if (gamepad1.dpad_left && !prevLeft) {
                F -= fStep;
            }

            // I adjustment
            if (gamepad1.x && !prevX) {
                I += iStep;
            }

            if (gamepad1.y && !prevY) {
                I -= iStep;
            }

            // D adjustment
            if (gamepad1.right_bumper && !prevRB) {
                D += dStep;
            }

            if (gamepad1.left_bumper && !prevLB) {
                D -= dStep;
            }

            // Prevent negative coefficients
            P = Math.max(0, P);
            I = Math.max(0, I);
            D = Math.max(0, D);
            F = Math.max(0, F);

            // Update previous button states
            prevA = gamepad1.a;
            prevB = gamepad1.b;

            prevUp = gamepad1.dpad_up;
            prevDown = gamepad1.dpad_down;

            prevLeft = gamepad1.dpad_left;
            prevRight = gamepad1.dpad_right;

            prevX = gamepad1.x;
            prevY = gamepad1.y;

            prevLB = gamepad1.left_bumper;
            prevRB = gamepad1.right_bumper;

            prevStick = gamepad1.left_stick_button;

            // Apply updated PIDF values
            shooter.setPIDFCoefficients(
                    DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(P, I, D, F)
            );

            // Set shooter velocity
            shooter.setVelocity(targetVelocity);

            // Read actual velocity
            double actualVelocity = shooter.getVelocity();

            // Calculate velocity error
            double error = targetVelocity - actualVelocity;

            double errorPercent = targetVelocity > 0
                    ? (Math.abs(error) / targetVelocity) * 100
                    : 0;

            // Telemetry
            telemetry.addLine("Shooter PIDF");

            telemetry.addLine("Velocity");
            telemetry.addData("Target Velocity", "%.2f ticks/sec", targetVelocity);
            telemetry.addData("Actual Velocity", "%.2f ticks/sec", actualVelocity);
            telemetry.addData("Velocity Error", "%.2f ticks/sec", error);
            telemetry.addData("Error Percentage", "%.3f%%", errorPercent);
            telemetry.addLine("");

            telemetry.addLine("PIDF Coefficients");
            telemetry.addData("P", "%.5f", P);
            telemetry.addData("I", "%.5f", I);
            telemetry.addData("D", "%.5f", D);
            telemetry.addData("F", "%.5f", F);
            telemetry.addLine("");

            telemetry.addData("Adjustment Mode", fineMode ? "FINE" : "NORMAL");

            telemetry.addData("Velocity Step", "%.2f", velocityStep);
            telemetry.addLine("");
            telemetry.addLine("A/B: Target Velocity +/-");
            telemetry.addLine("Dpad Left/Right: F +/-");
            telemetry.addLine("Dpad Up/Down: P +/-");
            telemetry.addLine("Bumpers: D +/-");
            telemetry.addLine("X/Y: I +/-");
            telemetry.addLine("Left Stick Button: Toggle Fine Mode");

            telemetry.update();

            sleep(2000);
            intakeMotor.setVelocity(2800);
            sleep(1000);
            shooterGate.setPosition(GATE_OUT);
            sleep(10000);
        }

        // Stop shooter
        shooter.setVelocity(0);
        shooter.setPower(0);
    }
}

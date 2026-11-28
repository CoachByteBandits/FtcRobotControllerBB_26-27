package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "Shooter PIDF Tuning")
public class ShooterPIDFTest1 extends LinearOpMode {
    private DcMotorEx shooter;
    private double targetVelocity = 1200; //TPS

    // PIDF coefficients
    private double P;
    private double I;
    private double D;
    private double F;

    // Adjustment modes
    private boolean fineMode = false;

    // Button press detection
    private boolean prevA, prevB;
    private boolean prevUp, prevDown;
    private boolean prevLeft, prevRight;
    private boolean prevX, prevY;
    private boolean prevLB, prevRB;
    private boolean prevStick;

    @Override
    public void runOpMode() {

        shooter = hardwareMap.get(
                DcMotorEx.class, "shooter"
        );

        shooter.setDirection(DcMotor.Direction.FORWARD);

        shooter.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        shooter.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );

        // Read existing motor controller PIDF values
        PIDFCoefficients currentPIDF =
                shooter.getPIDFCoefficients(
                        DcMotor.RunMode.RUN_USING_ENCODER
                );

        P = currentPIDF.p;
        I = currentPIDF.i;
        D = currentPIDF.d;
        F = currentPIDF.f;

        telemetry.addLine("SHOOTER PIDF TUNING");
        telemetry.addLine("-------------------------");
        telemetry.addData("Initial P", P);
        telemetry.addData("Initial I", I);
        telemetry.addData("Initial D", D);
        telemetry.addData("Initial F", F);
        telemetry.addLine("-------------------------");
        telemetry.addLine("Press START to begin");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // =================================
            // ADJUSTMENT STEP SIZES
            // =================================

            double pStep = fineMode ? 0.01 : 0.1;
            double iStep = fineMode ? 0.0001 : 0.001;
            double dStep = fineMode ? 0.0001 : 0.001;
            double fStep = fineMode ? 0.01 : 0.1;

            double velocityStep = fineMode ? 1 : 10;

            // Toggle fine adjustment mode
            if (gamepad1.left_stick_button && !prevStick) {
                fineMode = !fineMode;
            }

            // =================================
            // TARGET VELOCITY
            // =================================

            if (gamepad1.a && !prevA) {
                targetVelocity += velocityStep;
            }

            if (gamepad1.b && !prevB) {
                targetVelocity -= velocityStep;
            }

            targetVelocity = Math.max(0, targetVelocity);

            // =================================
            // P COEFFICIENT
            // =================================

            if (gamepad1.dpad_up && !prevUp) {
                P += pStep;
            }

            if (gamepad1.dpad_down && !prevDown) {
                P -= pStep;
            }

            // =================================
            // F COEFFICIENT
            // =================================

            if (gamepad1.dpad_right && !prevRight) {
                F += fStep;
            }

            if (gamepad1.dpad_left && !prevLeft) {
                F -= fStep;
            }

            // =================================
            // I COEFFICIENT
            // =================================

            if (gamepad1.x && !prevX) {
                I += iStep;
            }

            if (gamepad1.y && !prevY) {
                I -= iStep;
            }

            // =================================
            // D COEFFICIENT
            // =================================

            if (gamepad1.right_bumper && !prevRB) {
                D += dStep;
            }

            if (gamepad1.left_bumper && !prevLB) {
                D -= dStep;
            }

            // Prevent negative PIDF values
            P = Math.max(0, P);
            I = Math.max(0, I);
            D = Math.max(0, D);
            F = Math.max(0, F);

            // =================================
            // UPDATE PREVIOUS BUTTON STATES
            // =================================

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

            // =================================
            // APPLY PIDF TO MOTOR CONTROLLER
            // =================================

            shooter.setPIDFCoefficients(
                    DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(P, I, D, F)
            );

            // =================================
            // SHOOTER VELOCITY CONTROL
            // =================================

            shooter.setVelocity(targetVelocity);

            double actualVelocity = shooter.getVelocity();

            double error = targetVelocity - actualVelocity;

            double errorPercent = targetVelocity > 0? (Math.abs(error) / targetVelocity) * 100: 0;

            // Telemetry

            telemetry.addLine("========== SHOOTER PIDF ==========");
            telemetry.addLine("--- VELOCITY ---");
            telemetry.addData("Target Velocity", "%.2f ticks/sec", targetVelocity);
            telemetry.addData("Actual Velocity", "%.2f ticks/sec", actualVelocity);
            telemetry.addData("Velocity Error", "%.2f ticks/sec", error);
            telemetry.addData("Error Percentage", "%.3f%%", errorPercent);
            telemetry.addLine("-------------------------------");
            telemetry.addLine("--- PIDF COEFFICIENTS ---");
            telemetry.addData("P", "%.5f", P);
            telemetry.addData("I", "%.5f", I);
            telemetry.addData("D", "%.5f", D);
            telemetry.addData("F", "%.5f", F);
            telemetry.addLine("-------------------------------");
            telemetry.addData("Adjustment Mode", fineMode ? "FINE" : "NORMAL");
            telemetry.addData("Velocity Step", "%.2f", velocityStep);
            telemetry.addLine("-------------------------------");
            telemetry.addLine("A/B: Target Velocity +/-");
            telemetry.addLine("Dpad Up/Down: P +/-");
            telemetry.addLine("Dpad Left/Right: F +/-");
            telemetry.addLine("X/Y: I +/-");
            telemetry.addLine("Bumpers: D +/-");
            telemetry.addLine("Left Stick Button: Toggle Fine Mode");
            telemetry.update();

            sleep(50);
        }

        // Stop shooter
        shooter.setVelocity(0);
        shooter.setPower(0);
    }
}

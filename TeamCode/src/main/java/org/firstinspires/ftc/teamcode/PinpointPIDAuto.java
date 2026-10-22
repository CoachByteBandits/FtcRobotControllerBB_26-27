package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "PinpointPIDAuto", group = "PID")
public class PinpointPIDAuto extends LinearOpMode {

    // ---------------- HARDWARE ----------------

    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    private GoBildaPinpointDriver pinpoint;

    // ---------------- PID CONSTANTS ----------------
    // Position gains are per INCH of error. Turn gains are per RADIAN of error.

    static final double DRIVE_KP = 0.05;
    static final double DRIVE_KI = 0.0;
    static final double DRIVE_KD = 0.002;

    static final double TURN_KP = 2.0;
    static final double TURN_KI = 0.0;
    static final double TURN_KD = 0.05;

    static final double STRAFE_KP = 0.06;
    static final double STRAFE_KI = 0.0;
    static final double STRAFE_KD = 0.002;

    // Overall speed limit. Raise toward 1.0 once the Init test passes.
    static final double POWER_SCALE = 0.8;

    // Max turn power while driving/strafing/holding (high enough to recover from a hit)
    static final double HOLD_TURN_LIMIT = 0.5;

    // Minimum powers so the robot doesn't stall just short of the target.
    // Only apply while the error is bigger than the deadband.
    static final double MIN_STRAFE_POWER = 0.22;
    static final double MIN_DRIVE_POWER = 0.15;
    static final double MIN_TURN_POWER = 0.12;
    static final double MIN_POWER_DEADBAND_IN = 0.6;
    static final double MIN_TURN_DEADBAND_RAD = Math.toRadians(1.5);

    // ---------------- AXIS SETUP (set from the Init push test) ----------------
    // The code works in this frame:
    //   Field Y goes UP when you push the robot FORWARD
    //   Field X goes UP when you push the robot to the RIGHT
    //   Heading goes UP when you rotate the robot COUNTER-CLOCKWISE
    //
    // SWAP_XY  : true if pushing FORWARD changes raw X instead of raw Y
    // X_SIGN   : -1 if Field X goes DOWN when you push RIGHT
    // Y_SIGN   : -1 if Field Y goes DOWN when you push FORWARD
    // H_SIGN   : -1 if Heading goes DOWN when you rotate CCW
    static final boolean SWAP_XY = true;
    static final double X_SIGN = -1;
    static final double Y_SIGN = 1;
    static final double H_SIGN = 1;

    // Keep your existing Pinpoint offsets.
    static final double PINPOINT_OFFSET_X = -82;
    static final double PINPOINT_OFFSET_Y = -123;

    private final PIDController drivePID =
            new PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD);

    private final PIDController headingPID =
            new PIDController(TURN_KP, TURN_KI, TURN_KD);

    private final PIDController strafePID =
            new PIDController(STRAFE_KP, STRAFE_KI, STRAFE_KD);

    // ---------------- PERSISTENT FIELD TARGET ----------------

    private double targetFieldX;
    private double targetFieldY;
    private double targetHeadingDegrees;

    // ---------------- TOLERANCES ----------------

    private static final double POSITION_TOLERANCE_IN = 2.0;
    private static final double HEADING_TOLERANCE_DEG = 4.0;

    private static final long MOVEMENT_TIMEOUT_MS = 2000;

    // ---------------- BACKGROUND CONTROL THREAD ----------------

    private Thread controlThread;
    private final Object targetLock = new Object();
    private volatile boolean controlRunning = false;
    private volatile boolean resetPIDs = false;
    private volatile double maxTurnPower = HOLD_TURN_LIMIT;

    private volatile double posErrorNow = 999;
    private volatile double headingErrorNowDeg = 999;
    private volatile double currentXNow, currentYNow, currentHeadingNowDeg;

    // ---------------- INITIALIZATION ----------------

    @Override
    public void runOpMode() {

        leftFrontDrive =
                hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive =
                hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive =
                hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive =
                hardwareMap.get(DcMotor.class, "rightBackDrive");

        pinpoint =
                hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pinpoint.setOffsets(
                PINPOINT_OFFSET_X,
                PINPOINT_OFFSET_Y,
                DistanceUnit.MM
        );

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        // ---- INIT TEST: push the robot by hand and watch these numbers ----
        // Push FORWARD  -> Field Y must go UP
        // Push RIGHT    -> Field X must go UP
        // Rotate CCW    -> Heading must go UP
        while (!isStarted() && !isStopRequested()) {
            pinpoint.update();
            Pose2D p = pinpoint.getPosition();
            double[] f = readPose();
            telemetry.addLine("INIT TEST: push robot by hand (do not press Start)");
            telemetry.addData("Field X (up = RIGHT)", "%.2f", f[0]);
            telemetry.addData("Field Y (up = FORWARD)", "%.2f", f[1]);
            telemetry.addData("Heading (up = CCW)", "%.1f", f[2]);
            telemetry.addLine("--- raw Pinpoint values ---");
            telemetry.addData("Raw X", "%.2f", p.getX(DistanceUnit.INCH));
            telemetry.addData("Raw Y", "%.2f", p.getY(DistanceUnit.INCH));
            telemetry.addData("Raw Heading", "%.1f", p.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }

        if (isStopRequested()) {
            stopMotors();
            return;
        }

        pinpoint.resetPosAndIMU();
        sleep(500);

        double[] start = readPose();
        targetFieldX = start[0];
        targetFieldY = start[1];
        targetHeadingDegrees = start[2];

        // ---------------- AUTONOMOUS ROUTINE ----------------

        controlRunning = true;
        controlThread = new Thread(this::controlLoop);
        controlThread.start();

        try {
            driveStraight(24, 0);
            holdPoseFor(100, 0);

            turnToAngle(88);
            holdPoseFor(100, 88);

            driveStraight(24, 88);
            holdPoseFor(100, 88);

            strafeLeft(24, 88);
            holdPoseFor(100, 88);

            driveStraight(-24, 88);
            holdPoseFor(100, 88);

            driveStraight(24, 88);
            holdPoseFor(100, 88);

            strafeLeft(-24, 88);
            holdPoseFor(100, 88);

            driveStraight(-24, 88);
            holdPoseFor(100, 88);

            turnToAngle(0);
            holdPoseFor(100, 0);

            driveStraight(-24, 0);
            holdPoseFor(100, 0);

            // Keep holding the final pose until auto ends.
            while (opModeIsActive()) {
                updateTelemetry();
                idle();
            }
        } finally {
            controlRunning = false;
            try { controlThread.join(500); } catch (InterruptedException ignored) {}
            stopMotors();
        }
    }

    // ---------------- POSE READING ----------------
    // Returns {fieldX, fieldY, headingDeg} in the frame this code uses.

    private double[] readPose() {
        pinpoint.update();
        Pose2D p = pinpoint.getPosition();
        double rawX = p.getX(DistanceUnit.INCH);
        double rawY = p.getY(DistanceUnit.INCH);
        double h = p.getHeading(AngleUnit.DEGREES);

        double fx = SWAP_XY ? rawY : rawX;
        double fy = SWAP_XY ? rawX : rawY;

        return new double[]{X_SIGN * fx, Y_SIGN * fy, H_SIGN * h};
    }

    // ---------------- MOVEMENT COMMANDS ----------------

    private void driveStraight(double inches, double headingDegrees) {
        double angle = Math.toRadians(headingDegrees);
        synchronized (targetLock) {
            targetHeadingDegrees = headingDegrees;
            targetFieldX -= inches * Math.sin(angle);
            targetFieldY += inches * Math.cos(angle);
        }
        maxTurnPower = HOLD_TURN_LIMIT;
        resetPIDs = true;
        waitUntilSettled(MOVEMENT_TIMEOUT_MS);
    }

    private void strafeLeft(double inches, double headingDegrees) {
        double angle = Math.toRadians(headingDegrees);
        synchronized (targetLock) {
            targetHeadingDegrees = headingDegrees;
            targetFieldX -= inches * Math.cos(angle);
            targetFieldY -= inches * Math.sin(angle);
        }
        maxTurnPower = HOLD_TURN_LIMIT;
        resetPIDs = true;
        waitUntilSettled(MOVEMENT_TIMEOUT_MS);
    }

    private void turnToAngle(double targetAngleDegrees) {
        synchronized (targetLock) {
            targetHeadingDegrees = targetAngleDegrees;
        }
        maxTurnPower = 0.85;
        resetPIDs = true;
        waitUntilSettled(MOVEMENT_TIMEOUT_MS);
    }

    private void holdPoseFor(long durationMs, double headingDegrees) {
        synchronized (targetLock) {
            targetHeadingDegrees = headingDegrees;
        }
        maxTurnPower = HOLD_TURN_LIMIT;
        long end = System.currentTimeMillis() + durationMs;
        while (opModeIsActive() && System.currentTimeMillis() < end) {
            updateTelemetry();
            idle();
        }
    }

    private void waitUntilSettled(long timeoutMs) {
        sleep(30);
        long end = System.currentTimeMillis() + timeoutMs;
        while (opModeIsActive() && System.currentTimeMillis() < end) {
            if (posErrorNow < POSITION_TOLERANCE_IN
                    && Math.abs(headingErrorNowDeg) < HEADING_TOLERANCE_DEG) {
                break;
            }
            updateTelemetry();
            idle();
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Target X", "%.2f", targetFieldX);
        telemetry.addData("Actual X", "%.2f", currentXNow);
        telemetry.addData("Target Y", "%.2f", targetFieldY);
        telemetry.addData("Actual Y", "%.2f", currentYNow);
        telemetry.addData("Position Error", "%.2f in", posErrorNow);
        telemetry.addData("Target Heading", "%.1f", targetHeadingDegrees);
        telemetry.addData("Actual Heading", "%.1f", currentHeadingNowDeg);
        telemetry.addData("Heading Error", "%.1f deg", headingErrorNowDeg);
        telemetry.update();
    }

    // ---------------- CONTINUOUS POSE CONTROL (background thread) ----------------

    private void controlLoop() {
        while (controlRunning && !Thread.currentThread().isInterrupted()) {

            double targetX, targetY, targetHeadingDeg;
            synchronized (targetLock) {
                targetX = targetFieldX;
                targetY = targetFieldY;
                targetHeadingDeg = targetHeadingDegrees;
            }

            if (resetPIDs) {
                drivePID.reset();
                strafePID.reset();
                headingPID.reset();
                resetPIDs = false;
            }

            drivePID.setSetPoint(0);
            strafePID.setSetPoint(0);
            headingPID.setSetPoint(Math.toRadians(targetHeadingDeg));
            drivePID.setOutputRange(0.80);
            strafePID.setOutputRange(0.60);
            headingPID.setOutputRange(maxTurnPower);

            double[] pose = readPose();
            double currentX = pose[0];
            double currentY = pose[1];
            double headingRadians = Math.toRadians(pose[2]);

            double errorX = targetX - currentX;
            double errorY = targetY - currentY;

            double headingErrorRadians =
                    Math.toRadians(targetHeadingDeg) - headingRadians;
            while (headingErrorRadians > Math.PI) headingErrorRadians -= 2 * Math.PI;
            while (headingErrorRadians < -Math.PI) headingErrorRadians += 2 * Math.PI;

            // Field error -> robot-relative error
            double forwardError =
                    -errorX * Math.sin(headingRadians) + errorY * Math.cos(headingRadians);
            double leftError =
                    -errorX * Math.cos(headingRadians) - errorY * Math.sin(headingRadians);

            double forwardPower = -drivePID.getComputedOutput(forwardError);
            double leftPower = -strafePID.getComputedOutput(leftError);
            double turnPower = headingPID.getComputedAngleOutput(headingRadians);

            // Minimum power: keeps the robot from stalling just short of the target.
            leftPower = applyMinPower(leftPower, leftError, MIN_STRAFE_POWER);
            forwardPower = applyMinPower(forwardPower, forwardError, MIN_DRIVE_POWER);

            if (Math.abs(headingErrorRadians) > MIN_TURN_DEADBAND_RAD
                    && Math.abs(turnPower) < MIN_TURN_POWER) {
                turnPower = Math.signum(turnPower) * MIN_TURN_POWER;
            }

            posErrorNow = Math.hypot(errorX, errorY);
            headingErrorNowDeg = Math.toDegrees(headingErrorRadians);
            currentXNow = currentX;
            currentYNow = currentY;
            currentHeadingNowDeg = Math.toDegrees(headingRadians);

            double lf = forwardPower - leftPower - turnPower;
            double rf = forwardPower + leftPower + turnPower;
            double lb = forwardPower + leftPower - turnPower;
            double rb = forwardPower - leftPower + turnPower;

            double max = Math.max(1.0,
                    Math.max(Math.max(Math.abs(lf), Math.abs(rf)),
                            Math.max(Math.abs(lb), Math.abs(rb))));

            leftFrontDrive.setPower(POWER_SCALE * lf / max);
            rightFrontDrive.setPower(POWER_SCALE * rf / max);
            leftBackDrive.setPower(POWER_SCALE * lb / max);
            rightBackDrive.setPower(POWER_SCALE * rb / max);

            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                break;
            }
        }
        stopMotors();
    }

    // ---------------- MINIMUM POWER HELPER ----------------

    private double applyMinPower(double power, double error, double minPower) {
        if (Math.abs(error) > MIN_POWER_DEADBAND_IN && Math.abs(power) < minPower) {
            return Math.signum(error) * minPower;
        }
        return power;
    }

    // ---------------- STOP MOTORS ----------------

    private void stopMotors() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}
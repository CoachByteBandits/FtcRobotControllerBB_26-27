package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "AutoLayout7VTest", group = "Linear OPMODE")
public class AutoLayout7Vel extends LinearOpMode {

    // ---------------- HARDWARE ----------------

    private DcMotorEx leftFrontDrive;
    private DcMotorEx rightFrontDrive;
    private DcMotorEx leftBackDrive;
    private DcMotorEx rightBackDrive;

    private CRServo leftFlower = null;
    private CRServo rightFlower = null;

    private Servo shooterGate = null;

    private DcMotorEx shooter = null;
    private DcMotorEx intakeMotor;

    private DigitalChannel limitSwitch;

    private GoBildaPinpointDriver pinpoint;

    // ---------------- DRIVETRAIN PID CONSTANTS ----------------

    static final double DRIVE_KP = 0.05;
    static final double DRIVE_KI = 0.0;
    static final double DRIVE_KD = 0.003;

    static final double TURN_KP = 2.0;
    static final double TURN_KI = 0.0;
    static final double TURN_KD = 0.08;

    static final double STRAFE_KP = 0.06;
    static final double STRAFE_KI = 0.0;
    static final double STRAFE_KD = 0.003;

    // ---------------- SHOOTER SETTINGS ----------------

    static final double SHOOTER_VELOCITY = 1100;

    static final double GATE_OUT = 0.2;
    static final double GATE_IN = 0.0;

    // ---------------- INTAKE SETTINGS ----------------

    static final double INTAKE_POWER = 1.0;

    // ---------------- FLOWER SETTINGS ----------------

    static final double FLOWER_UP_POWER = -0.8;
    static final double FLOWER_UP_TIMEOUT_SEC = 3.0;

    static final double POWER_SCALE = 0.8;
    static final double HOLD_TURN_LIMIT = 0.5;

    static final double MIN_STRAFE_POWER = 0.22;
    static final double MIN_DRIVE_POWER = 0.15;
    static final double MIN_TURN_POWER = 0.12;
    static final double MIN_POWER_DEADBAND_IN = 0.6;
    static final double MIN_TURN_DEADBAND_RAD = Math.toRadians(1.5);

    // ---------------- AXIS SETUP ----------------

    static final boolean SWAP_XY = true;
    static final double X_SIGN = -1;
    static final double Y_SIGN = 1;
    static final double H_SIGN = 1;

    static final double PINPOINT_OFFSET_X = -82;
    static final double PINPOINT_OFFSET_Y = -123;

    // ---------------- FIELD COORDINATES & TIMING ----------------

    static final double TILE_IN = 24;

    static final double START_X = 0 * TILE_IN;
    static final double START_Y = 0 * TILE_IN;
    static final double START_HEADING_DEG = 0;

    private static final long GOTO_TIMEOUT_MS = 3500;
    private static final long MOVEMENT_TIMEOUT_MS = 2000;

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

    // ---------------- BACKGROUND CONTROL THREAD ----------------

    private Thread controlThread;
    private final Object targetLock = new Object();

    private volatile boolean controlRunning = false;
    private volatile boolean resetPIDs = false;
    private volatile double maxTurnPower = HOLD_TURN_LIMIT;

    private volatile double posErrorNow = 999;
    private volatile double headingErrorNowDeg = 999;
    private volatile double currentXNow, currentYNow, currentHeadingNowDeg;
    private volatile double shooterTargetVelocity = 0;

    // ---------------- FLOWER HOMING ----------------

    private boolean homeFlower() {

        long startTime = System.currentTimeMillis();

        telemetry.addLine("Flower moving UP to limit switch");
        addShooterTelemetry();
        telemetry.update();

        while (opModeIsActive()
                && !isStopRequested()
                && !limitSwitch.getState()
                && (System.currentTimeMillis() - startTime)
                < FLOWER_UP_TIMEOUT_SEC * 1000) {

            leftFlower.setPower(FLOWER_UP_POWER);
            rightFlower.setPower(FLOWER_UP_POWER);

            telemetry.addData("Flower", "Moving UP");
            telemetry.addData("Limit Switch", "NOT PRESSED");
            addShooterTelemetry();
            telemetry.update();

            idle();
        }

        leftFlower.setPower(0.0);
        rightFlower.setPower(0.0);

        boolean pressed = limitSwitch.getState();

        telemetry.addData("Flower", pressed ? "UP - PRESSED" : "HOMING TIMEOUT");
        telemetry.addData("Limit Switch", pressed ? "PRESSED" : "NOT PRESSED");
        addShooterTelemetry();
        telemetry.update();

        return pressed;
    }

    // ---------------- INITIALIZATION ----------------

    @Override
    public void runOpMode() {

        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rightBackDrive");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        leftFlower = hardwareMap.get(CRServo.class, "leftFlower");
        rightFlower = hardwareMap.get(CRServo.class, "rightFlower");

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooterGate = hardwareMap.get(Servo.class, "shooterGate");

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        limitSwitch = hardwareMap.get(DigitalChannel.class, "flowerLimit");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        // INTAKE SETUP

        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // DRIVE MOTOR DIRECTIONS

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // SHOOTER SETUP

        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // FLOWER DIRECTIONS - SAME AS TELEOP

        leftFlower.setDirection(CRServo.Direction.REVERSE);
        rightFlower.setDirection(CRServo.Direction.REVERSE);

        // SHOOTER GATE

        shooterGate.setDirection(Servo.Direction.FORWARD);
        shooterGate.setPosition(GATE_IN);

        // PINPOINT SETUP

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

        // ---- INIT TEST ----

        while (!isStarted() && !isStopRequested()) {

            pinpoint.update();

            double[] f = readPose();

            telemetry.addLine("Start Robot Parallel to Hive");
            telemetry.addData("Field X", "%.2f", f[0]);
            telemetry.addData("Field Y", "%.2f", f[1]);
            telemetry.addData("Heading", "%.1f", f[2]);
            telemetry.addData("Flower Limit", limitSwitch.getState()
                    ? "PRESSED" : "NOT PRESSED");
            addShooterTelemetry();
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

            // ---------------- FLOWER HOMING ----------------

            boolean flowerHomed = homeFlower();

            if (!flowerHomed) {
                telemetry.addLine("Flower did not reach the limit switch.");
                telemetry.addLine("Stopping autonomous for safety.");
                telemetry.update();
                return;
            }

            // ---------------- INTAKE ON BEFORE SHOOTING ----------------

            intakeMotor.setPower(INTAKE_POWER);

            // ---------------- FIRST SHOOTING SEQUENCE ----------------

            shooterGate.setPosition(GATE_IN);

            setShooterVelocity(SHOOTER_VELOCITY);

            // Allow flywheel to spin up
            waitWithTelemetry(4500);

            // Shoot 4 times
            for (int i = 0; i < 4; i++) {

                shooterGate.setPosition(GATE_OUT);
                waitWithTelemetry(600);

                shooterGate.setPosition(GATE_IN);
                waitWithTelemetry(350);
            }

            // Stop flywheel after shooting
            setShooterVelocity(0);

            // ---------------- MOVEMENT SEQUENCE ----------------

            goToTile(-0.5, 2, 0);
            holdPoseFor(100, 0);

            goToTile(0, 2, 0);
            holdPoseFor(100, 0);

            goToTile(-0.5, 1.8, 0);
            holdPoseFor(100, 0);

            goToTile(0, 0, 0);
            holdPoseFor(300, 0);

            // ---------------- SECOND SHOOTING SEQUENCE ----------------

            intakeMotor.setPower(INTAKE_POWER);
            shooterGate.setPosition(GATE_IN);

            // Run shooter at 1200 velocity
            setShooterVelocity(SHOOTER_VELOCITY);

            // Allow flywheel to spin up again
            waitWithTelemetry(2500);

            // Shoot another 4 balls
            for (int i = 0; i < 4; i++) {

                shooterGate.setPosition(GATE_OUT);
                waitWithTelemetry(100);

                shooterGate.setPosition(GATE_IN);
                waitWithTelemetry(350);
            }

            // Stop shooter
            setShooterVelocity(0);

            goToTile(-0.5, 1, 0);

            goToTile(-4, 2.5, 180);
            holdPoseFor(100, 180);

            // ---------------- STOP INTAKE ----------------

            intakeMotor.setPower(0.0);

            while (opModeIsActive()) {
                updateTelemetry();
                idle();
            }

        } finally {

            controlRunning = false;

            try {
                controlThread.join(500);
            } catch (InterruptedException ignored) {}

            stopMotors();
        }
    }

    // ---------------- POSE READING ----------------

    private double[] readPose() {

        pinpoint.update();

        Pose2D p = pinpoint.getPosition();

        double rawX = p.getX(DistanceUnit.INCH);
        double rawY = p.getY(DistanceUnit.INCH);
        double h = p.getHeading(AngleUnit.DEGREES);

        double fx = SWAP_XY ? rawY : rawX;
        double fy = SWAP_XY ? rawX : rawY;

        double ox = X_SIGN * fx;
        double oy = Y_SIGN * fy;
        double oh = H_SIGN * h;

        double s = Math.toRadians(START_HEADING_DEG);

        double x = START_X + ox * Math.cos(s) - oy * Math.sin(s);
        double y = START_Y + ox * Math.sin(s) + oy * Math.cos(s);

        double heading = START_HEADING_DEG + oh;

        while (heading > 180) heading -= 360;
        while (heading <= -180) heading += 360;

        return new double[]{x, y, heading};
    }

    // ---------------- MOVEMENT COMMANDS ----------------

    private void goTo(double x, double y, double headingDegrees) {

        synchronized (targetLock) {
            targetFieldX = x;
            targetFieldY = y;
            targetHeadingDegrees = headingDegrees;
        }

        maxTurnPower = HOLD_TURN_LIMIT;
        resetPIDs = true;

        waitUntilSettled(GOTO_TIMEOUT_MS);
    }

    private void goToTile(double tilesUp, double tilesRight,
                          double headingDegrees) {

        goTo(tilesRight * TILE_IN, tilesUp * TILE_IN, headingDegrees);
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

    // ---------------- TELEMETRY ----------------

    // Set the shooter target and keep track of the commanded velocity.
    private void setShooterVelocity(double velocity) {
        shooterTargetVelocity = velocity;
        shooter.setVelocity(velocity);
    }

    // Add shooter readings to the current telemetry packet.
    // This method does not call telemetry.update() by itself.
    private void addShooterTelemetry() {
        double actualVelocity = shooter.getVelocity();

        telemetry.addData("Shooter Target", "%.0f ticks/sec",
                shooterTargetVelocity);
        telemetry.addData("Shooter Actual", "%.0f ticks/sec",
                actualVelocity);
        telemetry.addData("Shooter Error", "%.0f ticks/sec",
                shooterTargetVelocity - actualVelocity);
    }

    // Wait while continuing to refresh telemetry on the OpMode thread.
    private void waitWithTelemetry(long durationMs) {
        long endTime = System.currentTimeMillis() + durationMs;

        while (opModeIsActive()
                && !isStopRequested()
                && System.currentTimeMillis() < endTime) {
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

        addShooterTelemetry();

        telemetry.update();
    }

    // ---------------- CONTINUOUS POSE CONTROL ----------------

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

            while (headingErrorRadians > Math.PI)
                headingErrorRadians -= 2 * Math.PI;

            while (headingErrorRadians < -Math.PI)
                headingErrorRadians += 2 * Math.PI;

            double forwardError =
                    -errorX * Math.sin(headingRadians)
                            + errorY * Math.cos(headingRadians);

            double leftError =
                    -errorX * Math.cos(headingRadians)
                            - errorY * Math.sin(headingRadians);

            double forwardPower =
                    -drivePID.getComputedOutput(forwardError);

            double leftPower =
                    -strafePID.getComputedOutput(leftError);

            double turnPower =
                    headingPID.getComputedAngleOutput(headingRadians);

            leftPower = applyMinPower(
                    leftPower, leftError, MIN_STRAFE_POWER);

            forwardPower = applyMinPower(
                    forwardPower, forwardError, MIN_DRIVE_POWER);

            if (Math.abs(headingErrorRadians) > MIN_TURN_DEADBAND_RAD
                    && Math.abs(turnPower) < MIN_TURN_POWER) {

                turnPower =
                        Math.signum(turnPower) * MIN_TURN_POWER;
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

            double max = Math.max(
                    1.0,
                    Math.max(
                            Math.max(Math.abs(lf), Math.abs(rf)),
                            Math.max(Math.abs(lb), Math.abs(rb))
                    )
            );

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

    private double applyMinPower(double power, double error,
                                 double minPower) {

        if (Math.abs(error) > MIN_POWER_DEADBAND_IN
                && Math.abs(power) < minPower) {
            return Math.signum(error) * minPower;
        }

        return power;
    }

    private void stopMotors() {

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        intakeMotor.setPower(0);
        leftFlower.setPower(0);
        rightFlower.setPower(0);

        setShooterVelocity(0);
    }
}
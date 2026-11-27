
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "CustomDT_CompRDY", group = "Linear OPMODE")
public class CustomDT_WFlowerIn extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime gateTimer = new ElapsedTime();
    private ElapsedTime flowerTimer = new ElapsedTime();

    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;
    private DcMotorEx intakeMotor = null;
    private DcMotorEx shooter = null;

    // Flower continuous rotation servos
    private CRServo leftFlower = null;
    private CRServo rightFlower = null;

    private Servo shooterGate = null;
    private Servo rgb = null;

    private DigitalChannel flowerLimit = null;

    private boolean previousDpadLeft = false;
    private boolean previousDpadUp = false;
    private boolean previousDpadDown = false;

    private boolean gateMoving = false;

    static final double TICKS_PER_REV = 537.7;
    static final double MAX_RPM = 312.0;

    static final double MAX_TICKS_PER_SEC =
            (MAX_RPM * TICKS_PER_REV) / 60.0;

    // RGB POSITIONS
    static final double CYAN = 0.56;
    static final double GREEN = 0.50;
    static final double PINK = 0.72;

    // RGB VELOCITY THRESHOLDS
    static final double GREEN_MIN = 0.40;
    static final double GREEN_MAX = 0.45;

    // SHOOTER
    static final double TRIGGER_MODE_MAX_POWER = 1.0;
    static final double TRIGGER_DEADBAND = 0.05;

    // SHOOTER GATE
    static final double GATE_OUT = 0.20;
    static final double GATE_IN = 0.0;
    static final double GATE_OUT_TIME = 0.25;

    // FLOWER SETTINGS
    static final double FLOWER_UP_POWER = -0.8;
    static final double FLOWER_DOWN_POWER = 0.3;

    static final double FLOWER_DOWN_TIME_SEC = 0.60;
    static final double FLOWER_UP_TIMEOUT_SEC = 3.0;

    private enum FlowerState {
        IDLE,
        GOING_UP,
        GOING_DOWN
    }

    private FlowerState flowerState = FlowerState.IDLE;

    private boolean flowerAtTop = false;
    private boolean flowerTimedOut = false;

    private void setFlowerPower(double power) {
        leftFlower.setPower(power);
        rightFlower.setPower(power);
    }

    @Override
    public void runOpMode() {

        // HARDWARE SETUP
        leftFrontDrive = hardwareMap.get(
                DcMotorEx.class, "leftFrontDrive");

        leftBackDrive = hardwareMap.get(
                DcMotorEx.class, "leftBackDrive");

        rightFrontDrive = hardwareMap.get(
                DcMotorEx.class, "rightFrontDrive");

        rightBackDrive = hardwareMap.get(
                DcMotorEx.class, "rightBackDrive");

        intakeMotor = hardwareMap.get(
                DcMotorEx.class, "intakeMotor");

        leftFlower = hardwareMap.get(
                CRServo.class, "leftFlower");

        rightFlower = hardwareMap.get(
                CRServo.class, "rightFlower");

        shooter = hardwareMap.get(
                DcMotorEx.class, "shooter");

        shooterGate = hardwareMap.get(
                Servo.class, "shooterGate");

        rgb = hardwareMap.get(
                Servo.class, "rgbFW");

        flowerLimit = hardwareMap.get(
                DigitalChannel.class, "flowerLimit");

        flowerLimit.setMode(DigitalChannel.Mode.INPUT);

        // MOTOR DIRECTIONS
        leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorEx.Direction.REVERSE);

        rightFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorEx.Direction.FORWARD);

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // REVERSED FLOWER SERVOS
        leftFlower.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFlower.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setDirection(DcMotorEx.Direction.REVERSE);

        shooterGate.setDirection(Servo.Direction.FORWARD);

        // SHOOTER ENCODER
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // INITIAL POSITIONS
        shooterGate.setPosition(GATE_IN);
        rgb.setPosition(CYAN);

        setFlowerPower(0.0);

        // FLOWER HOMING
        // Raises the flower until the limit switch is pressed.
        flowerAtTop = false;
        flowerTimedOut = false;
        flowerTimer.reset();

        telemetry.addData("Status", "Initialized - homing flower");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {

            // Assumes getState() returns true when physically pressed.
            boolean homePressed = flowerLimit.getState();

            if (!flowerAtTop && !flowerTimedOut) {

                if (homePressed) {

                    setFlowerPower(0.0);
                    flowerAtTop = true;

                } else if (flowerTimer.seconds()
                        >= FLOWER_UP_TIMEOUT_SEC) {

                    setFlowerPower(0.0);
                    flowerTimedOut = true;

                } else {

                    setFlowerPower(FLOWER_UP_POWER);
                }
            }

            telemetry.addData(
                    "Flower Homed",
                    flowerAtTop ? "YES - UP" : "NO - moving up...");

            telemetry.addData(
                    "Flower Limit Switch",
                    homePressed ? "PRESSED" : "NOT PRESSED");

            telemetry.addData(
                    "Flower Limit RAW getState",
                    flowerLimit.getState());

            if (flowerTimedOut) {
                telemetry.addData(
                        "WARNING",
                        "Timed out - check switch / direction.");
            }

            telemetry.update();
        }

        setFlowerPower(0.0);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            double max;

            // DRIVETRAIN
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            max = Math.max(
                    Math.abs(leftFrontPower),
                    Math.abs(rightFrontPower));

            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {

                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // MAIN INTAKE MOTOR
            if (gamepad2.a) {

                intakeMotor.setPower(1.0);

            } else if (gamepad2.b) {

                intakeMotor.setPower(-1.0);

            } else if (gamepad2.x) {

                intakeMotor.setPower(0.0);
            }

            // FLOWER INTAKE
            // Read the limit switch.
            boolean flowerLimitPressed = flowerLimit.getState();

            // Detect new button presses instead of holding the button.
            boolean dpadUpPressed =
                    gamepad2.dpad_up && !previousDpadUp;

            boolean dpadDownPressed =
                    gamepad2.dpad_down && !previousDpadDown;

            // D-PAD UP: RAISE FLOWER
            if (dpadUpPressed && !flowerLimitPressed) {

                flowerTimedOut = false;
                flowerAtTop = false;

                flowerState = FlowerState.GOING_UP;
                flowerTimer.reset();
            }

            // D-PAD DOWN: LOWER FLOWER
            else if (dpadDownPressed) {

                flowerAtTop = false;
                flowerTimedOut = false;

                flowerState = FlowerState.GOING_DOWN;
                flowerTimer.reset();
            }

            // FLOWER MOVEMENT CONTROL
            switch (flowerState) {

                case GOING_UP:

                    // Stop immediately when the limit switch is pressed.
                    if (flowerLimitPressed) {

                        setFlowerPower(0.0);

                        flowerAtTop = true;
                        flowerTimedOut = false;

                        flowerState = FlowerState.IDLE;

                    }

                    // Safety timeout if the switch never activates.
                    else if (flowerTimer.seconds()
                            >= FLOWER_UP_TIMEOUT_SEC) {

                        setFlowerPower(0.0);

                        flowerTimedOut = true;
                        flowerState = FlowerState.IDLE;

                    }

                    // Continue moving upward.
                    else {

                        setFlowerPower(FLOWER_UP_POWER);
                    }

                    break;

                case GOING_DOWN:

                    // Stop after the selected downward movement time.
                    if (flowerTimer.seconds()
                            >= FLOWER_DOWN_TIME_SEC) {

                        setFlowerPower(0.0);
                        flowerState = FlowerState.IDLE;

                    } else {

                        setFlowerPower(FLOWER_DOWN_POWER);
                    }

                    break;

                case IDLE:
                default:

                    setFlowerPower(0.0);
                    break;
            }

            // Remember button states for the next loop.
            previousDpadUp = gamepad2.dpad_up;
            previousDpadDown = gamepad2.dpad_down;

            // SHOOTER: TRIGGERS ONLY
            double shooterPower =
                    gamepad2.right_trigger
                            - gamepad2.left_trigger;

            boolean shooterRunning =
                    Math.abs(shooterPower) > TRIGGER_DEADBAND;

            shooterPower = Math.max(
                    -TRIGGER_MODE_MAX_POWER,
                    Math.min(TRIGGER_MODE_MAX_POWER, shooterPower));

            // SHOOTER SET VELOCITY
            if (shooterRunning) {

                shooter.setVelocity(
                        shooterPower * MAX_TICKS_PER_SEC);

            } else {

                shooter.setVelocity(0.0);
            }

            // ACTUAL SHOOTER VELOCITY
            double shooterVelocity =
                    Math.abs(shooter.getVelocity());

            double shooterRPM =
                    (shooterVelocity / TICKS_PER_REV) * 60.0;

            // RGB CHECKS ACTUAL MEASURED VELOCITY
            double velocityFraction =
                    shooterVelocity / MAX_TICKS_PER_SEC;

            String shooterStatus;

            if (!shooterRunning) {

                rgb.setPosition(CYAN);
                shooterStatus = "OFF - CYAN";

            } else if (velocityFraction >= GREEN_MIN
                    && velocityFraction <= GREEN_MAX) {

                rgb.setPosition(GREEN);
                shooterStatus = "IN RANGE - GREEN";

            } else if (velocityFraction > GREEN_MAX) {

                rgb.setPosition(PINK);
                shooterStatus = "OVER SPEED - PINK";

            } else {

                rgb.setPosition(CYAN);
                shooterStatus = "UNDER SPEED - CYAN";
            }

            // SHOOTER GATE
            // Press once: moves out and automatically returns.
            if (gamepad2.dpad_left
                    && !previousDpadLeft
                    && !gateMoving) {

                gateMoving = true;
                gateTimer.reset();

                shooterGate.setPosition(GATE_OUT);
            }

            if (gateMoving
                    && gateTimer.seconds() >= GATE_OUT_TIME) {

                shooterGate.setPosition(GATE_IN);
                gateMoving = false;
            }

            previousDpadLeft = gamepad2.dpad_left;

            // TELEMETRY
            telemetry.addData("FL Wheel", leftFrontPower);
            telemetry.addData("FR Wheel", rightFrontPower);
            telemetry.addData("BL Wheel", leftBackPower);
            telemetry.addData("BR Wheel", rightBackPower);

            telemetry.addData(
                    "Intake Motor Power",
                    intakeMotor.getPower());

            telemetry.addData("Flower State", flowerState);

            telemetry.addData(
                    "Left Flower Power",
                    leftFlower.getPower());

            telemetry.addData(
                    "Right Flower Power",
                    rightFlower.getPower());

            telemetry.addData(
                    "Flower Limit Switch",
                    flowerLimitPressed ? "PRESSED" : "NOT PRESSED");

            telemetry.addData("Flower At Top", flowerAtTop);

            if (flowerTimedOut) {

                telemetry.addData(
                        "WARNING",
                        "Flower timed out - switch never pressed");
            }

            telemetry.addData(
                    "Shooter Power Fraction",
                    "%.2f", shooterPower);

            telemetry.addData(
                    "Shooter RPM",
                    "%.1f", shooterRPM);

            telemetry.addData(
                    "Shooter Velocity",
                    "%.1f ticks/sec", shooterVelocity);

            telemetry.addData(
                    "Velocity Fraction",
                    "%.3f", velocityFraction);

            telemetry.addData(
                    "Shooter Status",
                    shooterStatus);

            telemetry.addData(
                    "Shooter Gate",
                    gateMoving ? "MOVING OUT" : "IN");

            telemetry.update();
        }

        // STOP MOTORS WHEN TELEOP ENDS
        setFlowerPower(0.0);
        intakeMotor.setPower(0.0);
        shooter.setPower(0.0);

        leftFrontDrive.setPower(0.0);
        leftBackDrive.setPower(0.0);
        rightFrontDrive.setPower(0.0);
        rightBackDrive.setPower(0.0);
    }
}

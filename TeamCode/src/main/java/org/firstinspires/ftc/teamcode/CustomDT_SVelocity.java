
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

@TeleOp(name = "CustomDT_VelocityMod", group = "Linear OPMODE")
public class CustomDT_SVelocity extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime flowerTimer = new ElapsedTime();

    private DcMotorEx leftFrontDrive;
    private DcMotorEx leftBackDrive;
    private DcMotorEx rightFrontDrive;
    private DcMotorEx rightBackDrive;
    private DcMotorEx intakeMotor;
    private DcMotorEx shooter;

    private CRServo leftFlower;
    private CRServo rightFlower;

    private Servo shooterGate;
    private Servo rgb;

    private DigitalChannel flowerLimit;

    private boolean previousDpadUp = false;
    private boolean previousDpadDown = false;
    private boolean previousY = false;
    private boolean yShooterOn = false;

    // ==========================================
    // SHOOTER SPECIFICATIONS
    // ==========================================

    static final double TICKS_PER_REV = 537.7;
    static final double MAX_RPM = 312.0;

    // SHOOTER MAX SPEED LIMIT: 50%
    static final double SHOOTER_SPEED_LIMIT = 0.50;

    static final double MAX_TICKS_PER_SEC =
            ((MAX_RPM * TICKS_PER_REV) / 60.0)
                    * SHOOTER_SPEED_LIMIT;

    static final double Y_BUTTON_POWER = 0.60;
    static final double TRIGGER_MODE_MAX_POWER = 1.0;
    static final double TRIGGER_DEADBAND = 0.05;

    // ==========================================
    // RGB SETTINGS
    // ==========================================

    static final double CYAN = 0.56;
    static final double GREEN = 0.50;

    // ==========================================
    // SHOOTER GATE
    // ==========================================

    static final double GATE_OUT = 0.2;
    static final double GATE_IN = 0.0;

    // ==========================================
    // FLOWER SETTINGS
    // ==========================================

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

        // ==========================================
        // HARDWARE SETUP
        // ==========================================

        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "leftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rightBackDrive");

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        leftFlower = hardwareMap.get(CRServo.class, "leftFlower");
        rightFlower = hardwareMap.get(CRServo.class, "rightFlower");

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooterGate = hardwareMap.get(Servo.class, "shooterGate");
        rgb = hardwareMap.get(Servo.class, "rgbFW");

        flowerLimit = hardwareMap.get(DigitalChannel.class, "flowerLimit");
        flowerLimit.setMode(DigitalChannel.Mode.INPUT);

        // ==========================================
        // MOTOR DIRECTIONS
        // ==========================================

        leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorEx.Direction.FORWARD);

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFlower.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFlower.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setDirection(DcMotorEx.Direction.REVERSE);
        shooterGate.setDirection(Servo.Direction.FORWARD);

        // ==========================================
        // SHOOTER MODE (RUN USING ENCODER)
        // ==========================================

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ==========================================
        // INITIAL POSITIONS
        // ==========================================

        shooterGate.setPosition(GATE_IN);
        rgb.setPosition(CYAN);
        setFlowerPower(0.0);

        // ==========================================
        // FLOWER HOMING (INIT PHASE)
        // ==========================================

        flowerAtTop = false;
        flowerTimedOut = false;
        flowerTimer.reset();

        telemetry.addData("Status", "Initializing - homing flower");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {

            boolean homePressed = flowerLimit.getState();

            if (!flowerAtTop && !flowerTimedOut) {
                if (homePressed) {
                    setFlowerPower(0.0);
                    flowerAtTop = true;
                } else if (flowerTimer.seconds() >= FLOWER_UP_TIMEOUT_SEC) {
                    setFlowerPower(0.0);
                    flowerTimedOut = true;
                } else {
                    setFlowerPower(FLOWER_UP_POWER);
                }
            }

            telemetry.addData("Flower Homed", flowerAtTop ? "YES - UP" : "NO - moving up");
            telemetry.addData("Flower Limit Switch", homePressed ? "PRESSED" : "NOT PRESSED");

            if (flowerTimedOut) {
                telemetry.addData("WARNING", "Check flower switch and direction");
            }

            telemetry.update();
        }

        setFlowerPower(0.0);

        waitForStart();
        runtime.reset();

        // ==========================================
        // MAIN TELEOP LOOP
        // ==========================================

        while (opModeIsActive()) {

            // ======================================
            // DRIVETRAIN
            // ======================================

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
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

            // ======================================
            // MAIN INTAKE
            // ======================================

            if (gamepad2.a) {
                intakeMotor.setPower(1.0);
            } else if (gamepad2.b) {
                intakeMotor.setPower(-1.0);
            } else if (gamepad2.x) {
                intakeMotor.setPower(0.0);
            }

            // ======================================
            // FLOWER TRANSFER STATE MACHINE
            // ======================================

            boolean flowerLimitPressed = flowerLimit.getState();
            boolean dpadUpPressed = gamepad2.dpad_up && !previousDpadUp;
            boolean dpadDownPressed = gamepad2.dpad_down && !previousDpadDown;

            if (dpadUpPressed && !flowerLimitPressed) {
                flowerTimedOut = false;
                flowerAtTop = false;
                flowerState = FlowerState.GOING_UP;
                flowerTimer.reset();
            } else if (dpadDownPressed) {
                flowerAtTop = false;
                flowerTimedOut = false;
                flowerState = FlowerState.GOING_DOWN;
                flowerTimer.reset();
            }

            switch (flowerState) {
                case GOING_UP:
                    if (flowerLimitPressed) {
                        setFlowerPower(0.0);
                        flowerAtTop = true;
                        flowerTimedOut = false;
                        flowerState = FlowerState.IDLE;
                    } else if (flowerTimer.seconds() >= FLOWER_UP_TIMEOUT_SEC) {
                        setFlowerPower(0.0);
                        flowerTimedOut = true;
                        flowerState = FlowerState.IDLE;
                    } else {
                        setFlowerPower(FLOWER_UP_POWER);
                    }
                    break;

                case GOING_DOWN:
                    if (flowerTimer.seconds() >= FLOWER_DOWN_TIME_SEC) {
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

            previousDpadUp = gamepad2.dpad_up;
            previousDpadDown = gamepad2.dpad_down;

            // ======================================
            // SHOOTER CONTROLS (SET VELOCITY)
            // ======================================

            boolean yPressed = gamepad2.y && !previousY;
            previousY = gamepad2.y;

            if (yPressed) {
                yShooterOn = !yShooterOn;
            }

            double shooterPower =
                    gamepad2.right_trigger - gamepad2.left_trigger;

            if (Math.abs(shooterPower) > TRIGGER_DEADBAND) {
                yShooterOn = false;
            } else if (yShooterOn) {
                shooterPower = Y_BUTTON_POWER;
            }

            boolean shooterRunning =
                    Math.abs(shooterPower) > TRIGGER_DEADBAND;

            shooterPower = Math.max(
                    -TRIGGER_MODE_MAX_POWER,
                    Math.min(TRIGGER_MODE_MAX_POWER, shooterPower));

            // Convert the trigger/preset value into target ticks per second.
            if (shooterRunning) {
                shooter.setVelocity(shooterPower * MAX_TICKS_PER_SEC);
                rgb.setPosition(GREEN);
            } else {
                shooter.setVelocity(0.0);
                rgb.setPosition(CYAN);
            }

            // MEASURED SHOOTER VELOCITY
            double shooterVelocity = Math.abs(shooter.getVelocity());

            double shooterRPM =
                    (shooterVelocity / TICKS_PER_REV) * 60.0;

            // shooter rpm telemetry
            telemetry.addData("Shooter RPM", "%.1f RPM", shooterRPM);

            // ======================================
            // SHOOTER GATE
            // ======================================

            if (gamepad2.dpad_left) {
                shooterGate.setPosition(GATE_OUT);
            } else {
                shooterGate.setPosition(GATE_IN);
            }

            // Telemetry
            telemetry.addData("FL Wheel", leftFrontPower);
            telemetry.addData("FR Wheel", rightFrontPower);
            telemetry.addData("BL Wheel", leftBackPower);
            telemetry.addData("BR Wheel", rightBackPower);

            telemetry.addData("Intake Power", intakeMotor.getPower());

            telemetry.addData("Flower State", flowerState);
            telemetry.addData("Flower Limit Switch", flowerLimitPressed ? "PRESSED" : "NOT PRESSED");

            telemetry.addData("Y Button Pressed", yShooterOn);

            telemetry.addData("Shooter Command", "%.3f", shooterPower);
            telemetry.addData("Shooter Measured RPM", "%.1f", shooterRPM);
            telemetry.addData("Shooter Measured Velocity", "%.1f ticks/sec",shooterVelocity);
            telemetry.addData("Shooter Target RPM","%.1f RPM",Math.abs(shooterPower * MAX_TICKS_PER_SEC) / TICKS_PER_REV * 60.0);
            telemetry.addData("Shooter Gate Position", shooterGate.getPosition());

            telemetry.update();
        }

        // ==========================================
        // STOP MOTORS
        // ==========================================

        setFlowerPower(0.0);
        intakeMotor.setPower(0.0);
        shooter.setVelocity(0.0);

        leftFrontDrive.setPower(0.0);
        leftBackDrive.setPower(0.0);
        rightFrontDrive.setPower(0.0);
        rightBackDrive.setPower(0.0);
    }
}

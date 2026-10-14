package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "Pinpoint PID Auto", group = "PID")
public class PinpointPIDAuto extends LinearOpMode {

    // =========================================================
    // HARDWARE
    // =========================================================

    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    private GoBildaPinpointDriver pinpoint;


    // =========================================================
    // YOUR PID CONTROLLERS
    // =========================================================

    private PIDController drivePID;
    private PIDController headingPID;
    private PIDController strafePID;


    // =========================================================
    // PID CONSTANTS
    // =========================================================

    static final double DRIVE_KP = 0.0025;
    static final double DRIVE_KI = 0.0000;
    static final double DRIVE_KD = 0.0005;

    static final double TURN_KP = 0.055;
    static final double TURN_KI = 0.0000;
    static final double TURN_KD = 0.0000;

    static final double STRAFE_KP = 0.0035;
    static final double STRAFE_KI = 0.0000;
    static final double STRAFE_KD = 0.0002;


    // =========================================================
    // PINPOINT OFFSETS
    //
    // LEAVE THESE 0 FOR NOW.
    // You can change them after you measure the pods.
    // =========================================================

    static final double PINPOINT_OFFSET_X = 0.0;
    static final double PINPOINT_OFFSET_Y = 0.0;


    @Override
    public void runOpMode() {

        // =====================================================
        // HARDWARE MAP
        // =====================================================

        leftFrontDrive =
                hardwareMap.get(
                        DcMotor.class,
                        "leftFrontDrive"
                );

        rightFrontDrive =
                hardwareMap.get(
                        DcMotor.class,
                        "rightFrontDrive"
                );

        leftBackDrive =
                hardwareMap.get(
                        DcMotor.class,
                        "leftBackDrive"
                );

        rightBackDrive =
                hardwareMap.get(
                        DcMotor.class,
                        "rightBackDrive"
                );

        pinpoint =
                hardwareMap.get(
                        GoBildaPinpointDriver.class,
                        "pinpoint"
                );


        // =====================================================
        // DRIVE MOTOR DIRECTIONS
        // =====================================================

        leftFrontDrive.setDirection(
                DcMotor.Direction.REVERSE
        );

        leftBackDrive.setDirection(
                DcMotor.Direction.REVERSE
        );

        rightFrontDrive.setDirection(
                DcMotor.Direction.FORWARD
        );

        rightBackDrive.setDirection(
                DcMotor.Direction.FORWARD
        );


        // =====================================================
        // BRAKE
        // =====================================================

        leftFrontDrive.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        rightFrontDrive.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        leftBackDrive.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        rightBackDrive.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );


        // =====================================================
        // IMPORTANT
        //
        // Drive motors are ONLY being used as motors.
        // Their encoders are NOT being used for localization.
        // =====================================================

        leftFrontDrive.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        rightFrontDrive.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        leftBackDrive.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        rightBackDrive.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );


        // =====================================================
        // PINPOINT SETUP
        // =====================================================

        // Pod offsets
        // KEEP 0,0 UNTIL YOU MEASURE THEM

        pinpoint.setOffsets(
                PINPOINT_OFFSET_X,
                PINPOINT_OFFSET_Y,
                DistanceUnit.MM
        );


        // =====================================================
        // ODOMETRY POD TYPE
        // =====================================================

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );


        // =====================================================
        // ODOMETRY POD DIRECTIONS
        //
        // Start with both FORWARD.
        // We can change these if one pod reads backwards.
        // =====================================================

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );


        // =====================================================
        // CREATE YOUR PID CONTROLLERS
        // =====================================================

        drivePID =
                new PIDController(
                        DRIVE_KP,
                        DRIVE_KI,
                        DRIVE_KD
                );

        headingPID =
                new PIDController(
                        TURN_KP,
                        TURN_KI,
                        TURN_KD
                );

        strafePID =
                new PIDController(
                        STRAFE_KP,
                        STRAFE_KI,
                        STRAFE_KD
                );


        // =====================================================
        // OUTPUT LIMITS
        // =====================================================

        drivePID.setOutputRange(0.80);

        headingPID.setOutputRange(0.25);

        strafePID.setOutputRange(0.60);


        telemetry.addLine("PINPOINT PID READY");
        telemetry.addLine("Odometry: 2 Pinpoint Pods");
        telemetry.addLine("IMU: Pinpoint Internal IMU");
        telemetry.addLine("Offsets: X=0 Y=0");

        telemetry.update();


        waitForStart();

        if (isStopRequested()) {
            return;
        }


        // =====================================================
        // RESET PINPOINT
        // =====================================================

        pinpoint.resetPosAndIMU();

        sleep(500);


        // =====================================================
        // AUTONOMOUS TEST
        // =====================================================

        driveStraight(115, 0);

        sleep(500);

        turnToAngle(88);

        sleep(500);

        driveStraight(20, 88);

        sleep(500);

        strafeLeft(20, 88);

        sleep(500);

        strafeLeft(-20, 88);

        sleep(500);

        driveStraight(-20, 88);

        sleep(500);

        turnToAngle(0);

        sleep(500);

        driveStraight(-115, 0);

        stopMotors();
    }


    // =========================================================
    // DRIVE STRAIGHT
    //
    // Distance comes from Pinpoint X/Y.
    // NOT from drive motor encoders.
    // =========================================================

    private void driveStraight(
            double inches,
            double headingDegrees
    ) {

        double direction = Math.signum(inches);

        double targetDistance = Math.abs(inches);


        // Reset PID
        drivePID.reset();
        headingPID.reset();


        // Set PID targets
        drivePID.setSetPoint(targetDistance);

        headingPID.setSetPoint(
                Math.toRadians(headingDegrees)
        );


        // =====================================================
        // RECORD STARTING POSITION
        // =====================================================

        pinpoint.update();

        Pose2D startPose =
                pinpoint.getPosition();

        double startX =
                startPose.getX(
                        DistanceUnit.INCH
                );

        double startY =
                startPose.getY(
                        DistanceUnit.INCH
                );


        // =====================================================
        // LOOP
        // =====================================================

        while (opModeIsActive()) {

            pinpoint.update();

            Pose2D pose =
                    pinpoint.getPosition();


            // Current Pinpoint position

            double currentX =
                    pose.getX(
                            DistanceUnit.INCH
                    );

            double currentY =
                    pose.getY(
                            DistanceUnit.INCH
                    );


            // =================================================
            // DISTANCE TRAVELED
            // =================================================

            double deltaX =
                    currentX - startX;

            double deltaY =
                    currentY - startY;


            double currentDistance =
                    Math.hypot(
                            deltaX,
                            deltaY
                    );


            // =================================================
            // DRIVE PID
            // =================================================

            double drivePower =
                    drivePID.getComputedOutput(
                            currentDistance
                    );


            drivePower *= direction;


            // =================================================
            // SLOW DOWN NEAR TARGET
            // =================================================

            double remaining =
                    targetDistance - currentDistance;

            double slowdownFactor =
                    Math.abs(remaining) / 12.0;

            slowdownFactor =
                    Math.max(
                            0.35,
                            Math.min(
                                    1.0,
                                    slowdownFactor
                            )
                    );

            drivePower *= slowdownFactor;


            // =================================================
            // HEADING PID
            // =================================================

            double heading =
                    pose.getHeading(
                            AngleUnit.RADIANS
                    );


            double turnCorrection =
                    headingPID.getComputedAngleOutput(
                            heading
                    );


            turnCorrection =
                    Math.max(
                            -0.15,
                            Math.min(
                                    0.15,
                                    turnCorrection
                            )
                    );


            // =================================================
            // TARGET REACHED
            // =================================================

            if (Math.abs(remaining) < 0.75) {
                break;
            }


            // =================================================
            // TANK DRIVE + HEADING CORRECTION
            // =================================================

            double leftPower =
                    drivePower + turnCorrection;

            double rightPower =
                    drivePower - turnCorrection;


            // Normalize

            double max =
                    Math.max(
                            1.0,
                            Math.max(
                                    Math.abs(leftPower),
                                    Math.abs(rightPower)
                            )
                    );


            leftPower /= max;
            rightPower /= max;


            // =================================================
            // MOTOR POWER
            // =================================================

            leftFrontDrive.setPower(leftPower);
            leftBackDrive.setPower(leftPower);

            rightFrontDrive.setPower(rightPower);
            rightBackDrive.setPower(rightPower);


            // =================================================
            // TELEMETRY
            // =================================================

            telemetry.addLine("PINPOINT DRIVE");

            telemetry.addData(
                    "Target Distance",
                    targetDistance
            );

            telemetry.addData(
                    "Current Distance",
                    currentDistance
            );

            telemetry.addData(
                    "X",
                    currentX
            );

            telemetry.addData(
                    "Y",
                    currentY
            );

            telemetry.addData(
                    "Drive Error",
                    drivePID.getCurrentError()
            );

            telemetry.addData(
                    "Heading",
                    Math.toDegrees(heading)
            );

            telemetry.addData(
                    "Heading Error",
                    Math.toDegrees(
                            headingPID.getCurrentError()
                    )
            );

            telemetry.addData(
                    "Drive Power",
                    drivePower
            );

            telemetry.addData(
                    "Turn Correction",
                    turnCorrection
            );

            telemetry.update();
        }


        stopMotors();

        sleep(200);
    }


    // =========================================================
    // TURN TO ANGLE
    //
    // Uses Pinpoint's internal IMU.
    // =========================================================

    private void turnToAngle(
            double targetAngleDegrees
    ) {

        headingPID.setOutputRange(0.85);

        headingPID.reset();

        headingPID.setSetPoint(
                Math.toRadians(
                        targetAngleDegrees
                )
        );


        while (opModeIsActive()) {

            pinpoint.update();

            Pose2D pose =
                    pinpoint.getPosition();


            double heading =
                    pose.getHeading(
                            AngleUnit.RADIANS
                    );


            // YOUR PID CONTROLLER
            // Angle wrapping is handled inside it.

            double turnPower =
                    headingPID.getComputedAngleOutput(
                            heading
                    );


            // =================================================
            // MINIMUM POWER
            // =================================================

            if (turnPower > 0 &&
                    turnPower < 0.15) {

                turnPower = 0.15;
            }

            if (turnPower < 0 &&
                    turnPower > -0.15) {

                turnPower = -0.15;
            }


            double errorDegrees =
                    Math.toDegrees(
                            headingPID.getCurrentError()
                    );


            // =================================================
            // TARGET REACHED
            // =================================================

            if (Math.abs(errorDegrees) < 1.0) {
                break;
            }


            // =================================================
            // TURN
            // =================================================

            leftFrontDrive.setPower(
                    -turnPower
            );

            leftBackDrive.setPower(
                    -turnPower
            );

            rightFrontDrive.setPower(
                    turnPower
            );

            rightBackDrive.setPower(
                    turnPower
            );


            // =================================================
            // TELEMETRY
            // =================================================

            telemetry.addLine("PINPOINT TURN");

            telemetry.addData(
                    "Target",
                    targetAngleDegrees
            );

            telemetry.addData(
                    "Current",
                    Math.toDegrees(heading)
            );

            telemetry.addData(
                    "Error",
                    errorDegrees
            );

            telemetry.addData(
                    "Turn Power",
                    turnPower
            );

            telemetry.update();
        }


        stopMotors();

        headingPID.setOutputRange(0.10);

        sleep(200);
    }


    // =========================================================
    // STRAFE
    //
    // Uses Pinpoint X/Y instead of motor encoders.
    // =========================================================

    private void strafeLeft(
            double inches,
            double headingDegrees
    ) {

        double direction =
                Math.signum(inches);

        double targetDistance =
                Math.abs(inches);


        strafePID.reset();

        headingPID.reset();


        strafePID.setSetPoint(
                targetDistance
        );

        headingPID.setSetPoint(
                Math.toRadians(
                        headingDegrees
                )
        );


        // =====================================================
        // STARTING POSITION
        // =====================================================

        pinpoint.update();

        Pose2D startPose =
                pinpoint.getPosition();


        double startX =
                startPose.getX(
                        DistanceUnit.INCH
                );

        double startY =
                startPose.getY(
                        DistanceUnit.INCH
                );


        while (opModeIsActive()) {

            pinpoint.update();

            Pose2D pose =
                    pinpoint.getPosition();


            double currentX =
                    pose.getX(
                            DistanceUnit.INCH
                    );

            double currentY =
                    pose.getY(
                            DistanceUnit.INCH
                    );


            // =================================================
            // DISTANCE TRAVELED
            // =================================================

            double deltaX =
                    currentX - startX;

            double deltaY =
                    currentY - startY;


            double currentDistance =
                    Math.hypot(
                            deltaX,
                            deltaY
                    );


            // =================================================
            // STRAFE PID
            // =================================================

            double strafePower =
                    strafePID.getComputedOutput(
                            currentDistance
                    );


            strafePower *= direction;


            // =================================================
            // HEADING CORRECTION
            // =================================================

            double heading =
                    pose.getHeading(
                            AngleUnit.RADIANS
                    );


            double turnCorrection =
                    headingPID.getComputedAngleOutput(
                            heading
                    );


            turnCorrection =
                    Math.max(
                            -0.12,
                            Math.min(
                                    0.12,
                                    turnCorrection
                            )
                    );


            // =================================================
            // TARGET REACHED
            // =================================================

            if (Math.abs(
                    targetDistance - currentDistance
            ) < 0.75) {

                break;
            }


            // =================================================
            // MECANUM STRAFE
            // =================================================

            double lf =
                    -strafePower
                            + turnCorrection;

            double rf =
                    strafePower
                            - turnCorrection;

            double lb =
                    strafePower
                            + turnCorrection;

            double rb =
                    -strafePower
                            - turnCorrection;


            // =================================================
            // NORMALIZE
            // =================================================

            double max =
                    Math.max(
                            1.0,
                            Math.max(
                                    Math.max(
                                            Math.abs(lf),
                                            Math.abs(rf)
                                    ),
                                    Math.max(
                                            Math.abs(lb),
                                            Math.abs(rb)
                                    )
                            )
                    );


            lf /= max;
            rf /= max;
            lb /= max;
            rb /= max;


            // =================================================
            // MOTOR POWER
            // =================================================

            leftFrontDrive.setPower(lf);
            rightFrontDrive.setPower(rf);

            leftBackDrive.setPower(lb);
            rightBackDrive.setPower(rb);


            // =================================================
            // TELEMETRY
            // =================================================

            telemetry.addLine("PINPOINT STRAFE");

            telemetry.addData(
                    "Target Distance",
                    targetDistance
            );

            telemetry.addData(
                    "Current Distance",
                    currentDistance
            );

            telemetry.addData(
                    "X",
                    currentX
            );

            telemetry.addData(
                    "Y",
                    currentY
            );

            telemetry.addData(
                    "Heading",
                    Math.toDegrees(heading)
            );

            telemetry.addData(
                    "Strafe Error",
                    strafePID.getCurrentError()
            );

            telemetry.update();
        }


        stopMotors();

        sleep(200);
    }


    // =========================================================
    // STOP ALL MOTORS
    // =========================================================

    private void stopMotors() {

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}
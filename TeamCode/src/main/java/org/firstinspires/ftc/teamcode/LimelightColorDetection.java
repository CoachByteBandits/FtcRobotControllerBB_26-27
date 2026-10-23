package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "LimelightColorDetection", group = "Linear OpMode")
public class LimelightColorDetection extends LinearOpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private Limelight3A limelight;
    private int activePipeline = 0;

    public double calculateDistanceFromArea(double area) {
        if (area == 0) {
            return Double.MAX_VALUE; // No target detected
        }

// Calibration factor based on testing (tune this for your setup)
        double calibrationFactor = 21.5;

// Calculate the distance using the area, assuming inverse-square relationship
        return calibrationFactor / Math.sqrt(area); // Distance in inches
    }

    @Override
    public void runOpMode() {
// Initialize the motors
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

// Set motor direction (flip these if your robot drives backwards)
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

// Initialize the Limelight object
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // Set polling rate for Limelight
        limelight.start();
        limelight.pipelineSwitch(activePipeline);
        telemetry.addData("Status", "Initialized");

// Variable Initialization
        boolean team = false; // false is red and true is blue
        double scale = 0;
        double drive = 0; // Forward/backward
        double rotate = 0; // Rotation
        double targetY = 0;
        double targetArea = 0;
        boolean hasTarget = false;
        double leftPower;
        double rightPower;
        double distance;

// Team selection during init
        while (opModeInInit()) {
            if (gamepad1.b) {
                team = false; // B is red
            } else if (gamepad1.x) {
                team = true; // X is blue
            }
            telemetry.addData("Team", team ? "Blue" : "Red");
            telemetry.update();
        }

        waitForStart();
        while (opModeIsActive()) {
// Arcade drive inputs
            drive = gamepad1.left_stick_y; // Forward/backward
            rotate = -gamepad1.right_stick_x; // Rotation

// Switch between color detection pipelines
            if (gamepad1.dpad_left) {
                activePipeline = 0; // Yellow
            } else if (gamepad1.dpad_right) { // Team Color
                if (team) {
                    activePipeline = 1; // Blue
                } else {
                    activePipeline = 2; // Red
                }
            }
            limelight.pipelineSwitch(activePipeline);

            LLResult result = limelight.getLatestResult();
            targetY = 0;
            targetArea = 0;
            hasTarget = false;

            if (result != null && result.isValid()) {
                targetY = result.getTy(); // Offset from target
                scale = Math.abs(targetY) / 5;
                targetArea = result.getTa(); // Area of the target (useful for distance)
                hasTarget = true;
                telemetry.addData("Target Y", targetY);
                telemetry.addData("Target Area", targetArea);
            }

// Auto-align: rotate toward the target while a pipeline button is held
            if ((gamepad1.dpad_right || gamepad1.dpad_left) && hasTarget) {
                telemetry.addData("Rotation without scale", -Range.clip(targetY / 150, -1.0, 1.0));
                telemetry.addData("Rotation with scale", scale * -Range.clip(targetY / 150, -1.0, 1.0));
                telemetry.addData("Scale", scale);

                rotate = scale * -Range.clip(targetY / 150, -1.0, 1.0);
            }

            distance = calculateDistanceFromArea(targetArea);
            telemetry.addData("Distance", distance);

// Two-wheel arcade drive mixing
            leftPower = drive + rotate;
            rightPower = drive - rotate;

// Keep motor powers within range [-1, 1]
            leftPower = Range.clip(leftPower, -1.0, 1.0);
            rightPower = Range.clip(rightPower, -1.0, 1.0);

// Set the motor powers
            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);

// Show telemetry
            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.addData("Target", hasTarget);
            telemetry.addData("Status", "Running");
            if (activePipeline == 0) {
                telemetry.addData("Pipeline", "Yellow");
            } else if (activePipeline == 1) {
                telemetry.addData("Pipeline", "Blue");
            } else if (activePipeline == 2) {
                telemetry.addData("Pipeline", "Red");
            }
            telemetry.update();
        }
    }
}
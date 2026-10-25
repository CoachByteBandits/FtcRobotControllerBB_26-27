package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

@Autonomous(name = "LL Color Detect Test", group = "Vision")
public class LLColorDetect_Test extends LinearOpMode {

    //Tuning constants
    private static final int pipelineIndex = 2;
    private static final double minArea = 0.05;
    private static final double stopArea = 8.0;
    private static final double turnKP = 0.02;
    private static final double maxTurn = 0.4;
    private static final double aimToleranceDegree = 2.0;
    private static final double maxDrive = 0.5;
    private static final double minDrive = 0.15;
    private static final double driveAllowedDegree = 10.0;
    private static final double searchTurn = 0.2;

    private Limelight3A limelight;
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void runOpMode() {
        //Hardware mapping
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(pipelineIndex);

        telemetry.addLine("Ready. Press START to chase pollen.");
        telemetry.update();

        waitForStart();
        limelight.start();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            LLResultTypes.ColorResult best = getLargestBlob(result);

            if (best == null) {
                // No pollen seen: slowly spin to look for some.
                setDrive(0, searchTurn);
                telemetry.addData("State", "SEARCHING");
            } else {
                double tx = best.getTargetXDegrees();
                double ty = best.getTargetYDegrees();
                double area = best.getTargetArea();

                if (area >= stopArea) {
                    setDrive(0, 0);
                    telemetry.addData("State", "AT POLLEN");
                } else {
                    double turn = 0;
                    if (Math.abs(tx) > aimToleranceDegree) {
                        turn = Range.clip(tx * turnKP, -maxTurn, maxTurn);
                    }

                    double drive = 0;
                    if (Math.abs(tx) < driveAllowedDegree) {
                        double closeness = Range.clip(area / stopArea, 0, 1);
                        drive = maxDrive - (maxDrive - minDrive) * closeness;
                    }

                    setDrive(drive, turn);
                    telemetry.addData("State", "CHASING");
                }

                telemetry.addData("Blob tx (deg)", "%.2f", tx);
                telemetry.addData("Blob ty (deg)", "%.2f", ty);
                telemetry.addData("Blob area (%)", "%.3f", area);
            }

            telemetry.addData("Blobs seen", countBlobs(result));
            telemetry.update();
        }

        //turn off when opmode ends
        setDrive(0, 0);
        limelight.stop();
    }

    private LLResultTypes.ColorResult getLargestBlob(LLResult result) {
        if (result == null || !result.isValid()) return null;

        List<LLResultTypes.ColorResult> blobs = result.getColorResults();
        if (blobs == null || blobs.isEmpty()) return null;

        LLResultTypes.ColorResult largest = null;
        double largestArea = minArea;
        for (LLResultTypes.ColorResult blob : blobs) {
            double area = blob.getTargetArea();
            if (area > largestArea) {
                largestArea = area;
                largest = blob;
            }
        }
        return largest;
    }

    private int countBlobs(LLResult result) {
        if (result == null || !result.isValid()) return 0;
        List<LLResultTypes.ColorResult> blobs = result.getColorResults();
        return blobs == null ? 0 : blobs.size();
    }

    private void setDrive(double forward, double turn) {
        double fl = forward + turn;
        double bl = forward + turn;
        double fr = forward - turn;
        double br = forward - turn;

        // Normalize so no motor exceeds 1.0
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))));

        frontLeft.setPower(fl / max);
        backLeft.setPower(bl / max);
        frontRight.setPower(fr / max);
        backRight.setPower(br / max);
    }
}
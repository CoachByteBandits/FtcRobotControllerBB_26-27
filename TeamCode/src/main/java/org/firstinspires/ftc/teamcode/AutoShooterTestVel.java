
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "Shooter Test Velocity")
public class AutoShooterTestVel extends LinearOpMode {

    private DcMotorEx shooter;

    private static final double TICKS_PER_REV = 28.0;

    @Override
    public void runOpMode() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        //shooter velocity: 1400 ~ 50% power
        shooter.setVelocity(1400);

        while (opModeIsActive()) {
            double ticksPerSecond = shooter.getVelocity();
            double rpm = (ticksPerSecond / TICKS_PER_REV) * 60.0;

            // Telemetry
            telemetry.addData("Target Velocity", 1400);
            telemetry.addData("Actual Velocity", "%.2f ticks/sec", ticksPerSecond);
            telemetry.addData("Shooter RPM", "%.2f RPM", rpm);

            telemetry.update();
        }

        // Stop motor
        shooter.setVelocity(0);
        shooter.setPower(0);
    }
}

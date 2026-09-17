    package org.firstinspires.ftc.teamcode;

    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.CRServo;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.DcMotorEx;
    import com.qualcomm.robotcore.util.ElapsedTime;

    @TeleOp(name="sixHourBuild")
    public class sixhrBuildPractice extends LinearOpMode {
        private ElapsedTime runtime = new ElapsedTime();
        private DcMotorEx leftFrontDrive = null;
        private DcMotorEx leftBackDrive = null;
        private DcMotorEx rightFrontDrive = null;
        private DcMotorEx rightBackDrive = null;
        private DcMotorEx intakeMotor = null;
        private DcMotorEx flyWheel = null;

        @Override
        public void runOpMode() {
            // Hardware setup
            leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "left_Front_Drive");
            leftBackDrive   = hardwareMap.get(DcMotorEx.class, "left_Back_Drive");
            rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_Front_Drive");
            rightBackDrive  = hardwareMap.get(DcMotorEx.class, "right_Back_Drive");
            intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
            flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");


            // Reverse left side motors for proper forward movement
            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

            intakeMotor.setDirection((DcMotorEx.Direction.FORWARD));
            flyWheel.setDirection((DcMotorEx.Direction.REVERSE));

            telemetry.addData("Status", "Initialized");
            telemetry.update();

            waitForStart();
            runtime.reset();

            while (opModeIsActive()) {
                double max;
                // Read joystick inputs
                double axial   = -gamepad1.left_stick_y;  // Forward/backward
                double lateral =  gamepad1.left_stick_x;  // Strafing
                double yaw     =  gamepad1.right_stick_x; // Turning

                // Calculate motor powers using mecanum equations
                double leftFrontPower  = axial + lateral + yaw;
                double rightFrontPower = axial - lateral - yaw;
                double leftBackPower   = axial - lateral + yaw;
                double rightBackPower  = axial + lateral - yaw;

                // Normalize powers to stay within [-1.0, 1.0]
                max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
                max = Math.max(max, Math.abs(leftBackPower));
                max = Math.max(max, Math.abs(rightBackPower));

                if (max > 1.0) {
                    leftFrontPower  /= max;
                    rightFrontPower /= max;
                    leftBackPower   /= max;
                    rightBackPower  /= max;
                }

                // Set motor powers
                leftFrontDrive.setPower(leftFrontPower);
                rightFrontDrive.setPower(rightFrontPower);
                leftBackDrive.setPower(leftBackPower);
                rightBackDrive.setPower(rightBackPower);

                double flyWheelUpPower = gamepad2.right_trigger;
                double flyWheelDownPower = gamepad2.left_trigger;
                double scaledFlyWheelPower = (flyWheelUpPower - flyWheelDownPower)*0.6;
                flyWheel.setPower(scaledFlyWheelPower);

                if (gamepad2.a) {
                    intakeMotor.setVelocity(3000);
                }else if (gamepad2.x) {
                    intakeMotor.setVelocity(-1800);
                }
                else {
                    intakeMotor.setVelocity(0);
                }



                // Telemetry
                telemetry.addData("FL Wheel", leftFrontPower);
                telemetry.addData("FR Wheel", rightFrontPower);
                telemetry.addData("BL Wheel", leftBackPower);
                telemetry.addData("BR Wheel", rightBackPower);
                telemetry.addData("Intake Velocity", intakeMotor.getVelocity());
                telemetry.addData("Fly Wheel Velocity", flyWheel.getVelocity());
                telemetry.update();
            }
        }
    }
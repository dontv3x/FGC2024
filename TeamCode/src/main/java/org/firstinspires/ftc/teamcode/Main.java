package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import global.first.FeedingTheFutureGameDatabase;

@TeleOp(name="Main")
@Disabled
public class Main extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private DcMotorEx A = null;
    private DcMotorEx B = null;
    private DcMotorEx C = null;
    private DcMotorEx D = null;
    private DcMotorEx left = null;
    private DcMotorEx right = null;

    private int L_ticks;
    private int L_previousTicks;

    private int R_ticks;
    private int R_previousTicks;

    private float yaw = 0f;
    private float previous_yaw;

    private PIDFController pidfController;
    private double setpoint = 25505;

    private static final double ALIGNMENT_TOLERANCE = 0.05; // Tolerance for alignment
    private static final double POSITION_TOLERANCE = 0.1;   // Tolerance for position
    private static final double TURN_POWER = 0.5;           // Power for turning
    private static final double MOVE_POWER = 0.5;           // Power for moving

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private int[] id = {104, 105, 103, 100, 101, 102, 106, 107};
    private String[] location = {"Blue back 1", "Blue back 2", "Blue center back", "Blue center front", "Red center front", "Red center back", "Red back 2", "Red back 1"};

    @Override
    public void runOpMode() {
        initAprilTag();

        A = hardwareMap.get(DcMotorEx.class, "A");
        B = hardwareMap.get(DcMotorEx.class, "B");
        C = hardwareMap.get(DcMotorEx.class, "C");
        D = hardwareMap.get(DcMotorEx.class, "D");
        left = hardwareMap.get(DcMotorEx.class, "left");
        right = hardwareMap.get(DcMotorEx.class, "right");
        pidfController = new PIDFController(1.0, 0.0, 0.0, 0.1);

        A.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        C.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        D.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        A.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        C.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        D.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BNO055IMU imu;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetryAprilTag();

            // Push telemetry to the Driver Station.
            telemetry.update();

            // Save CPU resources; can resume streaming when needed.
            if (gamepad1.dpad_down) {
                visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
                visionPortal.resumeStreaming();
            }

            // Share the CPU.
            sleep(20);
            Orientation angles = imu.getAngularOrientation();
            telemetry.addData("Yaw", "Yaw: " + angles.firstAngle); // Z
            telemetry.addData("Roll", "Roll: " + angles.secondAngle); // X
            telemetry.addData("Pitch", "Pitch: " + angles.thirdAngle); // Y

            Acceleration acceleration = imu.getLinearAcceleration();
            telemetry.addData("Accel X", "Acc X: " + acceleration.xAccel);
            telemetry.addData("Accel Y", "Acc Y: " + acceleration.yAccel);
            telemetry.addData("Accel Z", "Acc Z: " + acceleration.zAccel);
            telemetry.update();

            ChangeInDegrees(angles, imu);
            telemetry.addData("Yaw1", "Changing Yaw: " + yaw);

            commands();

            //Print coordinate of the joystick
            telemetry.addData("key1", "Left stick > (" + gamepad1.left_stick_x + "," + gamepad1.left_stick_y + ")");
            telemetry.addData("key2", "Right stick > (" + gamepad1.right_stick_x + "," + gamepad1.right_stick_y + ")");

            telemetry.update();
        }
        visionPortal.close();
    }
    private void commands(){

        double A_position = A.getCurrentPosition();
        double B_position = B.getCurrentPosition();
        double C_position = C.getCurrentPosition();
        double D_position = D.getCurrentPosition();

        double A_setpoint = gamepad1.left_stick_y + gamepad1.right_stick_x;
        double B_setpoint = -gamepad1.left_stick_y + gamepad1.right_stick_x;
        double C_setpoint = -(gamepad1.left_stick_y + gamepad1.right_stick_x);
        double D_setpoint = gamepad1.left_stick_y + gamepad1.right_stick_x;

        A.setPower(pidfController.compute(A_setpoint, A_position));
        B.setPower(pidfController.compute(B_setpoint, B_position));
        C.setPower(pidfController.compute(C_setpoint, C_position));
        D.setPower(pidfController.compute(D_setpoint, D_position));

        //Determine what direction it would rotate
        A.setPower(gamepad1.left_stick_x);
        B.setPower(gamepad1.left_stick_x);
        C.setPower(gamepad1.left_stick_x);
        D.setPower(gamepad1.left_stick_x);


        //Ternary operator to activate motor on dpad
        left.setPower(gamepad1.dpad_up ? 1 : gamepad1.dpad_down ? -1 : 0);
        right.setPower(gamepad1.dpad_up ? 1 : gamepad1.dpad_down ? -1 : 0);

        if(gamepad1.dpad_up || gamepad1.dpad_down){
            int L_currentTicks = left.getCurrentPosition();
            int R_currentTicks = right.getCurrentPosition();

            int LdeltaTicks = L_currentTicks - L_previousTicks;
            int RdeltaTicks = R_currentTicks - R_previousTicks;

            L_ticks += LdeltaTicks;
            R_ticks += RdeltaTicks;
        }

        if(gamepad1.cross){
            MoveToTicks();
        }

        if (gamepad1.triangle) {
            // Determine the direction to rotate (-1 for left, 1 for right)
            int direction = (yaw > 0) ? 1 : (yaw < 0) ? -1 : 0;

            // Rotate the robot until it reaches yaw = 0
            if (direction != 0) {
                A.setPower(direction); // Adjust power as needed
                B.setPower(direction);
                C.setPower(-direction); // Rotate opposite motors in reverse
                D.setPower(direction);
            } else {
                // Stop the motors when yaw = 0
                A.setPower(0);
                B.setPower(0);
                C.setPower(0);
                D.setPower(0);
            }
        }
    }

    //Encoder
    private void MoveToTicks() {
        // Initialize the PIDF controller
        PIDFController pidf = new PIDFController(1.0, 0.0, 0.0, 0.1); // Adjust coefficients as needed

        // Reset and configure motors
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for start command
        waitForStart();

        // Main loop
        while (Math.abs(left.getCurrentPosition() - L_ticks) > POSITION_TOLERANCE &&
                Math.abs(right.getCurrentPosition() - R_ticks) > POSITION_TOLERANCE) {
            // Get current positions
            double leftPosition = left.getCurrentPosition();
            double rightPosition = right.getCurrentPosition();

            // Compute the PIDF output
            double leftPower = pidf.compute(L_ticks, leftPosition);
            double rightPower = pidf.compute(R_ticks, rightPosition);

            // Set power to motors
            left.setPower(leftPower);
            right.setPower(rightPower);

            // Telemetry
            telemetry.addData("left velocity", left.getVelocity());
            telemetry.addData("right velocity", right.getVelocity());
            telemetry.addData("left position", left.getCurrentPosition());
            telemetry.addData("right position", right.getCurrentPosition());
            telemetry.addData("left power", leftPower);
            telemetry.addData("right power", rightPower);
            telemetry.update();
        }

        // Stop motors after reaching the target
        left.setPower(0);
        right.setPower(0);

        // Reset motor modes
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //Imu
    private void ChangeInDegrees(Orientation a, BNO055IMU imu){
        a = imu.getAngularOrientation();
        float currentYaw = a.firstAngle;
        float deltaYaw = currentYaw - previous_yaw;
        previous_yaw = currentYaw;
        this.yaw += deltaYaw;

    }

    //AprilTag
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(FeedingTheFutureGameDatabase.getFeedingTheFutureTagLibrary())
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)

                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the webcam (name is assumed to be "Webcam 1")
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d)", detection.id));
                telemetry.addLine(detection.metadata.name);
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (cm)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (cm, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                if(gamepad1.circle){
                    reposition(detection);
                }
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }
    private void reposition(AprilTagDetection detection) {
        if (detection == null || detection.ftcPose == null) {
            telemetry.addData("Error", "Detection or pose is null");
            return;
        }

        double horizontal = detection.ftcPose.x;
        double distance = detection.ftcPose.y;
        double yaw1 = detection.ftcPose.yaw;
        double chorizontal = detection.center.x;

        // Calculate alignment errors
        double horizontalError = chorizontal;
        double rotationError = yaw1;

        // Check if the tag is within the tolerance range
        if (Math.abs(horizontalError) > ALIGNMENT_TOLERANCE) {
            // Adjust movement to align horizontally
            double movePower = (horizontalError > 0) ? -MOVE_POWER : MOVE_POWER;
            A.setPower(movePower);
            B.setPower(movePower);
            C.setPower(-movePower);
            D.setPower(movePower);
        } else if (Math.abs(rotationError) > ALIGNMENT_TOLERANCE) {
            // Adjust rotation to face the tag
            double turnPower = (rotationError > 0) ? TURN_POWER : -TURN_POWER;
            A.setPower(-turnPower);
            B.setPower(turnPower);
            C.setPower(turnPower);
            D.setPower(-turnPower);
        } else {
            // Stop the robot when aligned
            A.setPower(0);
            B.setPower(0);
            C.setPower(0);
            D.setPower(0);
        }

        telemetry.addData("Horizontal", horizontal);
        telemetry.addData("Vertical", distance);
        telemetry.addData("Center Horizontal", chorizontal);
        telemetry.addData("Horizontal Error", horizontalError);
        telemetry.addData("Rotation Error", rotationError);
    }

    //PID controller

}

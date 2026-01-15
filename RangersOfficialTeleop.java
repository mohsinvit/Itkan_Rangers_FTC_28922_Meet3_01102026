package org.firstinspires.ftc.teamcode.Teleop;

import android.icu.text.UFormat;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;
@TeleOp(name="RangersOfficialTeleop", group="Linear Opmode")
public class RangersOfficialTeleop extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx shooterRight;
    private DcMotorEx shooterLeft;
    private DcMotor intake;
    private DcMotor transfer;
    private Servo blocker;
    private DcMotor shooterPIDF;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    double closed = 1;
    double open = 0.60;


    @Override
    public void runOpMode() {


        waitForStart();
        initHardware();
        while (opModeIsActive()) {
            double x = 0;
            double angle = 0;
            double distance = 0;
            int id = 0;
            List<AprilTagDetection> tags = aprilTag.getDetections();
            for(AprilTagDetection tag: tags){
                if(tag.metadata == null)
                    continue;
                if(tag.id == 20 || tag.id == 24){
                    x = tag.ftcPose.x;
                    angle = tag.ftcPose.bearing;
                    distance = tag.ftcPose.range;
                    id = tag.id;
                }
            }
            telemetry.addData("x",x);
            telemetry.addData("distance",distance);
            telemetry.addData("angle",angle);

            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            if(gamepad1.right_trigger > 0.1){
                blocker.setPosition(closed);
                intake.setPower(1);
                transfer.setPower(1);
            }
            else if(gamepad1.left_trigger > 0.1){
                intake.setPower(-1);
                transfer.setPower(-1);
            }
            else{
                intake.setPower(0);
                transfer.setPower(0);
            }
            if(gamepad1.right_bumper || gamepad1.a){
                transfer.setPower(1);
                intake.setPower(1);
                boolean aligned = false;
                double tolerance = 1;
                if(gamepad1.a){
                    /// this is because the camera was aligning better to this angle
                    /// when it was to far zone
                    /// I noticed that it was shooting towards the side of the goal instead
                    /// of the middle and thus we had to adjust "our target" a bit
                    /// obviously sense our target is 0 I just adjusted x
                    /// changing x is a cheap way to solve the problems .. that works :thumbs_up:
                    /// elhamdolelah this should be consistent and our alignment values were
                    /// retuned according to the new camera position I think it is good
                    /// luckily, you're adjusted shooter is now much better elhamdolelah
                    /// and therefore we don't need to do much calculations and conditions
                    /// for the camera .. good luck!
                    if(id == 20) x-=10;
                    if(id == 24) x+=10;
                }
                if(Math.abs(x) > tolerance){
                    double kp = 0.01;
                    double kf = 0.11;
                    turn = kp * x + kf * Math.signum(x);
                }
                else{
                    aligned = true;
                }

                int target = calcTarget(distance,angle);
                if(gamepad1.a)
                    target = 1800;
                telemetry.addData("target: ",target);
                double currentVelo = shooterRight.getVelocity();
                if(currentVelo < target){
                    shooterRight.setPower(1);
                    shooterLeft.setPower(1);
                }
                else{
                    shooterRight.setPower(0);
                    shooterLeft.setPower(0);
                }

                if((aligned || x == 0) && Math.abs(target - currentVelo) < 50){
                    blocker.setPosition(open);
                }
                else{
                    // blocker.setPosition(closed);
                }
            }
            else{
                shooterRight.setPower(0);
                shooterLeft.setPower(0);
                blocker.setPosition(closed);
            }
            double frpower = drive + turn - strafe;
            double brpower = drive + turn + strafe;
            double flpower = drive - turn + strafe;
            double blpower = drive - turn - strafe;
            frontRight.setPower(frpower);
            backRight.setPower(brpower);
            frontLeft.setPower(flpower);
            backLeft.setPower(blpower);
            telemetry.update();
        }
    }
    int calcTarget(double dist,double ang){
        if(dist > 100){
            return 2000;
        }
        /// apparently the angle was no longer needed with the new shooter
        /// and we needed to lower down our speeds a bit .. 1500 was the norm now 1400
        /// would score almost anywhere and 1500 is for the a bit further places on the field
        /// driver practice is important. no matter how much we tune this;
        /// driver practice would be of more importance
        if(dist > 65)
            return 1500;
       
        // if(ang < 1){
        //     if(dist < 50){
        //         return 1500;
        //     }
        // }
        // else{
        //     if(dist > 62){
        //         return 1600;
        //     }
        // }
        return 1400;
    }
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        // CHECK: Webcam name matches your configuration
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }
    void initHardware(){

        initAprilTag();

        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterright");
        shooterLeft  = hardwareMap.get(DcMotorEx.class, "shooterleft");

        intake   = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");

        blocker = hardwareMap.get(Servo.class, "blocker");


        // ===== Motor Directions (common mecanum setup) =====
        // ADJUST: If driving is wrong, flip these directions
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // ===== Drive settings =====
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ===== Shooter settings =====
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Start with blocker closed
        blocker.setPosition(closed);
    }

}

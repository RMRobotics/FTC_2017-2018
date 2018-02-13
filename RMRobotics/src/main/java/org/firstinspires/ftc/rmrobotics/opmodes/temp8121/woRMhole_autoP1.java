package org.firstinspires.ftc.rmrobotics.opmodes.temp8121;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static com.sun.tools.javac.util.Constants.format;

/**
 * Created by rotom on 10/17/2017.
 */

@Autonomous(name="testVuAuto", group ="woRMholeConfig")
public class woRMhole_autoP1 extends LinearOpMode {
    private ColorSensor colorSensor;
    private DcMotor wheelFL;
    private DcMotor wheelFR;
    private DcMotor wheelBL;
    private DcMotor wheelBR;
    private DcMotor lift;
    private Servo clawBL;
    private Servo clawBR;
    private Servo clawTL;
    private Servo clawTR;
    private Servo gemBar;
    private ElapsedTime runtime = new ElapsedTime();
    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        wheelFL = hardwareMap.dcMotor.get("wheelFL");
        wheelFR = hardwareMap.dcMotor.get("wheelFR");
        wheelBL = hardwareMap.dcMotor.get("wheelBL");
        wheelBR = hardwareMap.dcMotor.get("wheelBR");
        lift = hardwareMap.dcMotor.get("lift");
        clawBL = hardwareMap.servo.get("clawBL");
        clawBR = hardwareMap.servo.get("clawBR");
        clawTL = hardwareMap.servo.get("clawTL");
        clawTR = hardwareMap.servo.get("clawTR");
        gemBar = hardwareMap.servo.get("gemBar");

        //encoder setup and reset
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();
        wheelBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //starting vuforia
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AckoWtn/////AAAAGan7WAnq/0UVmQZG3sp7smBgRCNBnU1p+HmsTrC+W9TyxqaMlhFirDXglelvJCX4yBiO8oou6n7UWBfdRFbKHDqz0NIo5VcNHyhelmm0yK0vGKxoU0NZbQzjh5qVWnI/HRoFjM3JOq/LB/FTXgCcEaNGhXAqnz7nalixMeP8oRQlgX5nRVX4uE6w0K4yqIc5/FIDh1tn7PldiflmvNPhOW6FukPQD3d02wEnZB/JEchSSBzDbFA10XSgtYzXiweQI5tj+D5llLRrLh0mcWeouv55oSmya5RxUC26uEuO7bCAwyolWIuUr2Wh5oAG483nTD4vFhdjVMT7f0ovLO73C6xr2AXpNwen9IExRxBeosQ4";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        telemetry.update();
        waitForStart();
        relicTrackables.activate();

        //values used for vuforia navigation
        float mmPerInch = 25.4f;
        float mmBotWidth = 18 * mmPerInch;
        float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch;
        OpenGLMatrix camera2bot = OpenGLMatrix
                .translation(3.75f * mmPerInch, 5.37f * mmPerInch, 6.248f * mmPerInch) //temporary and arbitrary values. goal is roughly closer to the front, far right side, near the bottom ish
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZY,
                        AngleUnit.DEGREES, 0, 0, 0));

        //trans: bottom left of field is 0,0. red bottom picture is (0,34,0) in inches. blue bottom (144,34,0) red top (0,106,0) blue top (144,106,0)
        //rot: (0,0,90) (0,0,-90), (0,0,0), (0,0,0)
        //this program is testing red top
        OpenGLMatrix pic2field = OpenGLMatrix
                .translation(0, 106.0f * mmPerInch, 0) //temporary and arbitrary values. goal is roughly closer to the front, far right side, near the bottom ish
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZY,
                        AngleUnit.DEGREES, 0, 0, 90));

        waitForStart();
        while (opModeIsActive()) {

            //gemBar

            //gyro alignment

            //loops until the camera can find the vuforia image
            OpenGLMatrix camera2pic = null;
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;
            while (vuMark == RelicRecoveryVuMark.UNKNOWN) {
                vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                    telemetry.addData("VuMark", "%s visible", vuMark);
                    camera2pic = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose(); //creates an openGLMatrix for the position of the picture relative to the phone
                } else {
                    telemetry.addData("VuMark", "not visible");
                }
            }

            camera2pic.rotated(AngleUnit.DEGREES, -90, 0, 1, 0); //puts the picture on the right axis

            //finds bot 2 pic and bot 2 field using the measurements from camera 2 pic
            OpenGLMatrix bot2pic = camera2pic.multiplied(camera2bot);
            OpenGLMatrix bot2field = new OpenGLMatrix();
            bot2field = bot2pic.multiplied(pic2field);
            telemetry.addData("c2p", format(camera2bot));
            telemetry.addData("b2p", format(bot2pic));
            telemetry.addData("b2f", format(bot2field));

            VectorF move;
            //setting the movement vectors for getting to the right column
            if (vuMark == RelicRecoveryVuMark.LEFT) {
                move = getVector(bot2field, 609.6f, 1939.798f);
            }
            if (vuMark == RelicRecoveryVuMark.CENTER) {
                move = getVector(bot2field, 609.6f, 2133.6f);
            }
            if (vuMark == RelicRecoveryVuMark.RIGHT) {
                move = getVector(bot2field, 609.6f, 2327.402f);
            }
            telemetry.update();

            wheelFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Send telemetry message to indicate successful Encoder reset
            telemetry.addData("Path0", "Starting at %7d :%7d",
                    wheelFL.getCurrentPosition());
            telemetry.update();
        }
    }

    public VectorF getVector(OpenGLMatrix bot, float desX, float desY) {
        VectorF amHere = bot.getTranslation();
        VectorF goThere = new VectorF(0, desX, desY);
        return goThere.subtracted(amHere);
    }
}
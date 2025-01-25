package org.firstinspires.ftc.teamcode.DUC_auto;

import static org.firstinspires.ftc.teamcode.lib.Hardware.closeClawAngle;
import static org.firstinspires.ftc.teamcode.lib.Hardware.openClawAngle;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;

@Config
@Disabled
@Autonomous(name = "TEST", group = "Autonomous")
public class TEST_AUTO extends LinearOpMode {

    int armTickPosition = 0;
    int normalArmTickPosition = 100;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(25, -64, Math.toRadians(270));
        MecanumDrive drive = new PinpointDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        NormalClaw normalClaw = new NormalClaw(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        NormalArm normalArm = new NormalArm(hardwareMap);


        Vector2d highRung = new Vector2d(10, -28);

        TrajectoryActionBuilder trajectory0 = drive.actionBuilder(new Pose2d(24.30, -63.21, Math.toRadians(270.00)))
                .setReversed(true)
                .splineTo(new Vector2d(5.29, -31.29), Math.toRadians(90.00));

        TrajectoryActionBuilder trajectory1 = drive.actionBuilder(new Pose2d(5.29, -31.29, Math.toRadians(270.00)))
                .setReversed(false)
                .splineTo(new Vector2d(34.88, -31.29), Math.toRadians(90.00))
                .splineTo(new Vector2d(47.79, -14.95), Math.toRadians(90.00));





        Actions.runBlocking(claw.closeClaw());
        Actions.runBlocking(normalClaw.openClaw());

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                trajectory0.build(),
                                trajectory1.build()
                                /*new ParallelAction(
                                        arm.low(),
                                        getFirstSpecimen.build()
                                ),
                                claw.closeClaw(),
                                new ParallelAction(
                                        arm.highRung(),
                                        hangFirstSpecimen.build()
                                ),
                                arm.highRung2(),
                                new SleepAction(0.25),
                                claw.openClaw(),
                                new ParallelAction(
                                        arm.low(),
                                        goPark.build()
                                ),
                                claw.closeClaw()
                                /*new ParallelAction( I HATE OPTIMIZING FOR TIME!!!!!!!! GIVE ME DEAD ENCODERS NOW!!!!
                                        arm.highRung(),
                                        hangSecondSpecimen.build()
                                ),
                                arm.highRung2(),
                                new SleepAction(0.25),
                                claw.openClaw()*/
                        ),
                        arm.keepPosition(),
                        normalArm.keepPosition()
                )

        );

    }
    public class Arm {
        private Motor arm;

        public Arm(HardwareMap hardwareMap) {
            arm = new Motor(hardwareMap, "specimenArm");
            arm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            arm.resetEncoder();
        }

        public class KeepPosition implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                arm.setRunMode(Motor.RunMode.PositionControl);
                arm.setPositionCoefficient(0.01);
                arm.setTargetPosition(armTickPosition);
                arm.set(1);
                arm.setPositionTolerance(10);
                return true;
            }
        }
        public Action keepPosition() {
            return new KeepPosition();
        }

        public class HighRung implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armTickPosition = 2000;
                keepPosition();
                if (arm.atTargetPosition()) {
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action highRung() {
            return new HighRung();
        }
        public class HighRung2 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armTickPosition = 1300;
                if (arm.atTargetPosition()) {
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action highRung2() {
            return new HighRung2();
        }

        public class Low implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armTickPosition = 0;
                if (arm.atTargetPosition()) {
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action low() {
            return new Low();
        }
    }
    public class Claw {
        private ServoEx claw;

        public Claw(HardwareMap hardwareMap) {
            claw = new SimpleServo(hardwareMap, "specimenClaw", 0, 180);
        }
        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.turnToAngle(closeClawAngle);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }
        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.turnToAngle(openClawAngle);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }
    }

    public class NormalClaw {
        private ServoEx claw;

        public NormalClaw(HardwareMap hardwareMap) {
            claw = new SimpleServo(hardwareMap, "claw", 0, 180);
        }
        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.turnToAngle(closeClawAngle);
                if (claw.getAngle() == closeClawAngle) {
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }
        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.turnToAngle(openClawAngle);
                if (claw.getAngle() == openClawAngle) {
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }
    }


    public class NormalArm {
        private Motor arm;

        public NormalArm(HardwareMap hardwareMap) {
            arm = new Motor(hardwareMap, "arm");
            arm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            arm.setInverted(true);
            arm.resetEncoder();
        }

        public class KeepPosition implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                arm.setRunMode(Motor.RunMode.PositionControl);
                arm.setPositionCoefficient(0.01);
                arm.setTargetPosition(normalArmTickPosition);
                arm.set(0.75);
                arm.setPositionTolerance(10);
                return true;
            }
        }
        public Action keepPosition() {
            return new KeepPosition();
        }

        public class HighBasket implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armTickPosition = 2500;
                keepPosition();
                if (arm.atTargetPosition()) {
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action highBasket() {
            return new HighBasket();
        }
        public class Mid implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armTickPosition = 1900;
                keepPosition();
                if (arm.atTargetPosition()) {
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action mid() {
            return new Mid();
        }

        public class Low implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armTickPosition = 0;
                if (arm.atTargetPosition()) {
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action low() {
            return new Low();
        }
    }

}


package org.firstinspires.ftc.teamcode.main;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class PID {
    public DcMotorEx motor;
    private final float dir;
    private double vel;
    public PID(DcMotor pid_motor, float motor_dir, double p, double i, double d, double f) {
        motor = (DcMotorEx)pid_motor;
        dir = motor_dir;
        motor.setVelocityPIDFCoefficients(p, i, d, f);
    }
    public void setRPM(int rpm) {
        vel = rpm * 6 * dir / abs(dir);
        motor.setVelocity(vel, AngleUnit.DEGREES);
    };
}

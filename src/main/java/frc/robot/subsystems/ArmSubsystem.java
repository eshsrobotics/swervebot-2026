package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
//import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants.WheelIndex;

public class ArmSubsystem extends SubsystemBase {

    private InputSubsystem input;

    /**
     * Controls the left motor of the 4 bar lift.
     */
    private SparkMax LeftLift;

    /**
     * Controls the right motor of the 4 bar lift, which will be
     * a follower to move both of the motors simultaneously.
     */
    private SparkMax RightLift;

    /**
     * Left coral intake motor.
     */
    private SparkMax LeftCoral;

    /**
     * Right coral intake motor. Similar to the lift motors,
     * this will be a follower of the left coral intake motor.
     */
    private SparkMax RightCoral;


    private RelativeEncoder leftLiftEncoder;

    private SparkClosedLoopController pidController;

    private double armSpeed;

    private boolean isOuttaking;

    public ArmSubsystem(InputSubsystem inputSubsystem) {
        input = inputSubsystem;
        LeftLift = new SparkMax(Constants.ArmConstants.LEFT_LIFT_CAN_ID, MotorType.kBrushless);
        RightLift = new SparkMax(Constants.ArmConstants.RIGHT_LIFT_CAN_ID, MotorType.kBrushless);
        LeftCoral = new SparkMax(Constants.ArmConstants.LEFT_CORAL_CAN_ID, MotorType.kBrushless);
        RightCoral = new SparkMax(Constants.ArmConstants.RIGHT_CORAL_CAN_ID, MotorType.kBrushless);

        leftLiftEncoder = LeftLift.getEncoder();

        pidController = LeftLift.getClosedLoopController();

        // The idle mode of the coral intake and lift motors are set to brake to prevent them from
        // continuing to move after the movement is halted. The algae config idle mode is set to
        // coast due to the motors being in control of flywheels, meaning that the flywheels will
        // slowly spin to a halt rather than fully stopping immediately.
        SparkMaxConfig liftConfig = new SparkMaxConfig();
        liftConfig.idleMode(IdleMode.kBrake);
        liftConfig.closedLoop.pid(0.1, 0, 0);
        //liftConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        SparkMaxConfig followLiftConfig = new SparkMaxConfig();
        //followLiftConfig.follow(Constants.ArmConstants.LEFT_LIFT_CAN_ID, true).apply(liftConfig);
        followLiftConfig.inverted(true).apply(liftConfig);

        SparkMaxConfig coralConfig = new SparkMaxConfig();
        coralConfig.idleMode(IdleMode.kBrake);

        SparkMaxConfig followCoralConfig = new SparkMaxConfig();
        followCoralConfig.follow(Constants.ArmConstants.LEFT_CORAL_CAN_ID, true).apply(coralConfig);


        LeftLift.configure(liftConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        RightLift.configure(followLiftConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        LeftCoral.configure(coralConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        RightCoral.configure(followCoralConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }



    @Override
    public void periodic() {
        if (DriverStation.isTeleopEnabled()) {
            this.setArmSpeed(input.getArmMovement());
            this.spinOuttake(input.isCoralIntakeActivated());
        }
        if (isOuttaking) {
            LeftCoral.set(Constants.ArmConstants.CORAL_OUTTAKE_SPEED);
            System.out.println("Gong outtaking at " + Constants.ArmConstants.CORAL_OUTTAKE_SPEED * 100 + "%");
        } else {
            LeftCoral.stopMotor();
        }

        // Move the arm according to input from teleop or autonomous.
        if (Math.abs(armSpeed) >= Constants.MathConstants.EPSILON) {
            LeftLift.set(armSpeed * 1.0);
            RightLift.set(armSpeed * 1.1);
        } else {
            LeftLift.stopMotor();
            RightLift.stopMotor();
        }
    }

    /**
     * Moves the arm by the specified speed.
     *
     * @param speed The arm's speed as percentage between -1.0 (full speed
       down) and +1.0 (full speed up.) 0 stops the arm from moving.
    */
    public void setArmSpeed(double speed) {
        armSpeed = speed;
        //System.out.println("Moving arm at " + (speed * 100) + "%");
    }

    /**
     * Spins the outtake.
     *
     * @param speed The outtake speed as a percentage between 0 and 1.0.
     * Negative values are ignored.
    */
   public void spinOuttake(boolean active) {
        isOuttaking = active;
   }

   @Override
   public void initSendable(SendableBuilder builder) {
        // The RightMotor are always having the same speed as the left motors
        // as they are following them. As such, we only get the speed of the
        // leader, the LeftMotor. The right motor will be spinning in the opposite direction though.
        builder.addDoubleProperty("Lift", () -> LeftLift.get(), (armSpeed) -> LeftLift.set(armSpeed));
        builder.addDoubleProperty("Coral", () -> LeftCoral.get(), (intakeSpeed) -> LeftCoral.set(intakeSpeed));
        initSendable(builder);
    }

}
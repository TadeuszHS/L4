package frc.robot.subsystems.vision;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class Vision extends SubsystemBase {
    private static final String klimelight = "limelight-three";
    private final CommandSwerveDrivetrain m_drivetrain;
    private final Pigeon2 m_pigeon2;
    private final SwerveModule<TalonFX, TalonFX, CANcoder> m_frontLeftModule;
    private final SwerveModule<TalonFX, TalonFX, CANcoder> m_frontRightModule;
    private final SwerveModule<TalonFX, TalonFX, CANcoder> m_backLeftModule;
    private final SwerveModule<TalonFX, TalonFX, CANcoder> m_backRightModule;
    private final SwerveDrivePoseEstimator m_estimator;
    private final Field2d m_field = new Field2d();
    private Pose2d m_pose;

    public Vision(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        m_pigeon2 = drivetrain.getPigeon2();
        m_frontLeftModule = drivetrain.getModule(0);
        m_frontRightModule = drivetrain.getModule(1);
        m_backLeftModule = drivetrain.getModule(2);
        m_backRightModule = drivetrain.getModule(3);
        LimelightHelpers.setCameraPose_RobotSpace(klimelight, -0.356, 0, 0, 0, 19, 180);
        LimelightHelpers.SetRobotOrientation(klimelight, 180, 0, 19, 0, 0, 0);
        m_pose = LimelightHelpers.getBotPose2d_wpiRed(klimelight);
        m_estimator = new SwerveDrivePoseEstimator(
            drivetrain.getKinematics(),
            m_pigeon2.getRotation2d(),
            new SwerveModulePosition[] {
                m_frontLeftModule.getPosition(false),
                m_frontRightModule.getPosition(false),
                m_backLeftModule.getPosition(false),
                m_backRightModule.getPosition(false),
            },
            m_pose
        );
    }

    //TODO: Add autonomous commands

    @Override
    public void periodic() {
        m_pose = m_estimator.update(
            m_pigeon2.getRotation2d(),
            new SwerveModulePosition[] {
                m_frontLeftModule.getPosition(false),
                m_frontRightModule.getPosition(false),
                m_backLeftModule.getPosition(false),
                m_backRightModule.getPosition(false),
            }
        );

        if (LimelightHelpers.getTV(klimelight)) {
            m_estimator.addVisionMeasurement(LimelightHelpers.getBotPose2d_wpiRed(klimelight), Timer.getFPGATimestamp());
        }

        m_field.setRobotPose(m_pose);
        SmartDashboard.putNumber("Reported Position X", m_estimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Reported Position Y", m_estimator.getEstimatedPosition().getY());
        SmartDashboard.putData("Field", m_field);
    }
}

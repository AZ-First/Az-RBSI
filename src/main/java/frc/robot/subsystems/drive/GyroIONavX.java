// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Queue;

/** IO implementation for Pigeon2 */
public class GyroIONavX implements GyroIO<AHRS> {
  private final AHRS navx =
      new AHRS(NavXComType.kMXP_SPI, (byte) SwerveConstants.kOdometryFrequency);
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  // Constructor, taking default values
  public GyroIONavX() {

    navx.reset();
    // Set Angle Adjustment based on alliance
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      navx.setAngleAdjustment(0.0);
    } else {
      navx.setAngleAdjustment(180.0);
    }
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(navx::getYaw);
  }

  // Return the Pigeon2 instance
  @Override
  public AHRS getGyro() {
    return navx;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = navx.isConnected();
    inputs.yawPosition = Rotation2d.fromDegrees(-navx.getAngle());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(-navx.getRawGyroZ());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(-value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }

  // /**
  //  * Zero the NavX
  //  *
  //  * <p>This method should always rezero the pigeon in ALWAYS-BLUE-ORIGIN orientation. Testing,
  //  * however, shows that it's not doing what I think it should be doing. There is likely
  //  * interference with something else in the odometry
  //  */
  // @Override
  // public void zero() {
  //   // With the Pigeon facing forward, forward depends on the alliance selected.
  //   // Set Angle Adjustment based on alliance
  //   if (DriverStation.getAlliance().get() == Alliance.Blue) {
  //     navx.setAngleAdjustment(0.0);
  //   } else {
  //     navx.setAngleAdjustment(180.0);
  //   }
  //   System.out.println("Setting YAW to " + navx.getAngleAdjustment());
  //   navx.zeroYaw();
  // }
}

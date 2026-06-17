package frc.robot.generated;

import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public final class TunerFactory {
  public static final TunerView INSTANCE = create();

  private static TunerView create() {
    // Debugging
    Logger.recordOutput("Drive/EncoderOffsets/RobotType", Constants.getRobot());

    // Return the proper view into TunerConstants
    return switch (Constants.getRobot()) {
      case DEVBOT2 -> new DEVBOT2TunerView();
      case DEVBOT1 -> new DEVBOT1TunerView();
      case COMPBOT -> new COMPBOTTunerView();
      default -> new COMPBOTTunerView();
    };
  }
}

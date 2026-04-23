// package frc.robot.subsystems;

// import static edu.wpi.first.units.Units.*;

// import com.ctre.phoenix6.CANBus;
// import com.ctre.phoenix6.configs.CANdleConfiguration;
// import com.ctre.phoenix6.controls.*;
// import com.ctre.phoenix6.hardware.CANdle;
// import com.ctre.phoenix6.signals.AnimationDirectionValue;
// import com.ctre.phoenix6.signals.RGBWColor;
// import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
// import com.ctre.phoenix6.signals.StripTypeValue;

// import edu.wpi.first.wpilibj.RobotBase;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.util.AllianceShifts;

// public class betterled extends SubsystemBase {
//     private static final int ONBOARD_START = 0;
//     private static final int ONBOARD_END = 7;
//     private static final int STRIP_START = 8;
//     private static final int STRIP_END = 67;

//     private static final double SHIFT_WARN_SECONDS = 5.0;
//     private static final double FLASH_RATE_HZ = 4.0;

//     private static final RGBWColor FLASH_ORANGE = new RGBWColor(255, 60, 0, 0);
//     private static final RGBWColor FLASH_WHITE = new RGBWColor(255, 255, 255, 0);
//     private static final RGBWColor OFF = new RGBWColor(0, 0, 0, 0);

//     private final CANdle m_candle = new CANdle(Constants.LedConstants.kCandleId, CANBus.roboRIO());
//     private final Timer m_teleopTimer;

//     private boolean m_flashing = false;
//     private String m_currentState = "OFF";
//     private String m_lastAnimationState = "";

//     public betterled(Timer teleopTimer) {
//         m_teleopTimer = teleopTimer;

//         CANdleConfiguration cfg = new CANdleConfiguration();
//         cfg.LED.StripType = StripTypeValue.GRB;
//         cfg.LED.BrightnessScalar = 0.5;
//         cfg.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
//         cfg.CANdleFeatures.Enable5VRail = com.ctre.phoenix6.signals.Enable5VRailValue.Enabled;
//         m_candle.getConfigurator().apply(cfg);

//         for (int i = 0; i < 8; i++) {
//             m_candle.setControl(new EmptyAnimation(i));
//         }

//         setDefaultCommand(defaultCommand());
//     }

//     @Override
//     public void periodic() {
//         double matchTime = m_teleopTimer.get();
//         if (matchTime == 0) {
//             m_flashing = false;
//             publishSim("DEFAULT", false);
//             return;
//         }

//         double remaining = AllianceShifts.getRemainingShiftTime(m_teleopTimer);
//         AllianceShifts.AllianceShift currentShift = AllianceShifts.getCurrentShift(m_teleopTimer);

//         boolean beforeShiftChange = remaining > 0 && remaining <= SHIFT_WARN_SECONDS
//             && currentShift != AllianceShifts.AllianceShift.Endgame;

//         boolean beforeMatchEnd = currentShift == AllianceShifts.AllianceShift.Endgame
//             && remaining > 0 && remaining <= SHIFT_WARN_SECONDS;

//         m_flashing = beforeShiftChange || beforeMatchEnd;

//         if (m_flashing) {
//             String flashType = beforeMatchEnd ? "ENDGAME_WHITE" : "SHIFT_ORANGE";
            
//             if (!m_lastAnimationState.equals(flashType)) {
//                 m_lastAnimationState = flashType;
//                 if (beforeMatchEnd) {
//                     m_candle.setControl(new StrobeAnimation(STRIP_START, STRIP_END - STRIP_START + 1)
//                         .withColor(FLASH_WHITE)
//                         .withSlot(0));
//                 } else {
//                     m_candle.setControl(new StrobeAnimation(STRIP_START, STRIP_END - STRIP_START + 1)
//                         .withColor(FLASH_ORANGE)
//                         .withSlot(0));
//                 }
//             }

//             double cycle = (Timer.getFPGATimestamp() * FLASH_RATE_HZ) % 1.0;
//             boolean simLedOn = cycle < 0.5;

//             publishSim(flashType, simLedOn);
//         } else {
//             publishSim("DEFAULT", false);
//         }

//         publishMatchInfo(matchTime, remaining, currentShift);
//     }

//     private void publishSim(String state, boolean flashOn) {
//         m_currentState = state;
//         SmartDashboard.putString("LED/State", state);
//         SmartDashboard.putBoolean("LED/Strip On", !state.equals("OFF") || !m_flashing);
//         SmartDashboard.putBoolean("LED/Flashing", m_flashing);
//         SmartDashboard.putBoolean("LED/Flash Tick", flashOn);
//     }

//     private void publishMatchInfo(double matchTime, double remaining, AllianceShifts.AllianceShift shift) {
//         SmartDashboard.putNumber("LED/Match Time", matchTime);
//         SmartDashboard.putNumber("LED/Shift Remaining", remaining);
//         SmartDashboard.putString("LED/Current Shift", shift.toString());
//     }

//     public Command defaultCommand() {
//         return run(() -> {
//             if (!m_flashing) {
//                 if (!m_lastAnimationState.equals("DEFAULT")) {
//                     m_lastAnimationState = "DEFAULT";
//                     m_candle.setControl(
//                         new SolidColor(ONBOARD_START, ONBOARD_END)
//                             .withColor(OFF)
//                     );
//                     m_candle.setControl(
//                         new ColorFlowAnimation(STRIP_START, STRIP_END - STRIP_START + 1)
//                             .withColor(new RGBWColor(50, 45, 255, 0))
//                             .withDirection(AnimationDirectionValue.Forward)
//                             .withSlot(0)
//                     );
//                 }
//             }
//         });
//     }

//     public Command setSolidColor(int r, int g, int b) {
//         return runOnce(() -> {
//             m_candle.setControl(
//                 new SolidColor(STRIP_START, STRIP_END)
//                     .withColor(new RGBWColor(r, g, b, 0))
//             );
//         });
//     }

//     public Command setRainbow() {
//         return runOnce(() -> {
//             m_candle.setControl(
//                 new RainbowAnimation(STRIP_START, STRIP_END).withSlot(0)
//             );
//         });
//     }

//     public Command setOff() {
//         return runOnce(() -> {
//             for (int i = 0; i < 8; i++) {
//                 m_candle.setControl(new EmptyAnimation(i));
//             }
//             m_candle.setControl(
//                 new SolidColor(ONBOARD_START, ONBOARD_END).withColor(OFF)
//             );
//             m_candle.setControl(
//                 new SolidColor(STRIP_START, STRIP_END).withColor(OFF)
//             );
//         });
//     }
// }
package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class Zones {
    public interface Zone {
        BooleanSupplier contains(Supplier<Pose2d> pose);
        Pose2d[] getCorners();
    }

    public interface PredictiveXZone extends Zone {
        BooleanSupplier willContain(Supplier<Pose2d> pose, Supplier<ChassisSpeeds> fieldSpeeds, double dt);
    }

    public static class BaseZone implements Zone {
        protected final double xMin, xMax, yMin, yMax;

        public BaseZone(double xMin, double xMax, double yMin, double yMax) {
            this.xMin = xMin;
            this.xMax = xMax;
            this.yMin = yMin;
            this.yMax = yMax;
        }

        @Override
        public BooleanSupplier contains(Supplier<Pose2d> poseSupplier) {
            return () -> this.containsPoint(poseSupplier.get().getTranslation());
        }

        protected boolean containsPoint(Translation2d point) {
            return point.getX() >= xMin && point.getX() <= xMax && point.getY() >= yMin && point.getY() <= yMax;
        }

        public BaseZone mirroredX(double fieldLength) {
            return new BaseZone(fieldLength - xMax, fieldLength - xMin, yMin, yMax);
        }

        @Override
        public Pose2d[] getCorners() {
            return new Pose2d[] {
                new Pose2d(xMin, yMin, new edu.wpi.first.math.geometry.Rotation2d()),
                new Pose2d(xMax, yMin, new edu.wpi.first.math.geometry.Rotation2d()),
                new Pose2d(xMax, yMax, new edu.wpi.first.math.geometry.Rotation2d()),
                new Pose2d(xMin, yMax, new edu.wpi.first.math.geometry.Rotation2d()),
                new Pose2d(xMin, yMin, new edu.wpi.first.math.geometry.Rotation2d()) 
            };
        }

        public BaseZone mirroredY(double fieldWidth) {
            return new BaseZone(xMin, xMax, fieldWidth - yMax, fieldWidth - yMin);
        }
    }

    public static class PredictiveXBaseZone extends BaseZone implements PredictiveXZone {
        public PredictiveXBaseZone(double xMin, double xMax, double yMin, double yMax) {
            super(xMin, xMax, yMin, yMax);
        }

        public PredictiveXBaseZone(BaseZone baseZone) {
            super(baseZone.xMin, baseZone.xMax, baseZone.yMin, baseZone.yMax);
        }

        @Override
        public BooleanSupplier willContain(Supplier<Pose2d> pose, Supplier<ChassisSpeeds> fieldSpeeds, double dt) {
            return () -> willContainPoint(pose.get().getTranslation(), fieldSpeeds.get(), dt);
        }

        protected boolean willContainPoint(Translation2d point, ChassisSpeeds fieldSpeeds, double dt) {
            return (point.getY() >= yMin && point.getY() <= yMax)
                    && ((point.getX() >= xMin && point.getX() <= xMax)
                            || (point.getX() < xMin && fieldSpeeds.vxMetersPerSecond * dt >= xMin - point.getX())
                            || (point.getX() > xMax && fieldSpeeds.vxMetersPerSecond * dt <= xMax - point.getX()));
        }

        @Override
        public PredictiveXBaseZone mirroredX(double fieldLength) {
            return new PredictiveXBaseZone(super.mirroredX(fieldLength));
        }

        @Override
        public PredictiveXBaseZone mirroredY(double fieldWidth) {
            return new PredictiveXBaseZone(super.mirroredY(fieldWidth));
        }
    }

    public static class ZoneCollection implements Zone {
        protected final Zone[] zones;

        public ZoneCollection(Zone... zones) {
            this.zones = zones;
        }

        @Override
        public BooleanSupplier contains(Supplier<Pose2d> pose) {
            return () -> {
                for (Zone zone : zones) {
                    if (zone.contains(pose).getAsBoolean()) return true;
                }
                return false;
            };
        }

        @Override
        public Pose2d[] getCorners() {
            int total = 0;
            for (Zone zone : zones) total += zone.getCorners().length;
            Pose2d[] allCorners = new Pose2d[total];
            int idx = 0;
            for (Zone zone : zones) {
                Pose2d[] corners = zone.getCorners();
                System.arraycopy(corners, 0, allCorners, idx, corners.length);
                idx += corners.length;
            }
            return allCorners;
        }
    }

    public static class PredictiveXZoneCollection extends ZoneCollection implements PredictiveXZone {
        public PredictiveXZoneCollection(PredictiveXZone... zones) {
            super(zones);
        }

        @Override
        public BooleanSupplier willContain(Supplier<Pose2d> pose, Supplier<ChassisSpeeds> fieldSpeeds, double dt) {
            return () -> {
                for (Zone zone : zones) {
                    if (((PredictiveXZone) zone).willContain(pose, fieldSpeeds, dt).getAsBoolean()) return true;
                }
                return false;
            };
        }
    }

    private static final double FIELD_LENGTH = 16.541;
    private static final double FIELD_WIDTH = 8.211;

    private static final PredictiveXBaseZone BLUE_BOTTOM_TRENCH = new PredictiveXBaseZone(
            Constants.DriveAssistConstants.TRENCH_BUMP_X - (Constants.DriveAssistConstants.TRENCH_BUMP_LENGTH / 2.0) - (Constants.RobotDimensions.FULL_LENGTH / 2.0),
            Constants.DriveAssistConstants.TRENCH_BUMP_X + (Constants.DriveAssistConstants.TRENCH_BUMP_LENGTH / 2.0) + (Constants.RobotDimensions.FULL_LENGTH / 2.0),
            0.0,
            Constants.DriveAssistConstants.TRENCH_WIDTH
    );

    private static final PredictiveXBaseZone BLUE_TOP_TRENCH = BLUE_BOTTOM_TRENCH.mirroredY(FIELD_WIDTH);
    private static final PredictiveXBaseZone RED_BOTTOM_TRENCH = BLUE_BOTTOM_TRENCH.mirroredX(FIELD_LENGTH);
    private static final PredictiveXBaseZone RED_TOP_TRENCH = BLUE_TOP_TRENCH.mirroredX(FIELD_LENGTH);

    public static final PredictiveXZoneCollection TRENCH_ZONES = new PredictiveXZoneCollection(
            BLUE_BOTTOM_TRENCH, BLUE_TOP_TRENCH, RED_BOTTOM_TRENCH, RED_TOP_TRENCH
    );

    private static final PredictiveXBaseZone BLUE_BOTTOM_BUMP = new PredictiveXBaseZone(
            Constants.DriveAssistConstants.TRENCH_BUMP_X - (Constants.DriveAssistConstants.TRENCH_BUMP_LENGTH / 2.0) - (Constants.RobotDimensions.FULL_LENGTH / 2.0),
            Constants.DriveAssistConstants.TRENCH_BUMP_X + (Constants.DriveAssistConstants.TRENCH_BUMP_LENGTH / 2.0) + (Constants.RobotDimensions.FULL_LENGTH / 2.0),
            Constants.DriveAssistConstants.TRENCH_WIDTH + Constants.DriveAssistConstants.TRENCH_BLOCK_WIDTH,
            Constants.DriveAssistConstants.TRENCH_WIDTH + Constants.DriveAssistConstants.TRENCH_BLOCK_WIDTH + Constants.DriveAssistConstants.BUMP_WIDTH
    );

    private static final PredictiveXBaseZone BLUE_TOP_BUMP = BLUE_BOTTOM_BUMP.mirroredY(FIELD_WIDTH);
    private static final PredictiveXBaseZone RED_BOTTOM_BUMP = BLUE_BOTTOM_BUMP.mirroredX(FIELD_LENGTH);
    private static final PredictiveXBaseZone RED_TOP_BUMP = BLUE_TOP_BUMP.mirroredX(FIELD_LENGTH);

    public static final PredictiveXZoneCollection BUMP_ZONES = new PredictiveXZoneCollection(
            BLUE_BOTTOM_BUMP, BLUE_TOP_BUMP, RED_BOTTOM_BUMP, RED_TOP_BUMP
    );
}

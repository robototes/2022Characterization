package dc;

import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

public class Mk4Configuration {

    private final Mk4SwerveModuleHelper.GearRatio ratio;
    private final int drive, angle, encoder;
    private final double offset;
    private final String canBus;

    public Mk4Configuration(Mk4SwerveModuleHelper.GearRatio rat, int dr, int ang, int enc, double off, String canBus) {
        ratio = rat;
        drive = dr;
        angle = ang;
        encoder = enc;
        offset = off;
        this.canBus = canBus;
    }

    public SwerveModule create() {
        return create(true);
    }

    public SwerveModule create(boolean steer) {
        return Mk4SwerveModuleHelper.createFalcon500(ratio, drive, angle, steer ? encoder : -1, steer ? offset : 0, canBus);
    }

    public Mk4SwerveModuleHelper.GearRatio getRatio() {
        return ratio;
    }

    public int getDrive() {
        return drive;
    }

    public int getAngle() {
        return angle;
    }

    public int getEncoder() {
        return encoder;
    }

    public double getOffset() {
        return offset;
    }
}
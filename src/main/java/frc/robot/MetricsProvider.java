package frc.robot;

import org.springframework.beans.factory.InitializingBean;
import org.springframework.stereotype.Component;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@Component
public class MetricsProvider implements InitializingBean{

    private final Field2d location = new Field2d();

    public void updateLocation(Pose2d pose) {
        location.setRobotPose(pose);
    }

    @Override
    public void afterPropertiesSet() throws Exception {
        SmartDashboard.putData("location", location);
    }

}

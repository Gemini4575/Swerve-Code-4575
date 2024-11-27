package frc.robot;

import org.springframework.beans.factory.InitializingBean;
import org.springframework.context.annotation.ComponentScan;
import org.springframework.context.annotation.Configuration;

@Configuration
@ComponentScan(basePackages = "frc.robot")
public class SpringConfig implements InitializingBean {

    @Override
    public void afterPropertiesSet() throws Exception {
        System.out.println("Spring context is loaded");
    }

}

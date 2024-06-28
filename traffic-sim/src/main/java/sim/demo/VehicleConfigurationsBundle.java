package sim.demo;

import org.opentrafficsim.core.gtu.GtuType;
import org.opentrafficsim.core.parameters.ParameterFactoryByType;

/**
 * Class to encapsulate created automation vehicle types and corresponding parameters.
 */
public class VehicleConfigurationsBundle {
    private final GtuType hdvCar;
    private final GtuType avCar;
    private final ParameterFactoryByType parameterFactory;

    public VehicleConfigurationsBundle(GtuType hdvCar, GtuType avCar, ParameterFactoryByType parameterFactory) {
        this.hdvCar = hdvCar;
        this.avCar = avCar;
        this.parameterFactory = parameterFactory;
    }

    public GtuType getHdvCar() {
        return hdvCar;
    }

    public GtuType getAvCar() {
        return avCar;
    }

    public ParameterFactoryByType getParameterFactory() {
        return parameterFactory;
    }
}

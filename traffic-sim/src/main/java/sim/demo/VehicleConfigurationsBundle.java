package sim.demo;

import java.util.ArrayList;

import org.opentrafficsim.core.gtu.GtuType;
import org.opentrafficsim.core.parameters.ParameterFactoryByType;

/**
 * Class to encapsulate created automation vehicle types and corresponding parameters.
 */
public class VehicleConfigurationsBundle {
    private final ArrayList<GtuType> automationGtuTypes;
    private final ParameterFactoryByType parameterFactory;

    public VehicleConfigurationsBundle(ArrayList<GtuType> automationGtuTypes, ParameterFactoryByType parameterFactory)
    {
        this.automationGtuTypes = automationGtuTypes;
        this.parameterFactory = parameterFactory;
    }

    public ArrayList<GtuType> getAutomationGtuTypes()
    {
    	return automationGtuTypes;
    }

    public ParameterFactoryByType getParameterFactory()
    {
        return parameterFactory;
    }
}

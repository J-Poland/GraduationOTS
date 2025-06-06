package sim.demo.mental;

import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterTypes;
import org.opentrafficsim.base.parameters.Parameters;
import org.opentrafficsim.road.gtu.lane.perception.mental.Fuller;
import org.opentrafficsim.road.gtu.lane.perception.mental.AdaptationSpeed;

import sim.demo.vehicleconfigurations.VehicleAutomationConfigurations;

/**
 * Behavioral adaptation which reduces the desired speed to reduce task-demand.
 * <p>
 * Copyright (c) 2013-2024 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved. <br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * </p>
 * @author <a href="https://github.com/averbraeck">Alexander Verbraeck</a>
 * @author <a href="https://tudelft.nl/staff/p.knoppers-1">Peter Knoppers</a>
 * @author <a href="https://github.com/wjschakel">Wouter Schakel</a>
 */
public class CustomAdaptationSpeed extends AdaptationSpeed
{

    /** Base value for the desired speed. */
    private Double fSpeed0;

    /** {@inheritDoc} */
    @Override
    public void adapt(final Parameters parameters, final double taskSaturation) throws ParameterException
    {
    	// only perform adaptation when the car-following task is not automated (thus level 0)
    	if (parameters.getParameterOrNull(VehicleAutomationConfigurations.AUTOMATION_LEVEL).contains("LEVEL0")) {
    		
	        if (this.fSpeed0 == null)
	        {
	            this.fSpeed0 = parameters.getParameter(ParameterTypes.FSPEED);
	        }
	        double eps = parameters.getParameter(Fuller.TS) - parameters.getParameter(Fuller.TS_CRIT);
	        eps = eps < 0.0 ? 0.0 : (eps > 1.0 ? 1.0 : eps);
	        double factor = 1.0 - parameters.getParameter(BETA_V0) * eps;
	        parameters.setParameter(ParameterTypes.FSPEED, this.fSpeed0 * factor);
    	}
    }

}


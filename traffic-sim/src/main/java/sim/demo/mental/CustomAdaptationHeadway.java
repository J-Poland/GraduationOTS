package sim.demo.mental;

import org.djunits.value.vdouble.scalar.Duration;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterTypes;
import org.opentrafficsim.base.parameters.Parameters;
import org.opentrafficsim.road.gtu.lane.perception.mental.AdaptationHeadway;
import org.opentrafficsim.road.gtu.lane.perception.mental.Fuller;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.LmrsParameters;

import sim.demo.vehicleconfigurations.VehicleAutomationConfigurations;

/**
 * Behavioral adaptation which increases the desired headway to reduce task-demand.
 * <p>
 * Copyright (c) 2013-2024 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved. <br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * </p>
 * @author <a href="https://github.com/averbraeck">Alexander Verbraeck</a>
 * @author <a href="https://tudelft.nl/staff/p.knoppers-1">Peter Knoppers</a>
 * @author <a href="https://github.com/wjschakel">Wouter Schakel</a>
 */
public class CustomAdaptationHeadway extends AdaptationHeadway
{
    /** Base value for the minimum desired headway. */
    private Duration t0Min;

    /** Base value for the maximum desired headway. */
    private Duration t0Max;

    /** {@inheritDoc} */
    @Override
    public void adapt(final Parameters parameters, final double taskSaturation) throws ParameterException
    {
    	// only perform adaptation when the car-following task is not automated (thus level 0)
    	if (parameters.getParameterOrNull(VehicleAutomationConfigurations.AUTOMATION_LEVEL).contains("LEVEL0")) {
    		
	        if (this.t0Min == null)
	        {
	            this.t0Min = parameters.getParameterOrNull(ParameterTypes.TMIN);
	            this.t0Max = parameters.getParameterOrNull(ParameterTypes.TMAX);
	        }
	        
	        double eps = parameters.getParameter(Fuller.TS) - parameters.getParameter(Fuller.TS_CRIT);
	        eps = eps < 0.0 ? 0.0 : (eps > 1.0 ? 1.0 : eps);
	        double factor = 1.0 + parameters.getParameter(BETA_T) * eps;
	        Duration tMin = this.t0Min.times(factor);
	        Duration tMax = this.t0Max.times(factor);
	        
	        
	        // account for human headway adaptations around level-3 vehicles
	        // base headway settings on own Tmin and level 3 Tmin
	        boolean inBetweenLevel3 = parameters.getParameter(VehicleAutomationConfigurations.IN_BETWEEN_LEVEL3);
	        // only adapt when the GTU is surrounded by level 3
	        if (inBetweenLevel3) {
		        Duration tMinLevel3 = parameters.getParameter(VehicleAutomationConfigurations.TMIN_LEVEL3);
	        	double socio_cf = parameters.getParameter(VehicleAutomationConfigurations.SOCIO_CF);
	        	// interpolating between tMin and tMinLevel3 based on socio parameter
	            tMin = tMin.times(1 - socio_cf).plus(tMinLevel3.times(socio_cf));
	        }
	        
	        
	        if (tMax.si <= parameters.getParameter(ParameterTypes.TMIN).si)
	        {
	            parameters.setParameter(ParameterTypes.TMIN, tMin);
	            parameters.setParameter(ParameterTypes.TMAX, tMax);
	        }
	        else
	        {
	            parameters.setParameter(ParameterTypes.TMAX, tMax);
	            parameters.setParameter(ParameterTypes.TMIN, tMin);
	        }
    	}
    }

}

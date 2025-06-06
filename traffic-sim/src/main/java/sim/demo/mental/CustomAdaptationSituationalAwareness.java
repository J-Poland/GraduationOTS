package sim.demo.mental;

import org.djunits.value.vdouble.scalar.Duration;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterTypes;
import org.opentrafficsim.base.parameters.Parameters;
import org.opentrafficsim.road.gtu.lane.perception.mental.AdaptationSituationalAwareness;
import org.opentrafficsim.road.gtu.lane.perception.mental.Fuller;

import sim.demo.vehicleconfigurations.VehicleAutomationConfigurations;

/**
 * Behavioral adaptation which sets parameters for situational awareness and reaction time.
 * 
 * <pre>
 *      / SA_MAX,                                                                         taskSaturation &lt; TS_CRIT
 * SA = | SA_MAX - (SA_MAX - SA_MIN) * (taskSaturation - TS_CRIT) / (TS_MAX - TS_CRIT),   TS_CRIT &lt;= taskSaturation &lt; TS_MAX 
 *      \ SA_MIN,                                                                         taskSaturation &gt;= TS_MAX
 * 
 * TR = (S_MAX - SA) * TR_MAX
 * </pre>
 * <p>
 * Copyright (c) 2013-2024 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved. <br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * </p>
 * @author <a href="https://github.com/averbraeck">Alexander Verbraeck</a>
 * @author <a href="https://tudelft.nl/staff/p.knoppers-1">Peter Knoppers</a>
 * @author <a href="https://github.com/wjschakel">Wouter Schakel</a>
 */
public class CustomAdaptationSituationalAwareness extends AdaptationSituationalAwareness
{

	/** {@inheritDoc} */
    @Override
    public void adapt(final Parameters parameters, final double taskSaturation) throws ParameterException
    {
        // situational awareness
        double tsCrit = parameters.getParameter(Fuller.TS_CRIT);
        double tsMax = parameters.getParameter(Fuller.TS_MAX);
        double saMin = parameters.getParameter(SA_MIN);
        double saMax = parameters.getParameter(SA_MAX);
        double sa = taskSaturation < tsCrit ? saMax
                : (taskSaturation >= tsMax ? saMin : saMax - (saMax - saMin) * (taskSaturation - tsCrit) / (tsMax - tsCrit));
        parameters.setParameter(SA, sa);
        // reaction time
        double trMin = parameters.getParameterOrNull(VehicleAutomationConfigurations.MIN_TR).si;
        double trMax = parameters.getParameterOrNull(VehicleAutomationConfigurations.MAX_TR).si;
        parameters.setParameter(ParameterTypes.TR, Duration.instantiateSI(trMin + (trMax - trMin) * (saMax - sa)));
//        parameters.setParameter(ParameterTypes.TR, Duration.instantiateSI(trMax * (saMax - sa)));
    }

}

package sim.demo.behavior;

import org.djunits.value.vdouble.scalar.Duration;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterTypes;
import org.opentrafficsim.base.parameters.Parameters;
import org.opentrafficsim.road.gtu.lane.perception.mental.Fuller;

/**
 * This class contains behavioural adaptation classes to change GTU's driving behaviour according to the 
 * driver's cognitive load. The Fuller class is responsible for keeping track of all driving tasks 
 * (which are managed in TaskManagerAr), and the load of these tasks affect driving behaviour 
 * through the behavioural adaptation classes defined here.
 */
public class CognitiveBehavioralAdaptations {

    // class for adapting reaction time
    public static class AdaptationTr implements Fuller.BehavioralAdaptation
    {
		@Override
		public void adapt(Parameters parameters, double taskSaturation) throws ParameterException
		{
			Duration currentTr = parameters.getParameter(ParameterTypes.TR);
            Duration adjustedTr = Duration.instantiateSI(currentTr.si * (1 - 0.5 * taskSaturation));
            parameters.setParameter(ParameterTypes.TR, adjustedTr);
		}
    }

}

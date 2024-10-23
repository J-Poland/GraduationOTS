package sim.demo.object;

import org.djunits.value.vdouble.scalar.Length;
import org.opentrafficsim.base.parameters.ParameterTypes;
import org.opentrafficsim.core.dsol.OtsSimulatorInterface;
import org.opentrafficsim.core.network.NetworkException;
import org.opentrafficsim.road.gtu.lane.LaneBasedGtu;
import org.opentrafficsim.road.network.lane.Lane;
import org.opentrafficsim.road.network.lane.object.Distraction;

/**
 * Distraction following a distance profile.
 * <p>
 * Copyright (c) 2013-2024 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved. <br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * </p>
 * @author <a href="https://github.com/averbraeck">Alexander Verbraeck</a>
 * @author <a href="https://tudelft.nl/staff/p.knoppers-1">Peter Knoppers</a>
 * @author <a href="https://github.com/wjschakel">Wouter Schakel</a>
 */
public class CustomDistraction extends Distraction
{

    /** */
    private static final long serialVersionUID = 20180405L;

    /**
     * @param id String; id
     * @param lane Lane; lane
     * @param longitudinalPosition Length; longitudinal position
     * @param simulator OtsSimulatorInterface; simulator
     * @param profile DistractionProfile; distraction profile
     * @throws NetworkException on network exception
     */
    public CustomDistraction(final String id, final Lane lane, final Length longitudinalPosition,
            final OtsSimulatorInterface simulator, final DistractionProfile profile) throws NetworkException
    {
        super(id, lane, longitudinalPosition, simulator, profile);
    }

    /**
     * Returns the level of distraction at the given distance.
     * @param distance Distance to distraction; negative when approaching
     * @return Double; level of distraction (task-demand), or {@code null} if the distraction is no longer important
     */
    public Double getDistraction(LaneBasedGtu gtu, final Length distance)
    {
    	// get look-ahead and look-back from GTU
    	double lookAhead = gtu.getParameters().getParameterOrNull(ParameterTypes.LOOKAHEAD).si;
    	double lookBack = gtu.getParameters().getParameterOrNull(ParameterTypes.LOOKBACK).si;
    	
    	// use these distances for the distraction profile
    	DistractionProfile profile = new TrapezoidProfile(0.5, Length.instantiateSI(-lookAhead), Length.instantiateSI(0.0), Length.instantiateSI(lookBack));
    	
        return profile.getDistraction(distance);
    }
    
    // ensure that the user always provides the GTU parameters to create a distraction profile
    @Override
    public Double getDistraction(final Length distance)
    {
    	System.err.println("To use the CustomDistraction class, you should use the "
    					   + "getDistraction(LaneBasedGtu gtu, Length distance) method, not getDistraction(Length distance).");
    	return null;
    }

}

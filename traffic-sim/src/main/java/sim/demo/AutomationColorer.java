package sim.demo;

import java.awt.Color;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.djunits.value.vdouble.scalar.Acceleration;
import org.djutils.immutablecollections.ImmutableMap;
import org.djutils.immutablecollections.ImmutableMap.ImmutableEntry;
import org.opentrafficsim.animation.gtu.colorer.GtuColorer;
import org.opentrafficsim.core.gtu.Gtu;
import org.opentrafficsim.core.gtu.GtuType;
import org.opentrafficsim.draw.ColorInterpolator;

import sim.demo.vehicleconfigurations.VehicleAutomationConfigurations;


/**
 * Color GTUs based on their current acceleration.
 * <p>
 * Copyright (c) 2013-2024 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved. <br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * </p>
 * @author <a href="https://github.com/averbraeck">Alexander Verbraeck</a>
 * @author <a href="https://tudelft.nl/staff/p.knoppers-1">Peter Knoppers</a>
 */
public class AutomationColorer implements GtuColorer, Serializable
{
    /** */
    private static final long serialVersionUID = 201500001L;

    private final ArrayList<LegendEntry> legend = new ArrayList<LegendEntry>();
    
    private ImmutableMap<String, Color> GTU_TYPE_COLORS;

    /**
     * Construct a new AutomationColorer.
     */
    public AutomationColorer(ImmutableMap<String, Color> GTU_TYPE_COLORS)
    {
    	this.GTU_TYPE_COLORS = GTU_TYPE_COLORS;
    	
    	for (ImmutableEntry<String, Color> entry : GTU_TYPE_COLORS.entrySet()) {
    		String name = entry.getKey();
    		Color color = entry.getValue();
    		
    		LegendEntry legendEntry = new LegendEntry(color, name, "Vehicle automation level: " + name);
    		this.legend.add(legendEntry);
    	}
    }

    /** {@inheritDoc} */
    @Override
    public final Color getColor(final Gtu gtu)
    {
        String gtuTypeString = gtu.getParameters().getParameterOrNull(VehicleAutomationConfigurations.AUTOMATION_LEVEL);
        return GTU_TYPE_COLORS.get(gtuTypeString);
    }

    /** {@inheritDoc} */
    @Override
    public final List<LegendEntry> getLegend()
    {
        return Collections.unmodifiableList(this.legend);
    }

    /** {@inheritDoc} */
    @Override
    public final String toString()
    {
        return "Automation level";
    }

}


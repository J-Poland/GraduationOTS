package sim.demo;

import java.awt.Color;
import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.function.BiFunction;

import org.djunits.unit.AccelerationUnit;
import org.djunits.unit.SpeedUnit;
import org.djunits.value.vdouble.scalar.Acceleration;
import org.djunits.value.vdouble.scalar.Duration;
import org.djunits.value.vdouble.scalar.Length;
import org.djunits.value.vdouble.scalar.Speed;
import org.djutils.immutablecollections.Immutable;
import org.djutils.immutablecollections.ImmutableLinkedHashMap;
import org.djutils.immutablecollections.ImmutableMap;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterTypes;
import org.opentrafficsim.core.definitions.Defaults;
import org.opentrafficsim.core.definitions.DefaultsNl;
import org.opentrafficsim.core.distributions.ConstantGenerator;
import org.opentrafficsim.core.gtu.GtuTemplate;
import org.opentrafficsim.core.gtu.GtuType;
import org.opentrafficsim.core.parameters.ParameterFactoryByType;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.LmrsParameters;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.Tailgating;

import nl.tudelft.simulation.jstats.distributions.DistNormal;
import nl.tudelft.simulation.jstats.streams.StreamInterface;

/**
 * Class to define vehicle types. Containing vehicle types to represent SAE automation levels 1 - 5.
 */
public class VehicleConfigurations extends Defaults implements BiFunction<GtuType, StreamInterface, GtuTemplate> {

	    /**
	     * Constructor setting locale nl_NL.
	     */
	    protected VehicleConfigurations() {
	        super(new Locale("nl", "NL"));
	    }
	    
	    // drawing colors for GTU types
	    public static final ImmutableMap<GtuType, Color> GTU_TYPE_COLORS;

	    // get existing opentrafficsim GTU types
	    public static final GtuType ROAD_USER = DefaultsNl.ROAD_USER;
	    public static final GtuType WATERWAY_USER = DefaultsNl.WATERWAY_USER;
	    public static final GtuType RAILWAY_USER = DefaultsNl.RAILWAY_USER;
	    public static final GtuType PEDESTRIAN = DefaultsNl.PEDESTRIAN;
	    public static final GtuType BICYCLE = DefaultsNl.BICYCLE;
	    public static final GtuType MOPED = DefaultsNl.MOPED;
	    public static final GtuType VEHICLE = DefaultsNl.VEHICLE;
	    public static final GtuType EMERGENCY_VEHICLE = DefaultsNl.EMERGENCY_VEHICLE;
	    public static final GtuType SHIP = DefaultsNl.SHIP;
	    public static final GtuType TRAIN = DefaultsNl.TRAIN;
	    public static final GtuType CAR = DefaultsNl.CAR;
	    public static final GtuType VAN = DefaultsNl.VAN;
	    public static final GtuType BUS = DefaultsNl.BUS;
	    public static final GtuType TRUCK = DefaultsNl.TRUCK;
	    public static final GtuType SCHEDULED_BUS = DefaultsNl.SCHEDULED_BUS;

	    // add automation GTU types
	    // Human Driven Vehicle (Car)
	    public static final GtuType HDV_CAR = new GtuType("NL.HDVCAR", CAR);
	    // Autonomous Vehicle (Car)
	    public static final GtuType AV_CAR = new GtuType("NL.AVCAR", CAR);
	    
	    @Override
	    public GtuTemplate apply(final GtuType gtuType, final StreamInterface randomStream)
	    {
	        GtuTemplate template = null;
	        if (gtuType.equals(HDV_CAR))
	        {
	            // from "Maatgevende normen in de Nederlandse richtlijnen voor wegontwerp", R-2014-38, SWOV
	            template = new GtuTemplate(gtuType, new ConstantGenerator<>(Length.instantiateSI(4.19)),
	                    new ConstantGenerator<>(Length.instantiateSI(1.7)),
	                    new ConstantGenerator<>(new Speed(180, SpeedUnit.KM_PER_HOUR)));
	        }
	        if (gtuType.equals(AV_CAR))
	        {
	            // from "Maatgevende normen in de Nederlandse richtlijnen voor wegontwerp", R-2014-38, SWOV
	            template = new GtuTemplate(gtuType, new ConstantGenerator<>(Length.instantiateSI(4.19)),
	                    new ConstantGenerator<>(Length.instantiateSI(1.7)),
	                    new ConstantGenerator<>(new Speed(180, SpeedUnit.KM_PER_HOUR)));
	        }
	        return template;
	    };
	    
	    // create color legend for different GTU types
	    static {
	    	LinkedHashMap<GtuType, Color> map = new LinkedHashMap<>();
	    	// custom automation GTU types
	    	map.put(HDV_CAR, Color.RED);
	    	map.put(AV_CAR, Color.BLUE);
	    	// existing GTU types
	        map.put(CAR, Color.BLACK);
	        map.put(TRUCK, Color.BLACK);
	        map.put(VEHICLE, Color.GRAY);
	        map.put(PEDESTRIAN, Color.YELLOW);
	        map.put(BICYCLE, Color.GREEN);
	        GTU_TYPE_COLORS = new ImmutableLinkedHashMap<>(map, Immutable.WRAP);
	    }

	/**
	 * Create vehicle type variables and set applicable parameters.
	 * 
	 * @param stream
	 * @return Map<String, Object>
	 * @throws ParameterException
	 */
	public static VehicleConfigurationsBundle setVehicleTypes(StreamInterface stream) throws ParameterException {
		
		// Select corresponding vehicle characteristics for vehicle types
		GtuType hdvCar = HDV_CAR;
		GtuType avCar = AV_CAR;
		
		// Create parameter factory for vehicle types
		ParameterFactoryByType paramFactory = new ParameterFactoryByType();
		
		// Set parameters for HDV
		// speed limit adherence
		paramFactory.addParameter(hdvCar, ParameterTypes.FSPEED, new DistNormal(stream, 1, 13.0 / 130));
		// headway
		paramFactory.addParameter(hdvCar, ParameterTypes.TMAX, Duration.instantiateSI(1.2));
		paramFactory.addParameter(hdvCar, ParameterTypes.TMIN, Duration.instantiateSI(0.56));
		// reaction time
		paramFactory.addParameter(hdvCar, ParameterTypes.TR, Duration.instantiateSI(1.5));
		// lookahead and lookback
		paramFactory.addParameter(hdvCar, ParameterTypes.LOOKAHEAD, Length.instantiateSI(295.0));
		paramFactory.addParameter(hdvCar, ParameterTypes.LOOKBACK, Length.instantiateSI(200.0));
		// affected by other vehicle speed
		paramFactory.addParameter(hdvCar, LmrsParameters.SOCIO, new DistNormal(stream, 0.5, 0.1)); 
		paramFactory.addParameter(hdvCar, Tailgating.RHO, 0.6);
		
		// Set parameters for AV
		// speed limit adherence
		paramFactory.addParameter(avCar, ParameterTypes.FSPEED, 1.05);
		// headway
		paramFactory.addParameter(avCar, ParameterTypes.TMAX, Duration.instantiateSI(2.0));
		paramFactory.addParameter(avCar, ParameterTypes.TMIN, Duration.instantiateSI(0.4));
		// reaction time
		paramFactory.addParameter(avCar, ParameterTypes.TR, Duration.instantiateSI(0.5));
//		// lookahead and lookback
		paramFactory.addParameter(avCar, ParameterTypes.LOOKAHEAD, Length.instantiateSI(100.0));
		paramFactory.addParameter(avCar, ParameterTypes.LOOKBACK, Length.instantiateSI(60.0));
		// affected by other vehicle speed
		paramFactory.addParameter(avCar, LmrsParameters.SOCIO, 0.8);
		paramFactory.addParameter(avCar, Tailgating.RHO, 0.0);
		
		// return encapsulated vehicle types and parameters
        return new VehicleConfigurationsBundle(hdvCar, avCar, paramFactory);
	}
}



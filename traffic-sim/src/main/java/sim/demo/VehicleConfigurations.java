package sim.demo;

import java.awt.Color;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.function.BiFunction;

import org.djunits.unit.AccelerationUnit;
import org.djunits.unit.SpeedUnit;
import org.djunits.value.vdouble.scalar.Acceleration;
import org.djunits.value.vdouble.scalar.Duration;
import org.djunits.value.vdouble.scalar.Length;
import org.djunits.value.vdouble.scalar.Speed;
import org.djutils.exceptions.Throw;
import org.djutils.immutablecollections.Immutable;
import org.djutils.immutablecollections.ImmutableLinkedHashMap;
import org.djutils.immutablecollections.ImmutableMap;
import org.djutils.logger.CategoryLogger;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterTypeAcceleration;
import org.opentrafficsim.base.parameters.ParameterTypeDouble;
import org.opentrafficsim.base.parameters.ParameterTypeDuration;
import org.opentrafficsim.base.parameters.ParameterTypes;
import org.opentrafficsim.base.parameters.Parameters;
import org.opentrafficsim.core.definitions.Defaults;
import org.opentrafficsim.core.definitions.DefaultsNl;
import org.opentrafficsim.core.distributions.ConstantGenerator;
import org.opentrafficsim.core.gtu.GtuTemplate;
import org.opentrafficsim.core.gtu.GtuType;
import org.opentrafficsim.core.parameters.ParameterFactoryByType;
import org.opentrafficsim.road.gtu.lane.perception.categories.neighbors.Estimation;
import org.opentrafficsim.road.gtu.lane.perception.mental.AdaptationHeadway;
import org.opentrafficsim.road.gtu.lane.perception.mental.AdaptationSituationalAwareness;
import org.opentrafficsim.road.gtu.lane.perception.mental.AdaptationSpeed;
import org.opentrafficsim.road.gtu.lane.perception.mental.Fuller;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.LmrsParameters;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.Tailgating;

import nl.tudelft.simulation.jstats.distributions.DistTriangular;
import nl.tudelft.simulation.jstats.streams.StreamInterface;
import sim.demo.mental.CarFollowingTask;
import sim.demo.mental.CustomAdaptationHeadway;
import sim.demo.mental.CustomAdaptationSituationalAwareness;
import sim.demo.mental.TaskManagerAr;

/**
 * Class to define vehicle types. Containing vehicle types to represent SAE automation levels 1 - 5.
 */
public class VehicleConfigurations extends Defaults implements BiFunction<GtuType, StreamInterface, GtuTemplate> {
	
	// new parameters
    public static final ParameterTypeDuration INITIAL_TMIN = new ParameterTypeDuration("INITIAL_TMIN", "Initial minimal headway time");
	    
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

    // add automation level GTU types
    static ArrayList<GtuType> automationGtuTypes = new ArrayList<GtuType>();
    public static final GtuType LEVEL0_CAR = new GtuType("NL.LEVEL0CAR", CAR);
    public static final GtuType LEVEL1_CAR = new GtuType("NL.LEVEL1CAR", CAR);
    public static final GtuType LEVEL2_CAR = new GtuType("NL.LEVEL2CAR", CAR);
    public static final GtuType LEVEL3_CAR = new GtuType("NL.LEVEL3CAR", CAR);
    
    /**
     * Constructor setting locale nl_NL.
     */
    protected VehicleConfigurations() {
        super(new Locale("nl", "NL"));
        
        // store automation level gtu types (important: index and automation level should be equal)
        automationGtuTypes.add(LEVEL0_CAR);
        automationGtuTypes.add(LEVEL1_CAR);
        automationGtuTypes.add(LEVEL2_CAR);
        automationGtuTypes.add(LEVEL3_CAR);
    }
    
    @Override
    public GtuTemplate apply(final GtuType gtuType, final StreamInterface randomStream)
    {
        GtuTemplate template = null;
        
        // set gtu template for automation levels
        if (automationGtuTypes.contains(gtuType))
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
    	map.put(LEVEL0_CAR, Color.WHITE);
    	map.put(LEVEL1_CAR, Color.YELLOW);
    	map.put(LEVEL2_CAR, Color.ORANGE);
    	map.put(LEVEL3_CAR, Color.RED);
    	// existing GTU types
//        map.put(CAR, Color.BLACK);
//        map.put(TRUCK, Color.BLACK);
//        map.put(VEHICLE, Color.GRAY);
//        map.put(PEDESTRIAN, Color.BLUE);
//        map.put(BICYCLE, Color.GREEN);
        GTU_TYPE_COLORS = new ImmutableLinkedHashMap<>(map, Immutable.WRAP);
    }

	/**
	 * Create vehicle type variables and set (non-distribution) parameters.
	 * 
	 * @param stream
	 * @return Map<String, Object>
	 * @throws ParameterException
	 */
	public VehicleConfigurationsBundle setVehicleTypes() throws ParameterException {
		
		// create parameter factory for vehicle types
		ParameterFactoryByType paramFactory = new ParameterFactoryByType();
		
		// define fixed parameters (not drawn from distribution)
		// GTU headway
		Duration[] tMinValues = {
				Duration.instantiateSI(0.6),
				Duration.instantiateSI(1.2),
				Duration.instantiateSI(0.8),
				Duration.instantiateSI(0.8)};
		Duration[] tMaxValues = {
				Duration.instantiateSI(1.2),
				Duration.instantiateSI(1.6),
				Duration.instantiateSI(1.2),
				Duration.instantiateSI(1.2)};
		
		// lookahead and lookback
		Length[] lookAheadValues = {
				Length.instantiateSI(295.0),
				Length.instantiateSI(140.0),
				Length.instantiateSI(250.0),
				Length.instantiateSI(300.0)};
		Length[] lookBackValues = {
				Length.instantiateSI(200.0),
				Length.instantiateSI(20.0),
				Length.instantiateSI(50.0),
				Length.instantiateSI(100.0)};
		
		// loop through automation GTU types to set their parameters
		for (int i = 0; i < automationGtuTypes.size(); i++) {
			
			// get current gtuType
			GtuType gtuType = automationGtuTypes.get(i);
			
			// speed adherence factor
			paramFactory.addParameter(gtuType, ParameterTypes.FSPEED, ParameterTypes.FSPEED.getDefaultValue());
			// headway
			paramFactory.addParameter(gtuType, ParameterTypes.TMAX, tMaxValues[i]);
			paramFactory.addParameter(gtuType, ParameterTypes.TMIN, tMinValues[i]);
			paramFactory.addParameter(gtuType, INITIAL_TMIN, tMinValues[i]);
			// reaction time (and simulation steps)
			paramFactory.addParameter(gtuType, ParameterTypes.TR, ParameterTypes.TR.getDefaultValue());
			paramFactory.addParameter(gtuType, ParameterTypes.DT, ParameterTypes.DT.getDefaultValue());
			// lookahead and lookback
			paramFactory.addParameter(gtuType, ParameterTypes.LOOKAHEAD, lookAheadValues[i]);
			paramFactory.addParameter(gtuType, ParameterTypes.LOOKBACK, lookBackValues[i]);
			// affected by other vehicles
			paramFactory.addParameter(gtuType, LmrsParameters.SOCIO, LmrsParameters.SOCIO.getDefaultValue()); 
			paramFactory.addParameter(gtuType, Tailgating.RHO, Tailgating.RHO.getDefaultValue());
			// perception
			paramFactory.addParameter(gtuType, Fuller.TC, Fuller.TC.getDefaultValue());
 			paramFactory.addParameter(gtuType, Fuller.TS_CRIT, Fuller.TS_CRIT.getDefaultValue());
 			paramFactory.addParameter(gtuType, Fuller.TS_MAX, Fuller.TS_MAX.getDefaultValue());
 			paramFactory.addParameter(gtuType, AdaptationSituationalAwareness.SA_MIN, AdaptationSituationalAwareness.SA_MIN.getDefaultValue());
 			paramFactory.addParameter(gtuType, AdaptationSituationalAwareness.SA, AdaptationSituationalAwareness.SA.getDefaultValue());
 			paramFactory.addParameter(gtuType, AdaptationSituationalAwareness.SA_MAX, AdaptationSituationalAwareness.SA_MAX.getDefaultValue());
 			paramFactory.addParameter(gtuType, AdaptationSituationalAwareness.TR_MAX, AdaptationSituationalAwareness.TR_MAX.getDefaultValue());
 			paramFactory.addParameter(gtuType, AdaptationHeadway.BETA_T, AdaptationHeadway.BETA_T.getDefaultValue());
 			paramFactory.addParameter(gtuType, AdaptationSpeed.BETA_V0, AdaptationSpeed.BETA_V0.getDefaultValue());
			// tasks
			paramFactory.addParameter(gtuType, TaskManagerAr.ALPHA, TaskManagerAr.ALPHA.getDefaultValue());
			paramFactory.addParameter(gtuType, TaskManagerAr.BETA, TaskManagerAr.BETA.getDefaultValue());
			paramFactory.addParameter(gtuType, CarFollowingTask.HEXP, CarFollowingTask.HEXP.getDefaultValue());
		}
		
		// return encapsulated vehicle types
        return new VehicleConfigurationsBundle(automationGtuTypes, paramFactory);
	}
	
	/** Method to draw GTU parameters from distributions.
	 * @throws ParameterException
	 */
	public Parameters drawDistributionParameters(StreamInterface stream, GtuType gtuType, Parameters gtuParameters) throws ParameterException {
		// define specific type parameters { level0, level1, level2, level3 }
		// GTU speed adherence factor
		DistTriangular[] fSpeedValues = {
				new DistTriangular(stream, 0.8, 1.0, 1.2),
				new DistTriangular(stream, 0.92, 1.0, 1.08),
				new DistTriangular(stream, 0.94, 1.0, 1.06),
				new DistTriangular(stream, 0.98, 1.0, 1.02)};
		
		// reaction time
		DistTriangular[] trValues = {
				new DistTriangular(stream, 0.8, 0.9, 1.0),
				new DistTriangular(stream, 0.1, 0.15, 0.2),
				new DistTriangular(stream, 0.1, 0.12, 0.14),
				new DistTriangular(stream, 0.05, 0.075, 0.1)};
		
		// social
		DistTriangular[] socioValues = {
				new DistTriangular(stream, 0.4, 0.5, 0.6),
				new DistTriangular(stream, 0.45, 0.55, 0.65),
				new DistTriangular(stream, 0.50, 0.60, 0.70),
				new DistTriangular(stream, 0.55, 0.65, 0.75)};
		DistTriangular[] rhoValues = {
				new DistTriangular(stream, 0.5, 0.6, 0.7),
				new DistTriangular(stream, 0.5, 0.6, 0.7),
				new DistTriangular(stream, 0.5, 0.6, 0.7),
				new DistTriangular(stream, 0.5, 0.6, 0.7)};
		
		
		// select parameter index by automation type
		int typeIndex = automationGtuTypes.indexOf(gtuType);
		if (typeIndex == -1) {
			CategoryLogger.always().error("GTU automation type is not found. Parameters cannot be adjusted according to automation types.");
		}
		else {
			// set parameters for this GTU type
			gtuParameters.setParameter(ParameterTypes.FSPEED, fSpeedValues[typeIndex].draw());
			gtuParameters.setParameter(ParameterTypes.TR, Duration.instantiateSI(trValues[typeIndex].draw()));
			gtuParameters.setParameter(LmrsParameters.SOCIO, socioValues[typeIndex].draw());
			gtuParameters.setParameter(Tailgating.RHO, rhoValues[typeIndex].draw());
		}
		
		// return new parameter values
		return gtuParameters;
	}
}



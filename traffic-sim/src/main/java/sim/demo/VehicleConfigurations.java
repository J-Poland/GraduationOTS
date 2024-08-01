package sim.demo;

import java.awt.Color;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.function.BiFunction;

import org.djunits.unit.SpeedUnit;
import org.djunits.value.vdouble.scalar.Duration;
import org.djunits.value.vdouble.scalar.Length;
import org.djunits.value.vdouble.scalar.Speed;
import org.djutils.immutablecollections.Immutable;
import org.djutils.immutablecollections.ImmutableLinkedHashMap;
import org.djutils.immutablecollections.ImmutableMap;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterTypeDouble;
import org.opentrafficsim.base.parameters.ParameterTypes;
import org.opentrafficsim.core.definitions.Defaults;
import org.opentrafficsim.core.definitions.DefaultsNl;
import org.opentrafficsim.core.distributions.ConstantGenerator;
import org.opentrafficsim.core.gtu.GtuTemplate;
import org.opentrafficsim.core.gtu.GtuType;
import org.opentrafficsim.core.parameters.ParameterFactoryByType;
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
    public static final ParameterTypeDouble INITIAL_T = new ParameterTypeDouble("INITIAL_T", "Initial desired time headway");
	    
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
        
        // store automation level gtu types
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
        map.put(CAR, Color.BLACK);
        map.put(TRUCK, Color.BLACK);
        map.put(VEHICLE, Color.GRAY);
        map.put(PEDESTRIAN, Color.BLUE);
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
	public VehicleConfigurationsBundle setVehicleTypes(StreamInterface stream) throws ParameterException {
		
		// create parameter factory for vehicle types
		ParameterFactoryByType paramFactory = new ParameterFactoryByType();
		
		// set parameters per GTU type { level0, level1, level2, level3 }
		
		// GTU speed adherence factor
		DistTriangular[] fSpeedValues = {
				new DistTriangular(stream, 0.9, 1.0, 1.1),
				new DistTriangular(stream, 0.95, 1.0, 1.05),
				new DistTriangular(stream, 0.97, 1.0, 1.03),
				new DistTriangular(stream, 0.99, 1.0, 1.01)};
		
		// GTU headway
		Duration[] tMinValues = {
				Duration.instantiateSI(0.56),
				Duration.instantiateSI(0.56),
				Duration.instantiateSI(0.56),
				Duration.instantiateSI(0.56)};
		Duration[] tMaxValues = {
				Duration.instantiateSI(1.2),
				Duration.instantiateSI(1.2),
				Duration.instantiateSI(1.2),
				Duration.instantiateSI(1.2)};
		
		// reaction time
		Duration[] trValues = {
				Duration.instantiateSI(0.8),
				Duration.instantiateSI(0.8),
				Duration.instantiateSI(0.8),
				Duration.instantiateSI(0.6)};
		
		// lookahead and lookback
		Length[] lookAheadValues = {
				Length.instantiateSI(295.0),
				Length.instantiateSI(295.0),
				Length.instantiateSI(295.0),
				Length.instantiateSI(100.0)};
		Length[] lookBackValues = {
				Length.instantiateSI(200.0),
				Length.instantiateSI(200.0),
				Length.instantiateSI(200.0),
				Length.instantiateSI(60.0)};
		
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
		
		// perception
		double tcValue = 1.0;
		double tsCritValue = 0.8;
		double tsMaxValue = 1.0;
		double saMinValue = 0.1;
		DistTriangular saValue = new DistTriangular(stream, 0.0, 0.2, 0.8);
		double saMaxValue = 1.0;
		Duration trBestValue = Duration.instantiateSI(1.5);
		double betaTValue = 1.0;
		double betaV0Value = 1.0;
		
		// tasks
		double alphaValue = 0.8;
		double betaValue = 0.6;
		Duration hexpValue = Duration.instantiateSI(1.0);
		
		// loop through automation GTU types to set their parameters
		for (int i = 0; i < automationGtuTypes.size(); i++) {
			
			// get current gtuType
			GtuType currentType = automationGtuTypes.get(i);
			
			// speed adherence factor
			paramFactory.addParameter(currentType, ParameterTypes.FSPEED, fSpeedValues[i]);
			// headway
			// (set initial desired time headway T equal to reaction time)
			paramFactory.addParameter(currentType, ParameterTypes.TMAX, tMaxValues[i]);
			paramFactory.addParameter(currentType, ParameterTypes.TMIN, tMinValues[i]);
			paramFactory.addParameter(currentType, INITIAL_T, trValues[i].si);
			// reaction time
			paramFactory.addParameter(currentType, ParameterTypes.TR, trValues[i]);
			// lookahead and lookback
			paramFactory.addParameter(currentType, ParameterTypes.LOOKAHEAD, lookAheadValues[i]);
			paramFactory.addParameter(currentType, ParameterTypes.LOOKBACK, lookBackValues[i]);
			// affected by other vehicles
			paramFactory.addParameter(currentType, LmrsParameters.SOCIO, socioValues[i]); 
			paramFactory.addParameter(currentType, Tailgating.RHO, rhoValues[i]);
			// perception
			paramFactory.addParameter(currentType, Fuller.TC, tcValue);
			paramFactory.addParameter(currentType, Fuller.TS_CRIT, tsCritValue);
			paramFactory.addParameter(currentType, Fuller.TS_MAX, tsMaxValue);
			paramFactory.addParameter(currentType, CustomAdaptationSituationalAwareness.SA_MIN, saMinValue);
			paramFactory.addParameter(currentType, CustomAdaptationSituationalAwareness.SA, saValue);
			paramFactory.addParameter(currentType, CustomAdaptationSituationalAwareness.SA_MAX, saMaxValue);
			paramFactory.addParameter(currentType, CustomAdaptationSituationalAwareness.TR_MAX, trBestValue);
			paramFactory.addParameter(currentType, CustomAdaptationHeadway.BETA_T, betaTValue);
			paramFactory.addParameter(currentType, AdaptationSpeed.BETA_V0, betaV0Value);
			// tasks
			paramFactory.addParameter(currentType, TaskManagerAr.ALPHA, alphaValue);
			paramFactory.addParameter(currentType, TaskManagerAr.BETA, betaValue);
			paramFactory.addParameter(currentType, CarFollowingTask.HEXP, hexpValue);
		}
		
		// return encapsulated vehicle types and parameters
        return new VehicleConfigurationsBundle(automationGtuTypes, paramFactory);
		
		// OLD:
//		GtuType hdvCar = HDV_CAR;
//		GtuType avCar = AV_CAR;
//		
//		// set parameters for HDV
//		// speed limit adherence
//		paramFactory.addParameter(hdvCar, ParameterTypes.FSPEED, new DistTriangular(stream, 1, 13.0 / 130));
//		// headway
//		paramFactory.addParameter(hdvCar, ParameterTypes.TMAX, Duration.instantiateSI(1.2));
//		paramFactory.addParameter(hdvCar, ParameterTypes.TMIN, Duration.instantiateSI(0.56));
//		// reaction time
//		paramFactory.addParameter(hdvCar, ParameterTypes.TR, Duration.instantiateSI(1.5));
//		// lookahead and lookback
//		paramFactory.addParameter(hdvCar, ParameterTypes.LOOKAHEAD, Length.instantiateSI(295.0));
//		paramFactory.addParameter(hdvCar, ParameterTypes.LOOKBACK, Length.instantiateSI(200.0));
//		// affected by other vehicles
//		paramFactory.addParameter(hdvCar, LmrsParameters.SOCIO, new DistTriangular(stream, 0.5, 0.1)); 
//		paramFactory.addParameter(hdvCar, Tailgating.RHO, 0.6);
//		// perception
//		paramFactory.addParameter(hdvCar, Fuller.TC, 1.0);
//		paramFactory.addParameter(hdvCar, Fuller.TS_CRIT, 0.8);
//		paramFactory.addParameter(hdvCar, Fuller.TS_MAX, 1.0);
//		paramFactory.addParameter(hdvCar, AdaptationSituationalAwareness.SA_MIN, 0.1);
//		paramFactory.addParameter(hdvCar, AdaptationSituationalAwareness.SA, 0.5);
//		paramFactory.addParameter(hdvCar, AdaptationSituationalAwareness.SA_MAX, 1.0);
//		paramFactory.addParameter(hdvCar, AdaptationSituationalAwareness.TR_MAX, Duration.instantiateSI(1.5));
//		paramFactory.addParameter(hdvCar, AdaptationHeadway.BETA_T, 1.0);
//		paramFactory.addParameter(hdvCar, AdaptationSpeed.BETA_V0, 1.0);
//		// tasks
//		paramFactory.addParameter(hdvCar, TaskManagerAr.ALPHA, 0.8);
//		paramFactory.addParameter(hdvCar, TaskManagerAr.BETA, 0.6);
//		paramFactory.addParameter(hdvCar, CarFollowingTask.HEXP, Duration.instantiateSI(1.0));
//		
//		// set parameters for AV
//		// speed limit adherence
//		paramFactory.addParameter(avCar, ParameterTypes.FSPEED, 1.05);
//		// headway
//		paramFactory.addParameter(avCar, ParameterTypes.TMAX, Duration.instantiateSI(2.0));
//		paramFactory.addParameter(avCar, ParameterTypes.TMIN, Duration.instantiateSI(0.4));
//		// reaction time
//		paramFactory.addParameter(avCar, ParameterTypes.TR, Duration.instantiateSI(0.5));
//		// lookahead and lookback
//		paramFactory.addParameter(avCar, ParameterTypes.LOOKAHEAD, Length.instantiateSI(100.0));
//		paramFactory.addParameter(avCar, ParameterTypes.LOOKBACK, Length.instantiateSI(60.0));
//		// affected by other vehicles
//		paramFactory.addParameter(avCar, LmrsParameters.SOCIO, 0.8);
//		paramFactory.addParameter(avCar, Tailgating.RHO, 0.0);
//		// perception
//		paramFactory.addParameter(avCar, Fuller.TC, 0.8);
//		paramFactory.addParameter(avCar, Fuller.TS_CRIT, 0.8);
//		paramFactory.addParameter(avCar, Fuller.TS_MAX, 1.0);
//		paramFactory.addParameter(avCar, AdaptationSituationalAwareness.SA_MIN, 0.1);
//		paramFactory.addParameter(avCar, AdaptationSituationalAwareness.SA, 0.5);
//		paramFactory.addParameter(avCar, AdaptationSituationalAwareness.SA_MAX, 1.0);
//		paramFactory.addParameter(avCar, AdaptationSituationalAwareness.TR_MAX, Duration.instantiateSI(1.5));
//		paramFactory.addParameter(avCar, AdaptationHeadway.BETA_T, 1.0);
//		paramFactory.addParameter(avCar, AdaptationSpeed.BETA_V0, 1.0);
//		// tasks
//		paramFactory.addParameter(avCar, TaskManagerAr.ALPHA, 0.8);
//		paramFactory.addParameter(avCar, TaskManagerAr.BETA, 0.6);
//		paramFactory.addParameter(avCar, CarFollowingTask.HEXP, Duration.instantiateSI(1.0));
	}
}



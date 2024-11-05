package sim.demo.vehicleconfigurations;

import java.awt.Color;
import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;
import java.util.function.BiFunction;

import org.djunits.unit.SpeedUnit;
import org.djunits.value.vdouble.scalar.Acceleration;
import org.djunits.value.vdouble.scalar.Duration;
import org.djunits.value.vdouble.scalar.Length;
import org.djunits.value.vdouble.scalar.Speed;
import org.djutils.immutablecollections.Immutable;
import org.djutils.immutablecollections.ImmutableLinkedHashMap;
import org.djutils.immutablecollections.ImmutableMap;
import org.djutils.logger.CategoryLogger;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterTypeBoolean;
import org.opentrafficsim.base.parameters.ParameterTypeDouble;
import org.opentrafficsim.base.parameters.ParameterTypeDuration;
import org.opentrafficsim.base.parameters.ParameterTypeString;
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
import sim.demo.mental.TaskCarFollowing;
import sim.demo.mental.TaskManagerAr;

public class VehicleAutomationConfigurations extends Defaults implements BiFunction<GtuType, StreamInterface, GtuTemplate> {
	
	/** New parameters. */
	public static final ParameterTypeString AUTOMATION_LEVEL = new ParameterTypeString("AUTOMATION_LEVEL", "Automation level of vehicle.");
    public static final ParameterTypeString ORIGIN = new ParameterTypeString("ORIGIN", "Node on which the GTU was generated.");
    public static final ParameterTypeDuration INITIAL_TMIN = new ParameterTypeDuration("INITIAL_TMIN", "Initial minimal headway time.");
    public static final ParameterTypeDuration MAX_TR = new ParameterTypeDuration("MAX_TR", "Custom parameter for maximum reaction time.");
    public static final ParameterTypeDouble SOCIO_CF = new ParameterTypeDouble("SOCIO_CF", "Social parameter for car-following.");
    public static final ParameterTypeDouble CF_TASK_DEMAND = new ParameterTypeDouble("CF_TASK_DEMAND", "Car-following cognitive task demand.");
    public static final ParameterTypeDouble LC_TASK_DEMAND = new ParameterTypeDouble("LC_TASK_DEMAND", "Lane-chaning cognitive task demand.");
    public static final ParameterTypeBoolean SECONDARY_TASK_DISTRACTED = new ParameterTypeBoolean("SECONDARY_TASK_DISTRACTED", "Secondary task demand.");
    public static final ParameterTypeBoolean ROAD_SIDE_DISTRACTED = new ParameterTypeBoolean("ROAD_SIDE_DISTRACTED", "Road side distraction demand.");
    public static final ParameterTypeBoolean IN_BETWEEN_LEVEL3 = new ParameterTypeBoolean("IN_BETWEEN_LEVEL3", "Is this GTU surrounded by level 3 vehicles.");
    public static final ParameterTypeDuration TMIN_LEVEL3 = new ParameterTypeDuration("TMIN_LEVEL3", "Minimum time headway of level 3 vehicles.");
	
	/** Vehicle types. */
	public static final GtuType LEVEL0_CAR = new GtuType("NL.LEVEL0CAR", DefaultsNl.CAR);
    public static final GtuType LEVEL1_CAR = new GtuType("NL.LEVEL1CAR", DefaultsNl.CAR);
    public static final GtuType LEVEL2_CAR = new GtuType("NL.LEVEL2CAR", DefaultsNl.CAR);
    public static final GtuType LEVEL3_CAR = new GtuType("NL.LEVEL3CAR", DefaultsNl.CAR);
    
	/** Automation levels map. */
	Map<String, GtuType> vehicleAutomationLevels = new LinkedHashMap<>();
    
    /** Drawing colours for GTU type animations. */
    public static final ImmutableMap<String, Color> GTU_TYPE_COLORS;
	static {
    	LinkedHashMap<String, Color> map = new LinkedHashMap<>();
		map.put("LEVEL0", Color.WHITE);
    	map.put("LEVEL1", Color.YELLOW);
    	map.put("LEVEL2", Color.ORANGE);
    	map.put("LEVEL3", Color.RED);
    	GTU_TYPE_COLORS = new ImmutableLinkedHashMap<>(map, Immutable.WRAP);
    }
    
    /** Create parameter factory for GTU types. */
 	ParameterFactoryByType parameterFactoryByType = new ParameterFactoryByType();
 	
 	/** Maximum and minimum allowed acceleration. */
	double maxAcceleration;
	double minAcceleration;
	double sensitivityAnalysisValue;
 	
    
    /**
     * Constructor.
     * @throws ParameterException 
     */
	public VehicleAutomationConfigurations(double maxAcceleration, double minAcceleration, double sensitivityAnalysisValue) throws ParameterException
	{
		super(new Locale("nl", "NL"));
		
		this.maxAcceleration = maxAcceleration;
		this.minAcceleration = minAcceleration;
		this.sensitivityAnalysisValue = sensitivityAnalysisValue;
		
		// create GTU types
		createVehicleTypes();
	
		// set initial parameters
		parameterFactoryByType = setVehicleParameters();
	}
	
	@Override
    public GtuTemplate apply(final GtuType gtuType, final StreamInterface randomStream)
    {
        GtuTemplate template = null;
        
        // set gtu template for automation levels
        if (vehicleAutomationLevels.values().contains(gtuType))
        {
            // from "Maatgevende normen in de Nederlandse richtlijnen voor wegontwerp", R-2014-38, SWOV
            template = new GtuTemplate(gtuType, new ConstantGenerator<>(Length.instantiateSI(4.19)),
                    new ConstantGenerator<>(Length.instantiateSI(1.7)),
                    new ConstantGenerator<>(new Speed(180, SpeedUnit.KM_PER_HOUR)),
            		new ConstantGenerator<>(Acceleration.instantiateSI(maxAcceleration)),
                    new ConstantGenerator<>(Acceleration.instantiateSI(minAcceleration)));
        }
        return template;
    };
	
	
	/**
	 * Getter for vehicle automation levels GTU types.
	 * @return vehicleAutomationLevels.values();
	 */
	public Map<String, GtuType> getVehicleAutomationLevelsMap() {
		return vehicleAutomationLevels;
	}
	
	/**
	 * Getter for ParameterFactoryByType for vehicle automation levels.
	 * @return parameterFactoryByType;
	 */
	public ParameterFactoryByType getVehicleAutomationParameterFactory() {
		return parameterFactoryByType;
	}
	
	
	/**
	 * Create vehicle types.
	 */
	private void createVehicleTypes() {
	    vehicleAutomationLevels.put("LEVEL0", LEVEL0_CAR);
	    vehicleAutomationLevels.put("LEVEL1", LEVEL1_CAR);
	    vehicleAutomationLevels.put("LEVEL2", LEVEL2_CAR);
	    vehicleAutomationLevels.put("LEVEL3", LEVEL3_CAR);
	}
	
	/**
	 * Set initial vehicle parameters.
	 * @throws ParameterException 
	 */
	private ParameterFactoryByType setVehicleParameters() throws ParameterException {
		// temporary parameter factory
		ParameterFactoryByType paramFactory = new ParameterFactoryByType();
		
		// define fixed parameters (not drawn from distribution)
		
		// GTU acceleration
		Acceleration[] maxAccValues = {
				Acceleration.instantiateSI(1.25),
				Acceleration.instantiateSI(1.17),
				Acceleration.instantiateSI(1.17),
				Acceleration.instantiateSI(1.12)};
		Acceleration[] maxComfDecValues = {
				Acceleration.instantiateSI(2.09),
				Acceleration.instantiateSI(1.95),
				Acceleration.instantiateSI(1.95),
				Acceleration.instantiateSI(1.87)};
		Acceleration[] maxCritDecValues = {
				Acceleration.instantiateSI(3.5),
				Acceleration.instantiateSI(3.5),
				Acceleration.instantiateSI(3.5),
				Acceleration.instantiateSI(3.5)};
		Acceleration[] maxAdjDecValues = {
				Acceleration.instantiateSI(0.5),
				Acceleration.instantiateSI(0.5),
				Acceleration.instantiateSI(0.5),
				Acceleration.instantiateSI(0.5)};
		
		// GTU headway
		Duration[] tMinValues = {
				Duration.instantiateSI(0.58),
				Duration.instantiateSI(0.8),
				Duration.instantiateSI(0.8),
				Duration.instantiateSI(0.522)};
		Duration[] tMaxValues = {
				Duration.instantiateSI(1.84),
				Duration.instantiateSI(1.5),
				Duration.instantiateSI(1.5),
				Duration.instantiateSI(1.104)};
//		Duration[] tMaxValues = {
//				Duration.instantiateSI(sensitivityAnalysisValue),
//				Duration.instantiateSI(1.5),
//				Duration.instantiateSI(1.5),
//				Duration.instantiateSI(1.104)};
		
		// GTU reaction time
		Duration[] reactionTimeMaxValues = {
				Duration.instantiateSI(2.0),
				Duration.instantiateSI(0.0),
				Duration.instantiateSI(0.0),
				Duration.instantiateSI(0.0)};
		
		// lookahead and lookback
		Length[] lookAheadValues = {
				Length.instantiateSI(295.0),
				Length.instantiateSI(140.0),
				Length.instantiateSI(250.0),
				Length.instantiateSI(300.0)};
		Length[] lookBackValues = {
				Length.instantiateSI(200.0),
				Length.instantiateSI(200.0),
				Length.instantiateSI(200.0),
				Length.instantiateSI(200.0)};
		
		
		// loop through automation GTU types to set their parameters
		int i = 0;
		for (Map.Entry<String, GtuType> entry : vehicleAutomationLevels.entrySet()) {
		    String automationLevel = entry.getKey();
		    GtuType gtuType = entry.getValue();
			
			// automation level
			paramFactory.addParameter(gtuType, AUTOMATION_LEVEL, automationLevel);
			// speed adherence factor
			paramFactory.addParameter(gtuType, ParameterTypes.FSPEED, ParameterTypes.FSPEED.getDefaultValue());
			// acceleration
			paramFactory.addParameter(gtuType, ParameterTypes.A, maxAccValues[i]);
			paramFactory.addParameter(gtuType, ParameterTypes.BCRIT, maxCritDecValues[i]);
			paramFactory.addParameter(gtuType, ParameterTypes.B, maxComfDecValues[i]);
			paramFactory.addParameter(gtuType, ParameterTypes.B0, maxAdjDecValues[i]);
			// headway
			paramFactory.addParameter(gtuType, ParameterTypes.TMAX, tMaxValues[i]);
			paramFactory.addParameter(gtuType, ParameterTypes.TMIN, tMinValues[i]);
			paramFactory.addParameter(gtuType, INITIAL_TMIN, tMinValues[i]);
			// reaction time (and simulation steps)
			paramFactory.addParameter(gtuType, ParameterTypes.TR, ParameterTypes.TR.getDefaultValue());
			paramFactory.addParameter(gtuType, MAX_TR, reactionTimeMaxValues[i]);
			paramFactory.addParameter(gtuType, ParameterTypes.DT, ParameterTypes.DT.getDefaultValue());
			// LMRS
			paramFactory.addParameter(gtuType, LmrsParameters.VGAIN, new Speed(69.6, SpeedUnit.KM_PER_HOUR));
			// lookahead and lookback
			paramFactory.addParameter(gtuType, ParameterTypes.LOOKAHEAD, lookAheadValues[i]);
			paramFactory.addParameter(gtuType, ParameterTypes.LOOKBACK, lookBackValues[i]);
			// affected by other vehicles
			paramFactory.addParameter(gtuType, SOCIO_CF, 0.5);
			paramFactory.addParameter(gtuType, LmrsParameters.SOCIO, 0.5); 
			paramFactory.addParameter(gtuType, Tailgating.RHO, 0.0);
			// perception
            paramFactory.addParameter(ParameterTypes.PERCEPTION, ParameterTypes.PERCEPTION.getDefaultValue());
			paramFactory.addParameter(gtuType, Fuller.TC, Fuller.TC.getDefaultValue());
 			paramFactory.addParameter(gtuType, Fuller.TS_CRIT, Fuller.TS_CRIT.getDefaultValue());
 			paramFactory.addParameter(gtuType, Fuller.TS_MAX, Fuller.TS_MAX.getDefaultValue());
 			paramFactory.addParameter(gtuType, AdaptationSituationalAwareness.SA_MIN, AdaptationSituationalAwareness.SA_MIN.getDefaultValue());
 			paramFactory.addParameter(gtuType, AdaptationSituationalAwareness.SA, AdaptationSituationalAwareness.SA.getDefaultValue());
 			paramFactory.addParameter(gtuType, AdaptationSituationalAwareness.SA_MAX, AdaptationSituationalAwareness.SA_MAX.getDefaultValue());
 			paramFactory.addParameter(gtuType, AdaptationSituationalAwareness.TR_MAX, AdaptationSituationalAwareness.TR_MAX.getDefaultValue());
 			paramFactory.addParameter(gtuType, AdaptationHeadway.BETA_T, AdaptationHeadway.BETA_T.getDefaultValue());
 			paramFactory.addParameter(gtuType, AdaptationSpeed.BETA_V0, AdaptationSpeed.BETA_V0.getDefaultValue());
 			paramFactory.addParameter(gtuType, Estimation.OVER_EST, 1.0);
			// tasks
			paramFactory.addParameter(gtuType, TaskManagerAr.ALPHA, TaskManagerAr.ALPHA.getDefaultValue());
			paramFactory.addParameter(gtuType, TaskManagerAr.BETA, TaskManagerAr.BETA.getDefaultValue());
			paramFactory.addParameter(gtuType, TaskCarFollowing.HEXP, TaskCarFollowing.HEXP.getDefaultValue());
			// car-following task demand
			paramFactory.addParameter(gtuType, CF_TASK_DEMAND, 0.0);
			// lane-changing task demand
			paramFactory.addParameter(gtuType, LC_TASK_DEMAND, 0.0);
			// in-vehicle distraction demand
			paramFactory.addParameter(gtuType, SECONDARY_TASK_DISTRACTED, false);
			// road side distraction demand
			paramFactory.addParameter(gtuType, ROAD_SIDE_DISTRACTED, false);
			// parameters to enable level 0 vehicles to adapt headway settings to level 3 vehicles
			paramFactory.addParameter(gtuType, IN_BETWEEN_LEVEL3, false);
			paramFactory.addParameter(gtuType, TMIN_LEVEL3, tMinValues[3]);
			
			// increase index
			i += 1;
		}
		
		// return GTU parameter factory by type
		return paramFactory;
	}
	
	
	/** Method to draw GTU parameters from distributions.
	 * @throws ParameterException
	 */
	public Parameters drawDistributionParameters(StreamInterface stream, String gtuType, Parameters gtuParameters) throws ParameterException {
				
		// define specific type parameters { level0, level1, level2, level3 }
		// GTU speed adherence factor
		DistTriangular[] fSpeedValues = {
				new DistTriangular(stream, 0.8, 1.0, 1.2),
				new DistTriangular(stream, 0.9, 1.0, 1.1),
				new DistTriangular(stream, 0.9, 1.0, 1.1),
				new DistTriangular(stream, 0.95, 1.0, 1.05)};
		
		// social (rho values are set dynamically in tailgating class)
		DistTriangular humanSocioCarFollowingValues = new DistTriangular(stream, 0, 0.5, 1.0);
		DistTriangular humanSocioLaneChangingValues = new DistTriangular(stream, 0, 0.5, 1.0);
		double automatedSocioCarFollowingValue = 0.0;
		double automatedSocioLaneChangingValue = 0.0;
		
		// Task capacity for human drivers
		DistTriangular humanTaskCapacityValues = new DistTriangular(stream, 0.9, 1.0, 1.1);
				
		
		
		// determine automation level index by GTU type string
		int typeIndex = gtuType.contains("LEVEL0") ? 0 : gtuType.contains("LEVEL1") ? 1 : gtuType.contains("LEVEL2") ? 2 : gtuType.contains("LEVEL3") ? 3 : -1;
		
		// select parameter index by automation type
		if (typeIndex == -1) {
			CategoryLogger.always().error("GTU automation type is not found. Parameters cannot be adjusted to match automation types.");
		}
		else if (typeIndex == 0) {
			// set parameters for this GTU type
			gtuParameters.setParameter(ParameterTypes.FSPEED, fSpeedValues[typeIndex].draw());
			gtuParameters.setParameter(SOCIO_CF, humanSocioCarFollowingValues.draw());
			gtuParameters.setParameter(LmrsParameters.SOCIO, humanSocioLaneChangingValues.draw());
			gtuParameters.setParameter(Fuller.TC, humanTaskCapacityValues.draw());
		}
		else if (typeIndex == 1) {
			// set parameters for this GTU type
			gtuParameters.setParameter(ParameterTypes.FSPEED, fSpeedValues[typeIndex].draw());
			gtuParameters.setParameter(SOCIO_CF, automatedSocioCarFollowingValue);
			gtuParameters.setParameter(LmrsParameters.SOCIO, humanSocioLaneChangingValues.draw());
		}
		else if (typeIndex == 2) {
			// set parameters for this GTU type
			gtuParameters.setParameter(ParameterTypes.FSPEED, fSpeedValues[typeIndex].draw());
			gtuParameters.setParameter(SOCIO_CF, automatedSocioCarFollowingValue);
			gtuParameters.setParameter(LmrsParameters.SOCIO, automatedSocioLaneChangingValue);
		}
		else if (typeIndex == 3) {
			// set parameters for this GTU type
			gtuParameters.setParameter(ParameterTypes.FSPEED, fSpeedValues[typeIndex].draw());
			gtuParameters.setParameter(SOCIO_CF, automatedSocioCarFollowingValue);
			gtuParameters.setParameter(LmrsParameters.SOCIO, automatedSocioLaneChangingValue);
		}
		
		// return new parameter values
		return gtuParameters;
	}
}



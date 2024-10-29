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
import org.opentrafficsim.core.units.distributions.ContinuousDistSpeed;
import org.opentrafficsim.road.gtu.lane.perception.mental.AdaptationHeadway;
import org.opentrafficsim.road.gtu.lane.perception.mental.AdaptationSituationalAwareness;
import org.opentrafficsim.road.gtu.lane.perception.mental.AdaptationSpeed;
import org.opentrafficsim.road.gtu.lane.perception.mental.Fuller;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.LmrsParameters;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.Tailgating;

import nl.tudelft.simulation.jstats.distributions.DistLogNormal;
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
    public static final ParameterTypeDuration MIN_TR = new ParameterTypeDuration("MIN_TR", "Custom parameter for minimum reaction time.");
    public static final ParameterTypeDouble CF_TASK_DEMAND = new ParameterTypeDouble("CF_TASK_DEMAND", "Car-following cognitive task demand.");
    public static final ParameterTypeDouble LC_TASK_DEMAND = new ParameterTypeDouble("LC_TASK_DEMAND", "Lane-chaning cognitive task demand.");
    public static final ParameterTypeBoolean SECONDARY_TASK_DISTRACTED = new ParameterTypeBoolean("SECONDARY_TASK_DISTRACTED", "Secondary task demand.");
    public static final ParameterTypeBoolean ROAD_SIDE_DISTRACTED = new ParameterTypeBoolean("ROAD_SIDE_DISTRACTED", "Road side distraction demand.");
	
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
 	
    
    /**
     * Constructor.
     * @throws ParameterException 
     */
	public VehicleAutomationConfigurations(double maxAcceleration, double minAcceleration) throws ParameterException
	{
		super(new Locale("nl", "NL"));
		
		this.maxAcceleration = maxAcceleration;
		this.minAcceleration = minAcceleration;
		
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
				Acceleration.instantiateSI(1.17)};
		Acceleration[] maxComfDecValues = {
				Acceleration.instantiateSI(2.09),
				Acceleration.instantiateSI(1.95),
				Acceleration.instantiateSI(1.95),
				Acceleration.instantiateSI(1.95)};
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
				Duration.instantiateSI(0.6),
				Duration.instantiateSI(1.2),
				Duration.instantiateSI(0.8),
				Duration.instantiateSI(0.8)};
		Duration[] tMaxValues = {
				Duration.instantiateSI(1.2),
				Duration.instantiateSI(1.6),
				Duration.instantiateSI(1.2),
				Duration.instantiateSI(1.2)};
		
		// GTU reaction time
		//TODO: COllisions???
		Duration[] reactionTimeMaxValues = {
				Duration.instantiateSI(0.5),
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
				Length.instantiateSI(20.0),
				Length.instantiateSI(50.0),
				Length.instantiateSI(100.0)};
		
		
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
			paramFactory.addParameter(gtuType, ParameterTypes.B, maxComfDecValues[i]);
			paramFactory.addParameter(gtuType, ParameterTypes.BCRIT, maxCritDecValues[i]);
			paramFactory.addParameter(gtuType, ParameterTypes.B0, maxAdjDecValues[i]);
			// headway
			paramFactory.addParameter(gtuType, ParameterTypes.TMAX, tMaxValues[i]);
			paramFactory.addParameter(gtuType, ParameterTypes.TMIN, tMinValues[i]);
			paramFactory.addParameter(gtuType, INITIAL_TMIN, tMinValues[i]);
			// reaction time (and simulation steps)
			paramFactory.addParameter(gtuType, ParameterTypes.TR, ParameterTypes.TR.getDefaultValue());
			paramFactory.addParameter(gtuType, MAX_TR, reactionTimeMaxValues[i]);
			paramFactory.addParameter(gtuType, MIN_TR, Duration.instantiateSI(0.0));
			paramFactory.addParameter(gtuType, ParameterTypes.DT, ParameterTypes.DT.getDefaultValue());
			// lookahead and lookback
			paramFactory.addParameter(gtuType, ParameterTypes.LOOKAHEAD, lookAheadValues[i]);
			paramFactory.addParameter(gtuType, ParameterTypes.LOOKBACK, lookBackValues[i]);
			// affected by other vehicles
			paramFactory.addParameter(gtuType, LmrsParameters.SOCIO, LmrsParameters.SOCIO.getDefaultValue()); 
			paramFactory.addParameter(gtuType, Tailgating.RHO, Tailgating.RHO.getDefaultValue());
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
				new DistTriangular(stream, 0.92, 1.0, 1.08),
				new DistTriangular(stream, 0.94, 1.0, 1.06),
				new DistTriangular(stream, 0.98, 1.0, 1.02)};
		
		// social (rho values are set dynamically in tailgating class)
		DistTriangular humanSocioValues = new DistTriangular(stream, 0, 0.5, 1.0);
		DistTriangular automatedSocioValues = new DistTriangular(stream, 0.0, 0.1, 0.2);
		
		// ???
		/*
         * In case of social interactions we 1) introduce the RHO status variable of social pressure, value only
         * important at vehicle generation as the model sets this, 2) increase the TMAX value from a normal 1.2s to
         * 1.6s, as the tailgating phenomenon will reduce this, leading to an average at around 1.2s, but now dynamic
         * depending on social pressure, 3) we introduce a distributed SOCIO parameter to represent sensitivity to
         * social pressure, 4) we lower VGAIN to increase ego-speed sensitivity, and distribute it (we now have a
         * population of drivers distributed on a plain of socio and ego speed sensitivity, and 5) something similar but
         * simple for trucks.
         */
		DistTriangular socioDist = new DistTriangular(stream, 0.0, 0.25, 1.0);
		ContinuousDistSpeed continousSpeedDist = new ContinuousDistSpeed(new DistLogNormal(stream, Math.log(25.0) + 0.4 * 0.4, 0.4), SpeedUnit.KM_PER_HOUR);
				
		// determine automation level index by GTU type string
		int typeIndex = gtuType.contains("LEVEL0") ? 0 : gtuType.contains("LEVEL1") ? 1 : gtuType.contains("LEVEL2") ? 2 : gtuType.contains("LEVEL3") ? 3 : -1;
		
		// select parameter index by automation type
		if (typeIndex == -1) {
			CategoryLogger.always().error("GTU automation type is not found. Parameters cannot be adjusted to match automation types.");
		}
		else {
			// set parameters for this GTU type
			gtuParameters.setParameter(ParameterTypes.FSPEED, fSpeedValues[typeIndex].draw());
			
			// ???
			gtuParameters.setParameter(LmrsParameters.SOCIO, socioDist.draw());
			gtuParameters.setParameter(LmrsParameters.VGAIN, // mu =~ 3.3789, sigma = 0.4, mode = 25.0
									   continousSpeedDist.draw());
			
			// set human minimum specific values
			if (gtuParameters.getParameter(AUTOMATION_LEVEL).contains("LEVEL0")) {
				gtuParameters.setParameter(LmrsParameters.SOCIO, humanSocioValues.draw());
			}
			
			// set automated specific values
			else {
				// set automated reaction times equal to their maximum reaction time, SA does not influence this reaction time
				gtuParameters.setParameter(LmrsParameters.SOCIO, automatedSocioValues.draw());
			}
			
		}
		
		// return new parameter values
		return gtuParameters;
	}
}



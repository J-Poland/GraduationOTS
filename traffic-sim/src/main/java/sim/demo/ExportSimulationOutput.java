package sim.demo;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import de.siegmar.fastcsv.writer.CsvWriter;


/**
 * Class to handle data export.
 */
public class ExportSimulationOutput
{
	/** Sequence file writer */
	CsvWriter sequenceWriter = null;
	
	/**
	 * Constructor for ExportSimulationOutput class.
	 */
	public ExportSimulationOutput()
	{
		
	}
	
	
	/**
	 * Function to check whether a file path exists
	 * @param filePath
	 */
	private void checkDirectory(String filePath)
	{
		File file = new File(filePath);
        File parentDir = file.getParentFile();
        if (parentDir != null && !parentDir.exists()) {
            if (!parentDir.mkdirs()) {
                System.err.println("Failed to create directories: " + parentDir.getAbsolutePath());
                return;
            }
        }
	}
	
	/**
	 * Function to create a sequence CDV writer.
	 * @param sequenceFilePath
	 */
	public void initializeSequenceFile(String sequenceFilePath) {
		// ensure that directory exists
     	checkDirectory(sequenceFilePath);
     	
        // write output data to given CSV file
		try {
			FileWriter fileWriter = new FileWriter(sequenceFilePath);
			// create sequence CSV writer
	        sequenceWriter = CsvWriter.builder().build(fileWriter);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	/**
	 * Function to add headers to the sequence data CSV
	 * @param headers
	 */
	public void writeSequenceDataHeaders(ArrayList<String> headers) {
		sequenceWriter.writeRecord(headers);
	}
	
	/**
	 * Function to add new data row to the sequence data CSV
	 * @param dataRow
	 */
	public void writeSequenceDataRow(Map<String, String> dataRow) {
		// list for all new values
		List<String> row = new ArrayList<>();
		
		// loop through all variables
		for (String variable : dataRow.keySet()) {
			// get value
			String value = dataRow.get(variable);
			// add to row values
			row.add(value);
		}
		
		// write this new row to the file (it is automatically flushed)
		sequenceWriter.writeRecord(row);
	}
	
	/**
	 * Function to export single (scalar) simulation data to CSV file.
	 * 
	 * @param inputValuesMap
	 * @param filePath
	 */
	public void exportInputToCsv(Map<String, String> inputValuesMap, String inputFilePath) {
		
		// only continue when input data is available
		if (!inputValuesMap.isEmpty()) {
			
			// user feedback
			System.out.println("\nInput parameters exporting to CSV file at \n" + "'" + inputFilePath + "'");
			
			// ensure that directories exist
			checkDirectory(inputFilePath);
			
			// write output data to given CSV file
	        try (FileWriter fileWriter = new FileWriter(inputFilePath)) {
	        	
	        	// create CSV writer
	            CsvWriter csvWriter = CsvWriter.builder().build(fileWriter);
	            		
	            // get headers and values
	            ArrayList<String> headerList = new ArrayList<String>();
	            List<String> valueList = new ArrayList<String>();
	            for (String key : inputValuesMap.keySet()) {
	                headerList.add(key);
	                valueList.add(inputValuesMap.get(key));
	            }
	            
	            // write headers in first row
	            csvWriter.writeRecord(headerList);
	            
	            // write values in second row
	            csvWriter.writeRecord(valueList);
	            
	            // user feedback
	            System.out.println("Input parameters CSV file created successfully.\n");
	        
	        } catch (IOException e) {
	            e.printStackTrace();
	        }
		}
	}
	
	/**
	 * Function to export single (scalar) simulation data to CSV file.
	 * 
	 * @param filePath
	 */
	public void exportSingleToCsv(Map<String, Object> singleOutputMap, String singleFilePath) {
		
		// only continue when single output data is available
		if (!singleOutputMap.isEmpty()) {
			
			// user feedback
			System.out.println("\nSingle output data exporting to CSV file at \n" + "'" + singleFilePath + "'");
			
			// ensure that directories exist
			checkDirectory(singleFilePath);
			
			// write output data to given CSV file
	        try (FileWriter fileWriter = new FileWriter(singleFilePath)) {
	        	
	        	// create CSV writer
	            CsvWriter csvWriter = CsvWriter.builder().build(fileWriter);
	            		
	            // get headers and values for singletOutputMap
	            ArrayList<String> headerList = new ArrayList<String>();
	            List<String> valueList = new ArrayList<String>();
	            for (String key : singleOutputMap.keySet()) {
	                headerList.add(key);
	                valueList.add(singleOutputMap.get(key).toString());
	            }
	            
	            // write headers in first row
	            csvWriter.writeRecord(headerList);
	            
	            // write values in second row
	            csvWriter.writeRecord(valueList);
	            
	            // user feedback
	            System.out.println("Single CSV file created successfully.");
	        
	        } catch (IOException e) {
	            e.printStackTrace();
	        }
		}
	}
	
//	/**
//	 * Function to export sequence (time series) simulation data to CSV file.
//	 * 
//	 * @param filePath
//	 */
//	public void exportSequenceToCsv(Map<String, GtuSequenceData> sequenceOutputMap, String sequenceFilePath) {
//	    // only continue when single output data is available
//	    if (!sequenceOutputMap.isEmpty()) {
//	        
//	        // user feedback
//	        System.out.println("\nSequence output data exporting to CSV file at \n" + "'" + sequenceFilePath + "'");
//	        
//	        // ensure that directories exist
//	     	checkDirectory(sequenceFilePath);
//	        
//	        // collect all unique variable names
//	        Set<String> variableNames = new HashSet<>();
//	        for (GtuSequenceData gtuData : sequenceOutputMap.values()) {
//	            for (DataPoint dataPoint : gtuData.getDataPoints()) {
//	                variableNames.addAll(dataPoint.getValues().keySet());
//	            }
//	        }
//	        
//	        // write output data to given CSV file
//	        try (FileWriter fileWriter = new FileWriter(sequenceFilePath)) {
//	            
//	            // create CSV writer
//	            CsvWriter csvWriter = CsvWriter.builder().build(fileWriter);
//	            
//	            // initialise headers list
//	            List<String> headers = new ArrayList<>();
//	            headers.add("gtu_id");
//	            headers.add("simulation_time");
//	            headers.addAll(variableNames);
//	            
//	            // write headers to CSV
//	            csvWriter.writeRecord(headers);
//	            
//	            // iterate over outer GtuSequenceData (GTU IDs)
//	            for (Map.Entry<String, GtuSequenceData> outerEntry : sequenceOutputMap.entrySet()) {
//	                String gtuId = outerEntry.getKey();
//	                GtuSequenceData gtuData = outerEntry.getValue();
//	                
//	                // iterate over data points in GtuData
//	                for (DataPoint dataPoint : gtuData.getDataPoints()) {
//	                    double simulationTime = dataPoint.getSimulationTime();
//	                    Map<String, Object> values = dataPoint.getValues();
//	                    
//	                    // create a row for each data point
//	                    List<String> row = new ArrayList<>();
//	                    row.add(gtuId);
//	                    row.add(String.valueOf(simulationTime));
//	                    
//	                    // add variable values to the row
//	                    for (String variable : variableNames) {
//	                        Object value = values.get(variable);
//	                        row.add(value != null ? value.toString() : "");
//	                    }
//	                    
//	                    // write the row to CSV
//	                    csvWriter.writeRecord(row);
//	                }
//	            }
//	            
//	            // user feedback
//	            System.out.println("Sequence CSV file created successfully.");
//	        
//	        } catch (IOException exception) {
//	            exception.printStackTrace();
//	        }
//	    }
//	}
	
	public void exportIntermediateMeanValuesToCsv(Map<String, ArrayList<Double>> meanMap, String intermediateMeanValuesFilePath) {
    	
		// only continue when output data is available
		if (!meanMap.isEmpty()) {
			
			// user feedback
			System.out.println("\nIntermediate output data exporting to CSV file at \n" + "'" + intermediateMeanValuesFilePath + "'"); 
			
			// ensure that directories exist
			checkDirectory(intermediateMeanValuesFilePath);
			
			// write output data to given CSV file
	        try (FileWriter fileWriter = new FileWriter(intermediateMeanValuesFilePath)) {
	        	
	        	// create CSV writer
	            CsvWriter csvWriter = CsvWriter.builder().build(fileWriter);
	    	
	            // determine max value count
	            int maxCount = 0;
	            for (ArrayList<Double> values : meanMap.values()) {
	            	if (values.size() > maxCount) {
	            		maxCount = values.size();
	            	}
	            }
	            
	            // set headers
	            ArrayList<String> headers = new ArrayList<String>();
	            for (String variable : meanMap.keySet()) {
	            	headers.add(variable);
	            }
	            // write headers to CSV
                csvWriter.writeRecord(headers);
	            
	            // iterate through rows (max count)
	            for (int i = 0; i < maxCount; i++) {
	            	// create value list for this row
	            	ArrayList<String> row = new ArrayList<String>();
			        // loop through stored variable values
			    	for (ArrayList<Double> values : meanMap.values()) {
			    		// check whether current index is valid for this variable
			    		// not valid? then save an empty value
			    		if (i >= values.size()) {
			    			row.add("");
			    			continue;
			    		}
			    		// get value for this variable for this row
			    		row.add(values.get(i).toString());
			    	}
			    	
			    	// write the row to CSV
                    csvWriter.writeRecord(row);
                }
            
	            // user feedback
	            System.out.println("Intermediate CSV file created successfully.");
            
	        } catch (IOException exception) {
	            exception.printStackTrace();
	        }
		}
    	
    }
	
	public void exportLaneChangesToCsv(ArrayList<String> laneChangeTime, ArrayList<String> laneChangeIds, ArrayList<String> laneChangeDirections,
			ArrayList<String> laneChangeLinks, ArrayList<String> laneChangeFromLanes, String laneChangeFilePath) {
		
		// only continue when output data is available
		if (!laneChangeIds.isEmpty()) {
			
			// user feedback
			System.out.println("\nLane change output data exporting to CSV file at \n" + "'" + laneChangeFilePath + "'"); 
			
			// ensure that directories exist
			checkDirectory(laneChangeFilePath);
			
			// write output data to given CSV file
	        try (FileWriter fileWriter = new FileWriter(laneChangeFilePath)) {
	        	
	        	// create CSV writer
	            CsvWriter csvWriter = CsvWriter.builder().build(fileWriter);
	            
				// go through all required row indexes
				int maxLength = laneChangeIds.size();
	            for (int i = -1; i < maxLength; i++) {
	            	// create list of values for this row index
	            	List<String> rowValues = new ArrayList<String>();
	            	// headers
	            	if (i == -1) {
		    			rowValues.add("time");
	            		rowValues.add("id");
						rowValues.add("direction");
		            	rowValues.add("link");
		            	rowValues.add("from_lane");
	            	}
	            	// values
	            	else {
		    			rowValues.add(laneChangeTime.get(i));
		            	rowValues.add(laneChangeIds.get(i));
						rowValues.add(laneChangeDirections.get(i));
		            	rowValues.add(laneChangeLinks.get(i));
		            	rowValues.add(laneChangeFromLanes.get(i));
	            	}
					
	            	// write this row into CSV file
		            csvWriter.writeRecord(rowValues);
	            }
	            
	            // user feedback
	            System.out.println("Lane change CSV file created successfully.");
	            
	        } catch (IOException e) {
				e.printStackTrace();
			}
		}
	}
	
}


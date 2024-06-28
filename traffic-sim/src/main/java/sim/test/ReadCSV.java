package sim.test;

import de.siegmar.fastcsv.reader.CsvRecord;
import de.siegmar.fastcsv.reader.CsvReader;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.io.IOException;

import java.util.ArrayList;
import java.util.List;

public class ReadCSV {
	
	public static void main(String[] args) throws IOException {
	
		// CSV file path
		Path csvFilePath = Paths.get("C:\\Users\\jesse\\Documents\\Java\\TrafficSimulation-workspace\\traffic-demo\\src\\main\\resources\\TestData.csv");
	
		// List to store CSV records
		List<CsvRecord> csvRecords = new ArrayList<>();

		try (CsvReader<CsvRecord> csv = CsvReader.builder().ofCsvRecord(csvFilePath)) {
		    csv.forEach(csvRecords::add);
		}
		
		// Print stored CSV records
		for (CsvRecord record : csvRecords) {
		    System.out.println(record.getFields());
		}

	}
	
}


import csv
import yaml

def convert_csv_to_yaml(csv_file, yaml_file):
    # Read the CSV file
    point_data = []
    with open(csv_file, 'r') as f:
        csv_reader = csv.reader(f)
        # Skip the header rows
        for _ in range(4):  # Skip the first 4 lines
            next(csv_reader)
        
        # Extract the point data
        for row in csv_reader:
            if row and row[0] == 'Point':
                point_name = row[1]
                x = float(row[2])
                y = float(row[3])
                z = float(row[4])
                point_data.append({"x": x, "y": y, "z": z})
    
    # Create the YAML structure
    yaml_data = {
        "tibia_marker": point_data
    }
    
    # Add reference coordinates as a placeholder
    # You can modify these values as needed
    yaml_data["tibia_ref"] = [
        {"x": 0.0, "y": 0.018, "z": -0.02},
        {"x": 0.025, "y": 0.0, "z": 0.0},
        {"x": 0.0, "y": -0.028, "z": -0.02},
        {"x": -0.015, "y": 0.0, "z": 0.0},
        {"x": 0.0, "y": 0.0, "z": 0.015}
    ]
    
    # Write to YAML file
    with open(yaml_file, 'w') as f:
        f.write("# Marker coordinates (from the marker \"tibia_body\")\n")
        yaml.dump(yaml_data, f, default_flow_style=False, sort_keys=False)

if __name__ == "__main__":
    convert_csv_to_yaml("config/tibia_tracker.csv", "test.yaml")
    print("Conversion completed. YAML file created successfully.")
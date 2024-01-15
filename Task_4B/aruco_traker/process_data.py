import csv

def process_csv(file_path):
    data_dict = {}

    with open(file_path, 'r', encoding='utf-8-sig') as csvfile:
        reader = csv.DictReader(csvfile)

        for row in reader:
            try:
                # Access each attribute by its name and convert to the appropriate data type
                ar_id = int(row['id'])
                lat = float(row['lat'])
                lon = float(row['lon'])

                # Create a dictionary for each ID if it doesn't exist
                if ar_id not in data_dict:
                    # data_dict[ar_id] = {'lat': lat, 'lon': lon}
                    data_dict[ar_id] = [lat, lon]
                else:
                    # If the ID already exists, you can decide how to handle duplicates
                    print(f"Warning: Duplicate ID {ar_id}. Skipping.")

            except (ValueError, KeyError) as e:
                print(f"Error processing row: {row}. {e}")

    return data_dict

# Example usage
csv_file_path = '/home/pradhyumna/hardware_round/lat_long.csv'
processed_data_dict = process_csv(csv_file_path)
# print(processed_data_dict)
lat, lon = processed_data_dict[23]
print(lat)

# Access the processed data (a dictionary of dictionaries)
# for ar_id, values in processed_data_dict.items():
#     print(f"ID: {ar_id}, lat: {values['lat']}, lon: {values['lon']}")
# print()

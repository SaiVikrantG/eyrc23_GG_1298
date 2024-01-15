import csv

def process_csv(file_path):
    data = []

    with open(file_path, 'r', encoding='utf-8-sig') as csvfile:
        reader = csv.DictReader(csvfile)

    # with open(file_path, 'r') as csvfile:
    #     reader = csv.DictReader(csvfile)

        # Loop through each row in the CSV file
        for row in reader:
            try:
                # Access each attribute by its name
                ar_id = int(row['id'])
                lat = float(row['lat'])
                lon = float(row['lon'])

                # Create a dictionary for each row
                row_data = {'id': ar_id, 'lat': lat, 'lon': lon}

                # Append the dictionary to the data list
                data.append(row_data)

            except (ValueError, KeyError) as e:
                print(f"Error processing row: {row}. {e}")

    return data

# Example usage
csv_file_path = '/home/pradhyumna/hardware_round/lat_long.csv'
processed_data = process_csv(csv_file_path)

# Access the processed data (a list of dictionaries)
for row in processed_data:
    print(row)

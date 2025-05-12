import yaml
import csv

with open("joint_states_data.txt", "r") as infile:
    docs = infile.read().split('---')  # separate YAML documents

with open("cart_pole_data.csv", "w", newline="") as outfile:
    writer = csv.writer(outfile)
    writer.writerow(["Time", "Cart Position", "Pole Angle"])

    time = 0
    for doc in docs:
        if 'position' in doc:
            data = yaml.safe_load(doc)
            try:
                cart_pos = data['position'][0]
                pole_angle = data['position'][1]
                writer.writerow([time, cart_pos, pole_angle])
                time += 0.1  # adjust if needed
            except (KeyError, IndexError, TypeError):
                continue




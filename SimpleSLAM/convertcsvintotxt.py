import csv

# Input CSV file and output TXT file
csv_filename = r"/home/jeewantha/Desktop/robot_pose_20250710_084326.csv"
output_txt = 'pose.txt'

with open(csv_filename, 'r', newline='') as csvfile, open(output_txt, 'w') as txtfile:
    reader = csv.DictReader(csvfile)
    for row in reader:
        x = float(row['x'])
        y = float(row['y'])
        theta = float(row['theta'])
        txtfile.write(f"{x} {y} {theta}\n")

print(f"Saved x, y, theta to {output_txt}")
import os
from datetime import datetime
from load_cell import LoadCell

print("Load Cells calibration...\n")

phidget_serial = int(input("Phidget Serial: "))
channel = int(input("Channel(default all): "))

if channel == "":
    load_cells = [LoadCell({'tendon_id': i, 'cal_offset': None, 'cal_factor': None, 'serial': phidget_serial, 'channel': i }) for i in range(4)]
else:
    load_cells = [LoadCell({'tendon_id': 0, 'cal_offset': None, 'cal_factor': None, 'serial': phidget_serial, 'channel': channel})]

weights = [0.0, 5.0]

for load_cell in load_cells:
    print(f"\nCalibrating load cell {load_cell.id}...")
    load_cell.openChannel()

    measurements = []
    for w in weights:
        input(f"Apply {w} Kg to the load cell and press enter...")
        measurements.append(load_cell.getVoltageRatio())

    load_cell.cal_offset = measurements[0]
    load_cell.cal_factor = (weights[1] - weights[0]) / (measurements[1] - measurements[0])

calibration_report =    "# Force (Kg) = cal_factor * (VoltageRatio + cal_offset)\n" \
                        f"phidget_serial: {phidget_serial}\n" \
                        f"channels: {[load_cell.getChannel() for load_cell in load_cells]}\n" \
                        f"cal_offset: {[load_cell.cal_offset for load_cell in load_cells]}\n" \
                        f"cal_factor: {[load_cell.cal_factor for load_cell in load_cells]}"

print("\nCalibration values:\n")
print(calibration_report)

with open(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../config/cal_values.yaml'), 'a+') as f:
    print(f"\nCalibration datetime: {datetime.now()}", file=f)
    print(calibration_report, file=f)

for load_cell in load_cells:
    load_cell.closeChannel()

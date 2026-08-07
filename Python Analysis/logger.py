import csv
import serial


SERIAL_PORT = "/dev/cu.usbmodem1303"
BAUD_RATE = 460800
OUTPUT_FILE = "logged.csv"

EXPECTED_FIELDS = 30

CSV_HEADER = [
    "sample_index",
    "timestamp_ms",
    "accel_x_raw",
    "accel_y_raw",
    "accel_z_raw",
    "gyro_x_raw",
    "gyro_y_raw",
    "gyro_z_raw",
    "accel_x_g",
    "accel_y_g",
    "accel_z_g",
    "gyro_x_dps",
    "gyro_y_dps",
    "gyro_z_dps",
    "roll_acc_deg",
    "pitch_acc_deg",
    "accel_magnitude_g",
    "roll_kf_deg",
    "pitch_kf_deg",
    "roll_bias_dps",
    "pitch_bias_dps",
    "roll_k_angle",
    "pitch_k_angle",
    "qw",
    "qx",
    "qy",
    "qz",
    "mahony_roll_deg",
    "mahony_pitch_deg",
    "mahony_yaw_deg",
]

def main() -> None:
    print(f"Opening {SERIAL_PORT} at {BAUD_RATE} baud...")

    valid_samples = 0

    with serial.Serial(
        SERIAL_PORT,
        BAUD_RATE,
        timeout=1,
    ) as ser, open(
        OUTPUT_FILE,
        "w",
        newline="",
    ) as csv_file:

        writer = csv.writer(csv_file)
        writer.writerow(CSV_HEADER)

        print("Logging started.")
        print("Press Ctrl-C to stop.\n")

        try:
            while True:
                raw_line = ser.readline()

                if not raw_line:
                    continue

                try:
                    line = raw_line.decode("utf-8").strip()
                except UnicodeDecodeError:
                    print("Ignored undecodable line")
                    continue

                fields = line.split(",")

                if len(fields) != EXPECTED_FIELDS:
                    print(
                        f"Ignored line with {len(fields)} fields "
                        f"(expected {EXPECTED_FIELDS}): {line}"
                    )
                    continue

                try:
                    row = (
                        [int(value) for value in fields[:8]]
                        + [float(value) for value in fields[8:]]
                    )
                    
                except ValueError:
                    print(f"Ignored non-numeric line: {line}")
                    continue

                writer.writerow(row)
                valid_samples += 1

                if valid_samples % 100 == 0:
                    csv_file.flush()
                    print(f"Logged {valid_samples} samples")

        except KeyboardInterrupt:
            print("\nLogging stopped.")

    print(f"Saved {valid_samples} samples to {OUTPUT_FILE}")


if __name__ == "__main__":
    main()
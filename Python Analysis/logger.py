import csv
import serial


SERIAL_PORT = "/dev/cu.usbmodem1403"
BAUD_RATE = 230400
OUTPUT_FILE = "imu_stationary_full.csv"


def main() -> None:
    print(f"Opening {SERIAL_PORT} at {BAUD_RATE} baud...")

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

        writer.writerow([
            "sample_index",
            "timestamp_ms",
            "ax_raw",
            "ay_raw",
            "az_raw",
            "gx_raw",
            "gy_raw",
            "gz_raw",
            "ax_calibrated_g",
            "ay_calibrated_g",
            "az_calibrated_g",
            "gx_dps",
            "gy_dps",
            "gz_dps",
        ])

        valid_samples = 0

        print("Logging started.")
        print("Keep the IMU completely stationary.")
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

                if len(fields) != 14:
                    print(f"Ignored malformed line: {line}")
                    continue

                try:
                    sample_index = int(fields[0])
                    timestamp_ms = int(fields[1])

                    ax_raw = int(fields[2])
                    ay_raw = int(fields[3])
                    az_raw = int(fields[4])

                    gx_raw = int(fields[5])
                    gy_raw = int(fields[6])
                    gz_raw = int(fields[7])

                    ax_calibrated_g = float(fields[8])
                    ay_calibrated_g = float(fields[9])
                    az_calibrated_g = float(fields[10])

                    gx_dps = float(fields[11])
                    gy_dps = float(fields[12])
                    gz_dps = float(fields[13])

                except ValueError:
                    print(f"Ignored non-numeric line: {line}")
                    continue

                writer.writerow([
                    sample_index,
                    timestamp_ms,
                    ax_raw,
                    ay_raw,
                    az_raw,
                    gx_raw,
                    gy_raw,
                    gz_raw,
                    ax_calibrated_g,
                    ay_calibrated_g,
                    az_calibrated_g,
                    gx_dps,
                    gy_dps,
                    gz_dps,
                ])

                valid_samples += 1

                if valid_samples % 100 == 0:
                    csv_file.flush()
                    print(f"Logged {valid_samples} samples")

        except KeyboardInterrupt:
            print("\nLogging stopped.")

    print(f"Saved {valid_samples} samples to {OUTPUT_FILE}")


if __name__ == "__main__":
    main()
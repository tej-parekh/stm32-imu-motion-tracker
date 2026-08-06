import csv
import serial


SERIAL_PORT = "/dev/cu.usbmodem11303"
BAUD_RATE = 230400
OUTPUT_FILE = "logged.csv"

EXPECTED_FIELDS = 23

CSV_HEADER = [
    "sample_index",
    "timestamp_ms",
    "ax_raw",
    "ay_raw",
    "az_raw",
    "gx_raw",
    "gy_raw",
    "gz_raw",
    "ax_g",
    "ay_g",
    "az_g",
    "gx_dps",
    "gy_dps",
    "gz_dps",
    "roll_acc_deg",
    "pitch_acc_deg",
    "accel_magnitude_g",
    "roll_kf_deg",
    "pitch_kf_deg",
    "roll_bias_dps",
    "pitch_bias_dps",
    "roll_k_angle",
    "pitch_k_angle",
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
                    row = [
                        int(fields[0]),    # sample_index
                        int(fields[1]),    # timestamp_ms
                        int(fields[2]),    # ax_raw
                        int(fields[3]),    # ay_raw
                        int(fields[4]),    # az_raw
                        int(fields[5]),    # gx_raw
                        int(fields[6]),    # gy_raw
                        int(fields[7]),    # gz_raw
                        float(fields[8]),  # ax_g
                        float(fields[9]),  # ay_g
                        float(fields[10]), # az_g
                        float(fields[11]), # gx_dps
                        float(fields[12]), # gy_dps
                        float(fields[13]), # gz_dps
                        float(fields[14]), # roll_acc_deg
                        float(fields[15]), # pitch_acc_deg
                        float(fields[16]), # accel_magnitude_g
                        float(fields[17]), # roll_kf_deg
                        float(fields[18]), # pitch_kf_deg
                        float(fields[19]), # roll_bias_dps
                        float(fields[20]), # pitch_bias_dps
                        float(fields[21]), # roll_k_angle
                        float(fields[22]), # pitch_k_angle
                    ]

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
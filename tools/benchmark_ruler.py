import argparse
import csv
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sie_core.quality.accuracy import RulerBenchmark


def load_depth_pairs(path, measured_column, ruler_column):
    measured = []
    ruler = []

    with open(path, newline="") as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            measured.append(float(row[measured_column]))
            ruler.append(float(row[ruler_column]))

    return measured, ruler


def main():
    parser = argparse.ArgumentParser(description="Benchmark measured depth against ruler depth.")
    parser.add_argument("csv_path")
    parser.add_argument("--measured-column", default="measured")
    parser.add_argument("--ruler-column", default="ruler")
    args = parser.parse_args()

    measured, ruler = load_depth_pairs(args.csv_path, args.measured_column, args.ruler_column)
    result = RulerBenchmark().evaluate(measured, ruler)

    for key, value in result.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()

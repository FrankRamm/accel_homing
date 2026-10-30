"""Generate clearly SYNTHETIC developer QA images; never contact a printer."""
import argparse
import json
from pathlib import Path
from unittest import mock

from test_filter_charts import charts, synthetic_accuracy_axis, synthetic_filter_home


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    # Portable artifact filenames; the chart's displayed timezone is unchanged.
    clock = charts._clock()
    clock["filename_time"] = clock["filename_time"].replace(":", "-")
    renderer = charts.PNGRenderer()
    with mock.patch.object(charts, "_clock", return_value=clock):
        capture, record = synthetic_filter_home()
        timeline = charts.write_filter_capture(args.output, "synthetic_filter_timeline", "x", capture, record,
                                                .8, .8, simulated=True, renderer=renderer)
        control = synthetic_accuracy_axis("synthetic_raw_control", "raw",
            travels=(175.00, 175.30, 174.90, 175.10, 175.22, 174.95, 175.08, 175.28, 174.92, 175.13))
        baseline = charts.write_accuracy(args.output, "synthetic_raw_control", {"x": control}, "complete",
                                          simulated=True, renderer=renderer)
        treatment = synthetic_accuracy_axis("synthetic_filtered_trial", "filtered", threshold=1.5,
            travels=(175.10, 175.16, 175.08, 175.12, 175.14, 175.09, 175.15, 175.11, 175.13, 175.10))
        comparison = charts.write_accuracy(args.output, "synthetic_filtered_trial", {"x": treatment}, "complete",
                                            simulated=True, renderer=renderer, baseline_run_id="synthetic_raw_control")
    print(json.dumps({"SYNTHETIC_NOT_PHYSICAL_RESULTS": True, "timeline": timeline["png"],
                      "raw_control": baseline["x_png"], "actual_series_ab": comparison["x_png"]}, indent=2))


if __name__ == "__main__":
    main()

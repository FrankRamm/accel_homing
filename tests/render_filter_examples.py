"""Generate clearly SYNTHETIC developer QA images; never contact a printer."""
import argparse
import json
from pathlib import Path
from unittest import mock

from test_filter_charts import charts, synthetic_accuracy_axis, synthetic_filter_home
from test_censored_response_charts import native_filtered_capture, synthetic_native_accuracy_axis


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
        capture, record = native_filtered_capture()
        censored_home = charts.write_capture(args.output, "synthetic_censored_home", "x", capture, record,
                                             simulated=True, renderer=renderer)
        censored_timeline = charts.write_filter_capture(args.output, "synthetic_censored_timeline", "x", capture,
                                                         record, 10., 1., simulated=True, renderer=renderer)
        generated_travels = (175.00, 175.30, 174.90, 175.10, 175.22, 174.95, 175.08, 175.28, 174.92, 175.13)
        native_control = synthetic_native_accuracy_axis("synthetic_native_control", generated_travels, censored_indices=())
        native_baseline = charts.write_accuracy(args.output, "synthetic_native_control", {"x": native_control},
                                                "complete", simulated=True, renderer=renderer)
        censored_series = synthetic_native_accuracy_axis("synthetic_censored_trial", generated_travels,
                                                          censored_indices=(1, 3, 6))
        censored_comparison = charts.write_accuracy(args.output, "synthetic_censored_trial", {"x": censored_series},
            "complete", simulated=True, renderer=renderer, baseline_run_id="synthetic_native_control")
    print(json.dumps({"SYNTHETIC_NOT_PHYSICAL_RESULTS": True, "timeline": timeline["png"],
                      "raw_control": baseline["x_png"], "actual_series_ab": comparison["x_png"],
                      "native_saturation_prefix": censored_home["png"],
                      "native_saturation_cutoff": censored_timeline["png"],
                      "native_uncensored_control": native_baseline["x_png"],
                      "censored_response_ab": censored_comparison["x_png"]}, indent=2))


if __name__ == "__main__":
    main()

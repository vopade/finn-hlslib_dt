# HLS Library Unit Tests

## Instructions
1. Generate `config.h` and `memdata.h` by running data/gen_weights.py (modify it if you need non-default precision for weights/activations)
1. Set the FINN_HLS_ROOT to the root folder of the repo, e.g. `setenv FINN_HLS_ROOT <path to repo root> or export FINN_HLS_ROOT=<path to repo root>
1. Run a unit test with Vitis HLS, e.g. `vitis_hls <testname>.tcl`


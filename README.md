# Disaster Relief Helicopter Routing

This repository contains our C++ solution for the disaster-relief helicopter routing assignment. We maximize total delivered value minus trip costs while respecting all problem constraints and the exact output format used by the autograder.

## Build & Run

1) Build solver and checker
```bash
cd src
make
make checker
```

2) Solve an instance and write output
```bash
./main ../SampleIniputOutput/input1.txt ../SampleIniputOutput/output1.txt
```

3) Validate format and score with the checker (same semantics as the base code)
```bash
./format_checker ../input/input1.txt ../output/output1.txt
```

## What the solver does

It constructs a feasible plan and then improve it with local search:

- Random Restart
  - For each helicopter, repeatedly select a route which might contain 1 or more villages
  - Load packages: with perisahble being the highest priority then dry then other package
  - Calculate the cost with this random start state

- Local Search
  - For each new champion soution found try doing local search similar to hill climbing by interchanging some of the villages in each trip

## How correctness is ensured

I mirror the TA's checker and validate constraints both while generating neighbors and during scoring:

- Output format: identical to the sample (helicopter lines with trip count; one line per trip; trailing -1 per helicopter). Our `src/format_checker.cpp` logic matches the base checker, and we link and use the base checker to validate every run.
- Capacity and distance constraints:
  - Per-trip weight ≤ helicopter weight capacity
  - Per-trip distance (home→…→home) ≤ helicopter per-trip distance capacity
  - Sum of trip distances per helicopter ≤ DMax
  - For every trip, total dropped of each type ≤ picked up
- Value capping semantics (exactly as the checker):
  - Food value counted up to 9 × population per village; within that food cap, perishable is prioritized over dry.
  - Other supplies value counted up to 1 × population per village.

## Files

- `src/solver.cpp / src/solver.h`: RandomRestart + LocalSearch
- `src/structures.h`: Problem and solution data structures and geometry.
- `src/io_handler.cpp / src/io_handler.h`: Input parser and output writer (exact required format).
- `src/main.cpp`: Entrypoint; reads input, runs solver within time, writes output.
- `src/format_checker.cpp`: Same behavior and scoring semantics as the provided checker; we also build and use the professor’s checker for validation.
- `src/Makefile`: Build targets for solver (`main`) and checker (`format_checker`).

## Example

```bash
cd src
make && make checker
./main ../SampleIniputOutput/input1.txt ../SampleIniputOutput/output1.txt
./format_checker ../SampleIniputOutput/input1.txt ../SampleIniputOutput/output1.txt
```

The checker prints the total value, total trip cost, and the final score, and confirms all constraints are satisfied.

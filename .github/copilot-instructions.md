# Copilot instructions for DubinsFleetPlanner

## Build and test commands

Use out-of-source CMake builds from the repository root.

```bash
cmake -S . -B build -DDubinsFleetPlanner_TESTING=ON -DDubinsFleetPlanner_VISUALISATION=OFF -DDubinsFleetPlanner_PARALLEL=OFF
cmake --build build -j
```

- `DubinsFleetPlanner_PARALLEL=ON` requires a prebuilt HiGHS package discoverable by CMake (`src/CMakeLists.txt` calls `find_package(HIGHS REQUIRED)` when parallel mode is enabled).
- There is no repository-level lint target configured in root CMake.

Run all tests:

```bash
ctest --test-dir build --output-on-failure
```

Run a single test executable:

```bash
ctest --test-dir build -R '^TemporalDistance\\.'
```

Or run a single test binary directly:

```bash
./build/test/TestTemporalDistance
```

Optional coverage target (when configured with `-DDubinsFleetPlanner_COVERAGE=ON` and Python `gcovr` available):

```bash
cmake --build build --target coverage
```

## High-level architecture

The solver pipeline is:

1. **CLI and orchestration (`src/main.cpp`)**  
   Parses planner options, supports single-file and directory batch mode, loads optional obstacle paths and geometric obstacles, and selects solver mode:
   - arrival deltas: `FleetPlanner::solve(...)`
   - explicit timeslots: `FleetPlanner::solve_with_timeslots(...)`

2. **Input/output boundary (`src/ioUtils.*`, `USAGE.md`)**  
   - CSV input is strict on the first 13 columns (`ac_id` … `dt`); extra columns are interpreted as per-aircraft timeslots.
   - Output is JSON (`print_paths_as_ModernJSON`) or sampled CSV (`print_paths_as_CSV`).

3. **Path representation and primitives (`src/Dubins.hpp`, `src/Primitives.hpp`, `src/BaseDubins.hpp`)**  
   - `Dubins` is the core trajectory container (ordered `DynamicPathShape` sections plus derived metrics).
   - Base generators cover 8 families (`LSL`, `RSR`, `RSL`, `LSR`, `RLR`, `LRL`, plus `SLS`, `SRS`).

4. **Composable path-generation stack (`src/PathGenerator.hpp`, `src/PathGenerators/*`)**  
   The main builds generators by wrapping decorators:
   - base/fitted generators
   - optional line fitting / variable-radius fitting
   - optional sampling (`EllipseSampling`, `FittedEllipseSampling`, random variants)
   - optional start/end extensions (`StraightExtended*` or `AllExtended*`)

5. **Fleet solving (`src/FleetPlanner.hpp`)**  
   - Generates per-aircraft candidate paths from timing targets.
   - Filters candidates against moving obstacle trajectories and geometric obstacles.
   - For delta-time mode, iteratively samples candidate global arrival times (`generate_linspace` + `weave_samples`) and tracks best feasible duration.

6. **Conflict evaluation and optimization (`src/ConflictList.hpp/.cpp`, `libs/HiGHS`)**  
   - Computes pairwise path conflicts/distances (single-thread or threaded).
   - Solves binary path assignment as an ILP in HiGHS:
     - one path per aircraft
     - conflict constraints forbid incompatible path pairs
     - objective can emphasize turning angle or duration-weighted cost

## Key codebase-specific conventions

- **Template-driven separation/distance plumbing**: core solver APIs take compile-time function pointers (`Dubins::DubinsSeparationFunction`, `Dubins::DubinsDistanceFunction`) rather than runtime strategy objects.
- **Ownership model for candidate paths**: generation uses `std::unique_ptr<Dubins>`; conversion to shared ownership happens only when parallel conflict/LP stages need shared views (`make_shared(...)` in `utils.hpp`).
- **Strict CSV contract**: parser expects exact header field names/order and uses `std::from_chars`; malformed numeric input triggers assertion-based failure paths.
- **2D-first collision model**: most planning and separation checks are XY-focused; Z is parsed and propagated but several docs/code paths still treat vertical separation as limited or not fully implemented.
- **Batch solving behavior** (`main.cpp`): when input is a directory, outputs are per-case solution files plus `summary.csv`; existing outputs are skipped unless `--recompute` is set.

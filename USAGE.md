# Dubins Fleet Planner usage

## Command line

```bash
DubinsFleetPlanner <input> <output> <separation> [wind-x] [wind-y] [options]
```

Positional arguments:

- `input` (**required**): CSV case file, or a directory of CSV case files.
- `output` (**required**): output file (single-case mode) or output directory (batch mode).
- `separation` (**required**): minimum required XY separation between aircraft.
- `wind-x` (optional, default `0`): wind speed along X.
- `wind-y` (optional, default `0`): wind speed along Y.

If `input` is a directory, the planner runs all regular files in that directory, writes one output file per case, and writes/updates `summary.csv` in the output directory. Existing case outputs are skipped unless `--recompute` is set.

## Options

| Option | Default | Meaning |
| --- | --- | --- |
| `-A`, `--reallocate` | `false` | Allow aircraft to switch endpoints (delta-time solving mode). |
| `-O`, `--obstacles <json>` | empty | JSON file with obstacle trajectories. |
| `-G`, `--geo-obstacles <json>` | empty | JSON file with geometric (time-independent) obstacles. |
| `--pathgen-config <json>` | empty | Per-aircraft path-generation overrides (see below). |
| `--extend-start <vals...>` | unset | Start extension lengths for extended Dubins generation. |
| `--extend-end <vals...>` | unset | End extension lengths for extended Dubins generation. |
| `--straights-only` | `false` | With extensions, only allow straight extensions. |
| `--allow-shortest` | `false` | Timeslot mode only: also allow shortest-path candidates. |
| `-l`, `--line` | `false` | Add minimal-turn-radius paths with fitted start/end straights. |
| `--line-ratios <vals...>` | auto | Ratios used by `--line`; defaults to `0 0.5 1` when `--line` is enabled. |
| `--fixed-radius` | `false` | With `--line`, keep only minimal-radius Dubins paths (disable radius fitting). |
| `-S`, `--samples <int>` | `0` | Path-generation sampling count (`0` disables sampling planner). |
| `-e`, `--ellipse <float>` | `NaN` | Ellipse ratio used by sampling planner (`NaN` disables ellipse mode). |
| `--ellipse-border-only` | `false` | In ellipse mode, sample only ellipse border points. |
| `-t`, `--threads <int>` | `-1` | Number of threads (`0` = auto). |
| `-p`, `--precision <float>` | `1e-3` | Numeric precision. |
| `-r`, `--max-r-length <float>` | `3` | Maximal relative path length factor. |
| `-I`, `--max-iters <int>` | `300` | Max solver iterations (must be `> 0`). |
| `-w`, `--weave <int>` | `2` | Number of samples inserted when weaving/resampling. |
| `--weave-dist <float>` | `0.1` | Minimum absolute time-gap to allow weaving between samples. |
| `--weave-dist-percent <float>` | `0.0001` | Minimum relative time-gap (fraction of max duration) to allow weaving. |
| `-T`, `--max-time <int>` | `60` | Maximum solver runtime in seconds. |
| `-R`, `--recompute` | `false` | In batch mode, overwrite/redo already solved cases. |
| `-v`, `--verbose <int>` | `1` | Verbosity level (`0` to `3`). |
| `-E`, `--export-samples <int>` | `1000` | Number of samples for CSV export/display (must be `>= 3`). |
| `--help` | - | Print help and exit. |

Behavioral defaults:

- If only one of `--extend-start` / `--extend-end` is provided, the other is auto-filled with `[0]`.
- If `--line` is set and `--line-ratios` is omitted, ratios default to `[0, 0.5, 1]`.
- `--samples` must be non-negative.

## JSON Paths format

We define a JSON format for encoding several trajectories. This format is used both for giving pre-existing trajectories to the solvers
as obstacles (the `-O` option), and as output format.

```json
{
   "separation": (Float Number: Minimal separation achieved),
   "z_alpha"   : (Float Number: Z distance modifier value),
   "wind_x"    : (Float Number: Wind speed along the X axis),
   "wind_y"    : (Float Number: Wind speed along the Y axis),
   "duration"  : (Float Number: time duration of the plan),
   "AC_num"    : (Int Number  : number of aircraft),
   "trajectories": [
      {
         "stats"  : Stats object,
         "path"   : Path object,
         "delay"  : (Float Number, optional: Time delay before the trajectory starts.)
      },...
   ]
}
```

A `trajectory` is the combination of both the `Path`, i.e. the geometry drawing; and the `Stats` which describe how the aircraft follows said geometry.
When a `trajectory` is given in input as obstacle, it may also contain a `delay` field, indicating when in the future the trajectory will begin.

A `Pose3D` object is defined as follows:

```json  
{
   "x"    : (Float Number: X coordinate),
   "y"    : (Float Number: Y coordinate),
   "z"    : (Float Number: Z coordinate),
   "theta": (Float Number: XY orientation, in radian)
}

A `Stats` object:

```json
{
   "id"         : (Int Number  : aircraft id),
   "airspeed"   : (Float Number: aircraft speed in air referential),
   "climb"      : (Float Number: aircraft climb rate),
   "turn_radius": (Float Number: aircraft minimal turn radius),
}
```

A `Path` object:

```json
{
   "total_length"  : (Float Number : total length of the path),
   "sections_count": (Int Number   : number of basic sections in this path),
   "start"         : (Pose3D object: starting pose),
   "end"           : (Pose3D object: ending pose),
   "sections": [
      BasicPath objects,...
   ]
}
```

A `BasicPath` object:

```json
{
   "length": (Float Number: Length of this section),
   "type"  : (String      : Either "LEFT","RIGHT" or "STRAIGHT", for describing a turn or a straight line movement),
   "m"     : (Int         : Either 0,1 or 2, the enum integer maching the type),
   "x"     : (Float Number: X coordinate of the refence point; For a Straight: Starting point | For a turn: Circle center),
   "y"     : (Float Number: Y coordinate of the refence point),
   "z"     : (Float Number: Z coordinate of the refence point),
   "p1"    : (Float Number: For a Straight: horizontal x speed | For a turn: Radius),
   "p2"    : (Float Number: For a Straight: horizontal y speed | For a turn: signed angular speed; positive when LEFT, negative when RIGHT),
   "p3"    : (Float Number: Vertical speed),
   "p4"    : (Float Number: For a Straight: unused | For a turn: initial angle),
}
```

Regarding the geometric obstacles (the `-G` option), the JSON file is expected to contain an array of `Path` objects.

## Input CSV format

The fleet path planning problem is expected to be described as a CSV file, separated with semi-colons.
The first 13 columns must match:

```text
ac_id;start_x;start_y;start_z;start_theta;end_x;end_y;end_z;end_theta;airspeed;climb;turn_radius;dt
```

These fields describe, for each aircraft:

- `ac_id`: integer aircraft id.
- `start_[x,y,z,theta]`: start pose (`theta` in radians).
- `end_[x,y,z,theta]`: end pose (`theta` in radians).
- `airspeed`: positive speed in air reference frame.
- `climb`: positive climb rate (used symmetrically for descent).
- `turn_radius`: positive minimum turn radius.
- `dt`: arrival-time difference from the previous aircraft (ignored for the first aircraft).

Any extra columns are interpreted as per-aircraft timeslots. If at least one aircraft has timeslots, the solver uses timeslot mode; otherwise it uses delta-time mode from `dt`.

> Note: core conflict handling is currently XY-focused. Z values are parsed and propagated but are not treated as a full 3D separation model.

## Output format

Output format is selected from the output extension:

- `.csv`: sampled trajectories (`--export-samples` controls sample count).
- anything else (or no extension): JSON trajectory output.

In batch mode, each case output name is derived from the input filename with extension:

```text
.sol<output_extension_or_.json_if_missing>
```

Example: if `output` path has extension `.csv`, each case is written as `*.sol.csv`.

### CSV output

If it ends with `.csv`, stores the results by sampling each path (see flag `-s`), and put them in a CSV with samples+1 lines
(data+header), the header being: `time;[X_{ac_id};Y_{ac_id};Z_{ac_id};theta_{ac_id}]*{number of aircraft}`

### JSON output

Otherwise, the results are stored in the JSON Paths format described previously.

## Per-aircraft path generation configuration (`--pathgen-config`)

The CSV format remains unchanged. Per-aircraft path-generation options can be overridden with a JSON file.

Supported keys:

- `extend-start` (array of numbers)
- `extend-end` (array of numbers)
- `straights-only` (bool)
- `line` (bool)
- `line-ratios` (array of numbers)
- `fixed-radius` (bool)
- `samples` (integer)
- `ellipse` (number, or string `"NaN"` to disable ellipse)
- `ellipse-border-only` (bool)

JSON schema example:

```json
{
  "default": {
    "line": true,
    "line-ratios": [0.0, 0.5, 1.0],
    "samples": 20,
    "ellipse": 0.95,
    "ellipse-border-only": true
  },
  "aircraft": {
    "1": {
      "extend-start": [0.0, 50.0, 100.0],
      "extend-end": [0.0, 40.0],
      "straights-only": true
    },
    "3": {
      "samples": 0,
      "line": false
    }
  }
}
```

Merge order per aircraft:

1. command-line options
2. `default` block overrides
3. matching `aircraft.<ac_id>` overrides

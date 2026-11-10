# so101-draw

Handwriting with an SO101.

## Calibrate the drawing area

With the robot2 server running, use Go 1.26.5 or later:

```sh
go run ./cmd/calibrate -base-url http://localhost:1337 -output calibration.json
```

The command relaxes all motors. Move the pen to the top-left corner of the
drawing area and press Enter. Then slowly shade in the rest of the rectangle,
drawing various lines through it or along its border. Press Enter again to save.
Motors remain relaxed after the command exits.

Positions are sampled every 250 ms (4 Hz); use `-interval 100ms` to change this.
The JSON contains `top_left`, a map of motor names to raw encoder positions,
and `samples`, an ordered array of `{ "elapsed_seconds": ..., "positions": ... }`.
The first sample is the top-left corner at time zero, followed by periodic
readings and a final reading when Enter is pressed. Positions use robot2's
raw motor units, not angles or Cartesian coordinates.

The output file is replaced only after a successful recording. Input EOF and
API errors abort without replacing it.

## Predict drawing positions

The `calibration` package reads the command's JSON format and predicts all six
joint angles, in radians. For example, to target the middle of the drawing area:

```go
// Imports: encoding/json, os, github.com/unixpickle/so101-draw/calibration
contents, err := os.ReadFile("calibration.json")
if err != nil {
    return err
}
var data calibration.Data
if err := json.Unmarshal(contents, &data); err != nil {
    return err
}
mapper, err := calibration.New(&data)
if err != nil {
    return err
}
rect := mapper.Rectangle()
angles, err := mapper.Angles(rect.Width/2, rect.Height/2)
if err != nil {
    return err
}
// angles is a *kinematics.MotorAngles; nothing has been sent to the robot.
```

Drawing coordinates are in millimeters, with `(0, 0)` at the recorded top-left
corner, X increasing rightward and Y increasing downward. `Rectangle.XY` and
`Rectangle.Local` convert between drawing coordinates and robot XY. The basis
rotates within ±45° of the robot axes; its signs are inferred from the samples,
so the right axis can point along either direction of robot X.

The fit projects robot2's forward-kinematics **gripper midpoint** onto XY. It
searches for a small-area orthogonal rectangle that contains at least 99% of the
recorded points, keeping the initial corner fixed. At each angle it optimizes
width and height jointly. The angle search uses a 0.25° grid, sample directions,
and local refinement. It returns an error if the samples do not span a rectangle
or the fixed corner makes 99% containment impossible.

For each target, the mapper finds the closest calibration point in XY and makes
a local numerical IK correction. Shoulder pan, shoulder lift, elbow flex, and
wrist flex may change. The reference point's Z and the sum of shoulder lift,
elbow flex, and wrist flex remain fixed, as do wrist roll and gripper angle.
The solver requires an XYZ residual below 0.0001 mm and rejects corrections
larger than 45° in any joint or targets outside the fitted rectangle.

This models the gripper midpoint, without a custom pen-tip offset or hardware
joint limits. Its numerical tolerance describes agreement with forward
kinematics, not physical accuracy. Keep the drawing area away from full arm
extension. Nearest-neighbor changes can introduce small jumps in reference
height, pitch, or wrist roll; this API predicts points rather than smoothing
trajectories.

Run the synthetic comparison against `HoverPositionToCoordAngles` with:

```sh
go test ./calibration -run TestSyntheticHoverRectangles -v
```

The test generates three rotated rectangles, shades them with horizontal and
vertical lines, converts the poses to raw encoder readings, and round-trips the
recording through JSON. Each rectangle uses 274 calibration samples and 98
held-out targets, including edges and the three unrecorded corners. The known
top-left corner is checked separately. Joint errors must stay below 10°;
XY, reference Z, and wrist pitch have separate tight checks. Verbose output
reports per-joint maximum and RMS errors and the hover solver's own FK residual.
It compares both the fitted robot XY target and the same local millimeter
coordinate in the original rectangle, so errors from fitting the rectangle
are measured separately from errors in the local joint correction.

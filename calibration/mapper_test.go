package calibration

import (
	"encoding/json"
	"math"
	"testing"

	"github.com/unixpickle/model3d/model2d"
	"github.com/unixpickle/model3d/model3d"
	"github.com/unixpickle/robot2/kinematics"
)

// This exercises the full recording format (including encoder quantization),
// rectangle fit, nearest-neighbor lookup, and constrained local IK. Hover IK is
// used only to generate training poses and independent held-out reference poses.
func TestSyntheticHoverRectangles(t *testing.T) {
	for _, tc := range []struct {
		name                                       string
		angle, width, height, z, sx, sy, wristRoll float64
	}{
		{"clockwise", -31.7, 90, 60, 5, 1, 1, 0.2},
		{"counterclockwise", 17.3, 70, 50, -5, -1, 1, -0.4},
		{"near45", 42.4, 95, 45, 15, 1, -1, 0.6},
	} {
		t.Run(tc.name, func(t *testing.T) {
			s, c := math.Sincos(tc.angle * math.Pi / 180)
			truth := Rectangle{
				Right: model2d.XY(tc.sx*c, tc.sx*s), Down: model2d.XY(-tc.sy*s, tc.sy*c),
				Width: tc.width, Height: tc.height,
			}
			// Keep these drawing areas comfortably away from full extension.
			truth.Origin = model2d.XY(0, 195).Sub(truth.Right.Scale(tc.width / 2)).Sub(truth.Down.Scale(tc.height / 2))
			min := kinematics.NewMotorAngles([6]float64{-1.5, -0.6, -1.5, -math.Pi, -math.Pi, 0})
			max := kinematics.NewMotorAngles([6]float64{1.5, 1.1, 1.0, math.Pi, math.Pi, 0})
			maxOracleResidual := 0.0
			oracle := func(xy model2d.Coord) kinematics.MotorAngles {
				p := model3d.XYZ(xy.X, xy.Y, tc.z)
				a := kinematics.HoverPositionToCoordAngles(min, max, p, tc.wristRoll)
				if a == nil {
					t.Fatalf("hover IK failed for %v", p)
				}
				err := kinematics.AnglesToCoords(a).Mid().Dist(p)
				maxOracleResidual = math.Max(maxOracleResidual, err)
				if err > 2 {
					t.Fatalf("hover fixture is unreachable: %v, residual %.3f mm", p, err)
				}
				return *a
			}
			data := Data{TopLeft: encoderPositions(oracle(truth.Origin))}
			data.Samples = append(data.Samples, Sample{Positions: data.TopLeft})
			// Hatch horizontally and vertically, with a 1.5 mm inset from the
			// ideal edges: the oracle's coarse angular grid plus raw encoders
			// perturb FK by sub-mm amounts, while the corner must stay fixed.
			const inset = 1.5
			add := func(x, y float64) {
				a := oracle(truth.XY(x, y))
				data.Samples = append(data.Samples, Sample{Positions: encoderPositions(a)})
			}
			for row := 0; row < 10; row++ {
				for col := 0; col < 21; col++ {
					add(inset+(tc.width-2*inset)*float64(col)/20, inset+(tc.height-2*inset)*float64(row)/9)
				}
			}
			for col := 0; col < 3; col++ {
				for row := 0; row < 21; row++ {
					add(inset+(tc.width-2*inset)*float64(col)/2, inset+(tc.height-2*inset)*float64(row)/20)
				}
			}
			encoded, err := json.Marshal(data)
			if err != nil {
				t.Fatal(err)
			}
			var decoded Data
			if err := json.Unmarshal(encoded, &decoded); err != nil {
				t.Fatal(err)
			}
			mapper, err := New(&decoded)
			if err != nil {
				t.Fatal(err)
			}
			r := mapper.Rectangle()
			contained := 0
			for _, p := range mapper.points[1:] {
				if r.contains(model2d.XY(p.coord.X, p.coord.Y)) {
					contained++
				}
			}
			if contained < len(data.Samples)-len(data.Samples)/100 {
				t.Fatalf("only %d/%d calibration samples are contained", contained, len(data.Samples))
			}
			if math.Abs(r.Angle*180/math.Pi-tc.angle) > 3 || math.Abs(r.Width-tc.width) > 5 || math.Abs(r.Height-tc.height) > 5 {
				t.Fatalf("poor rectangle fit: %+v", r)
			}
			corner, err := mapper.Angles(0, 0)
			if err != nil {
				t.Fatal(err)
			}
			for j, angle := range corner.Vec() {
				if math.Abs(angle-mapper.points[0].angles.Vec()[j]) > 1e-12 {
					t.Fatal("top-left angles changed")
				}
			}
			var maxima, sumSquares, fullMaxima, fullSumSquares [6]float64
			maxXY, maxZ, maxPitch := 0.0, 0.0, 0.0
			count := 0
			// A different grid, including fitted corners and edges. The known
			// top-left corner is tested separately, not counted as held-out.
			for row := 0; row < 9; row++ {
				for col := 0; col < 11; col++ {
					if row == 0 && col == 0 {
						continue
					}
					x, y := r.Width*float64(col)/10, r.Height*float64(row)/8
					xy := r.XY(x, y)
					want := oracle(xy)
					// Also compare the same local millimeter coordinate against
					// the original rectangle, so fit error is not hidden by only
					// evaluating the local IK at its own fitted world coordinate.
					fullWant := oracle(truth.XY(x, y))
					got, err := mapper.Angles(x, y)
					if err != nil {
						t.Fatalf("target (%g, %g): %v", x, y, err)
					}
					for j, angle := range got.Vec() {
						diff := math.Abs(math.Remainder(angle-want.Vec()[j], 2*math.Pi)) * 180 / math.Pi
						maxima[j] = math.Max(maxima[j], diff)
						sumSquares[j] += diff * diff
						if diff > 10 {
							t.Errorf("target (%g, %g), joint %d: error %.3f degrees exceeds 10", x, y, j, diff)
						}
						fullDiff := math.Abs(math.Remainder(angle-fullWant.Vec()[j], 2*math.Pi)) * 180 / math.Pi
						fullMaxima[j] = math.Max(fullMaxima[j], fullDiff)
						fullSumSquares[j] += fullDiff * fullDiff
						if fullDiff > 10 {
							t.Errorf("target (%g, %g), joint %d: including fit error %.3f degrees exceeds 10", x, y, j, fullDiff)
						}
					}
					ref := mapper.nearest(xy)
					coord := kinematics.AnglesToCoords(got).Mid()
					maxXY = math.Max(maxXY, math.Hypot(coord.X-xy.X, coord.Y-xy.Y))
					maxZ = math.Max(maxZ, math.Abs(coord.Z-ref.coord.Z))
					maxPitch = math.Max(maxPitch, math.Abs(pitch(*got)-pitch(ref.angles)))
					if got.WristRoll != ref.angles.WristRoll || got.Gripper != ref.angles.Gripper {
						t.Fatal("wrist roll or gripper changed")
					}
					count++
				}
			}
			if maxXY > positionTolerance || maxZ > positionTolerance || maxPitch > 1e-12 {
				t.Fatalf("constraint violation: XY=%g mm, Z=%g mm, pitch=%g rad", maxXY, maxZ, maxPitch)
			}
			var rms, fullRMS [6]float64
			for i := range rms {
				rms[i] = math.Sqrt(sumSquares[i] / float64(count))
				fullRMS[i] = math.Sqrt(fullSumSquares[i] / float64(count))
			}
			t.Logf("%d calibration / %d held-out; fit angle=%.4f deg, size=%.3f x %.3f mm", len(data.Samples), count, r.Angle*180/math.Pi, r.Width, r.Height)
			t.Logf("max error deg [pan lift elbow flex roll gripper]: %.4f", maxima)
			t.Logf("RMS error deg [pan lift elbow flex roll gripper]: %.4f", rms)
			t.Logf("including rectangle fit: max error deg %.4f; RMS error deg %.4f", fullMaxima, fullRMS)
			t.Logf("max XY residual=%.8f mm; Z drift=%.8f mm; pitch drift=%.3g rad; hover oracle residual=%.4f mm", maxXY, maxZ, maxPitch, maxOracleResidual)
			for _, xy := range [][2]float64{{-1, 0}, {0, -1}, {r.Width + 1, 0}, {0, r.Height + 1}, {math.NaN(), 0}, {0, math.Inf(1)}} {
				if _, err := mapper.Angles(xy[0], xy[1]); err == nil {
					t.Errorf("accepted out-of-range target %v", xy)
				}
			}
		})
	}
}

func TestAdjustNonHoverPose(t *testing.T) {
	ref := kinematics.MotorAngles{ShoulderPan: 0.1, ShoulderLift: 0.2, ElbowFlex: -0.15, WristFlex: 1.4, WristRoll: 0.4, Gripper: 0.1}
	want := ref
	want.ShoulderPan += 0.02
	want.ShoulderLift += 0.01
	want.ElbowFlex -= 0.02
	want.WristFlex += 0.01
	target := kinematics.AnglesToCoords(&want).Mid()
	got, err := adjust(ref, target)
	if err != nil {
		t.Fatal(err)
	}
	for i, x := range got.Vec() {
		if math.Abs(x-want.Vec()[i]) > 1e-5 {
			t.Fatalf("joint %d: got %g, want %g", i, x, want.Vec()[i])
		}
	}
	if _, err := adjust(ref, model3d.XYZ(10000, 10000, 0)); err == nil {
		t.Fatal("accepted unreachable target")
	}
}

func TestInvalidData(t *testing.T) {
	if _, err := New(nil); err == nil {
		t.Fatal("accepted nil data")
	}
	if _, err := New(&Data{}); err == nil {
		t.Fatal("accepted missing motors")
	}
	if _, err := New(&Data{TopLeft: encoderPositions(kinematics.MotorAngles{}), Samples: []Sample{{}}}); err == nil {
		t.Fatal("accepted sample with missing motors")
	}
	if _, err := NewFromAngles(kinematics.MotorAngles{Gripper: math.NaN()}, nil); err == nil {
		t.Fatal("accepted NaN angle")
	}
}

func encoderPositions(angles kinematics.MotorAngles) map[string]int16 {
	result := map[string]int16{}
	for i, name := range []string{"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"} {
		result[name] = int16(kinematics.AngleToPosition(angles.Vec()[i]))
	}
	return result
}

func pitch(a kinematics.MotorAngles) float64 { return a.ShoulderLift + a.ElbowFlex + a.WristFlex }

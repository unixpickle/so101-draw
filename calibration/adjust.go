package calibration

import (
	"fmt"
	"math"

	"github.com/unixpickle/model3d/model3d"
	"github.com/unixpickle/robot2/kinematics"
)

const positionTolerance = 1e-4 // mm

// Eliminate wrist flex using the fixed pitch sum, leaving three variables for
// the three XYZ constraints. Newton steps use the forward-kinematics Jacobian,
// with a step cap and backtracking to stay on the reference point's IK branch.
func adjust(reference kinematics.MotorAngles, target model3d.Coord3D) (*kinematics.MotorAngles, error) {
	pitch := reference.ShoulderLift + reference.ElbowFlex + reference.WristFlex
	pose := func(v [3]float64) *kinematics.MotorAngles {
		a := reference
		a.ShoulderPan, a.ShoulderLift, a.ElbowFlex = v[0], v[1], v[2]
		a.WristFlex = pitch - v[1] - v[2]
		return &a
	}
	coords := func(v [3]float64) model3d.Coord3D { return kinematics.AnglesToCoords(pose(v)).Mid() }
	v := [3]float64{reference.ShoulderPan, reference.ShoulderLift, reference.ElbowFlex}
	for iteration := 0; iteration < 40; iteration++ {
		current := coords(v)
		residual := target.Sub(current)
		if residual.Norm() <= positionTolerance {
			return pose(v), nil
		}
		const epsilon = 1e-5
		var jacobian [3][3]float64
		for col := 0; col < 3; col++ {
			plus, minus := v, v
			plus[col] += epsilon
			minus[col] -= epsilon
			d := coords(plus).Sub(coords(minus)).Scale(1 / (2 * epsilon))
			jacobian[0][col], jacobian[1][col], jacobian[2][col] = d.X, d.Y, d.Z
		}
		step, ok := solve3(jacobian, [3]float64{residual.X, residual.Y, residual.Z})
		if !ok {
			return nil, fmt.Errorf("local IK is singular near the reference sample")
		}
		// Include the dependent wrist-flex step in the per-iteration cap.
		maxStep := math.Abs(step[1] + step[2])
		for _, x := range step {
			maxStep = math.Max(maxStep, math.Abs(x))
		}
		scale := math.Min(1, 0.15/maxStep)
		improved := false
		for backtrack := 0; backtrack < 16; backtrack++ {
			candidate := v
			for i := range candidate {
				candidate[i] += scale * step[i]
			}
			// A correction beyond 45 degrees is no longer a local adjustment.
			local := true
			candidateVec := pose(candidate).Vec()
			for i, x := range candidateVec[:4] {
				if math.Abs(x-reference.Vec()[i]) > math.Pi/4 {
					local = false
				}
			}
			if local && coords(candidate).Dist(target) < residual.Norm() {
				v, improved = candidate, true
				break
			}
			scale /= 2
		}
		if !improved {
			return nil, fmt.Errorf("local IK cannot reach target while preserving reference height and pitch (residual %.6g mm)", residual.Norm())
		}
	}
	return nil, fmt.Errorf("local IK did not converge")
}

// Gaussian elimination with partial pivoting for the small Jacobian system.
func solve3(a [3][3]float64, b [3]float64) ([3]float64, bool) {
	for col := 0; col < 3; col++ {
		pivot := col
		for row := col + 1; row < 3; row++ {
			if math.Abs(a[row][col]) > math.Abs(a[pivot][col]) {
				pivot = row
			}
		}
		if math.Abs(a[pivot][col]) < 1e-10 {
			return [3]float64{}, false
		}
		a[col], a[pivot] = a[pivot], a[col]
		b[col], b[pivot] = b[pivot], b[col]
		for row := col + 1; row < 3; row++ {
			ratio := a[row][col] / a[col][col]
			for j := col; j < 3; j++ {
				a[row][j] -= ratio * a[col][j]
			}
			b[row] -= ratio * b[col]
		}
	}
	var x [3]float64
	for row := 2; row >= 0; row-- {
		sum := b[row]
		for col := row + 1; col < 3; col++ {
			sum -= a[row][col] * x[col]
		}
		x[row] = sum / a[row][row]
	}
	return x, true
}

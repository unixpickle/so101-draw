package calibration

import (
	"fmt"
	"math"

	"github.com/unixpickle/model3d/model2d"
	"github.com/unixpickle/model3d/model3d"
	"github.com/unixpickle/robot2/kinematics"
)

type calibrationPoint struct {
	angles kinematics.MotorAngles
	coord  model3d.Coord3D
}

// Mapper predicts joint angles within a fitted drawing rectangle. It uses the
// midpoint of robot2's two finger tips as the drawing point. It does not command
// motors, check hardware limits, or model an additional pen-tip offset.
// A Mapper is immutable and can be queried concurrently.
type Mapper struct {
	rectangle Rectangle
	points    []calibrationPoint
}

// New constructs a mapper from the JSON-compatible recording format.
func New(data *Data) (*Mapper, error) {
	if data == nil {
		return nil, fmt.Errorf("nil calibration data")
	}
	topLeft, err := motorAngles(data.TopLeft)
	if err != nil {
		return nil, fmt.Errorf("top-left: %w", err)
	}
	samples := make([]kinematics.MotorAngles, len(data.Samples))
	for i, sample := range data.Samples {
		angles, err := motorAngles(sample.Positions)
		if err != nil {
			return nil, fmt.Errorf("sample %d: %w", i, err)
		}
		samples[i] = angles
	}
	return NewFromAngles(topLeft, samples)
}

// NewFromAngles constructs a mapper from joint angles in radians, without
// quantizing them to encoder positions.
func NewFromAngles(topLeft kinematics.MotorAngles, samples []kinematics.MotorAngles) (*Mapper, error) {
	points := make([]calibrationPoint, 0, len(samples)+1)
	for i := -1; i < len(samples); i++ {
		angles := topLeft
		if i >= 0 {
			angles = samples[i]
		}
		for _, angle := range angles.Vec() {
			if !finite(angle) {
				return nil, fmt.Errorf("non-finite joint angle at sample %d (-1 is top-left)", i)
			}
		}
		points = append(points, calibrationPoint{angles: angles, coord: kinematics.AnglesToCoords(&angles).Mid()})
	}
	xys := make([]model2d.Coord, len(samples))
	for i, p := range points[1:] {
		xys[i] = model2d.XY(p.coord.X, p.coord.Y)
	}
	origin := model2d.XY(points[0].coord.X, points[0].coord.Y)
	rectangle, err := FitRectangle(origin, xys)
	if err != nil {
		return nil, err
	}
	return &Mapper{rectangle: rectangle, points: points}, nil
}

func (m *Mapper) Rectangle() Rectangle { return m.rectangle }

// Angles predicts all six motor angles for drawing coordinates in millimeters:
// x runs right from 0 to Width; y runs down from 0 to Height. The closest recorded
// point in robot XY supplies the initial angles, Z, and wrist pitch. Only pan,
// lift, elbow, and wrist flex change. Wrist roll and gripper remain unchanged.
// A failed or excessive local correction returns an error, never an unchecked
// approximate pose. The nearest-neighbor reference can change discontinuously
// across the rectangle, so this is a point predictor, not a trajectory smoother.
func (m *Mapper) Angles(x, y float64) (*kinematics.MotorAngles, error) {
	if !finite(x) || !finite(y) || x < 0 || y < 0 || x > m.rectangle.Width || y > m.rectangle.Height {
		return nil, fmt.Errorf("drawing coordinate (%g, %g) is outside [0, %g] x [0, %g]", x, y, m.rectangle.Width, m.rectangle.Height)
	}
	xy := m.rectangle.XY(x, y)
	nearest := m.nearest(xy)
	target := model3d.XYZ(xy.X, xy.Y, nearest.coord.Z)
	return adjust(nearest.angles, target)
}

func (m *Mapper) nearest(xy model2d.Coord) calibrationPoint {
	best := m.points[0]
	bestDist := math.Inf(1)
	for _, p := range m.points {
		dx, dy := p.coord.X-xy.X, p.coord.Y-xy.Y
		if dist := dx*dx + dy*dy; dist < bestDist {
			best, bestDist = p, dist
		}
	}
	return best
}

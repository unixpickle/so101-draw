package calibration

import (
	"container/heap"
	"fmt"
	"math"
	"sort"

	"github.com/unixpickle/model3d/model2d"
)

const containmentEpsilon = 1e-7

// Rectangle is anchored at the recorded top-left corner. Right and Down are
// orthogonal unit vectors in robot XY; their signs are inferred from the data.
// Right lies within 45 degrees of either direction of the robot X axis.
type Rectangle struct {
	Origin model2d.Coord
	Right  model2d.Coord
	Down   model2d.Coord
	Width  float64
	Height float64
	// Angle is the rotation from the robot's XY axes, in [-pi/4, pi/4].
	Angle float64
}

// XY converts drawing coordinates (right, down), in mm, to robot XY.
func (r Rectangle) XY(x, y float64) model2d.Coord {
	return r.Origin.Add(r.Right.Scale(x)).Add(r.Down.Scale(y))
}

// Local converts robot XY to drawing coordinates in mm.
func (r Rectangle) Local(p model2d.Coord) model2d.Coord {
	d := p.Sub(r.Origin)
	return model2d.XY(d.Dot(r.Right), d.Dot(r.Down))
}

func (r Rectangle) contains(p model2d.Coord) bool {
	q := r.Local(p)
	return q.X >= -containmentEpsilon && q.Y >= -containmentEpsilon &&
		q.X <= r.Width+containmentEpsilon && q.Y <= r.Height+containmentEpsilon
}

// FitRectangle searches for the smallest-area, corner-anchored rectangle
// containing at least ceil(0.99*len(points)) points. The corner never moves.
// Width/height are optimized jointly at each angle, rather than using separate
// axis percentiles (which need not contain 99% of the points jointly).
// The angle search uses a 0.25-degree grid, point directions where a sample
// crosses an axis, and local refinement to 0.00025 degrees. It is an approximate
// global search, not an exact minimum-area certificate.
func FitRectangle(origin model2d.Coord, points []model2d.Coord) (Rectangle, error) {
	if !finite(origin.X) || !finite(origin.Y) || len(points) < 3 {
		return Rectangle{}, fmt.Errorf("rectangle needs a finite corner and at least three samples")
	}
	offsets := make([]model2d.Coord, len(points))
	angles := make([]float64, 0, len(points)+361)
	for i, p := range points {
		if !finite(p.X) || !finite(p.Y) {
			return Rectangle{}, fmt.Errorf("sample %d has non-finite coordinates", i)
		}
		d := p.Sub(origin)
		offsets[i] = d
		if d.Norm() > containmentEpsilon {
			// Include these directions so even an arbitrarily narrow feasible
			// angle range between samples on two adjacent borders is found.
			a := math.Atan2(d.Y, d.X)
			for a < -math.Pi/4 {
				a += math.Pi / 2
			}
			for a > math.Pi/4 {
				a -= math.Pi / 2
			}
			angles = append(angles, a)
		}
	}
	const step = math.Pi / 720
	for i := 0; i <= 360; i++ {
		angles = append(angles, -math.Pi/4+float64(i)*step)
	}
	sort.Float64s(angles)
	// Integer arithmetic avoids rounding an exact 99% count up spuriously.
	keep := len(points) - len(points)/100
	bestArea := math.Inf(1)
	var best Rectangle
	evaluate := func(angle float64) {
		if angle < -math.Pi/4 || angle > math.Pi/4 {
			return
		}
		s, c := math.Sincos(angle)
		for _, sx := range []float64{1, -1} {
			for _, sy := range []float64{1, -1} {
				right, down := model2d.XY(sx*c, sx*s), model2d.XY(-sy*s, sy*c)
				projected := make([]model2d.Coord, 0, len(offsets))
				for _, d := range offsets {
					x, y := d.Dot(right), d.Dot(down)
					if x >= -containmentEpsilon && y >= -containmentEpsilon {
						projected = append(projected, model2d.XY(math.Max(0, x), math.Max(0, y)))
					}
				}
				if len(projected) < keep {
					continue
				}
				w, h := smallestBounds(projected, keep)
				if w*h < bestArea {
					bestArea = w * h
					best = Rectangle{Origin: origin, Right: right, Down: down, Width: w, Height: h, Angle: angle}
				}
			}
		}
	}
	for i, angle := range angles {
		if i == 0 || angle-angles[i-1] > 1e-12 {
			evaluate(angle)
		}
	}
	delta := step
	for refinement := 0; refinement < 3; refinement++ {
		delta /= 10
		center := best.Angle
		for i := -10; i <= 10; i++ {
			evaluate(center + float64(i)*delta)
		}
	}
	if math.IsInf(bestArea, 1) {
		return Rectangle{}, fmt.Errorf("no rectangle anchored at top-left contains 99%% of samples")
	}
	if best.Width < 1e-6 || best.Height < 1e-6 {
		return Rectangle{}, fmt.Errorf("calibration does not span a rectangle")
	}
	return best, nil
}

// Sweep possible widths while maintaining the keep smallest Y values. The
// largest of these is the minimum height for this width, so every candidate
// contains keep actual points. Complexity is O(n log n).
func smallestBounds(points []model2d.Coord, keep int) (float64, float64) {
	sort.Slice(points, func(i, j int) bool { return points[i].X < points[j].X })
	heights := make(maxHeap, 0, keep)
	area := math.Inf(1)
	var width, height float64
	for _, p := range points {
		if len(heights) < keep {
			heap.Push(&heights, p.Y)
		} else if p.Y < heights[0] {
			heights[0] = p.Y
			heap.Fix(&heights, 0)
		}
		if len(heights) == keep && p.X*heights[0] < area {
			width, height = p.X, heights[0]
			area = width * height
		}
	}
	return width, height
}

type maxHeap []float64

func (h maxHeap) Len() int           { return len(h) }
func (h maxHeap) Less(i, j int) bool { return h[i] > h[j] }
func (h maxHeap) Swap(i, j int)      { h[i], h[j] = h[j], h[i] }
func (h *maxHeap) Push(x any)        { *h = append(*h, x.(float64)) }
func (h *maxHeap) Pop() any {
	x := (*h)[len(*h)-1]
	*h = (*h)[:len(*h)-1]
	return x
}

func finite(x float64) bool { return !math.IsNaN(x) && !math.IsInf(x, 0) }

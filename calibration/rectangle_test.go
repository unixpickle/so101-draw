package calibration

import (
	"math"
	"math/rand"
	"testing"

	"github.com/unixpickle/model3d/model2d"
)

func TestFitRectangle(t *testing.T) {
	for _, signs := range [][2]float64{{1, 1}, {-1, 1}, {1, -1}, {-1, -1}} {
		angle := 17.321 * math.Pi / 180
		s, c := math.Sincos(angle)
		want := Rectangle{
			Origin: model2d.XY(15, 200), Right: model2d.XY(signs[0]*c, signs[0]*s),
			Down: model2d.XY(-signs[1]*s, signs[1]*c), Width: 70, Height: 55,
		}
		var points []model2d.Coord
		for x := 0; x <= 20; x++ {
			for y := 0; y <= 20; y++ {
				points = append(points, want.XY(float64(x)*want.Width/20, float64(y)*want.Height/20))
			}
		}
		// Four bad readings are less than 1% of the total. They must not rotate
		// or enlarge the drawing area, nor move the fixed corner.
		for i := 0; i < 4; i++ {
			points = append(points, want.XY(-100-float64(i), -200))
		}
		got, err := FitRectangle(want.Origin, points)
		if err != nil {
			t.Fatal(err)
		}
		if got.Origin != want.Origin || math.Abs(got.Angle-angle) > 1e-7 ||
			math.Abs(got.Width-want.Width) > 1e-5 || math.Abs(got.Height-want.Height) > 1e-5 ||
			got.Right.Dist(want.Right) > 1e-7 || got.Down.Dist(want.Down) > 1e-7 {
			t.Fatalf("fit mismatch: got %+v; want %+v", got, want)
		}
		contained := 0
		for _, p := range points {
			if got.contains(p) {
				contained++
			}
		}
		if contained < len(points)-len(points)/100 {
			t.Fatalf("only %d/%d points contained", contained, len(points))
		}
	}
}

func TestSmallestBounds(t *testing.T) {
	rng := rand.New(rand.NewSource(42))
	for trial := 0; trial < 30; trial++ {
		points := make([]model2d.Coord, 12)
		for i := range points {
			points[i] = model2d.XY(rng.Float64()*10, rng.Float64()*10)
		}
		const keep = 9
		wantArea := math.Inf(1)
		for _, x := range points {
			for _, y := range points {
				count := 0
				for _, p := range points {
					if p.X <= x.X && p.Y <= y.Y {
						count++
					}
				}
				if count >= keep {
					wantArea = math.Min(wantArea, x.X*y.Y)
				}
			}
		}
		w, h := smallestBounds(points, keep)
		if math.Abs(w*h-wantArea) > 1e-9 {
			t.Fatalf("area %g, want %g", w*h, wantArea)
		}
	}
}

func TestInvalidRectangle(t *testing.T) {
	for _, points := range [][]model2d.Coord{
		nil,
		{model2d.XY(1, 0), model2d.XY(2, 0), model2d.XY(3, 0)},
		{model2d.XY(1, 1), model2d.XY(2, 2), model2d.XY(math.NaN(), 3)},
		{model2d.XY(-1, -1), model2d.XY(1, 1), model2d.XY(-1, 1)},
	} {
		if _, err := FitRectangle(model2d.XY(0, 0), points); err == nil {
			t.Fatalf("expected invalid rectangle for %v", points)
		}
	}
}

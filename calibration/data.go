// Package calibration fits a drawing rectangle and predicts joint angles from
// recorded motor positions. Distances are in millimeters and angles in radians.
package calibration

import (
	"fmt"

	"github.com/unixpickle/robot2/kinematics"
)

// Data is the JSON format written by cmd/calibrate.
type Data struct {
	TopLeft map[string]int16 `json:"top_left"`
	Samples []Sample         `json:"samples"`
}

type Sample struct {
	ElapsedSeconds float64          `json:"elapsed_seconds"`
	Positions      map[string]int16 `json:"positions"`
}

func motorAngles(positions map[string]int16) (kinematics.MotorAngles, error) {
	var vec [6]float64
	for i, name := range []string{
		"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper",
	} {
		pos, ok := positions[name]
		if !ok {
			return kinematics.MotorAngles{}, fmt.Errorf("missing motor %q", name)
		}
		vec[i] = kinematics.PositionToAngle(pos)
	}
	return *kinematics.NewMotorAngles(vec), nil
}

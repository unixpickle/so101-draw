// Command calibrate records the drawing area's top-left corner and interior.
package main

import (
	"bufio"
	"encoding/json"
	"flag"
	"fmt"
	"io"
	"net/http"
	"os"
	"path/filepath"
	"time"

	"github.com/unixpickle/robot2/api"
	"github.com/unixpickle/so101-draw/calibration"
)

func main() {
	if err := run(); err != nil {
		fmt.Fprintln(os.Stderr, "calibrate:", err)
		os.Exit(1)
	}
}

func run() error {
	parseClient := api.AddClientFlags()
	output := flag.String("output", "calibration.json", "output JSON file (replaced on success)")
	interval := flag.Duration("interval", 250*time.Millisecond, "time between motor position samples")
	flag.Parse()
	if flag.NArg() != 0 {
		return fmt.Errorf("unexpected positional arguments: %v", flag.Args())
	}
	if *interval <= 0 {
		return fmt.Errorf("-interval must be positive")
	}
	client, err := parseClient()
	if err != nil {
		return err
	}
	// The robot2 client uses http.DefaultClient for its requests.
	http.DefaultClient.Timeout = 5 * time.Second

	// Check the destination before relaxing the robot, and replace an existing
	// recording only after the new recording has been written successfully.
	f, err := os.CreateTemp(filepath.Dir(*output), ".calibration-*.json")
	if err != nil {
		return fmt.Errorf("prepare output: %w", err)
	}
	defer os.Remove(f.Name())
	defer f.Close()

	data, err := record(client, os.Stdin, os.Stdout, *interval)
	if err != nil {
		return err
	}
	encoder := json.NewEncoder(f)
	encoder.SetIndent("", "  ")
	if err := encoder.Encode(data); err != nil {
		return fmt.Errorf("write calibration: %w", err)
	}
	if err := f.Close(); err != nil {
		return fmt.Errorf("close calibration: %w", err)
	}
	if err := os.Rename(f.Name(), *output); err != nil {
		return fmt.Errorf("save calibration: %w", err)
	}
	fmt.Printf("Saved %d samples to %s. Motors remain relaxed.\n", len(data.Samples), *output)
	return nil
}

func record(client *api.Client, input io.Reader, output io.Writer, interval time.Duration) (*calibration.Data, error) {
	if err := client.SetTorqueEnabledAll(false); err != nil {
		return nil, fmt.Errorf("relax motors: %w", err)
	}
	fmt.Fprintln(output, "Motors relaxed. Move the pen to the top-left corner of the drawing area.")
	fmt.Fprintln(output, "Press Enter when the pen is there.")
	reader := bufio.NewReader(input)
	if _, err := reader.ReadString('\n'); err != nil {
		return nil, fmt.Errorf("wait for top-left corner: %w", err)
	}
	topLeft, err := positions(client)
	if err != nil {
		return nil, err
	}
	start := time.Now()
	data := &calibration.Data{
		TopLeft: topLeft,
		Samples: []calibration.Sample{{Positions: topLeft}},
	}
	fmt.Fprintln(output, "Slowly shade in the rest of the rectangle, drawing various lines through it or along its border.")
	fmt.Fprintf(output, "Recording every %s. Press Enter again to finish and save.\n", interval)
	done := make(chan error, 1)
	go func() {
		_, err := reader.ReadString('\n')
		done <- err
	}()
	ticker := time.NewTicker(interval)
	defer ticker.Stop()
	for {
		finished := false
		select {
		case err := <-done:
			if err != nil {
				return nil, fmt.Errorf("wait for end of trace: %w", err)
			}
			finished = true
		case <-ticker.C:
		}
		// Also capture the final position, even when Enter precedes the next tick.
		pos, err := positions(client)
		if err != nil {
			return nil, err
		}
		if len(pos) != len(topLeft) {
			return nil, fmt.Errorf("motor set changed during recording")
		}
		for name := range topLeft {
			if _, ok := pos[name]; !ok {
				return nil, fmt.Errorf("motor %q missing during recording", name)
			}
		}
		data.Samples = append(data.Samples, calibration.Sample{
			ElapsedSeconds: time.Since(start).Seconds(),
			Positions:      pos,
		})
		if finished {
			return data, nil
		}
	}
}

func positions(client *api.Client) (map[string]int16, error) {
	statuses, err := client.MotorStatuses()
	if err != nil {
		return nil, err
	}
	if len(statuses) == 0 {
		return nil, fmt.Errorf("robot returned no motor positions")
	}
	result := make(map[string]int16, len(statuses))
	for name, status := range statuses {
		if status == nil {
			return nil, fmt.Errorf("robot returned no status for motor %q", name)
		}
		result[name] = status.Position
	}
	return result, nil
}

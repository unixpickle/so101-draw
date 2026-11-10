package main

import (
	"encoding/json"
	"io"
	"net/http"
	"net/http/httptest"
	"net/url"
	"strings"
	"testing"
	"time"

	"github.com/unixpickle/robot2/api"
)

func TestRecord(t *testing.T) {
	input, writer := io.Pipe()
	defer input.Close()
	defer writer.Close()
	periodicRead := make(chan struct{})
	requests := make(chan string, 100)
	reads := 0
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		requests <- r.Method + " " + r.URL.Path
		switch r.URL.Path {
		case "/motor/settorque":
			var body map[string]any
			if err := json.NewDecoder(r.Body).Decode(&body); err != nil {
				t.Error(err)
			}
			if body["enabled"] != false || body["motor"] != nil {
				t.Errorf("expected all motors relaxed, got %v", body)
			}
			io.WriteString(w, `{"data":true}`)
		case "/motor/status":
			reads++
			json.NewEncoder(w).Encode(map[string]any{"data": map[string]any{
				"shoulder_pan": map[string]int{"position": 2000 + reads},
				"wrist_roll":   map[string]int{"position": -reads},
			}})
			if reads == 3 {
				close(periodicRead)
			}
		default:
			t.Errorf("unexpected robot request: %s", r.URL.Path)
			http.NotFound(w, r)
		}
	}))
	defer server.Close()
	go func() {
		io.WriteString(writer, "\n")
		select {
		case <-periodicRead:
			io.WriteString(writer, "\n")
		case <-time.After(5 * time.Second):
			writer.CloseWithError(io.ErrNoProgress)
		}
	}()
	interval := 10 * time.Millisecond
	data, err := record(testClient(server), input, io.Discard, interval)
	if err != nil {
		t.Fatal(err)
	}
	if got := <-requests; got != "POST /motor/settorque" {
		t.Fatalf("first request = %s, want torque disabled", got)
	}
	for len(requests) > 0 {
		if got := <-requests; got != "GET /motor/status" {
			t.Errorf("unexpected subsequent request: %s", got)
		}
	}
	if data.TopLeft["shoulder_pan"] != 2001 || data.TopLeft["wrist_roll"] != -1 {
		t.Fatalf("wrong top-left positions: %v", data.TopLeft)
	}
	if len(data.Samples) < 4 {
		t.Fatalf("want initial, periodic, and final samples; got %d", len(data.Samples))
	}
	for i, s := range data.Samples {
		if s.Positions["shoulder_pan"] != int16(2001+i) || s.Positions["wrist_roll"] != int16(-1-i) {
			t.Errorf("sample %d: wrong positions: %v", i, s.Positions)
		}
		if i == 0 && s.ElapsedSeconds != 0 {
			t.Error("initial sample must be at time zero")
		}
		if i > 0 && s.ElapsedSeconds <= data.Samples[i-1].ElapsedSeconds {
			t.Error("sample timestamps must increase")
		}
	}
	if data.Samples[2].ElapsedSeconds < (2 * interval).Seconds() {
		t.Error("periodic samples were recorded too quickly")
	}
}

func TestRecordErrors(t *testing.T) {
	for _, tc := range []struct {
		name, input, status, want string
		torqueFails               bool
	}{
		{name: "relax fails", input: "\n\n", torqueFails: true, want: "relax motors"},
		{name: "initial EOF", want: "wait for top-left corner"},
		{name: "finish EOF", input: "\n", status: `{"joint":{"position":42}}`, want: "wait for end of trace"},
		{name: "empty status", input: "\n\n", status: `{}`, want: "no motor positions"},
		{name: "null status", input: "\n\n", status: `{"joint":null}`, want: "no status for motor"},
		{name: "status fails", input: "\n\n", want: "get motor statuses"},
	} {
		t.Run(tc.name, func(t *testing.T) {
			server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
				if r.URL.Path == "/motor/settorque" && !tc.torqueFails {
					io.WriteString(w, `{"data":true}`)
				} else if r.URL.Path == "/motor/status" && tc.status != "" {
					io.WriteString(w, `{"data":`+tc.status+`}`)
				} else {
					http.Error(w, "failed", http.StatusInternalServerError)
				}
			}))
			defer server.Close()
			_, err := record(testClient(server), strings.NewReader(tc.input), io.Discard, time.Hour)
			if err == nil || !strings.Contains(err.Error(), tc.want) {
				t.Fatalf("got %v, want error containing %q", err, tc.want)
			}
		})
	}
}

func testClient(server *httptest.Server) *api.Client {
	u, _ := url.Parse(server.URL)
	return api.NewClient(u)
}

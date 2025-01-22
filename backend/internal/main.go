package main

import (
	"encoding/json"
	"fmt"
	"math"
	"math/rand"
	"net/http"
	"strconv"
	"sync"
	"time"

	mqtt "github.com/eclipse/paho.mqtt.golang"
	"github.com/gin-contrib/cors"
	"github.com/gin-gonic/gin"
	"github.com/gorilla/websocket"
	"github.com/ostapula/smart-city-backend/internal/sensor"
)

// Vector3 (already used for traffic lights and stop zones)
type Vector3 struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
}

type TrafficLight struct {
	Index int    `json:"index"`
	State string `json:"state"` // "red", "yellow", or "green"
}

// Define phases for each intersection
type Phase struct {
	GreenLights []int
}

// IntersectionState tracks the current phase and timing for each intersection
type IntersectionState struct {
	CurrentPhase int
	TimeInPhase  float64
	MinGreen     float64
	MaxGreen     float64
	YellowTime   float64
	TimeInYellow float64
	InYellow     bool
	IsRedToGreen bool // Track red->yellow->green transition
	Demand       []float64
}

// Create an array of intersection states, one for each intersection
var intersectionStates = [4]IntersectionState{
	{
		CurrentPhase: 0,
		MinGreen:     4.0,
		MaxGreen:     10.0,
		YellowTime:   2.0,
		InYellow:     false,
		IsRedToGreen: false,
		Demand:       []float64{0, 0},
	},
	{
		CurrentPhase: 0,
		MinGreen:     4.0,
		MaxGreen:     10.0,
		YellowTime:   2.0,
		InYellow:     false,
		IsRedToGreen: false,
		Demand:       []float64{0, 0},
	},
	{
		CurrentPhase: 0,
		MinGreen:     4.0,
		MaxGreen:     10.0,
		YellowTime:   2.0,
		InYellow:     false,
		IsRedToGreen: false,
		Demand:       []float64{0, 0},
	},
	{
		CurrentPhase: 0,
		MinGreen:     4.0,
		MaxGreen:     10.0,
		YellowTime:   2.0,
		InYellow:     false,
		IsRedToGreen: false,
		Demand:       []float64{0, 0},
	},
}

// demand[0] => total demand on North-South
// demand[1] => total demand on East-West
var demand = []float64{0, 0}

// WebSocket management
var upgrader = websocket.Upgrader{}
var clients = make(map[*websocket.Conn]bool)
var clientsMutex = sync.Mutex{}
var broadcast = make(chan []byte)
var carCounter int64 = 8

var trafficLights = []TrafficLight{
	{Index: 0, State: "red"},
	{Index: 1, State: "red"},
	{Index: 2, State: "red"},
	{Index: 3, State: "red"},
	{Index: 4, State: "red"},
	{Index: 5, State: "red"},
	{Index: 6, State: "red"},
	{Index: 7, State: "red"},
	{Index: 8, State: "red"},
	{Index: 9, State: "red"},
	{Index: 10, State: "red"},
	{Index: 11, State: "red"},
	{Index: 12, State: "red"},
	{Index: 13, State: "red"},
	{Index: 14, State: "red"},
	{Index: 15, State: "red"},
}

// We'll define 8 EXACT spawn points, each one a single (x,z) coordinate
// plus one direction (no "range" for x,z).
var spawnPoints = []struct {
	X, Z               float64
	DirX, DirZ         float64
	SpeedMin, SpeedMax float64
}{
	// SOUTH (two lanes) -> heading north
	{X: -3, Z: -65, DirX: 0, DirZ: 1, SpeedMin: 3, SpeedMax: 4},
	{X: -95, Z: -65, DirX: 0, DirZ: 1, SpeedMin: 3, SpeedMax: 4},

	// EAST (two lanes) -> heading west
	{X: -145, Z: 2, DirX: 1, DirZ: 0, SpeedMin: 3, SpeedMax: 4},
	{X: -145, Z: 95, DirX: 1, DirZ: 0, SpeedMin: 3, SpeedMax: 4},

	// NORTH (two lanes) -> heading south
	{X: 3, Z: 165, DirX: 0, DirZ: -1, SpeedMin: 3, SpeedMax: 4},
	{X: -89, Z: 165, DirX: 0, DirZ: -1, SpeedMin: 3, SpeedMax: 4},

	// WEST (two lanes) -> heading east
	{X: 65, Z: -3, DirX: -1, DirZ: 0, SpeedMin: 3, SpeedMax: 4},
	{X: 65, Z: 89, DirX: -1, DirZ: 0, SpeedMin: 3, SpeedMax: 4},
}

type IntersectionZone struct {
	Index      int
	MinX, MaxX float64
	MinZ, MaxZ float64
}

var intersectionZones = []IntersectionZone{
	{
		Index: 0,
		MinX:  -30, MaxX: 30,
		MinZ: -30, MaxZ: 30,
	},
	{
		Index: 1,
		MinX:  -110, MaxX: -70,
		MinZ: -30, MaxZ: 30,
	},
	{
		Index: 2,
		MinX:  -110, MaxX: -70,
		MinZ: 70, MaxZ: 110,
	},
	{
		Index: 3,
		MinX:  -30, MaxX: 30,
		MinZ: 70, MaxZ: 110,
	},
}

// Our global cars array uses sensor.CarData
var (
	cars     = []sensor.CarData{
		{
			ID:        "car1",
			Position:  sensor.Vector3{X: -2, Y: 0.25, Z: -50},
			Direction: sensor.Vector3{X: 0, Y: 0, Z: 1},
			Speed:     5,
		},
		{
			ID:        "car2",
			Position:  sensor.Vector3{X: 2, Y: 0.25, Z: 50},
			Direction: sensor.Vector3{X: 0, Y: 0, Z: -1},
			Speed:     5,
		},
		{
			ID:        "car3",
			Position:  sensor.Vector3{X: -50, Y: 0.25, Z: 2},
			Direction: sensor.Vector3{X: 1, Y: 0, Z: 0},
			Speed:     5,
		},
		{
			ID:        "car4",
			Position:  sensor.Vector3{X: 50, Y: 0.25, Z: -2},
			Direction: sensor.Vector3{X: -1, Y: 0, Z: 0},
			Speed:     5,
		},
		{
			ID:        "car5",
			Position:  sensor.Vector3{X: -2, Y: 0.25, Z: -59},
			Direction: sensor.Vector3{X: 0, Y: 0, Z: 1},
			Speed:     3,
		},
		{
			ID:        "car6",
			Position:  sensor.Vector3{X: 2, Y: 0.25, Z: 59},
			Direction: sensor.Vector3{X: 0, Y: 0, Z: -1},
			Speed:     3,
		},
		{
			ID:        "car7",
			Position:  sensor.Vector3{X: -59, Y: 0.25, Z: 2},
			Direction: sensor.Vector3{X: 1, Y: 0, Z: 0},
			Speed:     3,
		},
		{
			ID:        "car8",
			Position:  sensor.Vector3{X: 59, Y: 0.25, Z: -2},
			Direction: sensor.Vector3{X: -1, Y: 0, Z: 0},
			Speed:     3,
		},
	}
	carsMutex sync.RWMutex
)

var sensorData = make(map[int]sensor.SensorDataMessage)
var sensorDataMutex = sync.Mutex{}

// We'll define a global slice of "stop zones" for each intersection approach
// so that getRecommendedSpeed can see them as obstacles.
type StopZone struct {
	Position        Vector3
	TrafficLightIdx int
}

var stopZones = []StopZone{
	// Intersection 0 (lights 0..3)
	// north approach => lightIndex = 0
	{Position: Vector3{X: 3, Y: 0, Z: 10}, TrafficLightIdx: 0},
	// south approach => lightIndex = 1
	{Position: Vector3{X: -3, Y: 0, Z: -10}, TrafficLightIdx: 1},
	// east approach  => lightIndex = 2
	{Position: Vector3{X: -10, Y: 0, Z: 3}, TrafficLightIdx: 2},
	// west approach  => lightIndex = 3
	{Position: Vector3{X: 10, Y: 0, Z: -3}, TrafficLightIdx: 3},

	// Intersection 1 (lights 4..7)
	// north approach => lightIndex = 4
	{Position: Vector3{X: -89, Y: 0, Z: 10}, TrafficLightIdx: 4},
	// south approach => lightIndex = 5
	{Position: Vector3{X: -95, Y: 0, Z: -10}, TrafficLightIdx: 5},
	// east approach  => lightIndex = 6
	{Position: Vector3{X: -102, Y: 0, Z: 3}, TrafficLightIdx: 6},
	// west approach  => lightIndex = 7
	{Position: Vector3{X: -82, Y: 0, Z: -3}, TrafficLightIdx: 7},

	// Intersection 2 (lights 8..11)
	{Position: Vector3{X: -89, Y: 0, Z: 102}, TrafficLightIdx: 8},
	{Position: Vector3{X: -95, Y: 0, Z: 82}, TrafficLightIdx: 9},
	{Position: Vector3{X: -102, Y: 0, Z: 95}, TrafficLightIdx: 10},
	{Position: Vector3{X: -82, Y: 0, Z: 89}, TrafficLightIdx: 11},

	// Intersection 3 (lights 12..15)
	{Position: Vector3{X: 3, Y: 0, Z: 102}, TrafficLightIdx: 12},
	{Position: Vector3{X: -3, Y: 0, Z: 82}, TrafficLightIdx: 13},
	{Position: Vector3{X: -10, Y: 0, Z: 95}, TrafficLightIdx: 14},
	{Position: Vector3{X: 10, Y: 0, Z: 89}, TrafficLightIdx: 15},
}

var rng = rand.New(rand.NewSource(time.Now().UnixNano()))

// Add new struct for settings
type Settings struct {
	Type                  string               `json:"type"`
	GlobalSpeedMultiplier float64              `json:"globalSpeedMultiplier"`
	IntersectionSettings  []IntersectionTiming `json:"intersectionSettings"`
}

type IntersectionTiming struct {
	MinGreen   float64 `json:"minGreen"`
	MaxGreen   float64 `json:"maxGreen"`
	YellowTime float64 `json:"yellowTime"`
}

// Add a response struct
type SettingsResponse struct {
	Type    string `json:"type"`
	Success bool   `json:"success"`
	Message string `json:"message"`
}

func main() {
	// MQTT setup
	opts := mqtt.NewClientOptions()
	opts.AddBroker("tcp://localhost:1883")
	opts.SetClientID("main_app")
	mqttClient := mqtt.NewClient(opts)
	if token := mqttClient.Connect(); token.Wait() && token.Error() != nil {
		panic(token.Error())
	}
	defer mqttClient.Disconnect(250)

	// Subscribe to sensor topics
	for i := 0; i < 16; i++ {  // Changed from 4 to 16 to subscribe to all sensors
		topic := "smartcity/sensors/" + fmt.Sprint(i)
		mqttClient.Subscribe(topic, 0, handleSensorData)
	}

	// Start sensor manager
	sensorManager := sensor.NewSensorManager("tcp://localhost:1883", &cars, &carsMutex)
	go sensorManager.Start()

	// Start goroutines
	go simulateTrafficFlow()
	go handleMessages()
	go spawnRandomCars()

	// Setup Gin server
	router := gin.Default()
	router.Use(cors.Default())
	router.GET("/ws", func(c *gin.Context) {
		serveWs(c.Writer, c.Request)
	})

	// Serve static files for frontend (adjust path if needed)
	router.Static("/static", "../frontend")
	router.GET("/", func(c *gin.Context) {
		c.File("../frontend/index.html")
	})

	// Run server on :8080
	router.Run(":8080")
}

func handleSensorData(client mqtt.Client, msg mqtt.Message) {
	var data sensor.SensorDataMessage
	json.Unmarshal(msg.Payload(), &data)

	sensorDataMutex.Lock()
	sensorData[data.SensorIndex] = data
	sensorDataMutex.Unlock()
}

// Main simulation loop
func simulateTrafficFlow() {
	ticker := time.NewTicker(100 * time.Millisecond)
	defer ticker.Stop()
	lastTime := time.Now()

	for {
		<-ticker.C
		currentTime := time.Now()
		deltaTime := currentTime.Sub(lastTime).Seconds()
		lastTime = currentTime

		updateCarPositions(deltaTime)
		updateTrafficLights(deltaTime)

		// Log position of car1
		logCarPositions()

		// Broadcast state to all WebSocket clients
		data := map[string]interface{}{
			"type":          "update",
			"cars":          cars,
			"trafficLights": trafficLights,
		}
		msg, _ := json.Marshal(data)
		broadcast <- msg
	}
}

func updateCarPositions(deltaTime float64) {
	carsMutex.Lock()
	defer carsMutex.Unlock()

	accel := 2.0
    minSpeed := 0.5
	
	for i := range cars {
		car := &cars[i]
		targetSpeed := getRecommendedSpeed(*car, cars, stopZones)

		// If target speed > 0 (e.g., green light) but car is stopped,
		// give it a minimum starting speed
		if targetSpeed > 0 && car.Speed < 0.1 {
			car.Speed = minSpeed
		}

		// Then apply normal acceleration/deceleration
		if car.Speed < targetSpeed {
			car.Speed = math.Min(car.Speed+accel*deltaTime, targetSpeed)
		} else {
			car.Speed = math.Max(car.Speed-accel*deltaTime, targetSpeed)
		}

		// Move
		car.Position.X += car.Direction.X * car.Speed * deltaTime
		car.Position.Z += car.Direction.Z * car.Speed * deltaTime
	}

	// Check if cars are out of bounds and respawn them
	for i := range cars {
		car := &cars[i]
		if car.Position.Z < -65 || car.Position.Z > 165 || car.Position.X > 65 || car.Position.X < -145 {
			// Pick a random spawn point
			sp := spawnPoints[rng.Intn(len(spawnPoints))]

			// Reset position to spawn point
			car.Position.X = sp.X
			car.Position.Y = 0.25
			car.Position.Z = sp.Z

			// Set direction
			car.Direction.X = sp.DirX
			car.Direction.Y = 0
			car.Direction.Z = sp.DirZ

			// Set random speed within spawn point's range
			car.Speed = sp.SpeedMin + rng.Float64()*(sp.SpeedMax-sp.SpeedMin)
		}
	}
}

// Simple adaptive traffic light logic
func updateTrafficLights(deltaTime float64) {
	for i := range intersectionStates {
		updateSingleIntersection(i, deltaTime)
	}
}

func updateSingleIntersection(intersectionIndex int, deltaTime float64) {
	state := &intersectionStates[intersectionIndex]

	// If currently in yellow, handle yellow timer
	if state.InYellow {
		state.TimeInYellow += deltaTime
		if state.TimeInYellow >= state.YellowTime {
			state.InYellow = false
			state.TimeInYellow = 0
			if state.IsRedToGreen {
				// Complete the transition to green
				state.IsRedToGreen = false
				switchToGreen(intersectionIndex)
			} else {
				// Complete the transition to red
				switchPhase(intersectionIndex)
			}
		}
		return
	} else {
		state.TimeInPhase += deltaTime
	}

	// Calculate demand
	calculateIntersectionDemand(intersectionIndex)

	// Minimum green time check
	if state.TimeInPhase < state.MinGreen {
		return
	}
	// Max green time check
	if state.TimeInPhase >= state.MaxGreen {
		triggerYellow(intersectionIndex)
		return
	}

	// Demand-based check
	currentDemand := state.Demand[state.CurrentPhase]
	otherPhase := (state.CurrentPhase + 1) % 2
	if state.Demand[otherPhase] > currentDemand*1.2 {
		triggerYellow(intersectionIndex)
		return
	}
}

// Evaluate sensor-based demand for each intersection
func calculateIntersectionDemand(intersectionIndex int) {
	state := &intersectionStates[intersectionIndex]
	// Reset demand
	state.Demand[0] = 0
	state.Demand[1] = 0

	sensorDataMutex.Lock()
	defer sensorDataMutex.Unlock()

	// Each intersection has 4 sensors => base index for that intersection
	baseIndex := intersectionIndex * 4

	// North-South demand => sensors 0 and 1
	northSensor := sensorData[baseIndex]
	southSensor := sensorData[baseIndex+1]
	northSouthDemand := calculatePhaseDemand(northSensor, southSensor)
	state.Demand[0] = northSouthDemand

	// East-West demand => sensors 2 and 3
	eastSensor := sensorData[baseIndex+2]
	westSensor := sensorData[baseIndex+3]
	eastWestDemand := calculatePhaseDemand(eastSensor, westSensor)
	state.Demand[1] = eastWestDemand

	// Log detailed demand information
	fmt.Printf("Intersection %d Demand:\n", intersectionIndex)
	fmt.Printf("  North-South (Phase 0):\n")
	fmt.Printf("    North: %d cars (wait: %.2fs)\n", len(northSensor.CarIDs), northSensor.WaitTime)
	fmt.Printf("    South: %d cars (wait: %.2fs)\n", len(southSensor.CarIDs), southSensor.WaitTime)
	fmt.Printf("    Total N-S Demand: %.2f\n", northSouthDemand)
	fmt.Printf("  East-West (Phase 1):\n")
	fmt.Printf("    East: %d cars (wait: %.2fs)\n", len(eastSensor.CarIDs), eastSensor.WaitTime)
	fmt.Printf("    West: %d cars (wait: %.2fs)\n", len(westSensor.CarIDs), westSensor.WaitTime)
	fmt.Printf("    Total E-W Demand: %.2f\n", eastWestDemand)
	fmt.Printf("  Current Phase: %d\n", state.CurrentPhase)
	fmt.Printf("  Time in Phase: %.2fs\n\n", state.TimeInPhase)
}

func calculatePhaseDemand(sensor1, sensor2 sensor.SensorDataMessage) float64 {
	cars1 := float64(len(sensor1.CarIDs))
	cars2 := float64(len(sensor2.CarIDs))
	// Weighted by wait time in sensor data
	return (cars1 * sensor1.WaitTime) + (cars2 * sensor2.WaitTime)
}

// Switch to yellow
func triggerYellow(intersectionIndex int) {
	state := &intersectionStates[intersectionIndex]
	state.InYellow = true
	state.TimeInYellow = 0.0
	state.IsRedToGreen = false // This is green->yellow->red

	// Set lights for this intersection to yellow/red
	baseIndex := intersectionIndex * 4
	for i := 0; i < 4; i++ {
		lightIndex := baseIndex + i
		if inCurrentPhase(lightIndex, intersectionIndex) {
			trafficLights[lightIndex].State = "yellow"
		} else {
			trafficLights[lightIndex].State = "red"
		}
	}
}

// Switch the phase from (say) NS -> EW
func switchPhase(intersectionIndex int) {
	state := &intersectionStates[intersectionIndex]
	state.CurrentPhase = (state.CurrentPhase + 1) % 2
	state.TimeInPhase = 0.0

	// Start the red->yellow->green transition
	state.InYellow = true
	state.TimeInYellow = 0.0
	state.IsRedToGreen = true

	// Set new phase lights to yellow (they were red before)
	baseIndex := intersectionIndex * 4
	for i := 0; i < 4; i++ {
		lightIndex := baseIndex + i
		if inCurrentPhase(lightIndex, intersectionIndex) {
			trafficLights[lightIndex].State = "yellow"
		} else {
			trafficLights[lightIndex].State = "red"
		}
	}
}

// Complete the move to green
func switchToGreen(intersectionIndex int) {
	baseIndex := intersectionIndex * 4
	for i := 0; i < 4; i++ {
		lightIndex := baseIndex + i
		if inCurrentPhase(lightIndex, intersectionIndex) {
			trafficLights[lightIndex].State = "green"
		} else {
			trafficLights[lightIndex].State = "red"
		}
	}
}

// Which lights are in the active phase?
func inCurrentPhase(lightIndex, intersectionIndex int) bool {
	state := &intersectionStates[intersectionIndex]
	localIndex := lightIndex % 4
	// Phase 0 => North-South lights (local indices 0,1)
	// Phase 1 => East-West lights   (local indices 2,3)
	if state.CurrentPhase == 0 {
		return localIndex < 2
	}
	return localIndex >= 2
}

// Just an example to retrieve a single stop zone by traffic light index (unused here)
// func getStopZoneForLight(lightIndex int) Vector3 {
// 	// (You already have a global stopZones slice; you can unify if you wish.)
// 	// This function is unused in the recommended-speed approach, but left for reference.
// 	// ...
// 	return Vector3{X: 0, Y: 0, Z: 0}
// }

// Car-spawning logic
func spawnRandomCars() {
	// For example, spawn a car every 3 seconds
	ticker := time.NewTicker(3 * time.Second)
	defer ticker.Stop()

	for i := 0; i < 70; i++ {
		<-ticker.C
		spawnCarRandomLane()
	}
}

func spawnCarRandomLane() {
	sp := spawnPoints[rng.Intn(len(spawnPoints))]
	speed := sp.SpeedMin + rng.Float64()*(sp.SpeedMax-sp.SpeedMin)
	carCounter++
	newCarID := "car" + strconv.FormatInt(carCounter, 10)

	newCar := sensor.CarData{
		ID: newCarID,
		Position: sensor.Vector3{
			X: sp.X,
			Y: 0.25,
			Z: sp.Z,
		},
		Direction: sensor.Vector3{
			X: sp.DirX,
			Y: 0,
			Z: sp.DirZ,
		},
		Speed: speed,
	}

	carsMutex.Lock()
	cars = append(cars, newCar)
	carsMutex.Unlock()
	// fmt.Printf("Spawned %v at (%.2f, %.2f) heading (%.2f, %.2f), speed=%.2f\n",
	// 	newCarID, sp.X, sp.Z, sp.DirX, sp.DirZ, speed)
}

// (Optional) Old approach: Dot-product checks for a near stop zone
func isCarApproachingStopZone(car sensor.CarData, stopZone Vector3) bool {
	threshold := 0.5
	dx := stopZone.X - car.Position.X
	dy := stopZone.Y - car.Position.Y
	dz := stopZone.Z - car.Position.Z
	projection := dx*car.Direction.X + dy*car.Direction.Y + dz*car.Direction.Z

	return (projection > 0 && projection < threshold)
}

// (Optional) Old approach: Simple check for cars in front
func isCarShouldMove(car sensor.CarData) bool {
	threshold := 3.1
	for i := range cars {
		car2 := &cars[i]
		if car2.ID != car.ID &&
			car.Direction.X == car2.Direction.X &&
			car.Direction.Z == car2.Direction.Z {

			dx := car2.Position.X - car.Position.X
			dy := car2.Position.Y - car.Position.Y
			dz := car2.Position.Z - car.Position.Z
			projection := dx*car.Direction.X + dy*car.Direction.Y + dz*car.Direction.Z

			if projection > 0 && projection < threshold {
				return false
			}
		}
	}
	return true
}

// --- Unified Stopping Logic (Recommended) ---

// getRecommendedSpeed returns a float speed after checking the distance
// to the nearest obstacle in front (car or stop zone).
func getRecommendedSpeed(car sensor.CarData, allCars []sensor.CarData, stopZones []StopZone) float64 {
	const minDist = 3.0   // stop if obstacle < 2m
	const maxDist = 10.0  // full speed if obstacle >= 10m
	const baseSpeed = 4.0 // normal cruising speed

	obstacleDist := findClosestObstacleDistance(car, allCars, stopZones)
	if obstacleDist < 0 {
		// No obstacle - return to base speed
		return baseSpeed
	}
	if obstacleDist <= minDist {
		return 0
	}
	if obstacleDist >= maxDist {
		return baseSpeed
	}
	// Gradual slowdown
	ratio := (obstacleDist - minDist) / (maxDist - minDist)
	return baseSpeed * ratio
}

// findClosestObstacleDistance returns the distance to the nearest obstacle
// (a car in front or a stop zone) or -1 if none is in front
func findClosestObstacleDistance(
	car sensor.CarData,
	allCars []sensor.CarData,
	stopZones []StopZone,
) float64 {
	closest := math.Inf(1)

	// 1) Check other cars
	for i := 0; i < len(allCars); i++ {
		other := allCars[i]
		if other.ID == car.ID {
			continue
		}
		// Skip if not same direction, etc.
		sameDir := (car.Direction.X == other.Direction.X &&
			car.Direction.Z == other.Direction.Z)
		if !sameDir {
			continue
		}
		dist := distanceInFront(car, other.Position.X, other.Position.Z)
		if dist >= 0 && dist < closest {
			closest = dist
		}
	}

	// 2) Check stop zones
	for _, sz := range stopZones {
		// If traffic light for that zone is green, skip it
		// (no reason to stop at a green light).
		if trafficLights[sz.TrafficLightIdx].State == "green" {
			continue
		}

		// If red or yellow, treat it as an obstacle:
		dist := distanceInFront(car, sz.Position.X, sz.Position.Z)
		if dist >= 0 && dist < closest {
			closest = dist
		}
	}

	if math.IsInf(closest, 1) {
		return -1
	}
	return closest
}

// distanceInFront checks if (tx, tz) is ahead of the car (dot > 0)
// and returns the distance if so, or -1 if behind.
func distanceInFront(car sensor.CarData, tx, tz float64) float64 {
	dx := tx - car.Position.X
	dz := tz - car.Position.Z
	dot := dx*car.Direction.X + dz*car.Direction.Z

	// If dot <= 0 => behind or perpendicular
	if dot <= 0 {
		return -1
	}
	return math.Sqrt(dx*dx + dz*dz)
}

// -------------- WebSocket Handling --------------

func serveWs(w http.ResponseWriter, r *http.Request) {
	upgrader.CheckOrigin = func(r *http.Request) bool {
		return true
	}

	ws, err := upgrader.Upgrade(w, r, nil)
	if err != nil {
		return
	}
	defer ws.Close()

	clientsMutex.Lock()
	clients[ws] = true
	clientsMutex.Unlock()

	for {
		_, msg, err := ws.ReadMessage()
		if err != nil {
			clientsMutex.Lock()
			delete(clients, ws)
			clientsMutex.Unlock()
			break
		}

		// Handle incoming messages
		var data map[string]interface{}
		if err := json.Unmarshal(msg, &data); err == nil {
			if msgType, ok := data["type"].(string); ok && msgType == "settings" {
				var settings Settings
				if err := json.Unmarshal(msg, &settings); err == nil {
					fmt.Printf("Received settings: %+v\n", settings)
					updateIntersectionSettings(settings)

					// Send response back to client
					response := SettingsResponse{
						Type:    "settings_response",
						Success: true,
						Message: "Settings updated successfully",
					}
					responseJSON, _ := json.Marshal(response)
					ws.WriteMessage(websocket.TextMessage, responseJSON)
				} else {
					fmt.Printf("Error unmarshaling settings: %v\n", err)
				}
			}
		}
	}
}

func updateIntersectionSettings(settings Settings) {
	fmt.Printf("Updating intersection settings with: %+v\n", settings)

	// (Optional) handle GlobalSpeedMultiplier if you want a global effect on speeds
	// for i := range cars {
	//    cars[i].Speed *= settings.GlobalSpeedMultiplier
	// }

	// Update intersection settings
	for i := range intersectionStates {
		if i < len(settings.IntersectionSettings) {
			timing := settings.IntersectionSettings[i]
			intersectionStates[i].MinGreen = timing.MinGreen
			intersectionStates[i].MaxGreen = timing.MaxGreen
			intersectionStates[i].YellowTime = timing.YellowTime
			fmt.Printf("Updated intersection %d: MinGreen=%v, MaxGreen=%v, YellowTime=%v\n",
				i, timing.MinGreen, timing.MaxGreen, timing.YellowTime)
		}
	}
}

// Broadcasting messages to all WebSocket clients
func handleMessages() {
	for {
		msg := <-broadcast
		clientsMutex.Lock()
		for client := range clients {
			err := client.WriteMessage(websocket.TextMessage, msg)
			if err != nil {
				client.Close()
				delete(clients, client)
			}
		}
		clientsMutex.Unlock()
	}
}

func logCarPositions() {
	carsMutex.RLock()
	defer carsMutex.RUnlock()
	
	// for _, car := range cars {
	//     if car.ID == "car1" {
	//         fmt.Printf("Car1 position: (%.2f, %.2f)\n", car.Position.X, car.Position.Z)
	//         break
	//     }
	// }
}

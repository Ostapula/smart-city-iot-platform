package main

import (
	"encoding/json"
	"fmt"
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

type Vector3 struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
}

type CarData struct {
	ID        string  `json:"id"`
	Position  Vector3 `json:"position"`
	Direction Vector3 `json:"direction"`
	Speed     float64 `json:"speed"`
}

type TrafficLight struct {
	Index int    `json:"index"`
	State string `json:"state"` // "red" or "green"
}

// Define phases for each intersection
type Phase struct {
	GreenLights []int
}

// IntersectionState tracks the current phase and timing for each intersection
type IntersectionState struct {
	CurrentPhase  int
	TimeInPhase   float64
	MinGreen      float64
	MaxGreen      float64
	YellowTime    float64
	TimeInYellow  float64
	InYellow      bool
	IsRedToGreen  bool  // New field to track red->yellow->green transition
	Demand        []float64
}

// Create an array of intersection states, one for each intersection
var intersectionStates = [4]IntersectionState{
	{
		CurrentPhase: 0,
		MinGreen:     4.0,
		MaxGreen:     10.0,
		YellowTime:   3.0,
		InYellow:     false,
		IsRedToGreen: false,
		Demand:       []float64{0, 0},
	},
	{
		CurrentPhase: 0,
		MinGreen:     4.0,
		MaxGreen:     10.0,
		YellowTime:   3.0,
		InYellow:     false,
		IsRedToGreen: false,
		Demand:       []float64{0, 0},
	},
	{
		CurrentPhase: 0,
		MinGreen:     4.0,
		MaxGreen:     10.0,
		YellowTime:   3.0,
		InYellow:     false,
		IsRedToGreen: false,
		Demand:       []float64{0, 0},
	},
	{
		CurrentPhase: 0,
		MinGreen:     4.0,
		MaxGreen:     10.0,
		YellowTime:   3.0,
		InYellow:     false,
		IsRedToGreen: false,
		Demand:       []float64{0, 0},
	},
}

// We'll store the "demand" from sensors for each phase
// demand[0] => total demand on North-South
// demand[1] => total demand on East-West
var demand = []float64{0, 0}

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
	{X: -3, Z: -65, DirX: 0, DirZ: 1, SpeedMin: 2, SpeedMax: 4},
	{X: -95, Z: -65, DirX: 0, DirZ: 1, SpeedMin: 2, SpeedMax: 4},

	// EAST (two lanes) -> heading west
	{X: -145, Z: 2, DirX: 1, DirZ: 0, SpeedMin: 2, SpeedMax: 4},
	{X: -145, Z: 95, DirX: 1, DirZ: 0, SpeedMin: 2, SpeedMax: 4},

	// NORTH (two lanes) -> heading south
	{X: 3, Z: 165, DirX: 0, DirZ: -1, SpeedMin: 2, SpeedMax: 4},
	{X: -89, Z: 165, DirX: 0, DirZ: -1, SpeedMin: 2, SpeedMax: 4},

	// WEST (two lanes) -> heading east
	{X: 65, Z: -3, DirX: -1, DirZ: 0, SpeedMin: 2, SpeedMax: 4},
	{X: 65, Z: 89, DirX: -1, DirZ: 0, SpeedMin: 2, SpeedMax: 4},
}

// var timeSinceLastChange = 0.0
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

var cars = []sensor.CarData{
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

var sensorData = make(map[int]sensor.SensorDataMessage)
var sensorDataMutex = sync.Mutex{}
var rng = rand.New(rand.NewSource(time.Now().UnixNano()))

func main() {
	opts := mqtt.NewClientOptions()
	opts.AddBroker("tcp://localhost:1883")
	opts.SetClientID("main_app")
	mqttClient := mqtt.NewClient(opts)
	if token := mqttClient.Connect(); token.Wait() && token.Error() != nil {
		panic(token.Error())
	}
	defer mqttClient.Disconnect(250)

	for i := 0; i < 4; i++ {
		topic := "smartcity/sensors/" + fmt.Sprint(i)
		mqttClient.Subscribe(topic, 0, handleSensorData)
	}

	sensorManager := sensor.NewSensorManager("tcp://localhost:1883", &cars)
	go sensorManager.Start()

	go simulateTrafficFlow()
	go handleMessages()
	go spawnRandomCars()

	router := gin.Default()
	router.Use(cors.Default())
	router.GET("/ws", func(c *gin.Context) {
		serveWs(c.Writer, c.Request)
	})
	router.Static("/static", "../frontend")
	router.GET("/", func(c *gin.Context) {
		c.File("../frontend/index.html")
	})
	router.Run(":8080")
}

func handleSensorData(client mqtt.Client, msg mqtt.Message) {
	var data sensor.SensorDataMessage
	json.Unmarshal(msg.Payload(), &data)

	sensorDataMutex.Lock()
	sensorData[data.SensorIndex] = data
	sensorDataMutex.Unlock()
}

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
		//log.Println(demand)
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
	//log.Println(cars)
	for i := range cars {
		car := &cars[i]
		canMove := true

		canMove = isCarShouldMove(*car)

		trafficLightIndex := getRelevantTrafficLight(*car)
		if trafficLightIndex != -1 {
			light := trafficLights[trafficLightIndex]
			if light.State == "red" || light.State == "yellow" {
				stopZone := getStopZoneForLight(light.Index)
				if isCarApproachingStopZone(*car, stopZone) {
					canMove = false
					//log.Println(car.ID, car.Direction.X, car.Direction.Y, car.Direction.Z, trafficLightIndex, stopZone, canMove)
				}
			}
		}

		if canMove {
			car.Position.X += car.Direction.X * car.Speed * deltaTime
			car.Position.Y += car.Direction.Y * car.Speed * deltaTime
			car.Position.Z += car.Direction.Z * car.Speed * deltaTime

			// intersectionIndex := getIntersectionForPosition(car.Position.X, car.Position.Z)
			// if intersectionIndex != -1 {
			// 	// Attempt random turning
			// 	maybeChangeDirection(car, intersectionIndex)
			// }
		}
	}

	for i := range cars {
		car := &cars[i]
		// Check if car is out of bounds
		if car.Position.Z < -65 || car.Position.Z > 165 || car.Position.X > 65 || car.Position.X < -145 {
			// Pick a random spawn point
			spawnPoint := spawnPoints[rng.Intn(len(spawnPoints))]
			
			// Reset position to spawn point
			car.Position.X = spawnPoint.X
			car.Position.Y = 0 // Assuming Y should be 0 for ground level
			car.Position.Z = spawnPoint.Z
			
			// Set direction based on spawn point
			car.Direction.X = spawnPoint.DirX
			car.Direction.Y = 0 // Assuming Y direction should be 0 for ground level
			car.Direction.Z = spawnPoint.DirZ

			// Set random speed within spawn point's range
			car.Speed = spawnPoint.SpeedMin + rng.Float64()*(spawnPoint.SpeedMax-spawnPoint.SpeedMin)
		}
	}
}

func updateTrafficLights(deltaTime float64) {
	// Update each intersection independently
	for i := range intersectionStates {
		updateSingleIntersection(i, deltaTime)
	}
}

func updateSingleIntersection(intersectionIndex int, deltaTime float64) {
	state := &intersectionStates[intersectionIndex]
	
	// 1. Update timing
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

	// 2. Calculate demand for this intersection
	calculateIntersectionDemand(intersectionIndex)

	// 3. Decision logic
	if state.TimeInPhase < state.MinGreen {
		return
	}

	if state.TimeInPhase >= state.MaxGreen {
		triggerYellow(intersectionIndex)
		return
	}

	currentDemand := state.Demand[state.CurrentPhase]
	otherPhase := (state.CurrentPhase + 1) % 2
	if state.Demand[otherPhase] > currentDemand*1.2 {
		triggerYellow(intersectionIndex)
		return
	}
}

func calculateIntersectionDemand(intersectionIndex int) {
	state := &intersectionStates[intersectionIndex]
	// Reset demand for this intersection
	state.Demand[0] = 0
	state.Demand[1] = 0

	sensorDataMutex.Lock()
	defer sensorDataMutex.Unlock()

	// Calculate demand for sensors associated with this intersection
	baseIndex := intersectionIndex * 4
	
	// North-South demand (sensors 0 and 1 for each intersection)
	northSensor := sensorData[baseIndex]
	southSensor := sensorData[baseIndex+1]
	state.Demand[0] = calculatePhaseDemand(northSensor, southSensor)

	// East-West demand (sensors 2 and 3 for each intersection)
	eastSensor := sensorData[baseIndex+2]
	westSensor := sensorData[baseIndex+3]
	state.Demand[1] = calculatePhaseDemand(eastSensor, westSensor)
}

func calculatePhaseDemand(sensor1, sensor2 sensor.SensorDataMessage) float64 {
	cars1 := float64(len(sensor1.CarIDs))
	cars2 := float64(len(sensor2.CarIDs))
	return (cars1 * sensor1.WaitTime) + (cars2 * sensor2.WaitTime)
}

func triggerYellow(intersectionIndex int) {
	state := &intersectionStates[intersectionIndex]
	state.InYellow = true
	state.TimeInYellow = 0.0
	state.IsRedToGreen = false  // This is a green->yellow->red transition
	
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

func switchPhase(intersectionIndex int) {
	state := &intersectionStates[intersectionIndex]
	state.CurrentPhase = (state.CurrentPhase + 1) % 2
	state.TimeInPhase = 0.0
	
	// Start the red->yellow transition for the new phase
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

func switchToGreen(intersectionIndex int) {
	baseIndex := intersectionIndex * 4
	
	// Set the new phase lights to green
	for i := 0; i < 4; i++ {
		lightIndex := baseIndex + i
		if inCurrentPhase(lightIndex, intersectionIndex) {
			trafficLights[lightIndex].State = "green"
		} else {
			trafficLights[lightIndex].State = "red"
		}
	}
}

func inCurrentPhase(lightIndex, intersectionIndex int) bool {
	state := &intersectionStates[intersectionIndex]
	localIndex := lightIndex % 4 // Convert to local intersection index (0-3)
	
	// Phase 0: North-South (indices 0,1)
	// Phase 1: East-West (indices 2,3)
	if state.CurrentPhase == 0 {
		return localIndex < 2
	}
	return localIndex >= 2
}

func getStopZoneForLight(lightIndex int) Vector3 {
	intersection := lightIndex / 4
	approach := lightIndex % 4

	// default or fallback:
	stopZone := Vector3{X: 0, Y: 0, Z: 0}

	// Example coordinates (adjust them to match your roads):
	switch intersection {
	case 0:
		switch approach {
		case 0: // north
			stopZone = Vector3{X: 3, Y: 0, Z: 10}
		case 1: // south
			stopZone = Vector3{X: -3, Y: 0, Z: -10}
		case 2: // east
			stopZone = Vector3{X: -10, Y: 0, Z: 3}
		case 3: // west
			stopZone = Vector3{X: 10, Y: 0, Z: -3}
		}
	case 1:
		switch approach {
		case 0: // north
			stopZone = Vector3{X: -89, Y: 0, Z: 10}
		case 1: // south
			stopZone = Vector3{X: -95, Y: 0, Z: -10}
		case 2: // east
			stopZone = Vector3{X: -102, Y: 0, Z: 3}
		case 3: // west
			stopZone = Vector3{X: -82, Y: 0, Z: -3}
		}
	case 2:
		switch approach {
		case 0: // north
			stopZone = Vector3{X: -89, Y: 0, Z: 102}
		case 1: // south
			stopZone = Vector3{X: -95, Y: 0, Z: 82}
		case 2: // east
			stopZone = Vector3{X: -102, Y: 0, Z: 95}
		case 3: // west
			stopZone = Vector3{X: -82, Y: 0, Z: 89}
		}
	case 3:
		switch approach {
		case 0: // north
			stopZone = Vector3{X: 3, Y: 0, Z: 102}
		case 1: // south
			stopZone = Vector3{X: -3, Y: 0, Z: 82}
		case 2: // east
			stopZone = Vector3{X: -10, Y: 0, Z: 95}
		case 3: // west
			stopZone = Vector3{X: 10, Y: 0, Z: 89}
		}
	}
	return stopZone
}

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
	// 1) Pick a random spawn point
	sp := spawnPoints[rng.Intn(len(spawnPoints))]

	// 2) Random speed within [SpeedMin..SpeedMax]
	speed := sp.SpeedMin + rng.Float64()*(sp.SpeedMax-sp.SpeedMin)

	// 3) Build a unique car ID
	carCounter++
	newCarID := "car" + strconv.FormatInt(carCounter, 10)

	// 4) Create the new car
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

	// 5) Add to global slice
	cars = append(cars, newCar)

	fmt.Printf("Spawned %v at (%.2f, %.2f) heading (%.2f, %.2f), speed=%.2f\n",
		newCarID, sp.X, sp.Z, sp.DirX, sp.DirZ, speed)
}

func isCarApproachingStopZone(car sensor.CarData, stopZone Vector3) bool {
	threshold := 0.5

	dx := stopZone.X - car.Position.X
	dy := stopZone.Y - car.Position.Y
	dz := stopZone.Z - car.Position.Z

	projection := dx*car.Direction.X + dy*car.Direction.Y + dz*car.Direction.Z

	if projection > 0 && projection < threshold {
		//log.Printf("Aproaching stop zone: %v | %v ", projection > 0, projection < threshold)
		return true
	}
	return false
}

func isCarShouldMove(car sensor.CarData) bool {
	threshold := 3.1
	for i := range cars {
		car2 := &cars[i]
		// Only check cars going in the same direction
		if car2.ID != car.ID && car.Direction.X == car2.Direction.X && car.Direction.Z == car2.Direction.Z {
			dx := car2.Position.X - car.Position.X
			dy := car2.Position.Y - car.Position.Y
			dz := car2.Position.Z - car.Position.Z

			projection := dx*car.Direction.X + dy*car.Direction.Y + dz*car.Direction.Z

			// If there's a car ahead within threshold distance, don't move
			if projection > 0 && projection < threshold {
				return false
			}
		}
	}

	// No cars ahead within threshold, safe to move
	return true
}

func getRelevantTrafficLight(car sensor.CarData) int {
	// 1) Which intersection zone is the car in (or near)?
	intersectionIndex := getIntersectionForPosition(car.Position.X, car.Position.Z)
	if intersectionIndex == -1 {
		return -1 // Car is not in any intersection zone
	}

	// 2) Which approach for that intersection?
	//    0 => north, 1 => south, 2 => east, 3 => west
	approach := -1
	if car.Direction.Z < 0 {
		approach = 0 // north
	} else if car.Direction.Z > 0 {
		approach = 1 // south
	} else if car.Direction.X > 0 {
		approach = 2 // east
	} else if car.Direction.X < 0 {
		approach = 3 // west
	} else {
		return -1
	}

	// 3) Map (intersectionIndex, approach) => trafficLightIndex
	//    intersection i has 4 lights, so we do: i*4 + approach
	return intersectionIndex*4 + approach
}

func getIntersectionForPosition(x, z float64) int {
	for _, zone := range intersectionZones {
		if x >= zone.MinX && x <= zone.MaxX &&
			z >= zone.MinZ && z <= zone.MaxZ {
			return zone.Index
		}
	}
	return -1
}

// func maybeChangeDirection(car *sensor.CarData, intersectionIndex int) {
// 	// If intersection is 0 (the center?), maybe skip turning to show it ignoring the center.
// 	if intersectionIndex == 0 {
// 		return
// 	}

// 	// Let's say 1/4 chance to turn
// 	if rng.Float64() < 0.25 {
// 		// car can choose left, right, or go straight
// 		// Weighted random among: left=33%, right=33%, straight=33%
// 		roll := rng.Float64()
// 		if roll < 0.33 {
// 			turnLeft(car)
// 		} else if roll < 0.66 {
// 			turnRight(car)
// 		} else {
// 			// do nothing (go straight)
// 		}
// 	}
// }

// func turnLeft(car *sensor.CarData) {
// 	// If you treat north=(0,1), east=(1,0), south=(0,-1), west=(-1,0),
// 	// then turning left is basically a rotation of (-dirZ, dirX).
// 	oldX := car.Direction.X
// 	oldZ := car.Direction.Z
// 	car.Direction.X = -oldZ
// 	car.Direction.Z = oldX
// }

// func turnRight(car *sensor.CarData) {
// 	// turning right is (dirZ, -dirX)
// 	oldX := car.Direction.X
// 	oldZ := car.Direction.Z
// 	car.Direction.X = oldZ
// 	car.Direction.Z = -oldX
// }

func serveWs(w http.ResponseWriter, r *http.Request) {
	upgrader.CheckOrigin = func(r *http.Request) bool { return true }

	ws, err := upgrader.Upgrade(w, r, nil)
	if err != nil {
		return
	}
	defer ws.Close()

	clientsMutex.Lock()
	clients[ws] = true
	clientsMutex.Unlock()

	for {
		_, _, err := ws.ReadMessage()
		if err != nil {
			clientsMutex.Lock()
			delete(clients, ws)
			clientsMutex.Unlock()
			break
		}
	}
}

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

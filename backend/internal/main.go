package main

import (
	"encoding/json"
	"fmt"
	"math"
	"net/http"
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

// Define phases: For a simple intersection, we have two phases:
//   - Phase 0: North-South traffic lights (indexes 0 and 1) green, East-West (2 and 3) red
//   - Phase 1: East-West traffic lights (2 and 3) green, North-South (0 and 1) red
type Phase struct {
	GreenLights []int
}

// IntersectionState tracks the current phase and timing
type IntersectionState struct {
	CurrentPhase int
	TimeInPhase  float64
	MinGreen     float64
	MaxGreen     float64
	YellowTime   float64
	TimeInYellow float64
	InYellow     bool
}

// Define our two-phase system for a 4-approach intersection:
var phases = []Phase{
	{GreenLights: []int{0, 1}}, // North & South
	{GreenLights: []int{2, 3}}, // East & West
}

var intersectionState = IntersectionState{
	CurrentPhase: 0,
	// You can tweak these times to suit your simulation:
	MinGreen:   4.0,  // minimum 4s of green
	MaxGreen:   10.0, // maximum 10s of green if demand is high
	YellowTime: 3.0,  // 2s of yellow before switching
	InYellow:   false,
}

// We'll store the "demand" from sensors for each phase
// demand[0] => total demand on North-South
// demand[1] => total demand on East-West
var demand = []float64{0, 0}

var upgrader = websocket.Upgrader{}
var clients = make(map[*websocket.Conn]bool)
var clientsMutex = sync.Mutex{}
var broadcast = make(chan []byte)

//var timeSinceLastChange = 0.0

var trafficLights = []TrafficLight{
	{Index: 0, State: "red"},
	{Index: 1, State: "red"},
	{Index: 2, State: "red"},
	{Index: 3, State: "red"},
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

		// TODO: check for the car in front
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
		}
	}

	for i := range cars {
		car := &cars[i]
		if math.Abs(car.Position.X) > 65 || math.Abs(car.Position.Z) > 65 {
			//log.Printf("Befor reset x: %v | z: %v- \n ", car.Position.X, car.Position.Z)
			if car.Direction.Z != 0 {
				car.Position.Z = car.Position.Z * -0.99
			} else if car.Direction.X != 0 {
				car.Position.X = car.Position.X * -0.99
			} else {
				car.Position.X = car.Position.X * -0.99
				car.Position.Z = car.Position.Z * -0.99
			}
			//log.Printf("After reset x: %v | z: %v- \n ", car.Position.X, car.Position.Z)
		}
	}
}

// func updateTrafficLights(deltaTime float64) {
// 	timeSinceLastChange += deltaTime
// 	maxWaitTime := 0.0
// 	maxCars := 0
// 	sensorIndexToGreen := -1

// 	sensorDataMutex.Lock()
// 	for _, data := range sensorData {
// 		numCars := len(data.CarIDs)
// 		if numCars > 0 {
// 			if data.WaitTime > maxWaitTime || (data.WaitTime == maxWaitTime && numCars > maxCars) {
// 				maxWaitTime = data.WaitTime
// 				maxCars = numCars
// 				sensorIndexToGreen = data.SensorIndex
// 			}
// 		}
// 	}
// 	sensorDataMutex.Unlock()

//		if timeSinceLastChange >= 6.0 && sensorIndexToGreen != -1 {
//			for i := range trafficLights {
//				if trafficLights[i].Index == sensorIndexToGreen {
//					trafficLights[i].State = "green"
//				} else {
//					trafficLights[i].State = "red"
//				}
//			}
//			timeSinceLastChange = 0.0
//		}
//	}
func updateTrafficLights(deltaTime float64) {
	// 1. Update how long we've been in the current phase (or yellow)
	if intersectionState.InYellow {
		intersectionState.TimeInYellow += deltaTime
		// Check if yellow time is over
		if intersectionState.TimeInYellow >= intersectionState.YellowTime {
			// Switch phase
			intersectionState.InYellow = false
			intersectionState.TimeInYellow = 0
			switchPhase()
		}
		return
	} else {
		intersectionState.TimeInPhase += deltaTime
	}

	// 2. Calculate demand from sensors
	calculateDemand()

	// 3. Decide if we can extend green or if we must switch to yellow
	//    - If below MinGreen, stay in green
	//    - If above MaxGreen or demand is higher in the other phase, switch

	if intersectionState.TimeInPhase < intersectionState.MinGreen {
		// Don't switch if we haven't met the minimum green
		return
	}

	// If we've surpassed MaxGreen, time to switch
	if intersectionState.TimeInPhase >= intersectionState.MaxGreen {
		triggerYellow()
		return
	}

	// If the other phase's demand is significantly higher, trigger yellow
	// (You can tune this condition or add thresholds)
	currentDemand := demand[intersectionState.CurrentPhase]
	otherPhase := (intersectionState.CurrentPhase + 1) % len(phases)
	if demand[otherPhase] > currentDemand*1.2 {
		// If the other phase's demand is 20% higher, switch
		triggerYellow()
		return
	}
}

// Trigger the yellow phase
func triggerYellow() {
	intersectionState.InYellow = true
	intersectionState.TimeInYellow = 0.0
	// Set all lights in the current phase to yellow, others to red
	for i := range trafficLights {
		if inCurrentPhase(i) {
			trafficLights[i].State = "yellow"
		} else {
			trafficLights[i].State = "red"
		}
	}
}

// Actually switch to the next phase after yellow completes
func switchPhase() {
	// Move to the next phase
	intersectionState.CurrentPhase = (intersectionState.CurrentPhase + 1) % len(phases)
	intersectionState.TimeInPhase = 0.0

	// Update traffic lights: the new phase’s lights go green, others go red
	for i := range trafficLights {
		if inCurrentPhase(i) {
			trafficLights[i].State = "green"
		} else {
			trafficLights[i].State = "red"
		}
	}
}

// Helper function to check if a traffic light index belongs to the current phase
func inCurrentPhase(lightIndex int) bool {
	for _, idx := range phases[intersectionState.CurrentPhase].GreenLights {
		if idx == lightIndex {
			return true
		}
	}
	return false
}

func getStopZoneForLight(lightIndex int) Vector3 {
	switch lightIndex {
	case 0: // North
		return Vector3{X: 3, Y: 0, Z: 10}
	case 1: // South
		return Vector3{X: -3, Y: 0, Z: -10}
	case 2: // East
		return Vector3{X: -10, Y: 0, Z: 3}
	case 3: // West
		return Vector3{X: 10, Y: 0, Z: -3}
	default:
		return Vector3{X: 0, Y: 0, Z: 0}
	}
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
	threshold := 5.1
	for i := range cars {
		car2 := &cars[i]
		if car2.ID != car.ID && car.Direction == car2.Direction {
			dx := car2.Position.X - car.Position.X
			dy := car2.Position.Y - car.Position.Y
			dz := car2.Position.Z - car.Position.Z

			projection := dx*car.Direction.X + dy*car.Direction.Y + dz*car.Direction.Z
			// if car.ID == "car1" || car.ID == "car5" {
			// 	log.Printf("%v CHECK car %v: %v | %v | dx %v | dy %v | dz %v | projection %v", car, car2, projection > 0, projection < threshold, dx, dy, dz, projection)
			// }
			if projection > 0 && projection < threshold {
				// if car.ID == "car1" || car.ID == "car5" {
				// 	log.Printf("%v aproaching car %v: %v | %v | dx %v | dy %v | dz %v | projection %v", car, car2, projection > 0, projection < threshold, dx, dy, dz, projection)
				// }
				return false
			}
			return true
		}
	}

	return true
}

func getRelevantTrafficLight(car sensor.CarData) int {
	if car.Direction.Z < 0 {
		// Car is moving north
		return 0 // North traffic light
	} else if car.Direction.Z > 0 {
		// Car is moving south
		return 1 // South traffic light
	} else if car.Direction.X > 0 {
		// Car is moving east
		return 2 // East traffic light
	} else if car.Direction.X < 0 {
		// Car is moving west
		return 3 // West traffic light
	} else {
		return -1
	}
}

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

func calculateDemand() {
	// Reset
	demand[0] = 0
	demand[1] = 0

	sensorDataMutex.Lock()
	defer sensorDataMutex.Unlock()

	// For a simple example, we combine wait time and number of cars
	// to come up with a demand metric: demand = #cars * (wait time)
	// You could also do a simpler or more advanced formula.
	for _, data := range sensorData {
		phaseIndex := -1
		if data.SensorIndex == 0 || data.SensorIndex == 1 {
			phaseIndex = 0 // North-South
		} else if data.SensorIndex == 2 || data.SensorIndex == 3 {
			phaseIndex = 1 // East-West
		}
		if phaseIndex != -1 {
			// Example metric: (# cars) * (average wait time)
			numCars := float64(len(data.CarIDs))
			demand[phaseIndex] += numCars * data.WaitTime
		}
	}
}

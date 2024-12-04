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

var upgrader = websocket.Upgrader{}
var clients = make(map[*websocket.Conn]bool)
var clientsMutex = sync.Mutex{}
var broadcast = make(chan []byte)
var timeSinceLastChange = 0.0

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
		Speed:     3,
	},
	{
		ID:        "car2",
		Position:  sensor.Vector3{X: 2, Y: 0.25, Z: 50},
		Direction: sensor.Vector3{X: 0, Y: 0, Z: -1},
		Speed:     3,
	},
	{
		ID:        "car3",
		Position:  sensor.Vector3{X: -50, Y: 0.25, Z: 2},
		Direction: sensor.Vector3{X: 1, Y: 0, Z: 0},
		Speed:     3,
	},
	{
		ID:        "car4",
		Position:  sensor.Vector3{X: 50, Y: 0.25, Z: -2},
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
	for i := range cars {
		car := &cars[i]
		canMove := true

		trafficLightIndex := getRelevantTrafficLight(*car)
		if trafficLightIndex != -1 {
			light := trafficLights[trafficLightIndex]
			if light.State == "red" {
				stopZone := getStopZoneForLight(light.Index)
				if isCarApproachingStopZone(*car, stopZone) {
					canMove = false
					fmt.Println(car.ID, car.Direction.X, car.Direction.Y, car.Direction.Z, trafficLightIndex, stopZone, canMove)
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
		if math.Abs(car.Position.X) > 51 || math.Abs(car.Position.Z) > 51 {
			fmt.Printf("Befor reset x: %v | z: %v- \n ", car.Position.X, car.Position.Z)
			if car.Direction.Z != 0 {
				car.Position.Z = car.Position.Z * -1
			} else if car.Direction.X != 0 {
				car.Position.X = car.Position.X * -1
			} else {
				car.Position.X = car.Position.X * -1
				car.Position.Z = car.Position.Z * -1
			}
			fmt.Printf("After reset x: %v | z: %v- \n ", car.Position.X, car.Position.Z)
		}
	}
}

func updateTrafficLights(deltaTime float64) {
	timeSinceLastChange += deltaTime
	maxWaitTime := 0.0
	maxCars := 0
	sensorIndexToGreen := -1

	sensorDataMutex.Lock()
	for _, data := range sensorData {
		numCars := len(data.CarIDs)
		if numCars > 0 {
			if data.WaitTime > maxWaitTime || (data.WaitTime == maxWaitTime && numCars > maxCars) {
				maxWaitTime = data.WaitTime
				maxCars = numCars
				sensorIndexToGreen = data.SensorIndex
			}
		}
	}
	sensorDataMutex.Unlock()

	if timeSinceLastChange >= 6.0 && sensorIndexToGreen != -1 {
		for i := range trafficLights {
			if trafficLights[i].Index == sensorIndexToGreen {
				trafficLights[i].State = "green"
			} else {
				trafficLights[i].State = "red"
			}
		}
		timeSinceLastChange = 0.0
	}
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
		fmt.Printf("Aproaching stop zone: %v | %v ", projection > 0, projection < threshold)
		return true
	}
	return false
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

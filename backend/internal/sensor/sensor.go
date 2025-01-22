package sensor

import (
	"encoding/json"
	"fmt"
	"math"
	"sync"
	"time"

	mqtt "github.com/eclipse/paho.mqtt.golang"
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

type TrafficSensor struct {
	Index        int
	Position     Vector3
	CarsOnSensor []string // List of car IDs on the sensor
	WaitTime     float64  // Total waiting time in seconds
}

type SensorDataMessage struct {
	SensorIndex int      `json:"sensorIndex"`
	CarIDs      []string `json:"carIDs"`
	WaitTime    float64  `json:"waitTime"`
}

type SensorManager struct {
	sensors        []TrafficSensor
	mqttClient     mqtt.Client
	carsMutex      *sync.RWMutex
	cars           *[]CarData
	lastUpdate     time.Time
	updateInterval time.Duration
}

func NewSensorManager(broker string, cars *[]CarData, carsMutex *sync.RWMutex) *SensorManager {
	manager := &SensorManager{
		sensors: []TrafficSensor{
			{Index: 0, Position: Vector3{X: 3, Y: 0, Z: 13}},     // North
			{Index: 1, Position: Vector3{X: -3, Y: 0, Z: -13}},   // South
			{Index: 2, Position: Vector3{X: -13, Y: 0, Z: 3}},    // East
			{Index: 3, Position: Vector3{X: 13, Y: 0, Z: -3}},    // West
			{Index: 4, Position: Vector3{X: -89, Y: 0, Z: 13}},   // North 1
			{Index: 5, Position: Vector3{X: -95, Y: 0, Z: -13}},  // South 1
			{Index: 6, Position: Vector3{X: -105, Y: 0, Z: 3}},   // East 1
			{Index: 7, Position: Vector3{X: -79, Y: 0, Z: -3}},   // West 1
			{Index: 8, Position: Vector3{X: -89, Y: 0, Z: 105}},  // North 2
			{Index: 9, Position: Vector3{X: -95, Y: 0, Z: 79}},   // South 2
			{Index: 10, Position: Vector3{X: -105, Y: 0, Z: 95}}, // East 2
			{Index: 11, Position: Vector3{X: -79, Y: 0, Z: 89}},  // West 2
			{Index: 12, Position: Vector3{X: 3, Y: 0, Z: 105}},   // North 3
			{Index: 13, Position: Vector3{X: -3, Y: 0, Z: 79}},   // South 3
			{Index: 14, Position: Vector3{X: -13, Y: 0, Z: 95}},  // East 3
			{Index: 15, Position: Vector3{X: 13, Y: 0, Z: 89}},   // West 3
		},
		cars:           cars,
		carsMutex:      carsMutex,
		lastUpdate:     time.Now(),
		updateInterval: 100 * time.Millisecond,
	}

	opts := mqtt.NewClientOptions()
	opts.AddBroker(broker)
	opts.SetClientID("sensor_manager")
	client := mqtt.NewClient(opts)
	if token := client.Connect(); token.Wait() && token.Error() != nil {
		panic(token.Error())
	}
	manager.mqttClient = client

	return manager
}

func (sm *SensorManager) Start() {
	ticker := time.NewTicker(sm.updateInterval)
	defer ticker.Stop()

	for {
		<-ticker.C
		sm.updateSensors()
	}
}

func (sm *SensorManager) updateSensors() {
	sm.carsMutex.RLock()
	defer sm.carsMutex.RUnlock()

	// fmt.Println("\n--- Current Car Positions ---")
	// for _, car := range *sm.cars {
	//     fmt.Printf("Car %s at (%.2f, %.2f)\n", car.ID, car.Position.X, car.Position.Z)
	// }

	deltaTime := time.Since(sm.lastUpdate).Seconds()
	sm.lastUpdate = time.Now()

	// Clear out the old data
	for i := range sm.sensors {
		sm.sensors[i].CarsOnSensor = []string{}
	}

	for _, car := range *sm.cars {
		for i := range sm.sensors {
			sensor := &sm.sensors[i]

			// Debug: Print sensor checking
			// if car.ID == "car1" {
			//     fmt.Printf("\nChecking Sensor %d at (%.2f, %.2f) for car %s at (%.2f, %.2f)\n",
			//         sensor.Index, sensor.Position.X, sensor.Position.Z,
			//         car.ID, car.Position.X, car.Position.Z)
			// }

			// Determine bounding box size by sensor orientation
			var halfX, halfZ float64
			sensorGroup := sensor.Index % 4  // This gives us 0,1,2,3 for any sensor index
			if sensorGroup < 2 {
				// North / South sensors
				halfX = 5  // Width of detection zone
				halfZ = 10 // Length of detection zone
			} else {
				// East / West sensors
				halfX = 10 // Length of detection zone
				halfZ = 5  // Width of detection zone
			}

			// Calculate distances
			distX := math.Abs(car.Position.X - sensor.Position.X)
			distZ := math.Abs(car.Position.Z - sensor.Position.Z)

			// Debug: Print distances and bounds
			// if car.ID == "car1" {
			//     fmt.Printf("  Distance X: %.2f (limit: %.2f)\n", distX, halfX)
			//     fmt.Printf("  Distance Z: %.2f (limit: %.2f)\n", distZ, halfZ)
			// }

			// Check if car is within this rectangular area
			if distX <= halfX && distZ <= halfZ {
				sensor.CarsOnSensor = append(sensor.CarsOnSensor, car.ID)
				sensor.WaitTime += deltaTime
				// if sensor.Index == 4 || sensor.Index == 5 || sensor.Index == 6 || sensor.Index == 7 {
				// 	fmt.Printf("  >>> Car %s DETECTED on sensor %d <<<\n", car.ID, sensor.Index)
				// }
			}
		}
	}

	// Reset wait time to 0 if no cars remain on sensor
	for i := range sm.sensors {
		sensor := &sm.sensors[i]
		if len(sensor.CarsOnSensor) == 0 {
			sensor.WaitTime = 0
		}
	}

	// Log sensor status and publish data
	for _, sensor := range sm.sensors {
		// Only log if there are cars on the sensor or wait time > 0
		// if len(sensor.CarsOnSensor) > 0 || sensor.WaitTime > 0 {
		// 	fmt.Printf("\nSensor %d Status:\n", sensor.Index)
		// 	fmt.Printf("  Position: (%.2f, %.2f)\n", sensor.Position.X, sensor.Position.Z)
		// 	fmt.Printf("  Cars detected: %d\n", len(sensor.CarsOnSensor))
		// 	fmt.Printf("  Car IDs: %v\n", sensor.CarsOnSensor)
		// 	fmt.Printf("  Wait time: %.2f seconds\n", sensor.WaitTime)
		// }

		msg := SensorDataMessage{
			SensorIndex: sensor.Index,
			CarIDs:      sensor.CarsOnSensor,
			WaitTime:    sensor.WaitTime,
		}
		payload, _ := json.Marshal(msg)
		topic := "smartcity/sensors/" + fmt.Sprint(sensor.Index)
		sm.mqttClient.Publish(topic, 0, false, payload)
	}
}

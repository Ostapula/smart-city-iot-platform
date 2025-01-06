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
	carsMutex      sync.Mutex
	cars           []CarData
	lastUpdate     time.Time
	updateInterval time.Duration
}

func NewSensorManager(broker string, cars *[]CarData) *SensorManager {
	manager := &SensorManager{
		sensors: []TrafficSensor{
			{Index: 0, Position: Vector3{X: 3, Y: 0, Z: 13}},   // North
			{Index: 1, Position: Vector3{X: -3, Y: 0, Z: -13}}, // South
			{Index: 2, Position: Vector3{X: -13, Y: 0, Z: 3}},  // East
			{Index: 3, Position: Vector3{X: 13, Y: 0, Z: -3}},  // West
		},
		cars:           *cars,
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
	deltaTime := time.Since(sm.lastUpdate).Seconds()
	sm.lastUpdate = time.Now()

	sm.carsMutex.Lock()
	defer sm.carsMutex.Unlock()

	// Clear out the old data
	for i := range sm.sensors {
		sm.sensors[i].CarsOnSensor = []string{}
	}

	// Instead of a single sensorRadius, we’ll define half-widths
	// for each sensor’s rectangular bounding box:
	//   - North/South sensors:  halfX = 2, halfZ = 5  (4×10 box)
	//   - East/West sensors:    halfX = 5, halfZ = 2 (10×4 box)

	for _, car := range sm.cars {
		for i := range sm.sensors {
			sensor := &sm.sensors[i]

			// Determine bounding box size by sensor index
			var halfX, halfZ float64
			if sensor.Index == 0 || sensor.Index == 1 {
				// North / South
				halfX = 2
				halfZ = 5
			} else {
				// East / West
				halfX = 5
				halfZ = 2
			}

			// Check if car is within this rectangular area
			// ignoring Y dimension (assuming it's approximately 0)
			if math.Abs(car.Position.X-sensor.Position.X) <= halfX &&
				math.Abs(car.Position.Z-sensor.Position.Z) <= halfZ {
				sensor.CarsOnSensor = append(sensor.CarsOnSensor, car.ID)
				sensor.WaitTime += deltaTime
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

	// Publish sensor data over MQTT
	for _, sensor := range sm.sensors {
		msg := SensorDataMessage{
			SensorIndex: sensor.Index,
			CarIDs:      sensor.CarsOnSensor,
			WaitTime:    sensor.WaitTime,
		}
		//fmt.Println(msg)
		payload, _ := json.Marshal(msg)
		topic := "smartcity/sensors/" + fmt.Sprint(sensor.Index)
		sm.mqttClient.Publish(topic, 0, false, payload)
		//log.Println("Sensor:", sensor.Index, "CarsOnSensor:", sensor.CarsOnSensor)
	}
}

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
			{Index: 0, Position: Vector3{X: 6, Y: 0, Z: 12}},   // North
			{Index: 1, Position: Vector3{X: -6, Y: 0, Z: -12}}, // South
			{Index: 2, Position: Vector3{X: -12, Y: 0, Z: 6}},  // East
			{Index: 3, Position: Vector3{X: 12, Y: 0, Z: -6}},  // West
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

	for i := range sm.sensors {
		sm.sensors[i].CarsOnSensor = []string{}
	}

	sensorRadius := 5.0

	for _, car := range sm.cars {
		carPos := car.Position
		for i := range sm.sensors {
			sensor := &sm.sensors[i]
			distance := math.Sqrt(
				math.Pow(carPos.X-sensor.Position.X, 2) +
					math.Pow(carPos.Y-sensor.Position.Y, 2) +
					math.Pow(carPos.Z-sensor.Position.Z, 2),
			)
			if distance <= sensorRadius {
				sensor.CarsOnSensor = append(sensor.CarsOnSensor, car.ID)
				sensor.WaitTime += deltaTime
			}
		}
	}

	for i := range sm.sensors {
		sensor := &sm.sensors[i]
		if len(sensor.CarsOnSensor) == 0 {
			sensor.WaitTime = 0
		}
	}

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
	}
}

func (sm *SensorManager) UpdateCars(cars []CarData) {
	sm.carsMutex.Lock()
	defer sm.carsMutex.Unlock()
	sm.cars = cars
}

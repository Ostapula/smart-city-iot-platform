package main

import (
	"encoding/json"
	"math/rand"
	"time"

	mqtt "github.com/eclipse/paho.mqtt.golang"
)

type SensorData struct {
	DeviceID  string  `json:"deviceId"`
	DataType  string  `json:"dataType"`
	Value     float64 `json:"value"`
	Timestamp int64   `json:"timestamp"`
}

func simulateDevices(client mqtt.Client) {
	go func() {
		for {
			data := SensorData{
				DeviceID:  "trafficSensor1",
				DataType:  "trafficDensity",
				Value:     rand.Float64() * 100, // Random traffic density
				Timestamp: time.Now().Unix(),
			}
			payload, _ := json.Marshal(data)
			client.Publish("smartcity/traffic", 0, false, payload)
			time.Sleep(3 * time.Second)
		}
	}()

	// Repeat similar goroutines for other sensors (weather, air quality, etc.)
	go func() {
		for {
			data := SensorData{
				DeviceID:  "weatherSensor1",
				DataType:  "temperature",
				Value:     rand.Float64()*15 + 10, // Random temp between 10-25°C
				Timestamp: time.Now().Unix(),
			}
			payload, _ := json.Marshal(data)
			client.Publish("smartcity/weather", 0, false, payload)
			time.Sleep(5 * time.Second)
		}
	}()

	go func() {
		for {
			if rand.Intn(100) < 10 { // 10% chance
				data := SensorData{
					DeviceID:  "trafficSensor1",
					DataType:  "status",
					Value:     0, // 0 indicates failure
					Timestamp: time.Now().Unix(),
				}
				payload, _ := json.Marshal(data)
				client.Publish("smartcity/traffic", 0, false, payload)
			}
			time.Sleep(10 * time.Second)
		}
	}()
}

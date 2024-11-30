package main

import (
	"encoding/json"
	"net/http"
	"sync"

	mqtt "github.com/eclipse/paho.mqtt.golang"
	"github.com/gin-contrib/cors"
	"github.com/gin-gonic/gin"
	"github.com/gorilla/websocket"
)

var upgrader = websocket.Upgrader{}
var clients = make(map[*websocket.Conn]bool)
var clientsMutex = sync.Mutex{}
var broadcast = make(chan []byte)

func main() {
	mqttClient := setupMQTT()
	defer mqttClient.Disconnect(250)

	simulateDevices(mqttClient)
	mqttClient.Subscribe("smartcity/traffic", 0, handleMQTTMessage)

	go handleMessages()

	router := gin.Default()

	// Use CORS middleware
	router.Use(cors.Default())

	// WebSocket endpoint
	router.GET("/ws", func(c *gin.Context) {
		serveWs(c.Writer, c.Request)
	})

	// Serve static files under a specific prefix, e.g., '/static'
	router.Static("/static", "../frontend")

	// Serve index.html for the root path
	router.GET("/", func(c *gin.Context) {
		c.File("../frontend/index.html")
	})

	router.Run(":8080")
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

var trafficLightState = "green"

func handleMQTTMessage(client mqtt.Client, msg mqtt.Message) {
	var data SensorData
	json.Unmarshal(msg.Payload(), &data)

	// Apply logic
	if data.DataType == "trafficDensity" {
		if data.Value > 50 {
			trafficLightState = "red"
		} else {
			trafficLightState = "green"
		}

		// Broadcast new state
		state := map[string]string{
			"trafficLightState": trafficLightState,
		}
		stateJSON, _ := json.Marshal(state)
		broadcast <- stateJSON
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

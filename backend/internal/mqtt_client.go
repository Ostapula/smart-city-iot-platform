package main

import (
	"log"

	mqtt "github.com/eclipse/paho.mqtt.golang"
)

func setupMQTT() mqtt.Client {
	opts := mqtt.NewClientOptions().AddBroker("mqtt://127.0.0.1:1883")
	opts.SetClientID("smart-city-backend")
	client := mqtt.NewClient(opts)
	if token := client.Connect(); token.Wait() && token.Error() != nil {
		log.Fatal(token.Error())
	}
	return client
}

run:
	go1.23.2 run ./internal/main.go

dev:
	go1.23.2 run internal/main.go internal/mqtt_client.go internal/device_simulator.go internal/simulator.go

docker-create-mos:
	docker run -it --name mosquitto -p 1883:1883 -v $(pwd)/mosquitto.conf:/mosquitto/config/mosquitto.conf eclipse-mosquitto

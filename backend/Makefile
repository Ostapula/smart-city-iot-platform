run:
	go1.23.2 run ./internal/main.go

docker-create-mos:
	sudo docker run -it --name mosquitto -p 1883:1883 -v $(CURDIR)/mosquitto.conf:/mosquitto/config/mosquitto.conf eclipse-mosquitto

docker-start-mos:
	sudo docker start mosquitto

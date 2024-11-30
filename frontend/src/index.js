import * as THREE from 'three';

let scene, camera, renderer;
let trafficLight;
let websocket;
let cars = [];

init();
animate();
connectWebSocket();

function init() {
    // Scene
    scene = new THREE.Scene();

    // Camera
    camera = new THREE.PerspectiveCamera(
        75, window.innerWidth / window.innerHeight, 0.1, 1000
    );
    camera.position.set(0, 5, 20);

    // Renderer
    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(window.innerWidth, window.innerHeight);
    document.body.appendChild(renderer.domElement);

    // Ambient Light
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
    scene.add(ambientLight);

    // Traffic Light
    const geometry = new THREE.BoxGeometry(1, 3, 1);
    const material = new THREE.MeshLambertMaterial({ color: 0x00ff00 });
    trafficLight = new THREE.Mesh(geometry, material);
    scene.add(trafficLight);

    createCar(new THREE.Vector3(0, 0, 20));
    createCar(new THREE.Vector3(2, 0, 22));

    // Resize Event
    window.addEventListener('resize', onWindowResize, false);
}

function animate() {
    requestAnimationFrame(animate);
    renderer.render(scene, camera);
}

function onWindowResize() {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
}

function connectWebSocket() {
    const socketUrl = `ws://${window.location.hostname}:8080/ws`;
    websocket = new WebSocket(socketUrl);

    websocket.onmessage = function(event) {
        const data = JSON.parse(event.data);
        updateVisualization(data);
        console.log(data);
    };

    websocket.onclose = function() {
        console.log('WebSocket connection closed');
    };
}




function createCar(position) {
    const geometry = new THREE.BoxGeometry(1, 0.5, 2);
    const material = new THREE.MeshLambertMaterial({ color: 0x0000ff });
    const car = new THREE.Mesh(geometry, material);
    car.position.copy(position);
    scene.add(car);
    cars.push(car);
}

function animateCars() {
    cars.forEach(car => {
        if (trafficLight.material.color.getHex() === 0x00ff00 || car.position.z < 0) {
            car.position.z -= 0.1; // Move car
        }
        if (car.position.z < -20) {
            car.position.z = 20;
        }
    });
}

// Call animateCars() in your animate() loop
function animate() {
    requestAnimationFrame(animate);
    animateCars();
    renderer.render(scene, camera);
}

function updateVisualization(data) {
    if (data.deviceId === 'trafficSensor1') {
        const trafficDensity = data.value;
        if (trafficDensity > 50) {
            // Change traffic light to red
            trafficLight.material.color.setHex(0xff0000);
        } else {
            // Change traffic light to green
            trafficLight.material.color.setHex(0x00ff00);
        }
    }

    if (data.trafficLightState) {
        if (data.trafficLightState === 'red') {
            trafficLight.material.color.setHex(0xff0000);
        } else {
            trafficLight.material.color.setHex(0x00ff00);
        }
    }

    if (data.deviceId === 'weatherSensor1') {
        // Adjust scene based on temperature
        if (data.value < 15) {
            scene.background = new THREE.Color(0x87CEFA); // Cool color
        } else {
            scene.background = new THREE.Color(0xFFE4B5); // Warm color
        }
    }

    if (data.deviceId === 'trafficSensor1' && data.dataType === 'status') {
        if (data.value === 0) {
            alert('Traffic Sensor 1 has failed!');
            // Optional: change the sensor's representation to indicate failure
        }
    }
}



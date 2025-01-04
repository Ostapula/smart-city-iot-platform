import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import smallStreet from './objects/small_street.gltf';

let scene, camera, renderer;
let controls;
let trafficLights = [];
let websocket;
let cars = [];

init();
animate();
connectWebSocket();

function init() {
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0xa0a0a0);

    camera = new THREE.PerspectiveCamera(
        45, window.innerWidth / window.innerHeight, 1, 1000
    );
    camera.position.set(0, 50, 100);
    camera.lookAt(0, 0, 0);

    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(window.innerWidth, window.innerHeight);
    document.body.appendChild(renderer.domElement);

    controls = new OrbitControls(camera, renderer.domElement);
    controls.target.set(0, 0, 0);
    controls.update();

    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    scene.add(ambientLight);

    const dirLight = new THREE.DirectionalLight(0xffffff, 0.6);
    dirLight.position.set(100, 100, 100);
    scene.add(dirLight);

    createStreet();
    createStreetLights();
    createSensors();

    window.addEventListener('resize', onWindowResize, false);
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
    };

    websocket.onclose = function() {
        console.log('WebSocket connection closed');
    };
}

function updateVisualization(data) {
    if (data.type === 'update') {
        if (data.cars !== undefined) {
            //console.log(data);
            updateCars(data.cars);
        }
        if (data.trafficLights !== undefined) {
            updateTrafficLights(data.trafficLights);
        }
    }
}

function updateCars(carDataList) {
    let color = 0xff0000
    carDataList.forEach((carData) => {
        let car = cars.find(c => c.id === carData.id);
        //console.log(car);
        if (car) {
            car.mesh.position.set(
                carData.position.x,
                carData.position.y,
                carData.position.z
            );
            car.direction.set(
                carData.direction.x,
                carData.direction.y,
                carData.direction.z
            );
        } else {
            color *= 0x09
            const geometry = new THREE.BoxGeometry(1, 0.5, 2);
            const material = new THREE.MeshLambertMaterial({ color: color });
            const carMesh = new THREE.Mesh(geometry, material);
            carMesh.position.set(
                carData.position.x,
                carData.position.y,
                carData.position.z
            );
            scene.add(carMesh);
            cars.push({
                id: carData.id,
                mesh: carMesh,
                direction: new THREE.Vector3(
                    carData.direction.x,
                    carData.direction.y,
                    carData.direction.z
                ),
                targetPosition: new THREE.Vector3(
                    carData.position.x,
                    carData.position.y,
                    carData.position.z
                ),
            });
        }
    });
}


function updateTrafficLights(trafficLightData) {
    trafficLights.forEach((lightObj) => {
        const lightData = trafficLightData.find(tl => tl.index === lightObj.index);
        if (lightData) {
            const box = lightObj.box;

            const redLight = box.children[0];
            const yellowLight = box.children[1];
            const greenLight = box.children[2];

            redLight.material.emissive.setHex(0x000000);
            yellowLight.material.emissive.setHex(0x000000);
            greenLight.material.emissive.setHex(0x000000);

            if (lightData.state === 'red') {
                redLight.material.emissive.setHex(0xff0000);
                lightObj.state = 'red';
            } else if (lightData.state === 'green') {
                greenLight.material.emissive.setHex(0x00ff00);
                lightObj.state = 'green';
            } else if (lightData.state === 'yellow') {
                yellowLight.material.emissive.setHex(0xffff00);
            }
        }
    });
}

function createStreet() {
    const loader = new GLTFLoader();
    loader.load(
        smallStreet,
        (gltf) => {
            const model = gltf.scene;
            scene.add(model);
            model.position.set(0, 0, 0);
            model.scale.set(2, 2, 2);
        },
        (xhr) => {
            console.log(`${(xhr.loaded / xhr.total) * 100}% loaded`);
        },
        (error) => {
            console.error('An error happened:', error.message || error);
        }
    );

    const groundGeometry = new THREE.PlaneGeometry(500, 500);
    const groundMaterial = new THREE.MeshLambertMaterial({ color: 0x228B22 });
    const ground = new THREE.Mesh(groundGeometry, groundMaterial);
    ground.rotation.x = -Math.PI / 2;
    scene.add(ground);
}

function createStreetLights() {
    const poleHeight = 8;
    const poleRadius = 0.1;
    const poleColor = 0x000000;

    const lightGeometry = new THREE.CylinderGeometry(poleRadius, poleRadius, poleHeight, 16);
    const lightMaterial = new THREE.MeshLambertMaterial({ color: poleColor });

    const positions = [
        { pos: new THREE.Vector3(8, poleHeight / 2, 8), lightPosX: 0, lightPosZ: 0.3, index: 0 },  // North
        { pos: new THREE.Vector3(-8, poleHeight / 2, -8), lightPosX: 0, lightPosZ: -0.3, index: 1 },  // South
        { pos: new THREE.Vector3(-8, poleHeight / 2, 8), lightPosX: -0.3, lightPosZ: 0, index: 2 },  // East 
        { pos: new THREE.Vector3(8, poleHeight / 2, -8), lightPosX: 0.3, lightPosZ: 0, index: 3 },  // West 
    ];

    positions.forEach((position) => {
        const pole = new THREE.Mesh(lightGeometry, lightMaterial);
        pole.position.copy(position.pos);

        const boxGeometry = new THREE.BoxGeometry(0.5, 1.5, 0.5);
        const boxMaterial = new THREE.MeshLambertMaterial({ color: 0x333333 });
        const box = new THREE.Mesh(boxGeometry, boxMaterial);
        box.position.set(0, poleHeight / 2 + 0.75, 0);
        pole.add(box);

        const lightRadius = 0.2;
        const lightPositions = [
            { color: 0xff0000, y: 0.5 },
            { color: 0xffff00, y: 0 },
            { color: 0x00ff00, y: -0.5 }
        ];

        lightPositions.forEach((lightPos) => {
            const sphereGeometry = new THREE.SphereGeometry(lightRadius, 16, 16);
            const sphereMaterial = new THREE.MeshLambertMaterial({ color: lightPos.color });
            const light = new THREE.Mesh(sphereGeometry, sphereMaterial);
            light.position.set(position.lightPosX, lightPos.y, position.lightPosZ);
            box.add(light);
        });

        scene.add(pole);
        trafficLights.push({ 
            pole: pole, 
            box: box, 
            index: position.index,
            state: 'red'
        });

        const stopZoneGeometry = new THREE.BoxGeometry(4, 0.1, 4);
        const stopZoneMaterial = new THREE.MeshBasicMaterial({
            color: 0xff0000,
            transparent: true,
            opacity: 0.5,
        });
        const stopZone = new THREE.Mesh(stopZoneGeometry, stopZoneMaterial);

        // Set the position of the stop zone based on the traffic light index
        let stopZonePosition = new THREE.Vector3();
        switch (position.index) {
            case 0: // North
                stopZonePosition.set(3, 1, 10);
                break;
            case 1: // South
                stopZonePosition.set(-3, 1, -10);
                break;
            case 2: // East
                stopZonePosition.set(-10, 1, 3);
                break;
            case 3: // West
                stopZonePosition.set(10, 1, -3);
                break;
        }

        // Adjust stop zone orientation and position based on direction
        if (position.index === 0 || position.index === 1) {
            stopZone.rotation.y = 0; // No rotation needed
        } else if (position.index === 2 || position.index === 3) {
            stopZone.rotation.y = Math.PI / 2;
        }

        stopZone.position.copy(stopZonePosition);
        scene.add(stopZone);
    });
}

function animate() {
    requestAnimationFrame(animate);

    cars.forEach((car) => {
        if (car.targetPosition) {
            car.mesh.position.lerp(car.mesh.position, 0.1);
        }
    });

    if (controls) controls.update();

    renderer.render(scene, camera);
}

function createSensors() {
    const sensorGeometry = new THREE.SphereGeometry(1, 16, 16);
    const sensorMaterial = new THREE.MeshBasicMaterial({
        color: 0x0000ff,
        transparent: true,
        opacity: 0.5,
    });

    const sensorRangeGeometry = new THREE.CylinderGeometry(5, 5, 0.1, 32);
    const sensorRangeMaterial = new THREE.MeshBasicMaterial({
        color: 0x0000ff,
        transparent: true,
        opacity: 0.2,
    });

    const sensorPositions = [
        { position: new THREE.Vector3(6, 0.5, 12), index: 0 },   // North
        { position: new THREE.Vector3(-6, 0.5, -12), index: 1 }, // South
        { position: new THREE.Vector3(-12, 0.5, 6), index: 2 },  // East
        { position: new THREE.Vector3(12, 0.5, -6), index: 3 },  // West
    ];

    sensorPositions.forEach((sensor) => {
        const sensorMesh = new THREE.Mesh(sensorGeometry, sensorMaterial);
        sensorMesh.position.copy(sensor.position);
        scene.add(sensorMesh);

        const sensorRangeMesh = new THREE.Mesh(sensorRangeGeometry, sensorRangeMaterial);
        sensorRangeMesh.position.copy(sensor.position);
        //sensorRangeMesh.rotation.x = Math.PI / 2;
        scene.add(sensorRangeMesh);
    });
}


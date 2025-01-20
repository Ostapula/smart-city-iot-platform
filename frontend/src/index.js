import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import smallStreet from './objects/city_4_intersections.gltf';

let scene, camera, renderer;
let controls;
let trafficLights = [];
let websocket;
let cars = [];
let globalSpeedMultiplier = 1.0;
let intersectionSettings = [
    { minGreen: 4.0, maxGreen: 10.0, yellowTime: 2.0 },
    { minGreen: 4.0, maxGreen: 10.0, yellowTime: 2.0 },
    { minGreen: 4.0, maxGreen: 10.0, yellowTime: 2.0 },
    { minGreen: 4.0, maxGreen: 10.0, yellowTime: 2.0 }
];

init();
animate();
connectWebSocket();
initializeControls();

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

    const ambientLight = new THREE.AmbientLight(0xffffff, 0.77);
    scene.add(ambientLight);

    const dirLight = new THREE.DirectionalLight(0xffffff, 0.77);
    dirLight.position.set(-100, 100, 100);
    scene.add(dirLight);

    createStreet();
    createStreetLights();
    createSensors();
    spawnPoints();

    window.addEventListener('resize', onWindowResize, false);
}

function spawnPoints() {
    //south
    const geometry = new THREE.BoxGeometry(1, 0.5, 2);
    const material = new THREE.MeshLambertMaterial({ color: 0xFF00FF });
    const carMesh = new THREE.Mesh(geometry, material);
    carMesh.position.set(-3, 0.25, -65);
    scene.add(carMesh);

    const geometry1 = new THREE.BoxGeometry(1, 0.5, 2);
    const material1 = new THREE.MeshLambertMaterial({ color: 0xFF00FF });
    const carMesh1 = new THREE.Mesh(geometry1, material1);
    carMesh1.position.set(-95, 0.25, -65);
    scene.add(carMesh1);

    //east
    const geometry2 = new THREE.BoxGeometry(1, 0.5, 2);
    const material2 = new THREE.MeshLambertMaterial({ color: 0xFF00FF });
    const carMesh2 = new THREE.Mesh(geometry2, material2);
    carMesh2.position.set(-145, 0.25, 2);
    scene.add(carMesh2);

    const geometry3 = new THREE.BoxGeometry(1, 0.5, 2);
    const material3 = new THREE.MeshLambertMaterial({ color: 0xFF00FF });
    const carMesh3 = new THREE.Mesh(geometry3, material3);
    carMesh3.position.set(-145, 0.25, 95);
    scene.add(carMesh3);

    //north
    const geometry4 = new THREE.BoxGeometry(1, 0.5, 2);
    const material4 = new THREE.MeshLambertMaterial({ color: 0xFF00FF });
    const carMesh4 = new THREE.Mesh(geometry4, material4);
    carMesh4.position.set(3, 0.25, 165);
    scene.add(carMesh4);

    const geometry5 = new THREE.BoxGeometry(1, 0.5, 2);
    const material5 = new THREE.MeshLambertMaterial({ color: 0xFF00FF });
    const carMesh5 = new THREE.Mesh(geometry5, material5);
    carMesh5.position.set(-89, 0.25, 165);
    scene.add(carMesh5);

    //west
    const geometry6 = new THREE.BoxGeometry(1, 0.5, 2);
    const material6 = new THREE.MeshLambertMaterial({ color: 0xFF00FF });
    const carMesh6 = new THREE.Mesh(geometry6, material6);
    carMesh6.position.set(65, 0.25, -3);
    scene.add(carMesh6);

    const geometry7 = new THREE.BoxGeometry(1, 0.5, 2);
    const material7 = new THREE.MeshLambertMaterial({ color: 0xFF00FF });
    const carMesh7 = new THREE.Mesh(geometry7, material7);
    carMesh7.position.set(65, 0.25, 89);
    scene.add(carMesh7);
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
        if (data.type === 'update') {
            updateVisualization(data);
        } else if (data.type === 'settings_response') {
            console.log('Settings updated:', data);
        }
    };

    websocket.onclose = function() {
        console.log('WebSocket connection closed');
    };

    websocket.onerror = function(error) {
        console.error('WebSocket error:', error);
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
        if (car) {
            const adjustedCarData = {
                ...carData,
                speed: carData.speed * globalSpeedMultiplier
            };
            car.mesh.position.set(
                adjustedCarData.position.x,
                adjustedCarData.position.y,
                adjustedCarData.position.z
            );
            car.direction.set(
                adjustedCarData.direction.x,
                adjustedCarData.direction.y,
                adjustedCarData.direction.z
            );
            // Update sprite position
            car.label.position.set(
                adjustedCarData.position.x,
                adjustedCarData.position.y + 2,
                adjustedCarData.position.z
            );
        } else {
            color *= 0x09
            const geometry = new THREE.BoxGeometry(1, 0.5, 2);
            const material = new THREE.MeshLambertMaterial({ color: color });
            const carMesh = new THREE.Mesh(geometry, material);
            const adjustedCarData = {
                ...carData,
                speed: carData.speed * globalSpeedMultiplier
            };
            carMesh.position.set(
                adjustedCarData.position.x,
                adjustedCarData.position.y,
                adjustedCarData.position.z
            );

            // Create text sprite
            const canvas = document.createElement('canvas');
            const context = canvas.getContext('2d');
            canvas.width = 256;
            canvas.height = 256;
            context.font = 'Bold 60px Arial';
            context.fillStyle = 'white';
            context.textAlign = 'center';
            context.fillText(carData.id, 128, 128);

            const texture = new THREE.CanvasTexture(canvas);
            const spriteMaterial = new THREE.SpriteMaterial({ map: texture });
            const sprite = new THREE.Sprite(spriteMaterial);
            sprite.position.set(
                adjustedCarData.position.x,
                adjustedCarData.position.y + 2,
                adjustedCarData.position.z
            );
            sprite.scale.set(2, 2, 1);
            
            scene.add(carMesh);
            scene.add(sprite);
            
            cars.push({
                id: carData.id,
                mesh: carMesh,
                label: sprite,
                direction: new THREE.Vector3(
                    adjustedCarData.direction.x,
                    adjustedCarData.direction.y,
                    adjustedCarData.direction.z
                ),
                targetPosition: new THREE.Vector3(
                    adjustedCarData.position.x,
                    adjustedCarData.position.y,
                    adjustedCarData.position.z
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

            // Update state sphere color
            if (lightData.state === 'red') {
                redLight.material.emissive.setHex(0xff0000);
                lightObj.stateSphere.material.color.setHex(0xff0000);
                lightObj.state = 'red';
            } else if (lightData.state === 'green') {
                greenLight.material.emissive.setHex(0x00ff00);
                lightObj.stateSphere.material.color.setHex(0x00ff00);
                lightObj.state = 'green';
            } else if (lightData.state === 'yellow') {
                yellowLight.material.emissive.setHex(0xffff00);
                lightObj.stateSphere.material.color.setHex(0xffff00);
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
        { pos: new THREE.Vector3(-84, poleHeight / 2, 8), lightPosX: 0, lightPosZ: 0.3, index: 4 },  // North 1
        { pos: new THREE.Vector3(-100, poleHeight / 2, -8), lightPosX: 0, lightPosZ: -0.3, index: 5 },  // South 1
        { pos: new THREE.Vector3(-100, poleHeight / 2, 8), lightPosX: -0.3, lightPosZ: 0, index: 6 },  // East 1
        { pos: new THREE.Vector3(-84, poleHeight / 2, -8), lightPosX: 0.3, lightPosZ: 0, index: 7 },  // West 1
        { pos: new THREE.Vector3(-84, poleHeight / 2, 100), lightPosX: 0, lightPosZ: 0.3, index: 8 },  // North 2
        { pos: new THREE.Vector3(-100, poleHeight / 2, 84), lightPosX: 0, lightPosZ: -0.3, index: 9 },  // South 2
        { pos: new THREE.Vector3(-100, poleHeight / 2, 100), lightPosX: -0.3, lightPosZ: 0, index: 10 },  // East 2
        { pos: new THREE.Vector3(-84, poleHeight / 2, 84), lightPosX: 0.3, lightPosZ: 0, index: 11 },  // West 2
        { pos: new THREE.Vector3(8, poleHeight / 2, 100), lightPosX: 0, lightPosZ: 0.3, index: 12 },  // North 3
        { pos: new THREE.Vector3(-8, poleHeight / 2, 84), lightPosX: 0, lightPosZ: -0.3, index: 13 },  // South 3
        { pos: new THREE.Vector3(-8, poleHeight / 2, 100), lightPosX: -0.3, lightPosZ: 0, index: 14 },  // East 3
        { pos: new THREE.Vector3(8, poleHeight / 2, 84), lightPosX: 0.3, lightPosZ: 0, index: 15 },  // West 3
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

        // Add large sphere above traffic light
        const stateSphereGeometry = new THREE.SphereGeometry(1, 32, 32);
        const stateSphereMaterial = new THREE.MeshBasicMaterial({ color: 0xff0000 });
        const stateSphere = new THREE.Mesh(stateSphereGeometry, stateSphereMaterial);
        stateSphere.position.set(0, poleHeight + 2, 0); // Position above the traffic light
        pole.add(stateSphere);

        scene.add(pole);
        trafficLights.push({ 
            pole: pole, 
            box: box,
            stateSphere: stateSphere, // Add reference to state sphere
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
            case 4: // North 1
                stopZonePosition.set(-89, 1, 10);
                break;
            case 5: // South 1
                stopZonePosition.set(-95, 1, -10);
                break;
            case 6: // East 1
                stopZonePosition.set(-102, 1, 3);
                break;
            case 7: // West 1
                stopZonePosition.set(-82, 1, -3);
                break;
            case 8: // North 2
                stopZonePosition.set(-89, 1, 102);
                break;
            case 9: // South 2
                stopZonePosition.set(-95, 1, 82);
                break;
            case 10: // East 2
                stopZonePosition.set(-102, 1, 95);
                break;
            case 11: // West 2
                stopZonePosition.set(-82, 1, 89);
                break;
            case 12: // North 3
                stopZonePosition.set(3, 1, 102);
                break;
            case 13: // South 3
                stopZonePosition.set(-3, 1, 82);
                break;
            case 14: // East 3
                stopZonePosition.set(-10, 1, 95);
                break;
            case 15: // West 3
                stopZonePosition.set(10, 1, 89);
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
    // Replace the sphere geometry with a box geometry for the sensor itself
    // Adjust the dimensions as needed.
    const sensorGeometry = new THREE.BoxGeometry(1, 1, 2);
    const sensorMaterial = new THREE.MeshBasicMaterial({
        color: 0x0000ff,
        transparent: true,
        opacity: 0.5,
    });

    // Replace the cylinder geometry with a box geometry for the sensor's range
    // Adjust the dimensions for your desired rectangular footprint.
    const sensorRangeGeometryEastWest = new THREE.BoxGeometry(10, 0.1, 4);
    const sensorRangeGeometryNorthSouth = new THREE.BoxGeometry(4, 0.1, 10);
    const sensorRangeMaterial = new THREE.MeshBasicMaterial({
        color: 0x0000ff,
        transparent: true,
        opacity: 0.2,
    });

    // Update sensor positions accordingly
    const sensorPositions = [
        { position: new THREE.Vector3(3, 0.5, 13), index: 0 },   // North
        { position: new THREE.Vector3(-3, 0.5, -13), index: 1 }, // South
        { position: new THREE.Vector3(-13, 0.5, 3), index: 2 },  // East
        { position: new THREE.Vector3(13, 0.5, -3), index: 3 },  // West
        { position: new THREE.Vector3(-89, 0.5, 13), index: 4 },   // North 1
        { position: new THREE.Vector3(-95, 0.5, -13), index: 5 }, // South 1
        { position: new THREE.Vector3(-105, 0.5, 3), index: 6 },  // East 1
        { position: new THREE.Vector3(-79, 0.5, -3), index: 7 },  // West 1
        { position: new THREE.Vector3(-89, 0.5, 105), index: 8 },   // North 2
        { position: new THREE.Vector3(-95, 0.5, 79), index: 9 }, // South 2
        { position: new THREE.Vector3(-105, 0.5, 95), index: 10 },  // East 2
        { position: new THREE.Vector3(-79, 0.5, 89), index: 11 },  // West 2
        { position: new THREE.Vector3(3, 0.5, 105), index: 12 },   // North 3
        { position: new THREE.Vector3(-3, 0.5, 79), index: 13 }, // South 3
        { position: new THREE.Vector3(-13, 0.5, 95), index: 14 },  // East 3
        { position: new THREE.Vector3(13, 0.5, 89), index: 15 },  // West 3
    ];

    sensorPositions.forEach((sensor) => {
        const sensorMesh = new THREE.Mesh(sensorGeometry, sensorMaterial);
        sensorMesh.position.copy(sensor.position);
        scene.add(sensorMesh);
        let sensorRangeMesh;
        if (sensor.index == 0 || sensor.index == 1 || sensor.index == 4 || sensor.index == 5 || sensor.index == 8 || sensor.index == 9 || sensor.index == 12 || sensor.index == 13) {
            sensorRangeMesh = new THREE.Mesh(sensorRangeGeometryNorthSouth, sensorRangeMaterial);
        } else if (sensor.index == 2 || sensor.index == 3 || sensor.index == 6 || sensor.index == 7 || sensor.index == 9 || sensor.index == 10 || sensor.index == 11 || sensor.index == 14 || sensor.index == 15) {
            sensorRangeMesh = new THREE.Mesh(sensorRangeGeometryEastWest, sensorRangeMaterial);
        }   else {
            sensorRangeMesh = new THREE.Mesh(sensorGeometry, sensorMaterial);
        }
        sensorRangeMesh.position.copy(sensor.position);
        // If you want to rotate it so that the longer side faces a certain direction, adjust here.
        // For example, rotate around Y by 90 degrees to change the long edge orientation:
        // sensorRangeMesh.rotation.y = Math.PI / 2;
        scene.add(sensorRangeMesh);
    });
}

function initializeControls() {
    // Car speed control
    const carSpeedSlider = document.getElementById('carSpeed');
    const carSpeedValue = document.getElementById('carSpeedValue');
    
    carSpeedSlider.addEventListener('input', (e) => {
        globalSpeedMultiplier = parseFloat(e.target.value);
        carSpeedValue.textContent = `${globalSpeedMultiplier}x`;
    });

    // Create intersection controls
    const intersectionControlsDiv = document.getElementById('intersectionControls');
    
    for (let i = 0; i < 4; i++) {
        const intersectionDiv = document.createElement('div');
        intersectionDiv.className = 'intersection-controls';
        intersectionDiv.innerHTML = `
            <h3>Intersection ${i + 1}</h3>
            <div class="control-group">
                <label>Min Green Time
                    <span class="value-display" id="minGreen${i}Value">${intersectionSettings[i].minGreen}s</span>
                </label>
                <input type="range" id="minGreen${i}" min="2" max="10" step="0.5" value="${intersectionSettings[i].minGreen}">
            </div>
            <div class="control-group">
                <label>Max Green Time
                    <span class="value-display" id="maxGreen${i}Value">${intersectionSettings[i].maxGreen}s</span>
                </label>
                <input type="range" id="maxGreen${i}" min="5" max="30" step="0.5" value="${intersectionSettings[i].maxGreen}">
            </div>
            <div class="control-group">
                <label>Yellow Time
                    <span class="value-display" id="yellow${i}Value">${intersectionSettings[i].yellowTime}s</span>
                </label>
                <input type="range" id="yellow${i}" min="1" max="5" step="0.5" value="${intersectionSettings[i].yellowTime}">
            </div>
        `;
        intersectionControlsDiv.appendChild(intersectionDiv);

        // Add listeners for this intersection's controls
        const minGreenSlider = document.getElementById(`minGreen${i}`);
        const maxGreenSlider = document.getElementById(`maxGreen${i}`);
        const yellowSlider = document.getElementById(`yellow${i}`);

        const minGreenValue = document.getElementById(`minGreen${i}Value`);
        const maxGreenValue = document.getElementById(`maxGreen${i}Value`);
        const yellowValue = document.getElementById(`yellow${i}Value`);

        minGreenSlider.addEventListener('input', (e) => {
            const value = parseFloat(e.target.value);
            minGreenValue.textContent = `${value}s`;
            intersectionSettings[i].minGreen = value;
            // Ensure min doesn't exceed max
            if (value > maxGreenSlider.value) {
                maxGreenSlider.value = value;
                maxGreenValue.textContent = `${value}s`;
                intersectionSettings[i].maxGreen = value;
            }
        });

        maxGreenSlider.addEventListener('input', (e) => {
            const value = parseFloat(e.target.value);
            maxGreenValue.textContent = `${value}s`;
            intersectionSettings[i].maxGreen = value;
            // Ensure max doesn't go below min
            if (value < minGreenSlider.value) {
                minGreenSlider.value = value;
                minGreenValue.textContent = `${value}s`;
                intersectionSettings[i].minGreen = value;
            }
        });

        yellowSlider.addEventListener('input', (e) => {
            const value = parseFloat(e.target.value);
            yellowValue.textContent = `${value}s`;
            intersectionSettings[i].yellowTime = value;
        });
    }

    // Apply changes button
    document.getElementById('applyChanges').addEventListener('click', () => {
        console.log('Current settings before sending:', intersectionSettings);
        sendSettingsToBackend();
    });
}

function sendSettingsToBackend() {
    if (websocket.readyState === WebSocket.OPEN) {
        const settings = {
            type: 'settings',
            globalSpeedMultiplier: parseFloat(globalSpeedMultiplier),
            intersectionSettings: intersectionSettings.map(setting => ({
                minGreen: parseFloat(setting.minGreen),
                maxGreen: parseFloat(setting.maxGreen),
                yellowTime: parseFloat(setting.yellowTime)
            }))
        };
        console.log('Sending settings to backend:', settings);
        websocket.send(JSON.stringify(settings));
    } else {
        console.error('WebSocket is not connected');
    }
}

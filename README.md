# 🚀 Smart City IoT Platform

A platform for simulating and managing smart city traffic using IoT-based technologies. The project features a complex traffic system with multiple phases and intersections to reflect real-world conditions.

---

## 📌 TODO List

- [ ] Update the map to increase the number of lanes at each intersection.
- [ ] Expand traffic phases to **8**, making intersections more complex and realistic.
- [ ] Enhance the algorithm to support **8 phases**.
- [ ] Fix car left-turn logic.

---

## 🛠️ Getting Started

### 📦 Prerequisites
Ensure you have the following installed:
- **Docker**
- **Node.js** (for frontend)
- **Make** (for running backend commands)
- **Go Lang**

### 🚀 Installation & Setup
Follow these steps to start the project:

#### 1️⃣ Start MQTT Broker (Mosquitto)
```sh
cd backend/
make docker-create-mos
```

#### 2️⃣ Run the Backend (in backend dir)
```sh
make run
```

#### 3️⃣ Start the Frontend
```sh
cd frontend/
npm start
```
🖥️ The frontend should start on **port 1234**.

---

The project is not dockerized because it is not yet complete.

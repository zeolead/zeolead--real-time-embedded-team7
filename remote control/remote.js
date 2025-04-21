let socket = new WebSocket("ws://192.168.1.254:8765"); // Raspberry PI's IP
let testStartTime = null;

socket.onopen = () => {
    document.getElementById("status").textContent = "✅ WebSocket connected";
};

socket.onmessage = (event) => {
    const now = Date.now();
    const message = event.data;

    // test
    if (message.includes("test")) {
        if (testStartTime) {
            const delay = now - testStartTime;
            document.getElementById("latency").textContent = `ping：${delay} ms`;
            testStartTime = null;
        }
    }

    document.getElementById("status").textContent = "received：" + message;
};

socket.onerror = (error) => {
    document.getElementById("status").textContent = +error;
};

socket.onclose = () => {
    document.getElementById("status").textContent = "disconnected";
};

function sendCommand(cmd) {
    if (socket.readyState === WebSocket.OPEN) {
        if (cmd === "test") {
            testStartTime = Date.now(); // 
        }
        socket.send(cmd);
    } else {
        alert("WebSocket disconnected！");
    }
}

document.getElementById("FWD").onclick = () => sendCommand("forward");
document.getElementById("BWD").onclick = () => sendCommand("backward");
document.getElementById("LEFT").onclick = () => sendCommand("left");
document.getElementById("RIGHT").onclick = () => sendCommand("right");
document.getElementById("STOP").onclick = () => sendCommand("stop");
document.getElementById("TEST").onclick = () => sendCommand("test");

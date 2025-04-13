let socket = new WebSocket("ws://192.168.1.254:8765"); // 树莓派的 IP
let testStartTime = null;

socket.onopen = () => {
    document.getElementById("status").textContent = "✅ WebSocket 已连接";
};

socket.onmessage = (event) => {
    const now = Date.now();
    const message = event.data;

    // 检测是否是测试响应
    if (message.includes("test")) {
        if (testStartTime) {
            const delay = now - testStartTime;
            document.getElementById("latency").textContent = `📡 时延：${delay} ms`;
            testStartTime = null;
        }
    }

    document.getElementById("status").textContent = "收到回应：" + message;
};

socket.onerror = (error) => {
    document.getElementById("status").textContent = "❌ 错误：" + error;
};

socket.onclose = () => {
    document.getElementById("status").textContent = "🔌 已断开连接";
};

function sendCommand(cmd) {
    if (socket.readyState === WebSocket.OPEN) {
        if (cmd === "test") {
            testStartTime = Date.now(); // 记录起始时间
        }
        socket.send(cmd);
    } else {
        alert("WebSocket 未连接！");
    }
}

// 绑定按钮事件
document.getElementById("FWD").onclick = () => sendCommand("forward");
document.getElementById("BWD").onclick = () => sendCommand("backward");
document.getElementById("LEFT").onclick = () => sendCommand("left");
document.getElementById("RIGHT").onclick = () => sendCommand("right");
document.getElementById("STOP").onclick = () => sendCommand("stop");
document.getElementById("TEST").onclick = () => sendCommand("test");

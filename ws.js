let ws = new WebSocket('ws://localhost:5566');
let startTime; // Variable to store the start time
// 開啟後執行的動作，指定一個 function 會在連結 WebSocket 後執行
var data = JSON.stringify({"cmd": "AzAlt","content": ""});
let az = 0, alt = 0;
let offset_pitch = 0.0, offset_yaw = 0.0;
ws.onopen = () => {
    console.log('open connection');
    startTime = new Date();
    const requestInfo = setInterval(() => {
        ws.send(data);
    }, 100);
};

// 關閉後執行的動作，指定一個 function 會在連結中斷後執行
ws.onclose = () => {
    console.log('close connection');
};

ws.onmessage = event => {
    let arr = JSON.parse(event.data);
    console.log(event.data);
    //document.getElementById("Log").innerHTML = arr.content;
    const info = arr.content.split(",");
    az = parseFloat(info[0]);
    alt = parseFloat(info[1]);
    if (arr.cmd === "AzAlt") 
    {
        document.getElementById("az").innerHTML = `Az (deg): <span class="parameter-value">${az.toFixed(3)}</span>`;
        document.getElementById("alt").innerHTML = `Alt (deg): <span class="parameter-value">${alt.toFixed(3)}</span>`;
    }
};


const syncButton = document.getElementById('sync-btn');
const gotoButton = document.getElementById('goto-btn');

const sendMessage = (message, content) => {
    // Check if WebSocket connection is open
    if (ws.readyState === WebSocket.OPEN) {
        // Send the message
        ws.send(JSON.stringify({ cmd: message, content: content }));
        document.getElementById("actionLog").innerHTML = `Action: ${message} ${content}`;
    } else {
        console.log("WebSocket connection is not open.");
    }
};

function sendCommand(command, content) {
    if (ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ cmd: command, content: content }));
        //document.getElementById("actionLog").innerHTML = `Action: ${command} ${content}`;
    } else {
        console.log("WebSocket connection is not open.");
    }
}

function syncData() {
    offset_pitch = alt - pitch;
    offset_yaw = az - yaw;
    console.log(`Offset:Pitch: ${offset_pitch}, Yaw: ${offset_yaw}`);
}

function gotoData() {   
    let targetAz = yaw + offset_yaw;
    while (targetAz < 0) targetAz += 360.0;
    while (targetAz >= 360) targetAz -= 360.0;
    let targetAlt = pitch + offset_pitch;
    if (targetAlt >= 90) targetAlt = 90.0;
    sendCommand("HGoto", `${targetAlt},${targetAz}`);
    console.log(`Target:Az: ${targetAz}, Alt: ${targetAlt}`);
}

// Event listeners for buttons
connectButton.addEventListener('click', connectToArduino);
syncButton.addEventListener('click', syncData);
gotoButton.addEventListener('click', gotoData);

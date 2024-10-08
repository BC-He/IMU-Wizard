const connectButton = document.getElementById('connect-btn');
const imuOutput = document.getElementById('IMU-output');

let port;
let reader;
let roll = 0.0, pitch = 0.0, yaw = 0.0; // Array to store the parsed data
let altitude = 0.0, azimuth = 0.0;
async function connectToArduino() {
    try {
        // Request a port and open a connection
        port = await navigator.serial.requestPort();
        await port.open({ baudRate: 115200 });
        const decoder = new TextDecoderStream();
        const inputDone = port.readable.pipeTo(decoder.writable);
        const inputStream = decoder.readable;
        reader = inputStream.getReader();

        // Begin the loop to read incoming serial data
        readLoop();
    } catch (error) {
        console.error('Connection error:', error);
    }
}

async function readLoop() {
    let buffer = '';

    while (true) {
        try {
            const { value, done } = await reader.read();
            if (done) {
                console.log('[Reader] Done', done);
                reader.releaseLock();
                break;
            }
            if (value) {
                buffer += value; // Append the data to the buffer

                // Try to parse each complete line of JSON
                const lines = buffer.split('\n');
                buffer = lines.pop(); // Retain any incomplete line

                for (const line of lines) {
                    try {
                        const jsonData = JSON.parse(line.trim()); // Trim to handle extra spaces
                        displayParsedData(jsonData);
                    } catch (e) {
                        console.error('Error parsing JSON:', e);
                    }
                }
            }
        } catch (error) {
            console.error('Read error:', error);
        }
    }
}

function displayParsedData(jsonData) {
    if (jsonData.hasOwnProperty('btn1')) {
        // Handle button press
        if (jsonData.btn1 === true) {
            console.log("Goto");
            gotoData()
            return;
        }
    }
    else if (jsonData.hasOwnProperty('btn2')) {
        // Handle button press
        if (jsonData.btn2 === true) {
            console.log("Sync");
            syncData();
            return;
        }
    }
    roll = parseDouble(jsonData.roll);
    pitch = parseDouble(jsonData.pitch);
    yaw = parseDouble(jsonData.yaw);
    azimuth = parseDouble(jsonData.azimuth);
    altitude = parseDouble(jsonData.altitude);
    yaw = parseDouble(jsonData.yaw);
    imuOutput.innerHTML = `
        <p><strong>Roll:</strong> <span class="parameter-value"> ${roll.toFixed(3)}</span> deg</p>
        <p><strong>Pitch:</strong> <span class="parameter-value">${pitch.toFixed(3)}</span> deg</p>
        <p><strong>Yaw:</strong> <span class="parameter-value"> ${yaw.toFixed(3)}</span> deg</p>
        <p><strong>Azimuth:</strong> <span class="parameter-value"> ${azimuth.toFixed(3)}</span> deg</p>
        <p><strong>Altitude:</strong> <span class="parameter-value"> ${altitude.toFixed(3)}</span> deg</p>
    `;
}
function parseDouble(value) {
    return parseFloat(value) || 0.0;
}
// Event listeners for buttons
connectButton.addEventListener('click', connectToArduino);

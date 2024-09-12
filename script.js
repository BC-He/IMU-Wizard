const connectButton = document.getElementById('connect-btn');
const imuOutput = document.getElementById('IMU-output');

let port;
let reader;
let roll = 0.0, pitch = 0.0, yaw = 0.0; // Array to store the parsed data

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
    
    roll = parseFloat(jsonData.roll);
    pitch = parseFloat(jsonData.pitch);
    yaw = parseFloat(jsonData.yaw);
    imuOutput.innerHTML = `
        <p><strong>Roll:</strong> <span class="parameter-value"> ${roll.toFixed(3)}</span> deg</p>
        <p><strong>Pitch:</strong> <span class="parameter-value">${pitch.toFixed(3)}</span> deg</p>
        <p><strong>Yaw:</strong> <span class="parameter-value"> ${yaw.toFixed(3)}</span> deg</p>
    `;
}
// Event listeners for buttons
connectButton.addEventListener('click', connectToArduino);

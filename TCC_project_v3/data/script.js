let needsUpdate = false;// responsável pelo tempo de atualização do mvChartUpdate
let tempoInicial = null;
const intervaloExibido = 60; // segundos amostrados

let socket = new WebSocket(`ws://${location.hostname}:8080`);

const mvLabels = [];
const mvData = [];
const pvLabels = [];
const pvData = [];
const spData = [];

const mvCtx = document.getElementById('mvChart').getContext('2d');
const pvCtx = document.getElementById('pvChart').getContext('2d');

const mvChart = new Chart(mvCtx, {
    type: 'line',
    data: {
        labels: mvLabels,
        datasets: [{
            label: 'MV (Tensão Aplicada)',
            borderColor: 'red',
            data: mvData,
            fill: false,
            tension: 0.1
        }]
    },
    options: {
        responsive: true,
        animation: false,
        scales: {
            y: {
                min: 0,
                max: 5,
                title: {
                    display: true,
                    text: 'Tensão (V)'
                }
            }
        }
    }
});

const pvChart = new Chart(pvCtx, {
    type: 'line',
    data: {
        labels: pvLabels,
        datasets: [
            {
                label: 'PV (Tensão Medida)',
                borderColor: 'blue',
                data: pvData,
                fill: false,
                tension: 0.1
            },
            {
                label: 'SP (Setpoint)',
                borderColor: 'green',
                borderDash: [5, 5],
                data: spData,
                fill: false,
                tension: 0.1
            }
        ]
    },
    options: {
        responsive: true,
        animation: false,
        scales: {
            y: {
                min: 0,
                max: 5,
                title: {
                    display: true,
                    text: 'Tensão (V)'
                }
            }
        }
    }
});

socket.onopen = () => {
    document.getElementById("status").textContent = "🟢 Conectado ao ESP32";
};

socket.onclose = () => {
    document.getElementById("status").textContent = "🔴 Desconectado";
};

socket.onerror = () => {
    document.getElementById("status").textContent = "❌ Erro na conexão";
};

//socket.onmessage = (event) => {
//    try {
//        const data = JSON.parse(event.data);
//
//        // Garante que tempo é um número válido
//        const tempo = parseFloat(data.time);
//        if (!Number.isFinite(tempo)) {
//            console.warn("Tempo inválido recebido:", data.time);
//            return;
//        }
//
//        const pv = parseFloat(data.PV);
//        const mv = parseFloat(data.MV);
//        const sp = parseFloat(document.getElementById("sp").value);
//
//        if (tempoInicial === null) tempoInicial = tempo;
//        const tempoRelativo = tempo - tempoInicial;
//
//        mvLabels.push(tempoRelativo.toFixed(1));
//        mvData.push(mv);
//        pvLabels.push(tempoRelativo.toFixed(1));
//        pvData.push(pv);
//        spData.push(sp);
//
//        while (tempoRelativo - parseFloat(mvLabels[0]) > intervaloExibido) {
//            mvLabels.shift(); mvData.shift();
//            pvLabels.shift(); pvData.shift(); spData.shift();
//        }
//
//        //mvChart.update(); //envio dos dados a cada mensagem
//        //pvChart.update();
//
//        
//    } catch (e) {
//        console.error("Erro ao processar mensagem:", e);
//    }
//};


setInterval(() => {
    if (needsUpdate) {
        mvChart.update();
        pvChart.update();
        needsUpdate = false;
    }
}, 100); // Atualiza os gráficos 10 vezes por segundo=10Hz

socket.onmessage = (event) => {
    try {
        const data = JSON.parse(event.data);
        // Garante que tempo é um número válido
        const tempo = parseFloat(data.time);
        if (!Number.isFinite(tempo)) {
            console.warn("Tempo inválido recebido:", data.time);
            return;
        }

        const pv = parseFloat(data.PV);
        const mv = parseFloat(data.MV);
        const sp = parseFloat(document.getElementById("sp").value);

        if (tempoInicial === null) tempoInicial = tempo;
        const tempoRelativo = tempo - tempoInicial;

        mvLabels.push(tempoRelativo.toFixed(1));
        mvData.push(mv);
        pvLabels.push(tempoRelativo.toFixed(1));
        pvData.push(pv);
        spData.push(sp);

        while (tempoRelativo - parseFloat(mvLabels[0]) > intervaloExibido) {
            mvLabels.shift(); mvData.shift();
            pvLabels.shift(); pvData.shift(); spData.shift();
        }

        needsUpdate = true;
    } catch (e) {
        console.error("Erro ao processar mensagem:", e);
    }
};


function send(msg) {
    if (socket.readyState === WebSocket.OPEN) socket.send(msg);
    else alert("Conexão WebSocket não está aberta.");
}

function sendParams() {
    const sp = document.getElementById("sp").value;
    const kp = document.getElementById("kp").value;
    const ki = document.getElementById("ki").value;
    const kd = document.getElementById("kd").value;
    const planta = document.getElementById("planta").value;
    const tipoMalha = document.getElementById("malha").value;

    send(`setSp:${sp}`);
    send(`setKp:${kp}`);
    send(`setKi:${ki}`);
    send(`setKd:${kd}`);
    send(`setPlanta:${planta}`);
    send(`setMalha:${tipoMalha}`);
}

function sendStart() {
    tempoInicial = null; // Reinicia o tempo no próximo dado recebido 
    send("start");
}
function sendStop() { send("stop"); }

const plantaGains = {
    1: { kp: 0.2042, ki: 2.2691, kd: 0.0 },
    2: { kp: 0.6219, ki: 3.3958, kd: 0.0 },
    3: { kp: 6.2532, ki: 3.4886, kd: 0.0 },
    4: { kp: 5.6813, ki: 8.0834, kd: 0.0 },
    5: { kp: 0.1048, ki: 0.8811, kd: 0.0102 },
    6: { kp: 0.2195, ki: 0.2211, kd: 0.1105 },
    7: { kp: 1.1500, ki: 0.4114, kd: 0.0905 },
};


function autoFillGains() {
    const planta = parseInt(document.getElementById("planta").value);
    const gains = plantaGains[planta];
    if (gains) {
        document.getElementById("kp").value = gains.kp;
        document.getElementById("ki").value = gains.ki;
        document.getElementById("kd").value = gains.kd;
    } else {
        document.getElementById("kp").value = "";
        document.getElementById("ki").value = "";
        document.getElementById("kd").value = "";
    }
}

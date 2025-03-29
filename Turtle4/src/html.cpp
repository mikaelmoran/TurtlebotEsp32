#include "html.h"

const char index_html_page[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="sv">
<head>
  <meta charset="UTF-8">
  <title>Drawing TurtleRobot</title>
  <!-- Google Fonts & Material Icons -->
  <link rel="preload" as="font" href="https://fonts.gstatic.com/s/poppins/v22/pxiEyp8kv8JHgFVrJJfecnFHGPc.woff2" type="font/woff2" crossorigin="anonymous">
  <link href="https://fonts.googleapis.com/css2?family=Poppins:wght@300;400;600&display=swap" rel="stylesheet" crossorigin="anonymous">
  <link rel="preconnect" href="https://fonts.googleapis.com">
  <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
  <link href="https://fonts.googleapis.com/icon?family=Material+Icons" rel="stylesheet">
  <style>
    /* GLOBAL STIL & RESET */
    * { margin: 0; padding: 0; box-sizing: border-box; font-family: 'Poppins', sans-serif; }
    body {
      background: linear-gradient(120deg, #f3f4f6, #e0f2fe);
      min-height: 100vh; display: flex; flex-direction: column; color: #333;
    }
    h1, h2 { font-weight: 600; letter-spacing: 0.5px; }
    h1 { margin: 20px 0; text-align: center; }
    a, button { text-decoration: none; transition: all 0.2s ease; cursor: pointer; }
    /* KONTAINER */
    .container { max-width: 1400px; margin: 0 auto; padding: 20px; }
    /* LAYOUT: TRE KOLUMNER */
    .mainLayout { display: flex; gap: 20px; flex-wrap: nowrap; justify-content: center; align-items: flex-start; }
    .toolsLayout, .manualSection, .advanceCalib { width: 280px; display: flex; flex-direction: column; gap: 20px; }
    .drawingSection { flex: 1; display: flex; flex-direction: column; align-items: center; }
    /* CARD */
    .card { background: #fff; border-radius: 16px; box-shadow: 0 4px 8px rgba(0,0,0,0.06); padding: 20px; }
    .card h2 { margin-bottom: 15px; font-size: 1.2rem; color: #2c3e50; }
    /* KNAPPAR */
    button {
      background: #4a90e2; color: #fff; border: none; border-radius: 8px;
      padding: 10px 14px; font-size: 14px; margin-bottom: 10px; box-shadow: 0 2px 5px rgba(0,0,0,0.1);
    }
    button:hover { transform: translateY(-2px); box-shadow: 0 6px 12px rgba(0,0,0,0.15); }
    button:active { transform: translateY(1px); box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
    button:disabled { background: #ccc; cursor: not-allowed; }
    .buttonPanel button,
    .buttonPanel label,
    .manualSection button { width: 100%; }
    .buttonPanel label { display: flex; align-items: center; margin-bottom: 10px; cursor: pointer; font-weight: 400; color: #333; }
    .buttonPanel label input { margin-right: 8px; transform: scale(1.2); cursor: pointer; }
    input[type="text"], input[type="number"] {
      background: #f9fafb; border: 1px solid #d1d5db; border-radius: 8px;
      padding: 8px 12px; width: 100%; margin-bottom: 10px; font-size: 14px; color: #333; outline: none; transition: border-color 0.2s;
    }
    input[type="text"]:focus, input[type="number"]:focus { border-color: #4a90e2; }
    /* CANVAS & RELATERAT */
    #drawCanvas {
      border: 2px solid #ccc; background: #fff; border-radius: 12px;
      box-shadow: 0 4px 8px rgba(0,0,0,0.05); cursor: crosshair; margin-bottom: 10px;
    }
    #loadingIndicator { display: none; margin-top: 10px; font-weight: 500; text-align: center; color: #e67e22; }
    #fullscreenBtn { background: #e67e22; margin-bottom: 10px; }
    /* Extra statusruta för meddelanden */
    #statusMsg { text-align: center; font-weight: 500; color: #e67e22; margin-bottom: 10px; }
  </style>
</head>
<body>
  <h1>Drawing turtle robot</h1>
  <div class="container">
    <div class="mainLayout">
      <!-- Verktygs-panel -->
      <div class="toolsLayout card">
        <div class="buttonPanel">
          <h2>Tools</h2>
          <button id="clearBtn">Clear Canavs</button>
          <button id="btnUndo">Undo</button>
          <button id="sendCmdBtn">Send drawing to robot</button>
          <hr style="margin: 20px 0;">
          <button id="btnFreehand">Lines</button>
          <button id="btnCircle">Circles</button>
          <button id="btnRect">Rectangles</button>
          <button id="btnTri">Triangle</button>
          <button id="btnText">Text</button>
          <button id="btnMove">Move object</button>
          <label>
            <input type="checkbox" id="gridToggle" checked>
            Grid
          </label>
          <!-- Textverktygsfält -->
          <div id="textToolRow">
            <input type="text" id="textInput" placeholder="Type character here" maxlength="1">
            <input type="number" id="textScale" placeholder="Scale" step="1" value="5">
            <button id="drawTextBtn">Draw text</button>
          </div>
        </div>
      </div>

      <!-- Canvas -->
      <div class="drawingSection card">
        <h2>Canvas 2x2 Meter</h2>
        <canvas id="drawCanvas" width="500" height="500"></canvas>
        <button id="fullscreenBtn">🖥️ Fullscreen</button>
        <div id="loadingIndicator">⏳ Robot wroking...</div>
      </div>

      <!-- Manuell styrning -->
      <div class="manualSection card">
        <h2>Manual Drive</h2>
        <!-- Extra statusmeddelande -->
        <div id="statusMsg"></div>
        <button id="btnUp"><span class="material-icons">arrow_upward</span></button>
        <button id="btnDown"><span class="material-icons">arrow_downward</span></button>
        <button id="btnLeft"><span class="material-icons">arrow_back</span></button>
        <button id="btnRight"><span class="material-icons">arrow_forward</span></button>
        <button id="btnResetPos">Reset position</button>
        <div id="infoBox">Robotstatus: unknown...</div>
        <div id="homeStatus"></div>
        <button id="btnReturnHome">Return home</button>
      </div>

      <!-- Avancerad kalibrering -->
      <div class="advanceCalib card">
        <div class="modalContent">
          <h2>Calibration</h2>
          <p>Distance-test (200 mm):</p>
          <button id="distTestBtn">Drive Forward (200 mm)</button>
          <label for="measuredDist">Measured distance (mm):</label>
          <input type="number" id="measuredDist" step="0.1" placeholder="t ex 195.5">
          <button id="saveDistBtn">Save Distance Calibration</button>
          <hr style="margin:20px 0;">
          <p>Rotation-test (360°):</p>
          <button id="rotTestBtn">Run Rotation Test (360°)</button>
          <label for="measuredAngle">Measured angle (°):</label>
          <input type="number" id="measuredAngle" step="1" placeholder="t ex 350">
          <button id="saveAngleBtn">Save Rotation Calibration</button>
        </div>
      </div>
    </div>
  </div>

  <script>
    /************************************************
     * Javascript för canvas-ritning & styrning
     ************************************************/
    let shapes = [];
    let robotBusy = false;
    let currentTool = "freehand";
    let isDrawing = false, tempLine = null;
    let robotPosX = 0, robotPosY = 0, robotTrail = [];
    let robotPenDown = false;
    let isDrawingCircle = false, circleCenter = null;
    let isDrawingRect = false, rectStart = null;
    let trianglePoints = [];
    let textPlacement = null;
    let selectedShapeIndex = -1, isMoving = false;
    let lastMovePos = { x: 0, y: 0 };
    let currentMousePos = { x: 0, y: 0 };

    const canvas = document.getElementById("drawCanvas");
    const ctx = canvas.getContext("2d");
    const gridToggle = document.getElementById("gridToggle");
    const infoBox = document.getElementById("infoBox");
    const loadingIndicator = document.getElementById("loadingIndicator");
    const fullscreenBtn = document.getElementById("fullscreenBtn");
    const textToolRow = document.getElementById("textToolRow");
    const textInput = document.getElementById("textInput");
    const textScale = document.getElementById("textScale");
    const drawTextBtn = document.getElementById("drawTextBtn");
    const btnMove = document.getElementById("btnMove");
    
    const gridCanvas = document.createElement("canvas");
    gridCanvas.width = canvas.width;
    gridCanvas.height = canvas.height;
    const gridCtx = gridCanvas.getContext("2d");
    const mmPerPixel = 2.0;

    function setCanvasTransform() {
      ctx.setTransform(1, 0, 0, -1, canvas.width / 2, canvas.height / 2);
    }

    function convertToPhysicalCoordinates(pos) {
      return { x: pos.x - canvas.width / 2, y: canvas.height / 2 - pos.y };
    }

    function updateUI() {
      document.getElementById("btnUp").disabled = robotBusy;
      document.getElementById("btnDown").disabled = robotBusy;
      document.getElementById("btnLeft").disabled = robotBusy;
      document.getElementById("btnRight").disabled = robotBusy;
      document.getElementById("btnResetPos").disabled = robotBusy;
      document.getElementById("btnReturnHome").disabled = robotBusy;
      document.getElementById("sendCmdBtn").disabled = robotBusy;
      document.getElementById("distTestBtn").disabled = robotBusy;
      document.getElementById("rotTestBtn").disabled = robotBusy;
      const statusMsg = document.getElementById("statusMsg");
      statusMsg.textContent = robotBusy ? "Robot is busy, wait..." : "";
    }

    function getBoundingBox(shape) {
      let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
      shape.segments.forEach(seg => {
        minX = Math.min(minX, seg.x1, seg.x2);
        minY = Math.min(minY, seg.y1, seg.y2);
        maxX = Math.max(maxX, seg.x1, seg.x2);
        maxY = Math.max(maxY, seg.y1, seg.y2);
      });
      return { minX, minY, maxX, maxY };
    }

    function isPointInBox(point, box) {
      return (point.x >= box.minX && point.x <= box.maxX &&
              point.y >= box.minY && point.y <= box.maxY);
    }

    function createGrid() {
      gridCtx.clearRect(0, 0, gridCanvas.width, gridCanvas.height);
      gridCtx.strokeStyle = "#ddd";
      const pxPerCm = 5;
      for (let i = 0; i < 100; i++) {
        let pos = i * pxPerCm;
        gridCtx.beginPath();
        gridCtx.moveTo(pos, 0);
        gridCtx.lineTo(pos, gridCanvas.height);
        gridCtx.stroke();
        gridCtx.beginPath();
        gridCtx.moveTo(0, pos);
        gridCtx.lineTo(gridCanvas.width, pos);
        gridCtx.stroke();
      }
    }
    createGrid();

    function redrawAll() {
      ctx.save();
      ctx.resetTransform();
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.restore();
      ctx.save();
      setCanvasTransform();
      if (gridToggle.checked) {
        ctx.drawImage(gridCanvas, -canvas.width / 2, -canvas.height / 2, canvas.width, canvas.height);
      }
      ctx.restore();

      ctx.save();
      setCanvasTransform();
      ctx.strokeStyle = "black";
      shapes.forEach(shape => {
        shape.segments.forEach(seg => {
          ctx.beginPath();
          ctx.moveTo(seg.x1, seg.y1);
          ctx.lineTo(seg.x2, seg.y2);
          ctx.stroke();
        });
      });
      ctx.restore();

      ctx.save();
      setCanvasTransform();
      ctx.strokeStyle = "red";
      if (currentTool === "freehand" && tempLine) {
        ctx.beginPath();
        ctx.moveTo(tempLine.x1, tempLine.y1);
        ctx.lineTo(tempLine.x2, tempLine.y2);
        ctx.stroke();
      }
      if (currentTool === "circle" && isDrawingCircle && circleCenter) {
        let dx = currentMousePos.x - circleCenter.x;
        let dy = currentMousePos.y - circleCenter.y;
        let radius = Math.sqrt(dx * dx + dy * dy);
        let previewSegs = addCircleSegments(circleCenter.x, circleCenter.y, radius, 36);
        ctx.beginPath();
        previewSegs.forEach((seg, idx) => {
          if (idx === 0) ctx.moveTo(seg.x1, seg.y1);
          ctx.lineTo(seg.x2, seg.y2);
        });
        ctx.closePath();
        ctx.stroke();
      }
      if (currentTool === "rectangle" && isDrawingRect && rectStart) {
        ctx.beginPath();
        ctx.rect(rectStart.x, rectStart.y, currentMousePos.x - rectStart.x, currentMousePos.y - rectStart.y);
        ctx.stroke();
      }
      if (currentTool === "triangle" && trianglePoints.length > 0) {
        ctx.beginPath();
        ctx.moveTo(trianglePoints[0].x, trianglePoints[0].y);
        if (trianglePoints.length === 1) {
          ctx.lineTo(currentMousePos.x, currentMousePos.y);
        } else if (trianglePoints.length === 2) {
          ctx.lineTo(trianglePoints[1].x, trianglePoints[1].y);
          ctx.lineTo(currentMousePos.x, currentMousePos.y);
        }
        ctx.stroke();
      }
      if (currentTool === "text" && textPlacement) {
        ctx.beginPath();
        ctx.arc(textPlacement.x, textPlacement.y, 3, 0, 2 * Math.PI);
        ctx.fillStyle = "blue";
        ctx.fill();
      }
      ctx.restore();

      ctx.save();
      setCanvasTransform();
      let posXInPixels = robotPosX / mmPerPixel;
      let posYInPixels = robotPosY / mmPerPixel;
      ctx.beginPath();
      ctx.arc(posXInPixels, posYInPixels, 5, 0, 2 * Math.PI);
      ctx.strokeStyle = "red";
      ctx.lineWidth = 2;
      ctx.stroke();
      ctx.restore();
    }

    function canvasAnimation() {
      redrawAll();
      requestAnimationFrame(canvasAnimation);
    }
    canvasAnimation();

    // Avancerad kalibrering: distans
    const distTestBtn   = document.getElementById("distTestBtn");
    const saveDistBtn   = document.getElementById("saveDistBtn");
    const measuredDistEl = document.getElementById("measuredDist");

    distTestBtn.addEventListener("click", () => {
      fetch("/calib_dist_test")
        .then(r => r.text())
        .then(txt => alert("Started: " + txt))
        .catch(e => alert("Fel vid dist-test: " + e));
    });

    saveDistBtn.addEventListener("click", () => {
      const val = measuredDistEl.value;
      if (!val || parseFloat(val) <= 0) {
        alert("Error.");
        return;
      }
      fetch("/calib_dist_save?faktiskDist=" + encodeURIComponent(val))
        .then(r => r.text())
        .then(txt => alert("Answer from servern: " + txt))
        .catch(e => alert("Error!: " + e));
    });

    // Avancerad kalibrering: rotation
    const rotTestBtn      = document.getElementById("rotTestBtn");
    const saveAngleBtn    = document.getElementById("saveAngleBtn");
    const measuredAngleEl = document.getElementById("measuredAngle");

    rotTestBtn.addEventListener("click", () => {
      fetch("/calib_rot_test")
        .then(r => r.text())
        .then(txt => alert("Startad: " + txt))
        .catch(e => alert("Error: " + e));
    });

    saveAngleBtn.addEventListener("click", () => {
      const val = measuredAngleEl.value;
      if (!val || parseFloat(val) <= 0) {
        alert("Put in the angle.");
        return;
      }
      fetch("/calib_rot_save?faktiskVinkel=" + encodeURIComponent(val))
        .then(r => r.text())
        .then(txt => alert("Answer from servern: " + txt))
        .catch(e => alert("Error: " + e));
    });

    function addCircleSegments(cx, cy, r, steps = 36) {
      let segs = [];
      let angleStep = (2 * Math.PI) / steps;
      let xPrev = cx + r, yPrev = cy;
      for (let i = 1; i <= steps; i++) {
        let angle = i * angleStep;
        let xNow = cx + r * Math.cos(angle);
        let yNow = cy + r * Math.sin(angle);
        segs.push({ x1: xPrev, y1: yPrev, x2: xNow, y2: yNow });
        xPrev = xNow;
        yPrev = yNow;
      }
      return segs;
    }
    
    function addRectangleSegments(x1, y1, x2, y2) {
      return [
        { x1: x1, y1: y1, x2: x2, y2: y1 },
        { x1: x2, y1: y1, x2: x2, y2: y2 },
        { x1: x2, y1: y2, x2: x1, y2: y2 },
        { x1: x1, y1: y2, x2: x1, y2: y1 },
      ];
    }
    
    function addTriangleSegments(xA, yA, xB, yB, xC, yC) {
      return [
        { x1: xA, y1: yA, x2: xB, y2: yB },
        { x1: xB, y1: yB, x2: xC, y2: yC },
        { x1: xC, y1: yC, x2: xA, y2: yA },
      ];
    }
    
const hersheyFont = {
  "A": [
    [0, 0, 10, 20],
    [10, 20, 20, 0],
    [4, 10, 16, 10]
  ],
  "B": [
    [0, 0, 0, 20],
    [0, 20, 12, 20],
    [12, 20, 14, 18],
    [14, 18, 14, 12],
    [14, 12, 12, 10],
    [12, 10, 0, 10]
  ],
  "C": [
    [14,20, 2,20],
    [2,20, 0,18],
    [0,18, 0,2],
    [0,2, 2,0],
    [2,0, 14,0]
  ],
  "D": [
    [0,0, 0,20],
    [0,20,12,20],
    [12,20,14,18],
    [14,18,14,2],
    [14,2, 12,0],
    [12,0, 0,0]
  ],
  "E": [
    [0,0, 0,20],
    [0,20,14,20],
    [0,10,10,10],
    [0,0, 14,0]
  ],
  "F": [
    [0,0, 0,20],
    [0,20,14,20],
    [0,10,10,10]
  ],
  "G": [
    [14,20, 2,20],
    [2,20, 0,18],
    [0,18, 0,2],
    [0,2, 2,0],
    [2,0, 14,0],
    [14,0, 14,10],
    [14,10,10,10]
  ],
  "H": [
    [0,0, 0,20],
    [14,0, 14,20],
    [0,10, 14,10]
  ],
  "I": [
    [2,0, 12,0],
    [2,20, 12,20],
    [7,0, 7,20]
  ],
  "J": [
    [2,20, 12,20],
    [7,20, 7,4],
    [7,4, 5,2],
    [5,2, 0,2]
  ],
  "K": [
    [0,0, 0,20],
    [0,10, 12,20],
    [0,10, 12,0]
  ],
  "L": [
    [0,0, 0,20],
    [0,0, 14,0]
  ],
  "M": [
    [0,0, 0,20],
    [0,20, 7,8],
    [7,8, 14,20],
    [14,20, 14,0]
  ],
  "N": [
    [0,0, 0,20],
    [0,20, 14,0],
    [14,0, 14,20]
  ],
  "O": [
    [4,0, 10,0],
    [10,0, 14,4],
    [14,4, 14,16],
    [14,16, 10,20],
    [10,20, 4,20],
    [4,20, 0,16],
    [0,16, 0,4],
    [0,4, 4,0]
  ],
  "P": [
    [0,0, 0,20],
    [0,20, 12,20],
    [12,20, 14,18],
    [14,18, 14,12],
    [14,12, 12,10],
    [12,10, 0,10]
  ],
  "Q": [
    [4,0, 10,0],
    [10,0, 14,4],
    [14,4, 14,16],
    [14,16, 10,20],
    [10,20, 4,20],
    [4,20, 0,16],
    [0,16, 0,4],
    [0,4, 4,0],
    [10,4, 14,0]
  ],
  "R": [
    [0,0, 0,20],
    [0,20, 12,20],
    [12,20, 14,18],
    [14,18, 14,12],
    [14,12, 12,10],
    [12,10, 0,10],
    [6,10, 14,0]
  ],
  "S": [
    [14,18, 12,20],
    [12,20, 2,20],
    [2,20, 0,18],
    [0,18, 0,12],
    [0,12, 2,10],
    [2,10, 12,10],
    [12,10, 14,8],
    [14,8, 14,2],
    [14,2, 12,0],
    [12,0, 2,0],
    [2,0, 0,2]
  ],
  "T": [
    [0,20, 14,20],
    [7,20, 7,0]
  ],
  "U": [
    [0,20, 0,4],
    [0,4, 2,2],
    [2,2, 12,2],
    [12,2, 14,4],
    [14,4, 14,20]
  ],
  "V": [
    [0,20, 7,0],
    [7,0, 14,20]
  ],
  "W": [
    [0,20, 3,0],
    [3,0, 7,12],
    [7,12, 11,0],
    [11,0, 14,20]
  ],
  "X": [
    [0,0, 14,20],
    [14,0, 0,20]
  ],
  "Y": [
    [0,20, 7,10],
    [7,10, 14,20],
    [7,10, 7,0]
  ],
  "Z": [
    [0,20, 14,20],
    [14,20, 0,0],
    [0,0, 14,0]
  ],
  "Å": [
    [0,0, 10,20],
    [10,20, 20,0],
    [4,10, 16,10],
    [8,22, 9,24],
    [9,24, 11,24],
    [11,24, 12,22],
    [12,22, 8,22]
  ],
  "Ä": [
    [0,0, 10,20],
    [10,20, 20,0],
    [4,10, 16,10],
    [6,24, 8,24],
    [12,24, 14,24]
  ],
  "Ö": [
    [4,0, 10,0],
    [10,0, 14,4],
    [14,4, 14,16],
    [14,16, 10,20],
    [10,20, 4,20],
    [4,20, 0,16],
    [0,16, 0,4],
    [0,4, 4,0],
    [4,24, 6,24],
    [8,24, 10,24]
  ],
  "1": [
    [7,0, 7,20],
    [5,0, 9,0],
    [5,20, 9,20]
  ],
  "2": [
    [0,16, 2,20],
    [2,20, 12,20],
    [12,20, 14,18],
    [14,18, 14,12],
    [14,12, 12,10],
    [12,10, 0,0],
    [0,0, 14,0]
  ],
  "3": [
    [0,20, 14,20],
    [14,20, 12,10],
    [12,10, 14,0],
    [14,0, 0,0],
    [12,10, 4,10]
  ],
  "4": [
    [0,20, 0,10],
    [0,10, 14,10],
    [14,20, 14,0]
  ],
  "5": [
    [14,20, 0,20],
    [0,20, 0,10],
    [0,10, 12,10],
    [12,10, 14,8],
    [14,8, 14,0],
    [14,0, 0,0]
  ],
  "6": [
    [14,20, 0,20],
    [0,20, 0,0],
    [0,0, 14,0],
    [14,0, 14,8],
    [14,8, 12,10],
    [12,10, 0,10]
  ],
  "7": [
    [0,20, 14,20],
    [14,20, 14,0]
  ],
  "8": [
    [0,0, 0,20],
    [0,20, 14,20],
    [14,20, 14,0],
    [14,0, 0,0],
    [0,10, 14,10]
  ],
  "9": [
    [14,0, 14,20],
    [14,20, 0,20],
    [0,20, 0,12],
    [0,12, 2,10],
    [2,10, 14,10]
  ],
  "0": [
    [4,0, 10,0],
    [10,0, 14,4],
    [14,4, 14,16],
    [14,16, 10,20],
    [10,20, 4,20],
    [4,20, 0,16],
    [0,16, 0,4],
    [0,4, 4,0]
  ],
  " ": []
};
    
    function convertTextToShape(text, startX = 0, startY = 0, scale = 1) {
      let shape = { type: "text", segments: [] };
      let xOffset = startX;
      for (let char of text) {
        if (hersheyFont[char]) {
          let isFirstSegment = true;
          for (let seg of hersheyFont[char]) {
            shape.segments.push({
              x1: seg[0] * scale + xOffset,
              y1: seg[1] * scale + startY,
              x2: seg[2] * scale + xOffset,
              y2: seg[3] * scale + startY,
              moveBefore: isFirstSegment
            });
            isFirstSegment = false;
          }
          xOffset += 25 * scale;
        } else if (char === " ") {
          xOffset += 15 * scale;
        }
      }
      return shape;
    }
    
    function getMousePosMagnet(e) {
      const rect = canvas.getBoundingClientRect();
      let x = e.clientX - rect.left;
      let y = e.clientY - rect.top;
      const snapRadius = 5, pxPerCm = 5;
      const snappedX = Math.round(x / pxPerCm) * pxPerCm;
      const snappedY = Math.round(y / pxPerCm) * pxPerCm;
      if (Math.abs(snappedX - x) < snapRadius) x = snappedX;
      if (Math.abs(snappedY - y) < snapRadius) y = snappedY;
      return { x, y };
    }
    
    canvas.addEventListener("mousedown", (e) => {
      let pos = convertToPhysicalCoordinates(getMousePosMagnet(e));
      if (currentTool === "freehand") {
        isDrawing = true;
        tempLine = { x1: pos.x, y1: pos.y, x2: pos.x, y2: pos.y };
      } else if (currentTool === "circle") {
        isDrawingCircle = true;
        circleCenter = pos;
      } else if (currentTool === "rectangle") {
        isDrawingRect = true;
        rectStart = pos;
      } else if (currentTool === "text") {
        textPlacement = pos;
      } else if (currentTool === "move") {
        let found = false;
        for (let i = shapes.length - 1; i >= 0; i--) {
          let box = getBoundingBox(shapes[i]);
          if (isPointInBox(pos, box)) {
            selectedShapeIndex = i;
            isMoving = true;
            lastMovePos = pos;
            found = true;
            break;
          }
        }
        if (!found) {
          selectedShapeIndex = -1;
          isMoving = false;
        }
      }
    });
    
    canvas.addEventListener("mousemove", (e) => {
      currentMousePos = convertToPhysicalCoordinates(getMousePosMagnet(e));
      if (currentTool === "freehand" && isDrawing) {
        tempLine.x2 = currentMousePos.x;
        tempLine.y2 = currentMousePos.y;
      } else if (currentTool === "move" && isMoving && selectedShapeIndex !== -1) {
        let dx = currentMousePos.x - lastMovePos.x;
        let dy = currentMousePos.y - lastMovePos.y;
        shapes[selectedShapeIndex].segments.forEach(seg => {
          seg.x1 += dx;
          seg.y1 += dy;
          seg.x2 += dx;
          seg.y2 += dy;
        });
        lastMovePos = currentMousePos;
      }
    });
    
    canvas.addEventListener("mouseup", (e) => {
      let pos = convertToPhysicalCoordinates(getMousePosMagnet(e));
      if (currentTool === "freehand" && isDrawing) {
        isDrawing = false;
        tempLine.x2 = pos.x;
        tempLine.y2 = pos.y;
        shapes.push({ type: "freehand", segments: [{ x1: tempLine.x1, y1: tempLine.y1, x2: tempLine.x2, y2: tempLine.y2 }] });
        tempLine = null;
      } else if (currentTool === "circle" && isDrawingCircle) {
        isDrawingCircle = false;
        let dx = pos.x - circleCenter.x;
        let dy = pos.y - circleCenter.y;
        let radius = Math.sqrt(dx * dx + dy * dy);
        let segs = addCircleSegments(circleCenter.x, circleCenter.y, radius, 36);
        shapes.push({ type: "circle", segments: segs });
        currentTool = "freehand";
      } else if (currentTool === "rectangle" && isDrawingRect) {
        isDrawingRect = false;
        let segs = addRectangleSegments(rectStart.x, rectStart.y, pos.x, pos.y);
        shapes.push({ type: "rectangle", segments: segs });
        currentTool = "freehand";
      } else if (currentTool === "move" && isMoving) {
        isMoving = false;
        selectedShapeIndex = -1;
      }
    });
    
    canvas.addEventListener("click", (e) => {
      if (currentTool === "triangle") {
        let pos = convertToPhysicalCoordinates(getMousePosMagnet(e));
        trianglePoints.push(pos);
        if (trianglePoints.length === 3) {
          let segs = addTriangleSegments(
            trianglePoints[0].x, trianglePoints[0].y,
            trianglePoints[1].x, trianglePoints[1].y,
            trianglePoints[2].x, trianglePoints[2].y
          );
          shapes.push({ type: "triangle", segments: segs });
          trianglePoints = [];
          currentTool = "freehand";
        }
      }
    });
    
    btnMove.addEventListener("click", () => { currentTool = "move"; });
    
    function resizeCanvas() {
      const size = Math.min(window.innerWidth - 40, 500);
      canvas.width = size;
      canvas.height = size;
      gridCanvas.width = size;
      gridCanvas.height = size;
      createGrid();
      redrawAll();
    }
    window.addEventListener("resize", resizeCanvas);
    resizeCanvas();
    
    function showLoading(show) {
      loadingIndicator.style.display = show ? "block" : "none";
    }
    
    fullscreenBtn.addEventListener("click", () => {
      if (canvas.requestFullscreen) canvas.requestFullscreen();
      else if (canvas.webkitRequestFullscreen) canvas.webkitRequestFullscreen();
      else if (canvas.msRequestFullscreen) canvas.msRequestFullscreen();
    });
    
    const socket = new WebSocket("ws://" + location.hostname + "/ws");
    socket.onopen = () => { console.log("WebSocket ansluten."); };
    socket.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        if (data.type === "status") {
          robotPosX = data.posX;
          robotPosY = data.posY;
          robotPenDown = (data.penDown === "true" || data.penDown === true);
          if (robotPenDown) {
            if (robotTrail.length === 0 ||
                Math.hypot(robotPosX - robotTrail[robotTrail.length - 1].x,
                           robotPosY - robotTrail[robotTrail.length - 1].y) > 2) {
              robotTrail.push({ x: robotPosX, y: robotPosY });
            }
          }
          infoBox.innerHTML =
            `📍 <b>X:</b> ${data.posX.toFixed(1)} mm<br>` +
            `📍 <b>Y:</b> ${data.posY.toFixed(1)} mm<br>` +
            `🔄 <b>Angle:</b> ${data.posThetaDeg.toFixed(1)}°<br>` +
            `✏️ <b>Pen:</b> ${data.penDown ? "⬇️ Down" : "⬆️ Up"}`;
        }
        if (data.isBusy) {
          robotBusy = true;
          updateUI();
        } else {
          robotBusy = false;
          updateUI();
        }
        if (data.type === "home") {
          document.getElementById("homeStatus").textContent = "✅ Robot hemma!";
          setTimeout(() => { document.getElementById("homeStatus").textContent = ""; }, 3000);
        }
      } catch (e) {
        console.error("Fel vid tolkning av WebSocket-data:", e);
      }
      updateUI();
    };
    
    // Verktygsval-knappar
    document.getElementById("btnFreehand").addEventListener("click", () => {
      currentTool = "freehand";
      textToolRow.style.display = "none";
    });
    document.getElementById("btnCircle").addEventListener("click", () => {
      currentTool = "circle";
      textToolRow.style.display = "none";
    });
    document.getElementById("btnRect").addEventListener("click", () => {
      currentTool = "rectangle";
      textToolRow.style.display = "none";
    });
    document.getElementById("btnTri").addEventListener("click", () => {
      currentTool = "triangle";
      trianglePoints = [];
      textToolRow.style.display = "none";
    });
    document.getElementById("btnText").addEventListener("click", () => {
      currentTool = "text";
      textToolRow.style.display = "flex";
      textPlacement = null;
    });
    
    drawTextBtn.addEventListener("click", () => {
      const letter = textInput.value.toUpperCase();
      if (letter === "") return;
      let scale = parseFloat(textScale.value);
      if (isNaN(scale) || scale <= 3) scale = 5;
      const startX = textPlacement ? textPlacement.x : 0;
      const startY = textPlacement ? textPlacement.y : 0;
      const letterShape = convertTextToShape(letter, startX, startY, scale);
      shapes.push(letterShape);
      textPlacement = { x: startX * scale, y: startY };
      textInput.value = "";
      redrawAll();
    });
    
    document.getElementById("btnUndo").addEventListener("click", () => {
      if (shapes.length > 0) {
        shapes.pop();
        redrawAll();
      }
    });
    
    document.getElementById("clearBtn").addEventListener("click", () => {
      shapes = [];
      robotTrail = [];
      fetch("/reset_position")
        .then(r => r.text())
        .then(txt => console.log(txt))
        .catch(e => console.error("Fel:", e));
      redrawAll();
    });
    
    document.getElementById("btnUp").addEventListener("click", () => {
      fetch("/manual?move=F").catch(e => alert("Fel: " + e));
    });
    document.getElementById("btnDown").addEventListener("click", () => {
      fetch("/manual?move=B").catch(e => alert("Fel: " + e));
    });
    document.getElementById("btnLeft").addEventListener("click", () => {
      fetch("/manual?move=L").catch(e => alert("Fel: " + e));
    });
    document.getElementById("btnRight").addEventListener("click", () => {
      fetch("/manual?move=R").catch(e => alert("Fel: " + e));
    });
    document.getElementById("btnResetPos").addEventListener("click", () => {
      fetch("/reset_position")
        .then(r => r.text())
        .then(txt => alert(txt))
        .catch(e => alert("Fel: " + e));
    });
    document.getElementById("btnReturnHome").addEventListener("click", () => {
      fetch("/return_home")
      robotTrail = []
        .then(r => r.text())
        .then(txt => {
          alert(txt);
          socket.send(JSON.stringify({ type: "home" }));
        })
        .catch(e => alert("Fel: " + e));
    });
    document.getElementById("sendCmdBtn").addEventListener("click", () => {
      showLoading(true);
      let allSegments = [];
      shapes.forEach(shape => { allSegments.push(...shape.segments); });
      let jsonData = JSON.stringify(allSegments);
      fetch("/execute?cmd=" + encodeURIComponent(jsonData))
        .then(r => r.text())
        .then(txt => { alert(txt); showLoading(false); })
        .catch(e => { alert("Fel: " + e); showLoading(false); });
    });
    
    function updateStatusUI() { requestAnimationFrame(updateStatusUI); }
    updateStatusUI();
  </script>
</body>
</html>
)rawliteral";

const char calibrate_input_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="sv">
  <head><meta charset="UTF-8"><title>Kalibrering</title></head>
  <body>
    <h1>Kalibrering</h1>
    <p>Mät den ritade 10×10 cm kvadraten noggrant och fyll i dess verkliga bredd och höjd i mm:</p>
    <form action="/calibrate_save" method="GET">
      <label>Uppmätt bredd (mm):
        <input type="number" name="width" step="0.1" required>
      </label><br><br>
      <label>Uppmätt höjd (mm):
        <input type="number" name="height" step="0.1" required>
      </label><br><br>
      <input type="submit" value="Spara & Justera">
    </form>
  </body>
</html>
)rawliteral";

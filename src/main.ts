import './style.css'

const matrix = {
  multiply: (a: number[][], b: number[][]) => {
    const result = Array(a.length).fill(0).map(() => Array(b[0].length).fill(0));
    return result.map((row, i) => {
      return row.map((_, j) => {
        return a[i].reduce((sum, elm, k) => sum + (elm * b[k][j]), 0);
      });
    });
  },
  add: (a: number[][], b: number[][]) => a.map((row, i) => row.map((val, j) => val + b[i][j])),
  subtract: (a: number[][], b: number[][]) => a.map((row, i) => row.map((val, j) => val - b[i][j])),
  transpose: (a: number[][]) => a[0].map((_, c) => a.map(r => r[c])),
  invert2x2: (a: number[][]) => {
    const det = a[0][0] * a[1][1] - a[0][1] * a[1][0];
    if (det === 0) return null;
    const invDet = 1 / det;
    return [
      [a[1][1] * invDet, -a[0][1] * invDet],
      [-a[1][0] * invDet, a[0][0] * invDet]
    ];
  }
};

class KalmanFilter {
  x: number[][];
  P: number[][];
  F: number[][] | null;
  H: number[][];
  Q: number[][];
  R: number[][];
  I: number[][];
  isInitialized: boolean;

  constructor({ processNoise, measurementNoise }: { processNoise: number; measurementNoise: number }) {
    this.x = [[0], [0], [0], [0]];
    this.P = [
      [1, 0, 0, 0],
      [0, 1, 0, 0],
      [0, 0, 1000, 0],
      [0, 0, 0, 1000]
    ];
    this.F = null;
    this.H = [
      [1, 0, 0, 0],
      [0, 1, 0, 0]
    ];
    this.Q = [
      [processNoise, 0, 0, 0],
      [0, processNoise, 0, 0],
      [0, 0, processNoise, 0],
      [0, 0, processNoise, 0]
    ];
    this.R = [
      [measurementNoise, 0],
      [0, measurementNoise]
    ];
    this.I = [
      [1, 0, 0, 0],
      [0, 1, 0, 0],
      [0, 0, 1, 0],
      [0, 0, 0, 1]
    ];
    this.isInitialized = false;
  }

  predict(dt: number) {
    this.F = [
      [1, 0, dt, 0],
      [0, 1, 0, dt],
      [0, 0, 1, 0],
      [0, 0, 0, 1]
    ];
    this.x = matrix.multiply(this.F, this.x);
    const F_T = matrix.transpose(this.F);
    const P_pred = matrix.multiply(matrix.multiply(this.F, this.P), F_T);
    this.P = matrix.add(P_pred, this.Q);
  }

  update(z: number[][], R: number[][]) {
    this.R = R;
    const y = matrix.subtract(z, matrix.multiply(this.H, this.x));
    const H_T = matrix.transpose(this.H);
    const S = matrix.add(matrix.multiply(matrix.multiply(this.H, this.P), H_T), this.R);
    const S_inv = matrix.invert2x2(S);
    if (!S_inv) return;
    const K = matrix.multiply(matrix.multiply(this.P, H_T), S_inv);
    this.x = matrix.add(this.x, matrix.multiply(K, y));
    const I_KH = matrix.subtract(this.I, matrix.multiply(K, this.H));
    this.P = matrix.multiply(I_KH, this.P);
  }

  getState() {
    return {
      lat: this.x[0][0],
      lng: this.x[1][0],
      velocity_lat: this.x[2][0],
      velocity_lng: this.x[3][0]
    };
  }

  initializeState(lat: number, lng: number) {
    this.x[0][0] = lat;
    this.x[1][0] = lng;
    this.isInitialized = true;
  }
}

let map: google.maps.Map;
let rawPath: google.maps.Polyline | null = null;
let filteredPath: google.maps.Polyline | null = null;
let rawCoordinates: google.maps.LatLng[] = [];
let filteredCoordinates: google.maps.LatLng[] = [];
let rawMarkers: google.maps.marker.AdvancedMarkerElement[] = [];
let filteredMarkers: google.maps.marker.AdvancedMarkerElement[] = [];
let kalmanFilter: KalmanFilter;
let lastTimestamp: number | null = null;

kalmanFilter = new KalmanFilter({ processNoise: 0.0001, measurementNoise: 0.001 });

async function initMap(): Promise<void> {
  const { Map } = await google.maps.importLibrary("maps") as google.maps.MapsLibrary
  const mapOptions : google.maps.MapOptions = {
    zoom: 14,
    center: { lat: 32.837554, lng: -117.123425 },
    mapId: "DEMO_MAP_ID"
  }
  map = new Map(document.getElementById('map') as HTMLElement, mapOptions)
  
  rawPath = new google.maps.Polyline({
    path: rawCoordinates,
    geodesic: true,
    strokeColor: '#FF0000',
    strokeOpacity: 1.0,
    strokeWeight: 3
  });
  rawPath.setMap(map);

  filteredPath = new google.maps.Polyline({
    path: filteredCoordinates,
    geodesic: true,
    strokeColor: '#00FF00',
    strokeOpacity: 1.0,
    strokeWeight: 3
  });
  filteredPath.setMap(map);
}

async function addPositionMarkers(rawPosition: { lat: number; lng: number }, filteredPosition: { lat: number; lng: number }) {
  const { AdvancedMarkerElement } = await google.maps.importLibrary("marker") as google.maps.MarkerLibrary;
  
  const rawDot = document.createElement('div');
  rawDot.style.width = '8px';
  rawDot.style.height = '8px';
  rawDot.style.backgroundColor = '#FF0000';
  rawDot.style.borderRadius = '50%';
  rawDot.style.border = '1px solid white';
  
  const rawPositionMarker = new AdvancedMarkerElement({
    map: map,
    position: rawPosition,
    content: rawDot,
    title: `Raw GPS Point ${sampleIndex + 1}`
  });
  rawMarkers.push(rawPositionMarker);

  const filteredDot = document.createElement('div');
  filteredDot.style.width = '8px';
  filteredDot.style.height = '8px';
  filteredDot.style.backgroundColor = '#00FF00';
  filteredDot.style.borderRadius = '50%';
  filteredDot.style.border = '1px solid white';
  
  const filteredPositionMarker = new AdvancedMarkerElement({
    map: map,
    position: filteredPosition,
    content: filteredDot,
    title: `Filtered Point ${sampleIndex + 1}`
  });
  filteredMarkers.push(filteredPositionMarker);
}


const sampleData = [
  { accuracy: 610.372, lat: 32.8280482, lng: -117.092462 },
  { accuracy: 145.8, lat: 32.8285671, lng: -117.091834 },
  { accuracy: 323.456, lat: 32.8290893, lng: -117.092215 },
  { accuracy: 89.12, lat: 32.8295245, lng: -117.091687 },
  { accuracy: 425.67, lat: 32.8301456, lng: -117.092123 },
  { accuracy: 142.891, lat: 32.8307034, lng: -117.091576 },
  { accuracy: 67.234, lat: 32.8312789, lng: -117.092467 },
  { accuracy: 298.432, lat: 32.8318567, lng: -117.091756 },
  { accuracy: 156.789, lat: 32.8324321, lng: -117.092234 },
  { accuracy: 387.654, lat: 32.8329912, lng: -117.091589 },
  { accuracy: 78.345, lat: 32.8335234, lng: -117.092112 },
  { accuracy: 532.876, lat: 32.8340567, lng: -117.091634 },
  { accuracy: 256.234, lat: 32.8345789, lng: -117.092198 },
  { accuracy: 167.891, lat: 32.8351023, lng: -117.091756 },
  { accuracy: 98.456, lat: 32.8356345, lng: -117.092323 },
  { accuracy: 389.567, lat: 32.8361678, lng: -117.091887 },
  { accuracy: 234.123, lat: 32.8366912, lng: -117.092445 },
  { accuracy: 456.789, lat: 32.8372156, lng: -117.091898 },
  { accuracy: 167.234, lat: 32.8377389, lng: -117.092467 },
  { accuracy: 123.567, lat: 32.8382623, lng: -117.091934 },
  { accuracy: 345.891, lat: 32.8387856, lng: -117.092401 },
  { accuracy: 278.234, lat: 32.8393089, lng: -117.091978 },
  { accuracy: 156.678, lat: 32.8398323, lng: -117.092445 },
  { accuracy: 489.345, lat: 32.8403556, lng: -117.091912 },
  { accuracy: 67.123, lat: 32.8408789, lng: -117.092489 },
  { accuracy: 389.567, lat: 32.8414023, lng: -117.091956 },
  { accuracy: 234.891, lat: 32.8419256, lng: -117.092423 },
  { accuracy: 123.234, lat: 32.8424489, lng: -117.091890 },
  { accuracy: 456.678, lat: 32.8429723, lng: -117.092467 },
  { accuracy: 167.345, lat: 32.8434956, lng: -117.091934 }
];

let sampleIndex = 0;

async function processNextSample(): Promise<void> {
  if (sampleIndex >= sampleData.length) {
    console.log('모든 샘플 데이터 처리 완료');
    return;
  }

  const currentTimestamp = Date.now();
  if (lastTimestamp) {
    const dt = (currentTimestamp - lastTimestamp) / 1000.0;
    kalmanFilter.predict(dt);
  }
  lastTimestamp = currentTimestamp;

  const sample = sampleData[sampleIndex];
  const { lat, lng, accuracy } = sample;

  if (!kalmanFilter.isInitialized) {
    kalmanFilter.initializeState(lat, lng);
  }
  
  const earthRadius = 111139;
  const stdDev = accuracy / 2;
  const variance = (stdDev / earthRadius) ** 2;
  const R = [
    [variance, 0],
    [0, variance]
  ];

  const z = [[lat], [lng]];
  kalmanFilter.update(z, R);
  
  const filteredState = kalmanFilter.getState();
  const filteredPosition = { lat: filteredState.lat, lng: filteredState.lng };

  map.setCenter(filteredPosition);
  map.setZoom(18);
  
  const rawLatLng = new google.maps.LatLng(lat, lng);
  const filteredLatLng = new google.maps.LatLng(filteredPosition.lat, filteredPosition.lng);
  
  rawCoordinates.push(rawLatLng);
  filteredCoordinates.push(filteredLatLng);
  
  if (rawPath) {
    rawPath.setPath(rawCoordinates);
  }
  
  if (filteredPath) {
    filteredPath.setPath(filteredCoordinates);
  }

  await addPositionMarkers({ lat, lng }, filteredPosition);
  
  console.log(`Sample ${sampleIndex + 1}:`, 'Raw Location:', { lat, lng, accuracy });
  console.log(`Sample ${sampleIndex + 1}:`, 'Filtered Location:', filteredPosition);

  sampleIndex++;
}

function resetData(): void {
  rawCoordinates = [];
  filteredCoordinates = [];
  sampleIndex = 0;
  lastTimestamp = null;
  kalmanFilter = new KalmanFilter({ processNoise: 0.0001, measurementNoise: 0.001 });
  
  if (rawPath) {
    rawPath.setPath([]);
  }
  
  if (filteredPath) {
    filteredPath.setPath([]);
  }

  rawMarkers.forEach(marker => {
    marker.map = null;
  });
  rawMarkers = [];

  filteredMarkers.forEach(marker => {
    marker.map = null;
  });
  filteredMarkers = [];
}

initMap().then(() => {
  const button = document.getElementById('geolocation-btn');
  const resetButton = document.getElementById('reset-btn');
  
  if (button) {
    button.addEventListener('click', processNextSample);
  }
  
  if (resetButton) {
    resetButton.addEventListener('click', resetData);
  }
});